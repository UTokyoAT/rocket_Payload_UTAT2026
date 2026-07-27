#pragma once
#include "shared.h"
#include <Deployer.h>

// ミッションのシーケンス状態機械（SETTING→LAUNCH→DETACH→UNFOLD→NAVIGATE→GOAL）。
// 「tick」は本タスク自身の周期（MISSION_TICK_MS=10ms）を指す。tick数（LAUNCH_CONFIRM_TICKS等）は
// 50ms周期だった頃の値のまま据え置いているため、各確認時間は5分の1に短縮されている
// （例: LAUNCH_CONFIRM_TICKS=5は50ms→10ms化に伴い0.25秒から0.05秒相当になった）。
// ただしNAVIGATEの停止判定だけはGPSの実際の更新間隔（約1Hz）に依存するため、
// tick数ではなく新規GPS fix（SensorData::gpsFixSeq）の到着回数で数える。

static const uint32_t MISSION_TICK_MS = 10;

// SETTING: 気圧ベースラインはSensor::begin()で校正済み。ここではGPS衛星捕捉と
// 目的地（打ち上げ地点）座標の取得を待つ。
static const int      SETTING_MIN_SATELLITES = 10;
static const uint32_t SETTING_TIMEOUT_MS = 10UL * 60 * 1000;

// LAUNCH: 打ち上げ検知（タイムアウトなし。発射操作を待ち続ける）
static const float LAUNCH_ALT_THRESHOLD_M = 8.0f;
static const int   LAUNCH_CONFIRM_TICKS   = 5;

// DETACH: ロケット分離後、着地（高度変化が収まる）を検知（タイムアウトなし）
static const float DETACH_ALT_DELTA_THRESHOLD_M = 1.0f;
static const int   DETACH_CONFIRM_TICKS         = 5;

// UNFOLD: パラシュート分離後、姿勢が安定し直立していることを確認
static const uint32_t UNFOLD_TIMEOUT_MS           = 5UL * 60 * 1000;
static const int      UNFOLD_STABLE_WINDOW_TICKS  = 10;   // この件数分のroll/pitch変化幅を見る
static const float    UNFOLD_STABLE_RANGE_DEG     = 10.0f;
static const int      UNFOLD_UPRIGHT_CONFIRM_TICKS = 5;
static const float    UNFOLD_UPRIGHT_ABS_DEG       = 20.0f;

// NAVIGATE: GNSS誘導で目的地（打ち上げ地点）へ走行し、停止（到達）を検知
static const uint32_t NAVIGATE_TIMEOUT_MS          = 10UL * 60 * 1000;
static const int      NAVIGATE_STOP_CONFIRM_FIXES  = 5;
static const double   NAVIGATE_STOP_DELTA_DEG      = 0.00001;

void taskMission(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    Deployer deployer;
    deployer.begin();

    MissionState state = MissionState::SETTING;
    uint32_t stateEnteredAt = millis();

    // LAUNCH用
    int launchConfirmCount = 0;

    // DETACH用
    bool  detachAltInitialized = false;
    float lastAltForDetach = 0.0f;
    int   detachConfirmCount = 0;

    // UNFOLD用（直近UNFOLD_STABLE_WINDOW_TICKS件のroll/pitchのリングバッファ）
    float unfoldRollBuf[UNFOLD_STABLE_WINDOW_TICKS];
    float unfoldPitchBuf[UNFOLD_STABLE_WINDOW_TICKS];
    int   unfoldBufCount = 0;
    int   unfoldBufIndex = 0;
    int   unfoldUprightCount = 0;

    // NAVIGATE用
    bool     navigateFixInitialized = false;
    uint32_t lastGpsFixSeq = 0;
    double   lastFixLat = 0.0;
    double   lastFixLon = 0.0;
    int      navigateStopCount = 0;

    // 状態遷移のたびに各状態専用のカウンタをリセットし、DETACH/UNFOLD突入時は
    // ニクロム線を1回だけ通電する（分離・パラシュート分離はやり直しがきかないため
    // ここでの発火漏れ・二重発火は避ける）。
    auto transitionTo = [&](MissionState next) {
        state = next;
        stateEnteredAt = millis();
        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->state = next;
            xSemaphoreGive(s->mutex);
        }

        launchConfirmCount = 0;
        detachAltInitialized = false;
        detachConfirmCount = 0;
        unfoldBufCount = 0;
        unfoldBufIndex = 0;
        unfoldUprightCount = 0;
        navigateFixInitialized = false;
        navigateStopCount = 0;

        if (next == MissionState::DETACH) {
            deployer.deployRocket();
        } else if (next == MissionState::UNFOLD) {
            deployer.deployParachute();
        }
    };

    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        float alt, roll, pitch;
        bool gpsValid;
        int gpsSatellites;
        double lat, lon;
        uint32_t gpsFixSeq;

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            alt           = s->latest.alt;
            roll          = s->latest.roll;
            pitch         = s->latest.pitch;
            gpsValid      = s->latest.gpsValid;
            gpsSatellites = s->latest.gpsSatellites;
            lat           = s->latest.lat;
            lon           = s->latest.lon;
            gpsFixSeq     = s->latest.gpsFixSeq;
            xSemaphoreGive(s->mutex);
        }

        uint32_t elapsed = millis() - stateEnteredAt;

        switch (state) {
            case MissionState::SETTING: {
                if (gpsSatellites >= SETTING_MIN_SATELLITES && gpsValid) {
                    // 目的地＝打ち上げ地点として、今取得できているGPS位置をそのまま設定する
                    // （地上局がGET /goalを送ればこの後いつでも優先的に上書きされる）
                    if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
                        s->goalLat = lat;
                        s->goalLon = lon;
                        xSemaphoreGive(s->mutex);
                    }
                    transitionTo(MissionState::LAUNCH);
                } else if (elapsed > SETTING_TIMEOUT_MS) {
                    transitionTo(MissionState::ABORTED);
                }
                break;
            }

            case MissionState::LAUNCH: {
                launchConfirmCount = (alt > LAUNCH_ALT_THRESHOLD_M) ? launchConfirmCount + 1 : 0;
                if (launchConfirmCount >= LAUNCH_CONFIRM_TICKS) {
                    transitionTo(MissionState::DETACH);
                }
                break;
            }

            case MissionState::DETACH: {
                if (!detachAltInitialized) {
                    lastAltForDetach = alt;
                    detachAltInitialized = true;
                }
                float delta = fabsf(alt - lastAltForDetach);
                lastAltForDetach = alt;
                detachConfirmCount = (delta < DETACH_ALT_DELTA_THRESHOLD_M) ? detachConfirmCount + 1 : 0;
                if (detachConfirmCount >= DETACH_CONFIRM_TICKS) {
                    transitionTo(MissionState::UNFOLD);
                }
                break;
            }

            case MissionState::UNFOLD: {
                unfoldRollBuf[unfoldBufIndex]  = roll;
                unfoldPitchBuf[unfoldBufIndex] = pitch;
                unfoldBufIndex = (unfoldBufIndex + 1) % UNFOLD_STABLE_WINDOW_TICKS;
                if (unfoldBufCount < UNFOLD_STABLE_WINDOW_TICKS) unfoldBufCount++;

                bool settled = false;
                if (unfoldBufCount >= UNFOLD_STABLE_WINDOW_TICKS) {
                    float rollMin = unfoldRollBuf[0], rollMax = unfoldRollBuf[0];
                    float pitchMin = unfoldPitchBuf[0], pitchMax = unfoldPitchBuf[0];
                    for (int i = 1; i < UNFOLD_STABLE_WINDOW_TICKS; i++) {
                        if (unfoldRollBuf[i]  < rollMin)  rollMin  = unfoldRollBuf[i];
                        if (unfoldRollBuf[i]  > rollMax)  rollMax  = unfoldRollBuf[i];
                        if (unfoldPitchBuf[i] < pitchMin) pitchMin = unfoldPitchBuf[i];
                        if (unfoldPitchBuf[i] > pitchMax) pitchMax = unfoldPitchBuf[i];
                    }
                    settled = (rollMax - rollMin <= UNFOLD_STABLE_RANGE_DEG) &&
                              (pitchMax - pitchMin <= UNFOLD_STABLE_RANGE_DEG);
                }

                bool upright = settled && fabsf(roll) <= UNFOLD_UPRIGHT_ABS_DEG &&
                                           fabsf(pitch) <= UNFOLD_UPRIGHT_ABS_DEG;
                unfoldUprightCount = upright ? unfoldUprightCount + 1 : 0;

                if (unfoldUprightCount >= UNFOLD_UPRIGHT_CONFIRM_TICKS) {
                    transitionTo(MissionState::NAVIGATE);
                } else if (elapsed > UNFOLD_TIMEOUT_MS) {
                    // 変化は収まった（安定した）が20度以内に収まらない＝回収不能と判断
                    transitionTo(MissionState::ABORTED);
                }
                break;
            }

            case MissionState::NAVIGATE: {
                // GPSは1Hz程度でしか更新されないため、tickではなく新規fixの到着回数で数える
                if (gpsValid && gpsFixSeq != lastGpsFixSeq) {
                    if (navigateFixInitialized) {
                        double deltaLat = fabs(lat - lastFixLat);
                        double deltaLon = fabs(lon - lastFixLon);
                        // 進行方向が南北/東西いずれかに近いと片方の軸だけほぼ変化しなくなるため、
                        // 両軸とも動いていないことを確認する（ORだと直進中でも誤検知する）
                        bool stopped = (deltaLat <= NAVIGATE_STOP_DELTA_DEG) &&
                                       (deltaLon <= NAVIGATE_STOP_DELTA_DEG);
                        navigateStopCount = stopped ? navigateStopCount + 1 : 0;
                    }
                    lastFixLat = lat;
                    lastFixLon = lon;
                    lastGpsFixSeq = gpsFixSeq;
                    navigateFixInitialized = true;
                }

                if (navigateStopCount >= NAVIGATE_STOP_CONFIRM_FIXES) {
                    transitionTo(MissionState::GOAL);
                } else if (elapsed > NAVIGATE_TIMEOUT_MS) {
                    transitionTo(MissionState::ABORTED);
                }
                break;
            }

            case MissionState::GOAL:
            case MissionState::ABORTED:
                // 回収支援のLED点滅を鳴らし続ける。GOAL/ABORTEDの区別はmission_stateの値で行う
                // （XIAO2はNAVIGATE以外では自律走行を行わないため、両状態ともモータは自然に停止する）
                deployer.beepPattern();
                break;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MISSION_TICK_MS));
    }
}
