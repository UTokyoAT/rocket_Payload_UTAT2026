#include <Arduino.h>
#include <Sensor.h>
#include <GPS.h>
#include <PID.h>
#include <Heading.h>
#include "spi_protocol.h"
#include <SpiLinkMaster.h>

// XIAO1側：GPS誘導ナビゲーション統合確認テスト。
// PlatformIOで env:test-navigation を選択してXIAO1へ書き込む。
//
// XIAO2側は本番ファームウェア（env:xiao2）をそのまま書き込んでおくこと。
// モーター・WiFi・SPIスレーブはすべて本番のXIAO2がそのまま使えるため、
// テスト専用のXIAO2ファームウェアは不要（配線も本番のXIAO1⇔XIAO2 SPI接続のまま）。
//
// 地上局がXIAO2へWiFiで目的地（GET /goal?lat=..&lon=..）を送ると、SPI応答経由でこのXIAO1に
// 転送される。GPS fixを取得できていて、かつ目的地も受信済みのときだけmission_state=NAVIGATEを
// XIAO2へ送る。XIAO2は本番と同じロジックでNAVIGATEのときしかモーターを動かさないため、
// fix未取得・目的地未受信の間は自動的にモーターが止まったままになる。
// 本番のtask_mission.h（SETTING〜UNFOLDの発射・分離シーケンス）は経由しない
// （誘導ロジック単体を素早く確認するためのテストなので、衛星ロックさえできれば
//   すぐNAVIGATE相当の判定に入る）。
//
// テレメトリ確認・目的地送信はXIAO2のWiFi経由（CanSat-AP、パスワード cansat2026）。
//   - ブラウザ: http://192.168.4.1 （pid_output・destination_yaw・yaw・GPS座標を表示）
//   - 目的地送信: http://192.168.4.1/goal?lat=..&lon=..
//   - まずは実走行させず、車輪を浮かせた状態で確認すること
//     （PIDゲインは実機でのチューニングが必要）。
//
// ヘディング（yaw）はBMM350（地磁気）ではなくHeading（ジャイロ+GPS courseの相補フィルタ）を使う。
// 実機ではBMM350の近くにモーター等の強い磁気源があり地磁気の生値がほとんど動かず、
// コンパスとして使えなかったため（lib/Heading/Heading.hのコメント参照）。
// GPS courseは真北基準なので磁気偏角の補正も不要になった。

// XIAO ESP32S3: D6(GPIO43)にGPSモジュールのTXを、D7(GPIO44)にGPSモジュールのRXを接続
// （回路図のネット名がD6=UART_RX、D7=UART_TXになっているため、それに合わせた割り当て）
static const int GPS_RX_PIN = 43;  // D6
static const int GPS_TX_PIN = 44;  // D7

static Sensor sensor;
static GPS gps;
static Heading heading;
static SpiLinkMaster spiLink;
// TODO: 実機でゲイン調整。100Hz周期(dt=0.01s)前提
static PID headingPid(2.0f, 0.0f, 0.5f, -255.0f, 255.0f);

// XIAO2がWiFiで受信した目的地（GET /goal）をSPI応答経由で受け取って保持する
static double   _goalLat  = 0.0;
static double   _goalLon  = 0.0;
static bool     _haveGoal = false;

static uint32_t _lastUpdateMs = 0;

static float normalizeAngle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void setup() {
    Serial.begin(115200);

    sensor.begin();
    gps.begin(GPS_RX_PIN, GPS_TX_PIN);
    spiLink.begin();

    // ジャイロのヨー軸バイアスを較正する。この間は機体を静止させておくこと。
    Serial.println("[TEST] Calibrating gyro yaw bias... keep the device still.");
    float biasSum = 0.0f;
    const int BIAS_SAMPLES = 200;
    for (int i = 0; i < BIAS_SAMPLES; i++) {
        sensor.update();
        biasSum += sensor.getGyroX();
        delay(5);
    }
    float gyroBias = biasSum / BIAS_SAMPLES;
    heading.begin(gyroBias);
    Serial.printf("[TEST] gyro yaw bias = %.3f deg/s\n", gyroBias);

    _lastUpdateMs = millis();
}

void loop() {
    gps.update();
    // getLat()/getLon()より前に呼ぶ（呼ぶと内部の更新フラグが消費される）
    bool gpsFixUpdated = gps.locationUpdated();
    sensor.update();

    uint32_t now = millis();
    float dt = (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    bool gpsValid = gps.isValid();

    heading.update(sensor.getGyroX(), dt,
                    gpsFixUpdated, gps.isCourseValid(), gps.getCourse(), gps.getSpeedMps());
    float yaw = heading.get();

    float pidOutput = 0.0f;
    float destinationYaw = 0.0f;

    if (gpsValid && _haveGoal) {
        destinationYaw = gps.bearingTo(_goalLat, _goalLon);
        float error = normalizeAngle(destinationYaw - yaw);
        pidOutput = headingPid.update(error, dt);
    } else {
        headingPid.reset();  // 目的地未受信・fix未取得の間は積分を溜め込まない
    }

    SpiFrameToXiao2 out{};
    out.timestamp_ms    = now;
    out.alt             = sensor.getAltitude();
    out.roll            = sensor.getRoll();
    out.pitch           = sensor.getPitch();
    out.yaw             = yaw;
    out.lat             = static_cast<float>(gps.getLat());
    out.lon             = static_cast<float>(gps.getLon());
    // fix未取得 or 目的地未受信の間はNAVIGATE以外を送り、XIAO2側にモーターを止めさせる
    out.mission_state   = static_cast<uint8_t>((gpsValid && _haveGoal) ? MissionState::NAVIGATE
                                                                        : MissionState::SETTING);
    out.pid_output       = pidOutput;
    out.destination_yaw  = destinationYaw;

    SpiFrameFromXiao2 in = spiLink.transfer(out);
    if (in.goal_valid) {
        _goalLat  = in.goal_lat;
        _goalLon  = in.goal_lon;
        _haveGoal = true;
    }

    delay(10);  // 100Hz目安（実測dtでPID計算するので多少ズレても問題ない）
}
