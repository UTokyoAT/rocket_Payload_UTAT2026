#include <Arduino.h>
#include <Sensor.h>
#include <Deployer.h>
#include "spi_protocol.h"
#include <SpiLinkMaster.h>

// XIAO1側：GPS・地磁気（BMM350）を使わない開ループ（推測航法）誘導テスト。
// 打ち上げ待ち〜ロケット分離〜パラシュート分離までは本番のtask_mission.hと全く同じ
// シーケンス（ニクロム発火含む）を行い、最後のNAVIGATE（GPS誘導）だけを
// 「10m直進→半径10mの円軌道」の推測航法に置き換えたもの。
// PlatformIOで env:test-dead-reckoning を選択してXIAO1へ書き込む。
//
// 背景: 実機のGPS・BMM350の調子が悪く、本番の誘導（task_navigation.h）や
// test_navigationのようなGPS/ヘディングに依存した誘導テストが行えない。
// そこで気圧（高度）・ジャイロ+加速度（roll/pitch）だけで打ち上げ〜分離〜着地〜
// パラシュート分離〜姿勢安定を検知し、そこから先はセンサーフィードバックなしの
// 時間ベース推測航法で走行する。
//
// ★本番と同じくニクロム線を実際に発火させる（LAUNCH/DETACH/UNFOLD/しきい値は
//   本番task_mission.hと同一値）。ロケット・パラシュートを実際に取り付けた
//   本番同様の状態でのみ書き込むこと。ニクロム発火なしで駆動ロジックだけを
//   机上で繰り返し試したい場合はtest_dead_reckoning_post_detachを使う。
//
// XIAO2側は本番ファームウェア（env:xiao2）をそのまま書き込んでおくこと
// （モーター・WiFiはすべて本番のXIAO2がそのまま使える。配線も本番のXIAO1⇔XIAO2
//   SPI接続のまま）。ただし直進フェーズを「最大出力」にするため、本番の
//   src/xiao2/main.cpp の BASE_SPEED を150→255に変更済みであることが前提
//   （このリポジトリでは既に変更済み）。
//
// フェーズ構成:
//   1. LAUNCH   … 高度3m超をLAUNCH_CONFIRM_TICKS連続検知するまで待つ（本番と同一。打ち上げ検知）
//   2. DETACH   … LAUNCH検知直後にdeployRocket()でロケットから分離。以後、
//                 高度変化が収まる（＝パラシュート降下後に着地）までDETACH_ALT_*で待つ（本番と同一）
//   3. UNFOLD   … DETACH確定直後にdeployParachute()でパラシュートを分離。
//                 roll/pitchが安定し直立していることを確認（本番と同一）
//   4. DRIVE_STRAIGHT … その場から最大出力（左右均等）で直進。STRAIGHT_DISTANCE_M分の
//                        時間だけ走ってから止める（距離センサーが無いため時間ベース）
//   5. DRIVE_CIRCLE   … 左右に一定の差をつけたまま走行し、半径CIRCLE_RADIUS_Mの円を描く
//   6. GOAL     … 走行完了。回収支援のLED点滅
//   7. ABORTED  … UNFOLDがタイムアウト（姿勢が20度以内に収まらない＝転倒等）した場合。
//                 モーターは動かさずLED点滅のみ
//
// SETTING（GPS衛星捕捉・目的地座標取得）は本番の前段にあるが、GPS自体を使わない
// このテストでは意味がないため丸ごと省略し、起動後すぐLAUNCH待ちに入る。
//
// SpiFrameToXiao2.mission_stateは本番のMissionStateをそのまま流用する
// （プロトコル自体は変更しない）。XIAO2はmission_state==NAVIGATEのときだけ
// pid_outputを左右差動としてモーターに反映する安全ゲートを持っているため、
// 走行させたいフェーズ（4・5）だけNAVIGATEを送り、それ以外はLAUNCH/DETACH/UNFOLD/
// GOAL/ABORTEDを送ってモーターを止めたままにする。
//
// 実機での事前確認・チューニング手順:
//   a. STRAIGHT_SPEED_MPS … 実機を平地でBASE_SPEED=255（最大出力）で走らせ、
//      実測の並進速度[m/s]に書き換える（直進10mの所要時間・円軌道の所要時間の
//      両方の計算に使う）
//   b. CIRCLE_TURN_PID_OUTPUT … DRIVE_CIRCLEだけ単独で走らせて実際の旋回半径を
//      測り、半径がCIRCLE_RADIUS_Mに近づくよう値を調整する
//      （値を大きくすると旋回がきつくなる＝半径が小さくなる）
//
// --- 走行軌跡の推定（着地位置を原点とした簡易マップ）---
// GPSが無いため、加速度センサー（水平面の並進加速度）とジャイロ（ヨー角速度の積分）
// だけで機体位置(x,y)を推測する開ループの推測航法（dead reckoning）。UNFOLD確定時
// （＝着地・パラシュート分離・姿勢安定を確認した瞬間）を原点(0,0)・ヘディング0度として
// リセットし、以後は加速度の2階積分で位置を、ジャイロの積分でヘディングを進める。
// ヘディングは地磁気・GPSを使わないため真北基準ではなく、リセット時に機体が向いていた
// 方向を0度とする相対角（このテスト内だけで閉じたローカル座標系）。
// 低価格IMUの積分だけに頼るため、時間とともに誤差が蓄積し実際の軌跡から大きくズレていく
// （特に加速度の2階積分は数秒でも無視できないドリフトが出る）。walkthrough確認・大まかな
// 軌跡の可視化用と割り切ること。
//
// SpiFrameToXiao2のlat/lonフィールド（本来はGPS度数）を、このテストでは
// 「原点からのx/y距離[m]」として流用して送る（プロトコル自体は変更しない。
// 数値の意味づけをこのテストだけ変えている）。yawフィールドも上記の相対ヘディング[deg]を
// 送る。地上局側はground/receiver_dead_reckoning.py（GPS度数変換をせず、
// 受信したlat/lonをそのままメートル座標として描画する版）を使うこと
// （本番用のreceiver.pyはGPS度数前提なのでこのテストには使えない）。

static Sensor sensor;
static Deployer deployer;
static SpiLinkMaster spiLink;

// --- LAUNCH ---
static const float LAUNCH_ALT_THRESHOLD_M = 5.0f;  // 本番task_mission.h（3.0f）から変更
static const int   LAUNCH_CONFIRM_TICKS   = 5;
// 電源投入直後は気圧センサーの値が安定していなかったり、設置作業中の持ち運びで誤検知しうる
// ため、起動からこの時間が経過するまでは高度判定（LAUNCH_ALT_THRESHOLD_M超の検知）自体を
// 開始しない。
static const uint32_t LAUNCH_ARM_DELAY_MS = 10UL * 60 * 1000;  // 10分

// --- DETACH（本番と同じ値）---
static const int   DETACH_ALT_SAMPLE_TICKS      = 100;  // 100tick(=1秒)ごとに間引いて高度差分を見る
static const float DETACH_ALT_DELTA_THRESHOLD_M = 1.0f;
static const int   DETACH_CONFIRM_TICKS         = 5;

// --- UNFOLD（本番と同じ値）---
static const uint32_t UNFOLD_TIMEOUT_MS           = 5UL * 60 * 1000;
static const int      UNFOLD_STABLE_WINDOW_TICKS  = 10;
static const float    UNFOLD_STABLE_RANGE_DEG     = 10.0f;
static const int      UNFOLD_UPRIGHT_CONFIRM_TICKS = 5;
static const float    UNFOLD_UPRIGHT_ABS_DEG       = 20.0f;

// --- 走行パラメータ（実機で要チューニング。上記コメント参照）---
static const float STRAIGHT_DISTANCE_M   = 10.0f;
static const float CIRCLE_RADIUS_M       = 10.0f;
static const float CIRCLE_LAPS           = 1.0f;   // 円軌道を何周走るか
// TODO: 実機のBASE_SPEED=255（最大出力）直進時の実測並進速度[m/s]に置き換える
static const float STRAIGHT_SPEED_MPS    = 0.5f;
// TODO: 実機で半径CIRCLE_RADIUS_Mに近づくよう調整するpid_output値（旋回量）。
// 正の値で左旋回（左が減速・右が最大のまま）になる。負にすると右旋回。
static const float CIRCLE_TURN_PID_OUTPUT = 15.0f;

static const uint32_t MISSION_TICK_MS = 10;  // 100Hz（本番task_mission.hと同一周期）

// --- 位置推定（推測航法）用 ---
static const int BIAS_CALIBRATION_SAMPLES = 200;  // 静止状態で平均を取るサンプル数

// 加速度のローパスフィルタ（指数移動平均）の重み。大きいほど反応が速いが荒く、
// 小さいほど滑らかだが遅れる。TODO: 実機で調整
static const float ACCEL_LPF_ALPHA = 0.2f;
// フィルタ後の加速度がこの値未満ならノイズとみなし0扱いにする（静止に近い状態での
// 見せかけの動きを抑える）。TODO: 実機で調整
static const float ACCEL_DEADBAND_MPS2 = 0.05f;

static float gGyroYawBiasDegPerSec = 0.0f;  // ヨー軸ジャイロの静止バイアス
static float gAccelForwardBias = 0.0f;      // 前後方向（Z軸）加速度の静止バイアス [m/s^2]
static float gAccelLateralBias = 0.0f;      // 左右方向（Y軸）加速度の静止バイアス [m/s^2]

static float gHeadingDeg = 0.0f;  // リセット時点を0度とする相対ヘディング
static float gVelX = 0.0f, gVelY = 0.0f;  // ローカル座標系での速度 [m/s]
static float gPosX = 0.0f, gPosY = 0.0f;  // 原点（リセット地点）からの位置 [m]
static uint32_t gLastPosUpdateUs = 0;
static float gFilteredForwardAccel = 0.0f;
static float gFilteredLateralAccel = 0.0f;

static float normalizeAngleDeg(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// ジャイロ・加速度のバイアスを較正する（機体を静止させた状態で呼ぶこと）。
// 二階積分は残留バイアスが時間の2乗で効いてくるため、実際に走り出す直前に取り直すほど
// 精度が上がる（起動直後の較正だけだと、発射〜分離〜着地の衝撃や温度変化でズレる）。
static void calibrateBias() {
    Serial.println("[TEST] calibrating gyro/accel bias... keep the device still.");
    float gyroSum = 0.0f, accelForwardSum = 0.0f, accelLateralSum = 0.0f;
    for (int i = 0; i < BIAS_CALIBRATION_SAMPLES; i++) {
        sensor.update();
        gyroSum          += sensor.getGyroX();
        accelForwardSum  += sensor.getAccelZ();
        accelLateralSum  += sensor.getAccelY();
        delay(5);
    }
    gGyroYawBiasDegPerSec = gyroSum / BIAS_CALIBRATION_SAMPLES;
    gAccelForwardBias     = accelForwardSum / BIAS_CALIBRATION_SAMPLES;
    gAccelLateralBias     = accelLateralSum / BIAS_CALIBRATION_SAMPLES;
    Serial.printf("[TEST] gyro yaw bias=%.3fdeg/s accel bias fwd=%.3f lat=%.3fm/s^2\n",
                  gGyroYawBiasDegPerSec, gAccelForwardBias, gAccelLateralBias);
}

// UNFOLD確定（＝着地位置）を原点・ヘディング0度としてリセットする
static void resetPositionEstimate() {
    gHeadingDeg = 0.0f;
    gVelX = 0.0f; gVelY = 0.0f;
    gPosX = 0.0f; gPosY = 0.0f;
    gFilteredForwardAccel = 0.0f;
    gFilteredLateralAccel = 0.0f;
    gLastPosUpdateUs = micros();
}

// 毎ループ呼ぶ。ジャイロでヘディングを、加速度（水平面・バイアス除去・平滑化・不感帯
// 処理済み）をヘディング方向へ回転してワールド座標系の加速度とし、2階積分で位置を進める。
static void updatePositionEstimate() {
    uint32_t nowUs = micros();
    float dt = (nowUs - gLastPosUpdateUs) / 1000000.0f;
    gLastPosUpdateUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) return;  // 起動直後・オーバーフロー等の異常dtは積分しない

    gHeadingDeg = normalizeAngleDeg(gHeadingDeg + (sensor.getGyroX() - gGyroYawBiasDegPerSec) * dt);

    float rawForwardAccel = sensor.getAccelZ() - gAccelForwardBias;  // 前方+
    float rawLateralAccel = sensor.getAccelY() - gAccelLateralBias;  // 右方+
    gFilteredForwardAccel += ACCEL_LPF_ALPHA * (rawForwardAccel - gFilteredForwardAccel);
    gFilteredLateralAccel += ACCEL_LPF_ALPHA * (rawLateralAccel - gFilteredLateralAccel);

    float forwardAccel = (fabsf(gFilteredForwardAccel) < ACCEL_DEADBAND_MPS2) ? 0.0f : gFilteredForwardAccel;
    float lateralAccel = (fabsf(gFilteredLateralAccel) < ACCEL_DEADBAND_MPS2) ? 0.0f : gFilteredLateralAccel;

    float headingRad = gHeadingDeg * DEG_TO_RAD;
    float worldAx = forwardAccel * sinf(headingRad) + lateralAccel * cosf(headingRad);
    float worldAy = forwardAccel * cosf(headingRad) - lateralAccel * sinf(headingRad);

    gVelX += worldAx * dt;
    gVelY += worldAy * dt;
    gPosX += gVelX * dt;
    gPosY += gVelY * dt;
}

enum class TestPhase {
    LAUNCH,
    DETACH,
    UNFOLD,
    DRIVE_STRAIGHT,
    DRIVE_CIRCLE,
    GOAL,
    ABORTED
};

static const char* phaseName(TestPhase p) {
    switch (p) {
        case TestPhase::LAUNCH:         return "LAUNCH";
        case TestPhase::DETACH:         return "DETACH";
        case TestPhase::UNFOLD:         return "UNFOLD";
        case TestPhase::DRIVE_STRAIGHT: return "DRIVE_STRAIGHT";
        case TestPhase::DRIVE_CIRCLE:   return "DRIVE_CIRCLE";
        case TestPhase::GOAL:           return "GOAL";
        case TestPhase::ABORTED:        return "ABORTED";
    }
    return "?";
}

static TestPhase gPhase = TestPhase::LAUNCH;
static uint32_t  gPhaseEnteredMs = 0;

// LAUNCH用
static int launchConfirmCount = 0;

// DETACH用
static bool  detachAltInitialized = false;
static float lastAltForDetach = 0.0f;
static int   detachConfirmCount = 0;
static int   detachTickCount = 0;

// UNFOLD用（直近UNFOLD_STABLE_WINDOW_TICKS件のroll/pitchのリングバッファ）
static float unfoldRollBuf[UNFOLD_STABLE_WINDOW_TICKS];
static float unfoldPitchBuf[UNFOLD_STABLE_WINDOW_TICKS];
static int   unfoldBufCount = 0;
static int   unfoldBufIndex = 0;
static int   unfoldUprightCount = 0;

static uint32_t straightDurationMs = 0;
static uint32_t circleDurationMs   = 0;

// 状態遷移のたびに各状態専用のカウンタをリセットし、DETACH/UNFOLD突入時は
// 本番と同じくニクロム線を1回だけ通電する（分離・パラシュート分離はやり直しがきかないため
// ここでの発火漏れ・二重発火は避ける）。
static void transitionTo(TestPhase next) {
    Serial.printf("[TEST] %s -> %s\n", phaseName(gPhase), phaseName(next));
    gPhase = next;
    gPhaseEnteredMs = millis();

    launchConfirmCount   = 0;
    detachAltInitialized = false;
    detachConfirmCount   = 0;
    detachTickCount      = 0;
    unfoldBufCount        = 0;
    unfoldBufIndex        = 0;
    unfoldUprightCount    = 0;

    if (next == TestPhase::DETACH) {
        Serial.println("[TEST] firing deployRocket() nichrome...");
        deployer.deployRocket();
    } else if (next == TestPhase::UNFOLD) {
        Serial.println("[TEST] firing deployParachute() nichrome...");
        deployer.deployParachute();
    } else if (next == TestPhase::DRIVE_STRAIGHT) {
        // 着地・パラシュート分離・姿勢安定を確認した瞬間＝ここを地図の原点とする。
        // 走り出す直前でまだ静止しているはずなので、バイアスもここで取り直してから
        // リセットする（起動直後の較正だけに頼らず、残留誤差をできるだけ削る）。
        calibrateBias();
        resetPositionEstimate();
        Serial.println("[TEST] position estimate reset to origin (landing point)");
    }
}

void setup() {
    Serial.begin(115200);

    sensor.begin();  // BMM350（地磁気）が繋がっていなくても高度・roll/pitchは使える
    deployer.begin();
    spiLink.begin();

    // 位置推定用のジャイロ・加速度バイアスを較正する。打ち上げ前で機体は発射台上に
    // 静止している前提（この間は動かさないこと）。DRIVE_STRAIGHT突入直前にも取り直す
    // ため、これは主に「較正できていない状態でLAUNCH判定に入らない」ための初回較正。
    calibrateBias();
    resetPositionEstimate();

    straightDurationMs = static_cast<uint32_t>((STRAIGHT_DISTANCE_M / STRAIGHT_SPEED_MPS) * 1000.0f);
    // 円軌道走行中は片輪を減速するため並進速度がSTRAIGHT_SPEED_MPSよりわずかに落ちるが、
    // 未知数（実機のトレッド幅・PWM-速度特性）が多いため近似としてSTRAIGHT_SPEED_MPSを流用する。
    // 実測してズレが大きければCIRCLE_LAPS到達前後で手動停止するか、この式を調整すること。
    float circumferenceM = 2.0f * PI * CIRCLE_RADIUS_M * CIRCLE_LAPS;
    circleDurationMs = static_cast<uint32_t>((circumferenceM / STRAIGHT_SPEED_MPS) * 1000.0f);

    Serial.printf("[TEST] straight: %.1fm @ %.2fm/s -> %lums\n",
                  STRAIGHT_DISTANCE_M, STRAIGHT_SPEED_MPS, (unsigned long)straightDurationMs);
    Serial.printf("[TEST] circle: r=%.1fm x%.1flap -> %lums (pid_output=%.1f)\n",
                  CIRCLE_RADIUS_M, CIRCLE_LAPS, (unsigned long)circleDurationMs, CIRCLE_TURN_PID_OUTPUT);
    Serial.println("[TEST] waiting for launch (alt > 3m)...");

    gPhaseEnteredMs = millis();
}

void loop() {
    sensor.update();
    // NAVIGATE（DRIVE_STRAIGHT/DRIVE_CIRCLE）に入るまではマップを動かさない
    // （resetPositionEstimate()はDRIVE_STRAIGHT突入時に呼ばれるので、それより前は
    //   常に原点のまま。打ち上げ待ち中の振動やハンドリングをマップに反映させないため）。
    if (gPhase == TestPhase::DRIVE_STRAIGHT || gPhase == TestPhase::DRIVE_CIRCLE) {
        updatePositionEstimate();
    }

    float alt   = sensor.getAltitude();
    float roll  = sensor.getRoll();
    float pitch = sensor.getPitch();
    uint32_t elapsed = millis() - gPhaseEnteredMs;

    MissionState outState = MissionState::LAUNCH;  // 既定はモーター停止側
    float outPidOutput = 0.0f;

    switch (gPhase) {
        case TestPhase::LAUNCH: {
            outState = MissionState::LAUNCH;
            if (elapsed >= LAUNCH_ARM_DELAY_MS) {
                launchConfirmCount = (alt > LAUNCH_ALT_THRESHOLD_M) ? launchConfirmCount + 1 : 0;
                if (launchConfirmCount >= LAUNCH_CONFIRM_TICKS) {
                    transitionTo(TestPhase::DETACH);
                }
            }
            break;
        }

        case TestPhase::DETACH: {
            outState = MissionState::DETACH;
            if (!detachAltInitialized) {
                lastAltForDetach = alt;
                detachAltInitialized = true;
                detachTickCount = 0;
            } else if (++detachTickCount >= DETACH_ALT_SAMPLE_TICKS) {
                float delta = fabsf(alt - lastAltForDetach);
                lastAltForDetach = alt;
                detachTickCount = 0;
                detachConfirmCount = (delta < DETACH_ALT_DELTA_THRESHOLD_M) ? detachConfirmCount + 1 : 0;
                if (detachConfirmCount >= DETACH_CONFIRM_TICKS) {
                    transitionTo(TestPhase::UNFOLD);
                }
            }
            break;
        }

        case TestPhase::UNFOLD: {
            outState = MissionState::UNFOLD;
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

            // 直立確認はrollのみで判定する（pitchは見ない）
            bool upright = settled && fabsf(roll) <= UNFOLD_UPRIGHT_ABS_DEG;
            unfoldUprightCount = upright ? unfoldUprightCount + 1 : 0;

            if (unfoldUprightCount >= UNFOLD_UPRIGHT_CONFIRM_TICKS) {
                transitionTo(TestPhase::DRIVE_STRAIGHT);
            } else if (elapsed > UNFOLD_TIMEOUT_MS) {
                // 変化は収まった（安定した）が20度以内に収まらない＝転倒等で走行不能と判断
                transitionTo(TestPhase::ABORTED);
            }
            break;
        }

        case TestPhase::DRIVE_STRAIGHT: {
            outState = MissionState::NAVIGATE;
            outPidOutput = 0.0f;  // 左右差なし＝直進。base speedはXIAO2側で最大出力にしてある
            if (elapsed > straightDurationMs) {
                transitionTo(TestPhase::DRIVE_CIRCLE);
            }
            break;
        }

        case TestPhase::DRIVE_CIRCLE: {
            outState = MissionState::NAVIGATE;
            outPidOutput = CIRCLE_TURN_PID_OUTPUT;
            if (elapsed > circleDurationMs) {
                transitionTo(TestPhase::GOAL);
            }
            break;
        }

        case TestPhase::GOAL: {
            outState = MissionState::GOAL;
            deployer.beepPattern();
            break;
        }

        case TestPhase::ABORTED: {
            outState = MissionState::ABORTED;
            deployer.beepPattern();
            break;
        }
    }

    SpiFrameToXiao2 out{};
    out.timestamp_ms   = millis();
    out.alt            = alt;
    out.roll           = roll;
    out.pitch          = pitch;
    out.yaw            = gHeadingDeg;  // 相対ヘディング[deg]（真北基準ではない。上部コメント参照）
    out.lat            = gPosX;        // GPS度数ではなく原点からのx距離[m]として流用
    out.lon            = gPosY;        // 同じくy距離[m]
    out.mission_state  = static_cast<uint8_t>(outState);
    out.pid_output      = outPidOutput;
    // mission_stateはDRIVE_STRAIGHT/DRIVE_CIRCLEどちらもNAVIGATE固定になり地上局側で
    // 区別できないため、destination_yaw（本来は目的地方位角。この誘導方式では未使用）を
    // このテストの詳細フェーズ番号（TestPhaseの並び=0:LAUNCH〜6:ABORTED）として流用する。
    // 地上局はground/receiver_dead_reckoning.pyのPHASE_NAMESで表示する。
    out.destination_yaw = static_cast<float>(static_cast<int>(gPhase));

    spiLink.transfer(out);  // XIAO1からの応答（目的地）はこのテストでは使わない

    static uint32_t lastLogMs = 0;
    uint32_t now = millis();
    if (now - lastLogMs >= 500) {
        lastLogMs = now;
        Serial.printf("[TEST] phase=%s elapsed=%lums alt=%.2fm roll=%.1f pitch=%.1f pid_output=%.1f "
                      "pos=(%.2f,%.2f)m heading=%.1fdeg\n",
                      phaseName(gPhase), (unsigned long)elapsed, alt, roll, pitch, outPidOutput,
                      gPosX, gPosY, gHeadingDeg);
    }

    delay(MISSION_TICK_MS);  // 100Hz目安
}
