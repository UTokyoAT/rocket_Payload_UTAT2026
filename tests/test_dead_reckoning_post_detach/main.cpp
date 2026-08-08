#include <Arduino.h>
#include <Sensor.h>
#include "spi_protocol.h"
#include <SpiLinkMaster.h>

// XIAO1側：test_dead_reckoningの「detach後」だけを切り出した版。
// PlatformIOで env:test-dead-reckoning-post-detach を選択してXIAO1へ書き込む。
//
// test_dead_reckoning（WAIT_ALT_SETTLE→WAIT_ATTITUDE_SETTLE→DRIVE_STRAIGHT→DRIVE_CIRCLE）から
// WAIT_ALT_SETTLE（高度変化が収まるのを待つ＝分離検知）を省き、起動した時点で
// 「すでに分離済み・地面に置かれている」ものとして扱う。実機を落下させずに机上・平地に
// 直接置いて電源を入れ、姿勢安定確認から先の走行ロジックだけを繰り返し試したいときに使う
// （毎回落下・振動が収まるのを待たなくてよい分、素早く反復テストできる）。
//
// XIAO2側は本番ファームウェア（env:xiao2）をそのまま書き込んでおくこと。
// ニクロム線（Deployer）は使わない（test_dead_reckoningと同じ理由。
// 詳細・チューニング手順はtest_dead_reckoning/main.cppのコメントを参照）。
//
// フェーズ構成:
//   1. WAIT_ATTITUDE_SETTLE … roll/pitchが安定し直立していることを確認（本番UNFOLDと同じ判定）
//   2. DRIVE_STRAIGHT    … その場から最大出力（左右均等）で直進。STRAIGHT_DISTANCE_M分の
//                           時間だけ走ってから止める（距離センサーが無いため時間ベース）
//   3. DRIVE_CIRCLE      … 左右に一定の差をつけたまま走行し、半径CIRCLE_RADIUS_Mの円を描く
//   4. DONE              … 停止。以後は状態をシリアル出力し続けるだけ
//
// 走行軌跡の推定（着地位置を原点とした簡易マップ）・lat/lonフィールドの流用・
// 使用する地上局スクリプト（receiver_dead_reckoning.py）については
// test_dead_reckoning/main.cppの該当コメントを参照（このファイルも同じ方式）。

static Sensor sensor;
static SpiLinkMaster spiLink;

// --- WAIT_ATTITUDE_SETTLE（本番UNFOLDと同じ値）---
static const uint32_t ATTITUDE_TIMEOUT_MS         = 5UL * 60 * 1000;
static const int      ATTITUDE_STABLE_WINDOW_TICKS = 10;
static const float    ATTITUDE_STABLE_RANGE_DEG    = 10.0f;
static const int      ATTITUDE_UPRIGHT_CONFIRM_TICKS = 5;
static const float    ATTITUDE_UPRIGHT_ABS_DEG       = 20.0f;

// --- 走行パラメータ（実機で要チューニング。test_dead_reckoning/main.cppのコメント参照）---
static const float STRAIGHT_DISTANCE_M   = 10.0f;
static const float CIRCLE_RADIUS_M       = 10.0f;
static const float CIRCLE_LAPS           = 1.0f;   // 円軌道を何周走るか
// TODO: 実機のBASE_SPEED=255（最大出力）直進時の実測並進速度[m/s]に置き換える
static const float STRAIGHT_SPEED_MPS    = 0.5f;
// TODO: 実機で半径CIRCLE_RADIUS_Mに近づくよう調整するpid_output値（旋回量）。
// 正の値で左旋回（左が減速・右が最大のまま）になる。負にすると右旋回。
static const float CIRCLE_TURN_PID_OUTPUT = 15.0f;

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
// 精度が上がる。
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

// 姿勢安定確認（＝着地位置とみなす）を原点・ヘディング0度としてリセットする
static void resetPositionEstimate() {
    gHeadingDeg = 0.0f;
    gVelX = 0.0f; gVelY = 0.0f;
    gPosX = 0.0f; gPosY = 0.0f;
    gFilteredForwardAccel = 0.0f;
    gFilteredLateralAccel = 0.0f;
    gLastPosUpdateUs = micros();
}

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
    WAIT_ATTITUDE_SETTLE,
    DRIVE_STRAIGHT,
    DRIVE_CIRCLE,
    DONE
};

static const char* phaseName(TestPhase p) {
    switch (p) {
        case TestPhase::WAIT_ATTITUDE_SETTLE: return "WAIT_ATTITUDE_SETTLE";
        case TestPhase::DRIVE_STRAIGHT:       return "DRIVE_STRAIGHT";
        case TestPhase::DRIVE_CIRCLE:         return "DRIVE_CIRCLE";
        case TestPhase::DONE:                 return "DONE";
    }
    return "?";
}

// destination_yaw経由で送るテスト詳細フェーズ番号。test_dead_reckoning（フル版）の
// TestPhase（0:LAUNCH,1:DETACH,2:UNFOLD,3:DRIVE_STRAIGHT,4:DRIVE_CIRCLE,5:GOAL,6:ABORTED）と
// 番号を揃え、地上局（receiver_dead_reckoning.pyのPHASE_NAMES）がどちらのファームウェアでも
// 同じ表示になるようにする。
static int phaseCode(TestPhase p) {
    switch (p) {
        case TestPhase::WAIT_ATTITUDE_SETTLE: return 2;  // UNFOLD相当
        case TestPhase::DRIVE_STRAIGHT:       return 3;
        case TestPhase::DRIVE_CIRCLE:         return 4;
        case TestPhase::DONE:                 return 5;  // GOAL相当
    }
    return -1;
}

static TestPhase gPhase = TestPhase::WAIT_ATTITUDE_SETTLE;
static uint32_t  gPhaseEnteredMs = 0;

// WAIT_ATTITUDE_SETTLE用（直近ATTITUDE_STABLE_WINDOW_TICKS件のroll/pitchのリングバッファ）
static float attitudeRollBuf[ATTITUDE_STABLE_WINDOW_TICKS];
static float attitudePitchBuf[ATTITUDE_STABLE_WINDOW_TICKS];
static int   attitudeBufCount = 0;
static int   attitudeBufIndex = 0;
static int   attitudeUprightCount = 0;

static uint32_t straightDurationMs = 0;
static uint32_t circleDurationMs   = 0;

static void transitionTo(TestPhase next) {
    Serial.printf("[TEST] %s -> %s\n", phaseName(gPhase), phaseName(next));
    gPhase = next;
    gPhaseEnteredMs = millis();

    attitudeBufCount     = 0;
    attitudeBufIndex     = 0;
    attitudeUprightCount = 0;

    if (next == TestPhase::DRIVE_STRAIGHT) {
        // 姿勢安定確認が取れた瞬間＝ここを地図の原点とする。走り出す直前でまだ静止して
        // いるはずなので、バイアスもここで取り直してからリセットする。
        calibrateBias();
        resetPositionEstimate();
        Serial.println("[TEST] position estimate reset to origin");
    }
}

void setup() {
    Serial.begin(115200);

    sensor.begin();  // BMM350（地磁気）が繋がっていなくても高度・roll/pitchは使える
    spiLink.begin();

    // 位置推定用のジャイロ・加速度バイアスを較正する。この間は機体を静止させておくこと。
    // DRIVE_STRAIGHT突入直前にも取り直すため、これは初回較正。
    calibrateBias();
    resetPositionEstimate();

    straightDurationMs = static_cast<uint32_t>((STRAIGHT_DISTANCE_M / STRAIGHT_SPEED_MPS) * 1000.0f);
    float circumferenceM = 2.0f * PI * CIRCLE_RADIUS_M * CIRCLE_LAPS;
    circleDurationMs = static_cast<uint32_t>((circumferenceM / STRAIGHT_SPEED_MPS) * 1000.0f);

    Serial.println("[TEST] starting from post-detach assumption (no altitude-settle wait)");
    Serial.printf("[TEST] straight: %.1fm @ %.2fm/s -> %lums\n",
                  STRAIGHT_DISTANCE_M, STRAIGHT_SPEED_MPS, (unsigned long)straightDurationMs);
    Serial.printf("[TEST] circle: r=%.1fm x%.1flap -> %lums (pid_output=%.1f)\n",
                  CIRCLE_RADIUS_M, CIRCLE_LAPS, (unsigned long)circleDurationMs, CIRCLE_TURN_PID_OUTPUT);

    gPhaseEnteredMs = millis();
}

void loop() {
    sensor.update();
    // NAVIGATE（DRIVE_STRAIGHT/DRIVE_CIRCLE）に入るまではマップを動かさない
    // （resetPositionEstimate()はDRIVE_STRAIGHT突入時に呼ばれるので、それより前は
    //   常に原点のまま）。
    if (gPhase == TestPhase::DRIVE_STRAIGHT || gPhase == TestPhase::DRIVE_CIRCLE) {
        updatePositionEstimate();
    }

    float alt   = sensor.getAltitude();
    float roll  = sensor.getRoll();
    float pitch = sensor.getPitch();
    uint32_t elapsed = millis() - gPhaseEnteredMs;

    MissionState outState = MissionState::UNFOLD;  // 既定はモーター停止側
    float outPidOutput = 0.0f;

    switch (gPhase) {
        case TestPhase::WAIT_ATTITUDE_SETTLE: {
            outState = MissionState::UNFOLD;
            attitudeRollBuf[attitudeBufIndex]  = roll;
            attitudePitchBuf[attitudeBufIndex] = pitch;
            attitudeBufIndex = (attitudeBufIndex + 1) % ATTITUDE_STABLE_WINDOW_TICKS;
            if (attitudeBufCount < ATTITUDE_STABLE_WINDOW_TICKS) attitudeBufCount++;

            bool settled = false;
            if (attitudeBufCount >= ATTITUDE_STABLE_WINDOW_TICKS) {
                float rollMin = attitudeRollBuf[0], rollMax = attitudeRollBuf[0];
                float pitchMin = attitudePitchBuf[0], pitchMax = attitudePitchBuf[0];
                for (int i = 1; i < ATTITUDE_STABLE_WINDOW_TICKS; i++) {
                    if (attitudeRollBuf[i]  < rollMin)  rollMin  = attitudeRollBuf[i];
                    if (attitudeRollBuf[i]  > rollMax)  rollMax  = attitudeRollBuf[i];
                    if (attitudePitchBuf[i] < pitchMin) pitchMin = attitudePitchBuf[i];
                    if (attitudePitchBuf[i] > pitchMax) pitchMax = attitudePitchBuf[i];
                }
                settled = (rollMax - rollMin <= ATTITUDE_STABLE_RANGE_DEG) &&
                          (pitchMax - pitchMin <= ATTITUDE_STABLE_RANGE_DEG);
            }

            // 直立確認はrollのみで判定する（pitchは見ない）
            bool upright = settled && fabsf(roll) <= ATTITUDE_UPRIGHT_ABS_DEG;
            attitudeUprightCount = upright ? attitudeUprightCount + 1 : 0;

            if (attitudeUprightCount >= ATTITUDE_UPRIGHT_CONFIRM_TICKS) {
                transitionTo(TestPhase::DRIVE_STRAIGHT);
            } else if (elapsed > ATTITUDE_TIMEOUT_MS) {
                Serial.println("[TEST] attitude settle timed out. staying stopped.");
                transitionTo(TestPhase::DONE);
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
                transitionTo(TestPhase::DONE);
            }
            break;
        }

        case TestPhase::DONE: {
            outState = MissionState::GOAL;
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
    // このテストの詳細フェーズ番号として流用する（番号の意味はphaseCode()参照）。
    out.destination_yaw = static_cast<float>(phaseCode(gPhase));

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

    delay(10);  // 100Hz目安
}
