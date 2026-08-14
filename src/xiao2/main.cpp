#include <Arduino.h>
#include <Actuator.h>
#include <Radio.h>
#include "spi_protocol.h"
#include <SpiLinkSlave.h>

static const char* AP_SSID = "CanSat-AP";
static const char* AP_PASS = "cansat2026";

static SpiLinkSlave spiLink;
static Actuator actuator;
static Radio radio;

// XIAO1から最後に受信したフレーム。新しいフレームが届かない間もこれを使い続ける
// （モータ駆動・WiFiテレメトリの両方がこれを参照する）。
static SpiFrameToXiao2 lastFrame{};

// XIAO1からSPIフレームを最後に受信した時刻。SPI_LINK_TIMEOUT_MS以上更新が無ければ
// XIAO1側の異常（クラッシュ・配線断等）とみなしフェイルセイフでモータを強制停止する。
// 手動操作（Radio）より優先する（XIAO1が生きている前提でのみ機体を動かしてよいため）。
static uint32_t lastSpiFrameMs = 0;
static const uint32_t SPI_LINK_TIMEOUT_MS = 5000;

// TODO: ミッションステートに応じて前進速度を調整
// GPS/地磁気トラブル対応のtest_dead_reckoning（推測航法テスト）が直進フェーズを
// 「最大出力」で行う前提のため255（最大）にしてある。本番のGPS誘導NAVIGATEでも
// このBASE_SPEEDをそのまま使うため、誘導精度に問題が出るようなら要調整。
static const int16_t BASE_SPEED = 255;

// XIAO1が計算した誘導PID出力（旋回量）をbase±turnの左右差動出力に変換する。
static void computeAutonomousMotor(float pidOutput, int16_t& outLeft, int16_t& outRight) {
    outLeft  = constrain(static_cast<int>(BASE_SPEED - pidOutput), -255, 255);
    outRight = constrain(static_cast<int>(BASE_SPEED + pidOutput), -255, 255);
}

void setup() {
    Serial.begin(115200);
    Serial.println("XIAO2 (motor drive / WiFi telemetry) booting...");

    actuator.begin();
    spiLink.begin();
    radio.begin(AP_SSID, AP_PASS);
}

void loop() {
    if (spiLink.poll(lastFrame)) {
        SpiFrameFromXiao2 out{};
        out.goal_valid = radio.hasGoal() ? 1 : 0;
        out.goal_lat   = radio.getGoalLat();
        out.goal_lon   = radio.getGoalLon();
        spiLink.setResponse(out);
        lastSpiFrameMs = millis();
    }

    // 自律PID走行はNAVIGATE状態のときだけ有効にする（発射・分離・展開の最中に
    // pid_outputの値でモータが勝手に動き出さないようにする安全ゲート）。
    // 地上局からの手動操作コマンドはミッションステートに関わらず常に優先する。
    MissionState missionState = static_cast<MissionState>(lastFrame.mission_state);

    int16_t left, right;
    if (millis() - lastSpiFrameMs > SPI_LINK_TIMEOUT_MS) {
        left  = 0;
        right = 0;
    } else if (radio.hasRecentMotorCommand()) {
        left  = radio.getMotorCommandLeft();
        right = radio.getMotorCommandRight();
    } else if (missionState == MissionState::NAVIGATE) {
        computeAutonomousMotor(lastFrame.pid_output, left, right);
    } else {
        left  = 0;
        right = 0;
    }

    actuator.setMotorLeft(left);
    actuator.setMotorRight(right);

    radio.setData(lastFrame);
    radio.poll();
}
