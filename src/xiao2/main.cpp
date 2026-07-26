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

// TODO: ミッションステートに応じて前進速度を調整
static const int16_t BASE_SPEED = 150;

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
        spiLink.setResponse(out);
    }

    int16_t left, right;
    if (radio.hasRecentMotorCommand()) {
        left  = radio.getMotorCommandLeft();
        right = radio.getMotorCommandRight();
    } else {
        computeAutonomousMotor(lastFrame.pid_output, left, right);
    }

    actuator.setMotorLeft(left);
    actuator.setMotorRight(right);

    radio.setData(lastFrame);
    radio.poll();

    // TODO: XIAO1からの通信が一定時間途絶えたらフェイルセイフでモータ停止する
}
