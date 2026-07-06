#include <Arduino.h>
#include <Actuator.h>
#include <PID.h>
#include "spi_protocol.h"
#include <SpiLinkSlave.h>

static SpiLinkSlave spiLink;
static Actuator actuator;

// TODO: ゲインは実機調整が必要
static PID headingPid(2.0f, 0.0f, 0.5f, -255.0f, 255.0f);

// GPS方位（目標地点への方位）とXIAO1から受け取ったyaw（コンパス方位）の誤差をPIDで
// 補正し、左右モータの差動出力に変換する（tests/test_navigationと同じ考え方）。
// TODO: 現状は仮実装。目標座標の与え方・bearing計算・車体キネマティクスを詰める。
static void computeAutonomousMotor(const SpiFrameToXiao2& in, int16_t& outLeft, int16_t& outRight) {
    float headingError = 0.0f;  // TODO: 目標方位 - in.yaw を -180〜180 に正規化して算出
    const float dt = 0.02f;     // 50Hz想定
    float turn = headingPid.update(headingError, dt);

    const int16_t base = 150;   // TODO: ミッションステートに応じて前進速度を調整
    outLeft  = constrain(static_cast<int>(base - turn), -255, 255);
    outRight = constrain(static_cast<int>(base + turn), -255, 255);
}

void setup() {
    Serial.begin(115200);
    Serial.println("XIAO2 (motor PID / guidance) booting...");

    actuator.begin();
    spiLink.begin();
}

void loop() {
    SpiFrameToXiao2 in{};
    if (spiLink.poll(in)) {
        int16_t left, right;

        if (in.manual_override) {
            left  = in.manual_motor_left;
            right = in.manual_motor_right;
        } else {
            computeAutonomousMotor(in, left, right);
        }

        actuator.setMotorLeft(left);
        actuator.setMotorRight(right);

        SpiFrameFromXiao2 out{};
        out.motor_output_left  = left;
        out.motor_output_right = right;
        spiLink.setResponse(out);
    }

    // TODO: XIAO1からの通信が一定時間途絶えたらフェイルセイフでモータ停止する
}
