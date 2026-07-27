#include <Arduino.h>
#include <Actuator.h>
#include <DebugLog.h>

// 左右モーター（TB6612FNG、3ピン/モーター＋共有STBY方式）の動作確認
// PlatformIO で env:test-motor を選択して書き込む
// 左右それぞれを -255→255→-255 とゆっくりランプさせるだけのテスト。
// STBYはActuator::begin()内でソフト制御によりHIGHにする（ハード側の固定配線ではない）。
// デバッグ出力はWiFi経由。CanSat-AP（パスワード: cansat2026）に接続して
// http://192.168.4.1 を開くか、GET /log をポーリングする（USBシリアル不要）。

static Actuator actuator;
static DebugLog debug;

static void rampMotor(void (Actuator::*setSpeed)(int), const char* label) {
    for (int s = -255; s <= 255; s += 5) {
        (actuator.*setSpeed)(s);
        debug.printf("[TEST] %s speed=%4d", label, s);
        debug.poll();
        delay(30);
    }
    for (int s = 255; s >= -255; s -= 5) {
        (actuator.*setSpeed)(s);
        debug.printf("[TEST] %s speed=%4d", label, s);
        debug.poll();
        delay(30);
    }
    (actuator.*setSpeed)(0);
}

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] Motor (Left/Right) check starting...");
    actuator.begin();
}

void loop() {
    rampMotor(&Actuator::setMotorLeft,  "LEFT ");
    debug.poll();
    delay(500);
    rampMotor(&Actuator::setMotorRight, "RIGHT");
    debug.poll();
    delay(1000);
}
