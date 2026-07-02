#include <Arduino.h>
#include <Actuator.h>

// 左右モーター（TB6612FNG、2ピン/モーター方式）の動作確認
// PlatformIO で env:test-motor を選択して書き込む
// 左右それぞれを -255→255→-255 とゆっくりランプさせるだけのテスト。
// PWMA/PWMB/STBYはハード側で常時HIGHに固定配線しておくこと。

static Actuator actuator;

static void rampMotor(void (Actuator::*setSpeed)(int), const char* label) {
    for (int s = -255; s <= 255; s += 5) {
        (actuator.*setSpeed)(s);
        Serial.printf("[TEST] %s speed=%4d\n", label, s);
        delay(30);
    }
    for (int s = 255; s >= -255; s -= 5) {
        (actuator.*setSpeed)(s);
        Serial.printf("[TEST] %s speed=%4d\n", label, s);
        delay(30);
    }
    (actuator.*setSpeed)(0);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] Motor (Left/Right) check starting...");
    actuator.begin();
}

void loop() {
    rampMotor(&Actuator::setMotorLeft,  "LEFT ");
    delay(500);
    rampMotor(&Actuator::setMotorRight, "RIGHT");
    delay(1000);
}
