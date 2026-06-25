#include "Actuator.h"

// TODO: 実際のピン番号に変更する
static const int PIN_MOTOR_IN1  = 25;
static const int PIN_MOTOR_IN2  = 26;
static const int PIN_MOTOR_PWM  = 27;
static const int PIN_PARACHUTE  = 32;
static const int PIN_SEPARATE   = 33;
static const int PIN_BUZZER     = 14;
static const int PIN_LED        = 2;

void Actuator::begin() {
    pinMode(PIN_MOTOR_IN1,  OUTPUT);
    pinMode(PIN_MOTOR_IN2,  OUTPUT);
    pinMode(PIN_MOTOR_PWM,  OUTPUT);
    pinMode(PIN_PARACHUTE,  OUTPUT);
    pinMode(PIN_SEPARATE,   OUTPUT);
    pinMode(PIN_BUZZER,     OUTPUT);
    pinMode(PIN_LED,        OUTPUT);
}

void Actuator::setMotor(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        analogWrite(PIN_MOTOR_PWM, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
        analogWrite(PIN_MOTOR_PWM, -speed);
    } else {
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        analogWrite(PIN_MOTOR_PWM, 0);
    }
}

void Actuator::deployParachute() {
    // TODO: ニクロム線への通電時間・サーボ角度を調整
    digitalWrite(PIN_PARACHUTE, HIGH);
    delay(1000);
    digitalWrite(PIN_PARACHUTE, LOW);
}

void Actuator::separate() {
    // TODO: 分離機構のシーケンスを実装
    digitalWrite(PIN_SEPARATE, HIGH);
    delay(500);
    digitalWrite(PIN_SEPARATE, LOW);
}

void Actuator::setBuzzer(bool on) { digitalWrite(PIN_BUZZER, on); }
void Actuator::setLED(bool on)    { digitalWrite(PIN_LED, on); }

void Actuator::beepPattern() {
    // ピッ・ピッ・ポーのパターン（回収支援用）
    for (int i = 0; i < 2; i++) {
        setBuzzer(true);  setLED(true);  delay(100);
        setBuzzer(false); setLED(false); delay(200);
    }
    setBuzzer(true);  setLED(true);  delay(400);
    setBuzzer(false); setLED(false);
}
