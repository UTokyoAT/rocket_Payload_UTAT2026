#include "Actuator.h"

// TODO: 実際のピン番号に変更する（XIAO ESP32S3で使えるGPIOはD0〜D10の11本のみ）
// TB6612FNGのPWMA・PWMB・STBYはハード側で常時HIGHに固定配線しておくこと
// （2ピン/モーター方式で使わないため、MCU側のピンは消費しない）
static const int PIN_MOTOR_L_IN1 = 25;
static const int PIN_MOTOR_L_IN2 = 26;
static const int PIN_MOTOR_R_IN1 = 27;
static const int PIN_MOTOR_R_IN2 = 32;
static const int PIN_PARACHUTE   = 33;
static const int PIN_SEPARATE    = 14;
static const int PIN_BUZZER      = 2;
static const int PIN_LED         = 4;

void Actuator::begin() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    pinMode(PIN_PARACHUTE,   OUTPUT);
    pinMode(PIN_SEPARATE,    OUTPUT);
    pinMode(PIN_BUZZER,      OUTPUT);
    pinMode(PIN_LED,         OUTPUT);
}

// IN1・IN2のどちらか片方だけをPWM出力することで速度・方向を両方指定する
// （TB6612FNGを2ピン/モーター方式で駆動。もう片方は0=LOW相当にする）
static void driveMotor(int in1Pin, int in2Pin, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) {
        analogWrite(in1Pin, speed);
        analogWrite(in2Pin, 0);
    } else {
        analogWrite(in1Pin, 0);
        analogWrite(in2Pin, -speed);
    }
}

void Actuator::setMotorLeft(int speed)  { driveMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, speed); }
void Actuator::setMotorRight(int speed) { driveMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, speed); }

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
