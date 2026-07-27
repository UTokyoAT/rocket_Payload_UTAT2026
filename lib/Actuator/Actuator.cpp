#include "Actuator.h"

// XIAO2実配線（Seeed XIAO ESP32S3のD番号 -> GPIO番号）
// motor_PWMA: D0(GPIO1)  motor_AIN2: D1(GPIO2)  motor_AIN1: D2(GPIO3)
// motor_STBY: D3(GPIO4)  motor_BIN1: D4(GPIO5)  motor_BIN2: D5(GPIO6)
// motor_PWMB: D6(GPIO43)
// SPI（SpiLinkSlave）: SCK/MISO/MOSI=D8/D9/D10、CS=D7 → include/spi_protocol.h参照
static const int PIN_MOTOR_PWMA = 1;
static const int PIN_MOTOR_AIN2 = 2;
static const int PIN_MOTOR_AIN1 = 3;
static const int PIN_MOTOR_STBY = 4;
static const int PIN_MOTOR_BIN1 = 5;
static const int PIN_MOTOR_BIN2 = 6;
static const int PIN_MOTOR_PWMB = 43;

void Actuator::begin() {
    pinMode(PIN_MOTOR_PWMA, OUTPUT);
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    pinMode(PIN_MOTOR_BIN1, OUTPUT);
    pinMode(PIN_MOTOR_BIN2, OUTPUT);
    pinMode(PIN_MOTOR_PWMB, OUTPUT);

    digitalWrite(PIN_MOTOR_STBY, HIGH);  // スタンバイ解除（常時ドライバ有効）
}

// AIN1・AIN2で回転方向を決め、PWMAで速度を出力する（TB6612FNG標準の3ピン方式）
static void driveMotor(int in1Pin, int in2Pin, int pwmPin, int speed) {
    speed = constrain(speed, -255, 255);
    digitalWrite(in1Pin, speed >= 0 ? HIGH : LOW);
    digitalWrite(in2Pin, speed >= 0 ? LOW  : HIGH);
    analogWrite(pwmPin, abs(speed));
}

void Actuator::setMotorLeft(int speed) {
    driveMotor(PIN_MOTOR_AIN1, PIN_MOTOR_AIN2, PIN_MOTOR_PWMA, speed);
}

void Actuator::setMotorRight(int speed) {
    driveMotor(PIN_MOTOR_BIN1, PIN_MOTOR_BIN2, PIN_MOTOR_PWMB, speed);
}
