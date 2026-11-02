#include "Actuator.h"

// エクスパンダ・XIAO実配線
// motor_PWMA: D9(GPIO8)          
// motor_PWMB: D10(GPIO9)   … マイコン直結（PWM出力）
// motor_AIN2: Expander GP2  motor_AIN1: Expander GP3
// motor_STBY: Expander GP4
// motor_BIN1: Expander GP5  motor_BIN2: Expander GP6      … エキスパンダ経由（方向制御）

static const int PIN_MOTOR_PWMA = 8;
static const int PIN_MOTOR_AIN2 = 2;
static const int PIN_MOTOR_AIN1 = 3;
static const int PIN_MOTOR_STBY = 4;
static const int PIN_MOTOR_BIN1 = 5;
static const int PIN_MOTOR_BIN2 = 6;
static const int PIN_MOTOR_PWMB = 9;

// 前回の回転の向き（+1 / -1 / 0 = 未設定）。初回は必ず書き込む
static int lastDirLeft  = 0;
static int lastDirRight = 0;

void Actuator::begin(Expander& expander) {
    _expander = &expander;

    pinMode(PIN_MOTOR_PWMA, OUTPUT);
    pinMode(PIN_MOTOR_PWMB, OUTPUT);

    _expander->enableOutput(PIN_MOTOR_AIN1);
    _expander->enableOutput(PIN_MOTOR_AIN2);
    _expander->enableOutput(PIN_MOTOR_STBY);
    _expander->enableOutput(PIN_MOTOR_BIN1);
    _expander->enableOutput(PIN_MOTOR_BIN2);

    _expander->set_high(PIN_MOTOR_STBY);  // スタンバイ解除（常時ドライバ有効）
}

// AIN1・AIN2で回転方向を決め、PWMAで速度を出力する（TB6612FNG標準の3ピン方式）
// 方向ピンはI2Cエクスパンダ経由なので、向きが前回（lastDir）と変わったときだけ書き込む
static void driveMotor(Expander& expander, int in1Pin, int in2Pin, int pwmPin, int speed, int& lastDir) {
    speed = constrain(speed, -255, 255);
    const int dir = (speed >= 0) ? +1 : -1;
    if (dir != lastDir) {
        if (dir > 0) {
            expander.set_high(in1Pin);
            expander.set_low(in2Pin);
        } else {
            expander.set_low(in1Pin);
            expander.set_high(in2Pin);
        }
        lastDir = dir;
    }
    analogWrite(pwmPin, abs(speed));
}

void Actuator::setMotorLeft(int speed) {
    if (!_expander) return;
    driveMotor(*_expander, PIN_MOTOR_AIN1, PIN_MOTOR_AIN2, PIN_MOTOR_PWMA, speed, lastDirLeft);
}

void Actuator::setMotorRight(int speed) {
    if (!_expander) return;
    driveMotor(*_expander, PIN_MOTOR_BIN1, PIN_MOTOR_BIN2, PIN_MOTOR_PWMB, speed, lastDirRight);
}
