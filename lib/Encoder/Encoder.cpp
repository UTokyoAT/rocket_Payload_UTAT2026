#include "Encoder.h"

const int PIN_RA = 2;   // エンコーダ右A相
const int PIN_RB = 3;   // エンコーダ右B相
const int PIN_LA = 2;   // エンコーダ左A相
const int PIN_LB = 3;   // エンコーダ左B相

const float GEAR_RATIO = 100.0;     // モーターの減速比
const float PPR = 7.0;              // Pulses Per Revolution、軸が一回転する間に出るパルスの数
const float COUNTS_PER_REV = PPR * GEAR_RATIO * 4.0;  // 1回転あたりのカウント数

const int8_t transitionTable[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
    -1,  0,  0,  1,
    0,  1, -1,  0
}; // この表に合わせて相変化から回転方向を当てる

volatile long Encoder::rightEncoderCount = 0;
volatile uint8_t Encoder::rightLastState = 0;
volatile long Encoder::leftEncoderCount = 0;
volatile uint8_t Encoder::leftLastState = 0;

void Encoder::begin() {
    pinMode(PIN_RA, INPUT_PULLUP);   // 内部プルアップを有効化
    pinMode(PIN_RB, INPUT_PULLUP);
    pinMode(PIN_LA, INPUT_PULLUP);   
    pinMode(PIN_LB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_RA), updateRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RB), updateRight, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LA), updateLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LB), updateLeft, CHANGE);
}

void IRAM_ATTR Encoder::updateRight() {
    uint8_t ra = digitalRead(PIN_RA);
    uint8_t rb = digitalRead(PIN_RB);
    uint8_t rightCurrentState = (ra << 1) | rb;
    uint8_t index = (rightLastState << 2) | rightCurrentState;
    rightEncoderCount += transitionTable[index];
    rightLastState = rightCurrentState;
}

void IRAM_ATTR Encoder::updateLeft() {
    uint8_t la = digitalRead(PIN_LA);
    uint8_t lb = digitalRead(PIN_LB);
    uint8_t leftCurrentState = (la << 1) | lb;
    uint8_t index = (leftLastState << 2) | leftCurrentState;
    leftEncoderCount += transitionTable[index];
    leftLastState = leftCurrentState;
}

float Encoder::getRightRevolutions() {
    rightRevolutions = rightEncoderCount / COUNTS_PER_REV;
    return rightRevolutions;
}

float Encoder::getLeftRevolutions() {
    leftRevolutions = leftEncoderCount / COUNTS_PER_REV;
    return leftRevolutions;
}