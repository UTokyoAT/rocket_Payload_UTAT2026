#include "Deployer.h"

// XIAO1実配線（Seeed XIAO ESP32S3のD番号 -> GPIO番号）
// deployRocket: D1(GPIO2)  deployParachute: D3(GPIO4)  LED: D2(GPIO3、空きピン)
// I2C(SDA/SCL)=D4/D5、UART(RX/TX)=D7/D6、SPI(SpiLinkMaster)はSCK/MISO/MOSI=D8/D9/D10、CS=D0
static const int PIN_ROCKET    = 2;
static const int PIN_PARACHUTE = 4;
static const int PIN_LED       = 3;

void Deployer::begin() {
    pinMode(PIN_ROCKET,    OUTPUT);
    pinMode(PIN_PARACHUTE, OUTPUT);
    pinMode(PIN_LED,       OUTPUT);
}

void Deployer::deployRocket() {
    digitalWrite(PIN_ROCKET, HIGH);
    delay(3000);
    digitalWrite(PIN_ROCKET, LOW);
}

void Deployer::deployParachute() {
    // TODO: ニクロム線への通電時間を調整
    digitalWrite(PIN_PARACHUTE, HIGH);
    delay(1000);
    digitalWrite(PIN_PARACHUTE, LOW);
}

void Deployer::setLED(bool on) { digitalWrite(PIN_LED, on); }

void Deployer::beepPattern() {
    // 点滅パターン（回収支援用、ブザーなし）
    for (int i = 0; i < 2; i++) {
        setLED(true);  delay(100);
        setLED(false); delay(200);
    }
    setLED(true);  delay(400);
    setLED(false);
}
