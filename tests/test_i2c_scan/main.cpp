#include <Arduino.h>
#include <Wire.h>

// I2Cバス上の全アドレス(0x03-0x77)を総当たりでスキャンし、
// 応答があったアドレスを一覧表示する。
// PlatformIO で env:test-i2c-scan を選択して書き込む。
//
// 期待されるアドレス:
//   0x14       BMM350（地磁気）
//   0x68/0x69  MPU6050（6軸IMU）
//   0x76/0x77  BMP280（気圧）

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Wire.begin();
    Serial.println("[TEST] I2C scan starting...");
}

void loop() {
    int found = 0;

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("[I2C] device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            found++;
        }
    }

    if (found == 0) {
        Serial.println("[I2C] no device found on bus");
    } else {
        Serial.print("[I2C] scan done, ");
        Serial.print(found);
        Serial.println(" device(s) found");
    }

    // 0x68にMPU6050(またはクローンチップ)がいる場合、WHO_AM_I(reg 0x75)を読んで
    // 本物のMPU6050(0x68)かクローン(別ID)かを確認する
    Wire.beginTransmission(0x68);
    Wire.write(0x75);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
        if (Wire.available()) {
            uint8_t whoAmI = Wire.read();
            Serial.print("[I2C] 0x68 WHO_AM_I = 0x");
            Serial.println(whoAmI, HEX);
            Serial.println("       (Adafruit_MPU6050 expects 0x68 here; other values = clone chip)");
        }
    }

    Serial.println("---");
    delay(2000);
}
