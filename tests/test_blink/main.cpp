#include <Arduino.h>
#include <WiFi.h>

// 電池駆動の確認用 Lチカ。PlatformIOで env:test-blink を選択して書き込む。
// 消費電力をできるだけ抑えるため、Wi-Fi/BTを切り、CPUを80MHzに落とし、
// センサーやモーター周りは何も初期化しない。
// 内蔵のオレンジLED（GPIO21、LOWで点灯）を1秒ごとに50msだけ光らせる。
//
// 確認ポイント:
//   - 電池だけで（USBを抜いて）オレンジLEDが1秒周期で点滅し続けるか
//   - 途中で点滅が止まる・周期が乱れる → 電源が不安定（リセットしている）

static const int           PIN_LED   = LED_BUILTIN;  // XIAO ESP32S3 は GPIO21
static const unsigned long ON_MS     = 50;
static const unsigned long PERIOD_MS = 1000;

void setup() {
    WiFi.mode(WIFI_OFF);
    btStop();
    setCpuFrequencyMhz(80);  // Wi-Fiを使わないなら80MHzで十分

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // 消灯
}

void loop() {
    digitalWrite(PIN_LED, LOW);   // 点灯
    delay(ON_MS);
    digitalWrite(PIN_LED, HIGH);  // 消灯
    delay(PERIOD_MS - ON_MS);
}
