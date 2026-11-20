#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <Expander.h>

// I/Oエキスパンダ（MCP23008）の動作確認用 Lチカ。PlatformIOで env:test-blink を選択して書き込む。
// エキスパンダの PIN_EXP_LED に外付けLEDをつなぎ（GP → 抵抗(330Ω〜1kΩ程度) → LED → GND）、
// 1秒周期で点滅させる。モーター周りは初期化しない（PWMピンは触らない）。
//
// 内蔵オレンジLED（GPIO21、LOWで点灯）で初期化結果を示す:
//   - 外付けLEDと同期してゆっくり点滅 → エキスパンダ初期化OK
//   - 内蔵LEDが速く点滅（5Hz）         → MCP23008が見つからない（I2C配線・電源・アドレス・RESETを確認）
// USBシリアル(115200bps)にもI2Cスキャン結果と初期化結果を出す（MCP23008は通常 0x20）。
//
// 確認ポイント:
//   - 内蔵LEDがゆっくり点滅しているのに外付けLEDが光らない → LEDの向き・配線、またはエキスパンダ出力の不良
//   - PIN_EXP_LED を 4（STBY）に変えて書き込み、テスターでTB6612のSTBYピンが3.3V/0Vで切り替わるか見るのも有効

static const int           PIN_EXP_LED  = 0;            // 外付けLEDをつないだエキスパンダのピン（GP0〜GP7）
static const int           PIN_LED      = LED_BUILTIN;  // XIAO ESP32S3 は GPIO21（LOWで点灯）
static const unsigned long PERIOD_MS    = 1000;
static const unsigned long FAIL_HALF_MS = 100;          // 初期化失敗時の内蔵LED点滅（半周期）

static Expander expander;
static bool     gExpanderOk = false;

static void scanI2C() {
    Serial.println("[BLINK] I2C scan:");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[BLINK]   found 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("[BLINK]   no devices");
}

void setup() {
    WiFi.mode(WIFI_OFF);
    btStop();

    Serial.begin(115200);
    delay(1500);  // シリアルモニタを開く猶予
    Serial.println("[BLINK] Expander LED test starting...");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);  // 消灯

    gExpanderOk = expander.setup();  // 内部で RESET を HIGH にして MCP23008 を begin_I2C する
    scanI2C();

    if (gExpanderOk) {
        gExpanderOk = expander.enableOutput(PIN_EXP_LED);
    }
    Serial.printf("[BLINK] expander: %s (LED on GP%d)\n", gExpanderOk ? "OK" : "NOT FOUND", PIN_EXP_LED);
}

void loop() {
    if (!gExpanderOk) {
        // 初期化失敗: 内蔵LEDを速く点滅させ、定期的に結果を出し続ける
        static unsigned long lastPrint = 0;
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(FAIL_HALF_MS);
        if (millis() - lastPrint >= 2000) {
            lastPrint = millis();
            Serial.println("[BLINK] expander NOT FOUND");
        }
        return;
    }

    expander.set_high(PIN_EXP_LED);
    digitalWrite(PIN_LED, LOW);   // 内蔵LEDも同期して点灯
    Serial.println("[BLINK] ON");
    delay(PERIOD_MS / 2);

    expander.set_low(PIN_EXP_LED);
    digitalWrite(PIN_LED, HIGH);
    Serial.println("[BLINK] OFF");
    delay(PERIOD_MS / 2);
}
