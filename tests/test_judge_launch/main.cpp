#include <Arduino.h>
#include <Sensor.h>
#include <Expander.h>
#include <Deployer.h>
#include <Telemetry.h>
#include "secrets.h"

// LAUNCH判定〜ロケット分離（ニクロム線通電）確認テスト（単体完結版）。
// PlatformIOで env:test-judge-launch を選択して書き込む。
// 事前に include/secrets.example.h を include/secrets.h にコピーして値を設定し、
// ground/server で `docker compose up -d` しておく。
//
// 目的: 気圧センサ値取得 → LAUNCH判定 → Rocket分離ニクロム線通電（エクスパンダ経由）
//   → 地上局での状態確認、の一連の流れを単体で確認する。
// ニクロム線のピン配置は src/Deployer 側で定義している（ロケット分離: エクスパンダGP1）。
//
// LAUNCH判定: 高度がLAUNCH_ALT_THRESHOLD_Mを LAUNCH_CONFIRM_TICKS 回連続で超えたら確定。
// 周期は100Hz目安（Sensor::update()の想定周期）。
//
// 確認方法（InfluxDBのData Explorer / Grafana）:
//   rocket_test の test_name=judge_launch … 100msごとの以下のフィールド
//     alt            起動地点からの相対高度（m）
//     launched       LAUNCH確定済みか（1/0）。1になればLAUNCH判定＋Rocket通電が起きた
//     confirm_count  しきい値超えの連続tick数
//   rocket_log … LAUNCH検出・通電結果・1秒ごとの状態
//
// ベンチで気圧センサを手で覆う/持ち上げる等して繰り返し試せるよう、シリアルから
// 'r' でLAUNCH判定状態をリセットできる（再起動しなくても再試行可能）。
//
// コマンド（USBシリアル、改行区切り）:
//   r    LAUNCH判定状態をリセットし、再度LAUNCH待ちに戻す（Rocketは再通電しない）
//   ?    現在の状態を表示

static const float    LAUNCH_ALT_THRESHOLD_M = 3.0f;
static const int      LAUNCH_CONFIRM_TICKS   = 5;

static const uint32_t SAMPLE_PERIOD_MS = 10;    // 100Hz
static const uint32_t SEND_PERIOD_MS   = 100;   // 10Hz
static const uint32_t LOG_PERIOD_MS    = 1000;

static Sensor    sensor;
static Expander  expander;
static Deployer  deployer;
static Telemetry telemetry;

static bool launched = false;
static int  launchConfirmCount = 0;

static uint32_t _lastSendMs = 0;
static uint32_t _lastLogMs  = 0;

static void printStatus(float alt) {
    telemetry.log("[STATUS] alt=%.2fm launched=%s confirmCount=%d/%d mqtt=%s",
                  alt, launched ? "yes" : "no", launchConfirmCount, LAUNCH_CONFIRM_TICKS,
                  telemetry.isConnected() ? "yes" : "no");
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "r") {
        launched = false;
        launchConfirmCount = 0;
        telemetry.log("[RESET] LAUNCH judge state reset. Waiting for LAUNCH again.");
    } else if (cmd == "?") {
        printStatus(sensor.getAltitude());
    } else {
        Serial.println("[ERR] unknown command (r/?)");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    // begin()直後はまだ未接続なのでここでのログはSerialにしか出ない
    telemetry.log("[TEST] LAUNCH judge -> Rocket deploy check starting...");

    sensor.begin();
    telemetry.log("[TEST] BMP280=%s", sensor.isBmp280Ready() ? "OK" : "NG");

    if (!expander.setup()) {
        telemetry.log("[TEST] Expander init failed (nichrome cannot be driven)");
    }
    // begin内のenableOutputでOUTPUT+LOWになる（起動直後に通電しない）
    deployer.begin(expander);

    Serial.println("  r    LAUNCH判定状態をリセット（再度LAUNCH待ちに戻る）");
    Serial.println("  ?    現在の状態を表示");
}

void loop() {
    uint32_t start = millis();

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    sensor.update();
    float alt = sensor.getAltitude();

    if (!launched) {
        launchConfirmCount = (alt > LAUNCH_ALT_THRESHOLD_M) ? launchConfirmCount + 1 : 0;
        if (launchConfirmCount >= LAUNCH_CONFIRM_TICKS) {
            launched = true;
            telemetry.log("[TEST] LAUNCH detected! Firing rocket separation nichrome...");
            // deployRocket()は書き込み失敗を返さないため、ON/OFFを個別に呼んで結果を確認する
            bool onOk = deployer.setRocket(true);
            delay(1000);  // Deployer::deployRocket()と同じ通電時間
            bool offOk = deployer.setRocket(false);
            telemetry.log("[TEST] Rocket nichrome fired. on=%s off=%s",
                          onOk ? "OK" : "FAIL", offOk ? "OK" : "FAIL");
        }
    }

    if (start - _lastSendMs >= SEND_PERIOD_MS) {
        _lastSendMs = start;
        telemetry.sendTest("judge_launch", {{"alt", alt},
                                             {"launched", launched},
                                             {"confirm_count", launchConfirmCount}});
    }

    if (start - _lastLogMs >= LOG_PERIOD_MS) {
        _lastLogMs = start;
        printStatus(alt);
    }

    uint32_t elapsed = millis() - start;
    if (elapsed < SAMPLE_PERIOD_MS) delay(SAMPLE_PERIOD_MS - elapsed);
}
