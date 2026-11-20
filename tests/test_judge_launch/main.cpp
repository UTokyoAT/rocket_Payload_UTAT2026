#include <Arduino.h>
#include <Expander.h>
#include <Sensor.h>
#include <Deployer.h>
#include <Telemetry.h>
#include "secrets.h"

// LAUNCH判定〜ロケット分離（ニクロム線通電）確認テスト。
// PlatformIOで env:test-judge-launch を選択して書き込む。
// 目的: 気圧センサ値取得 → LAUNCH判定 → Rocket分離ニクロム線通電、の流れを単体で確認する。
//
// LAUNCH判定のしきい値・確認tick数はtest_allと揃えている
// （LAUNCH_ALT_THRESHOLD_M=3.0m、LAUNCH_CONFIRM_TICKS=5）。ただし本テストは100Hzで回す。
//
// 確認方法: 事前に include/secrets.h を設定し、ground/server で `docker compose up -d` しておく。
//   0.5秒ごとに Telemetry で地上局へ送る（USBシリアル 115200bps にも同じ内容が出る）。
//     <prefix>/test/launch_judge   alt_m, pressure_hpa, launched, confirm_cnt
//     <prefix>/log                 [STATUS] 行・LAUNCH検知などのイベント
//
// ベンチで気圧センサを手で覆う/開放する等して繰り返し試せるよう、
// 'r' でLAUNCH判定状態をリセットできる（実機のように再起動しなくても再試行可能）。
//
// コマンド（USBシリアル改行区切り、または MQTT の <prefix>/cmd）:
//   r    LAUNCH判定状態をリセットし、再度LAUNCH待ちに戻す（Rocketは再通電しない）
//   ?    現在の状態を表示
// 例: mosquitto_pub -h <PCのIP> -t rocket/cmd -q 1 -m r

static const float LAUNCH_ALT_THRESHOLD_M = 3.0f;
static const int   LAUNCH_CONFIRM_TICKS   = 5;
static const uint32_t SEND_PERIOD_MS      = 500;

static Expander  expander;
static Sensor    sensor;
static Deployer  deployer;
static Telemetry telemetry;

static bool launched = false;
static int  launchConfirmCount = 0;

static uint32_t _lastSendMs = 0;

// MQTT の内部タスクから呼ばれるので、ここでは受け取るだけにして loop() で処理する
static volatile char _pendingCmd = 0;

static void printStatus(float alt) {
    telemetry.log("[STATUS] alt=%.2fm p=%.2fhPa launched=%s confirmCount=%d/%d mqtt=%s",
                  alt, sensor.getPressure(), launched ? "yes" : "no",
                  launchConfirmCount, LAUNCH_CONFIRM_TICKS, telemetry.isConnected() ? "yes" : "no");
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
        telemetry.log("[ERR] unknown command (r/?)");
    }
}

static void onCommand(const char* cmd) {
    if (cmd[0] && cmd[1] == '\0') _pendingCmd = cmd[0];  // 1文字コマンドだけ受け付ける
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.onCommand(onCommand);
    telemetry.begin(cfg);

    telemetry.log("[TEST] LAUNCH judge -> Rocket deploy check starting...");

    sensor.begin();
    telemetry.log("[TEST] BMP280=%s", sensor.isBmp280Ready() ? "OK" : "NG");

    if (!expander.setup()) {
        telemetry.log("[TEST] Expander init failed (nichrome cannot be driven)");
    }
    deployer.begin(expander);  // enableOutputでOUTPUT+LOWになる（起動直後に通電しない）

    Serial.println("  r    LAUNCH判定状態をリセット（再度LAUNCH待ちに戻る）");
    Serial.println("  ?    現在の状態を表示");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }
    if (_pendingCmd) {
        char c = _pendingCmd;
        _pendingCmd = 0;
        handleCommand(String(c));
    }

    sensor.update();
    float alt = sensor.getAltitude();

    // 高度がしきい値を一定tick連続で超えたらLAUNCH確定
    if (!launched) {
        launchConfirmCount = (alt > LAUNCH_ALT_THRESHOLD_M) ? launchConfirmCount + 1 : 0;
        if (launchConfirmCount >= LAUNCH_CONFIRM_TICKS) {
            launched = true;
            telemetry.log("[TEST] LAUNCH detected! alt=%.2fm Firing rocket separation nichrome...", alt);
            deployer.deployRocket();  // 通電中（約3秒）はループが止まる
            telemetry.log("[TEST] Rocket nichrome fired.");
        }
    }

    uint32_t now = millis();
    if (now - _lastSendMs >= SEND_PERIOD_MS) {
        _lastSendMs = now;
        telemetry.sendTest("launch_judge", {{"alt_m", alt},
                                            {"pressure_hpa", sensor.getPressure()},
                                            {"launched", launched},
                                            {"confirm_cnt", launchConfirmCount}});
        printStatus(alt);
    }

    delay(10);  // 100Hz目安
}
