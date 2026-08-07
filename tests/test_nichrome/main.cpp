#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// XIAO1実配線（lib/Deployer/Deployer.cppと同じピン配置）。
// ニクロム線でテグスを溶断するのに必要な通電時間を実機で調整するためのテスト。
// PlatformIOで env:test-nichrome を選択して書き込む。
// 本番のXIAO1（task_mission.h）はWiFiを使わないが、この基板自体（ESP32S3）は
// WiFiを積んでいるため、ベンチでUSBを外して安全な距離から操作したい場合向けに
// USBシリアル(115200bps)に加えてWiFi（ブラウザ）からも同じ操作ができるようにしてある。
//
// シリアルコマンド（改行区切り）:
//   r        ロケット分離線(GPIO2)を現在の設定時間だけ通電
//   p        パラシュート分離線(GPIO4)を現在の設定時間だけ通電
//   s        通電中なら即座に停止
//   tr<ms>   ロケット分離線の通電時間を設定（例: tr800）
//   tp<ms>   パラシュート分離線の通電時間を設定（例: tp1200）
//   ?        現在の設定・状態を表示
//
// WiFi: CanSat-AP（パスワード cansat2026）に接続し、ブラウザで http://192.168.4.1
// を開くとボタンで同じ操作ができる（1秒ごとに状態を自動更新）。
// エンドポイントもcurl等から直接叩ける:
//   GET /fire?target=rocket|parachute   通電開始
//   GET /stop                           即座に停止
//   GET /set?target=rocket|parachute&ms=800   通電時間を設定
//   GET /status                         現在の設定・状態をプレーンテキストで返す

static const int PIN_ROCKET    = 2;
static const int PIN_PARACHUTE = 4;
static const int PIN_LED       = 3;

// 誤操作で長時間通電し続けて焼き切りすぎたり発熱しすぎたりしないための上限
static const unsigned long MAX_BURN_MS = 15000;

static const char* AP_SSID = "CanSat-AP";
static const char* AP_PASS = "cansat2026";

static WebServer server(80);

enum class BurnTarget { NONE, ROCKET, PARACHUTE };

static BurnTarget    gBurning = BurnTarget::NONE;
static unsigned long gBurnStartMs = 0;
static unsigned long gBurnDurationMs = 0;

static unsigned long gDurRocket    = 1000;  // とりあえず1秒流して様子を見るための暫定値
static unsigned long gDurParachute = 1000;  // Deployer::deployParachute()の初期値と合わせている

static void printStatus() {
    Serial.printf("[STATUS] rocket_dur=%lums parachute_dur=%lums burning=%s\n",
                  gDurRocket, gDurParachute,
                  gBurning == BurnTarget::NONE ? "none" :
                  (gBurning == BurnTarget::ROCKET ? "rocket" : "parachute"));
}

static void stopBurn(const char* reason) {
    digitalWrite(PIN_ROCKET, LOW);
    digitalWrite(PIN_PARACHUTE, LOW);
    digitalWrite(PIN_LED, LOW);
    if (gBurning != BurnTarget::NONE) {
        Serial.printf("[STOP] %s (elapsed=%lums)\n", reason, millis() - gBurnStartMs);
    }
    gBurning = BurnTarget::NONE;
}

static void startBurn(BurnTarget target, int pin, unsigned long durationMs) {
    if (gBurning != BurnTarget::NONE) {
        Serial.println("[WARN] already burning, ignoring request");
        return;
    }
    durationMs = constrain(durationMs, 0UL, MAX_BURN_MS);
    gBurning = target;
    gBurnStartMs = millis();
    gBurnDurationMs = durationMs;
    digitalWrite(pin, HIGH);
    digitalWrite(PIN_LED, HIGH);
    Serial.printf("[START] target=%s duration=%lums\n",
                  target == BurnTarget::ROCKET ? "rocket" : "parachute", durationMs);
}

static String statusText() {
    String s;
    s += "rocket_dur_ms=" + String(gDurRocket) + "\n";
    s += "parachute_dur_ms=" + String(gDurParachute) + "\n";
    s += "burning=" + String(gBurning == BurnTarget::NONE ? "none" :
                              (gBurning == BurnTarget::ROCKET ? "rocket" : "parachute")) + "\n";
    if (gBurning != BurnTarget::NONE) {
        s += "elapsed_ms=" + String(millis() - gBurnStartMs) + "\n";
    }
    return s;
}

// GET/シリアル共通のハンドラ。target文字列("rocket"/"parachute")はWebとシリアルの
// コマンド文字("r"/"p")の橋渡し用
static bool startBurnByName(const String& target) {
    if (target == "rocket") {
        startBurn(BurnTarget::ROCKET, PIN_ROCKET, gDurRocket);
        return true;
    } else if (target == "parachute") {
        startBurn(BurnTarget::PARACHUTE, PIN_PARACHUTE, gDurParachute);
        return true;
    }
    return false;
}

static void handleWebFire() {
    if (!server.hasArg("target") || !startBurnByName(server.arg("target"))) {
        server.send(400, "text/plain", "usage: /fire?target=rocket|parachute");
        return;
    }
    server.send(200, "text/plain", statusText());
}

static void handleWebStop() {
    stopBurn("web stop");
    server.send(200, "text/plain", statusText());
}

static void handleWebSet() {
    String target = server.arg("target");
    if (!server.hasArg("ms") || (target != "rocket" && target != "parachute")) {
        server.send(400, "text/plain", "usage: /set?target=rocket|parachute&ms=800");
        return;
    }
    unsigned long ms = constrain((unsigned long)server.arg("ms").toInt(), 0UL, MAX_BURN_MS);
    if (target == "rocket") {
        gDurRocket = ms;
        Serial.printf("[SET] rocket duration = %lums (via WiFi)\n", gDurRocket);
    } else {
        gDurParachute = ms;
        Serial.printf("[SET] parachute duration = %lums (via WiFi)\n", gDurParachute);
    }
    server.send(200, "text/plain", statusText());
}

static void handleWebStatus() {
    server.send(200, "text/plain", statusText());
}

static void handleWebRoot() {
    String html;
    html += "<!doctype html><html><head><meta charset=\"utf-8\">";
    html += "<meta http-equiv=\"refresh\" content=\"1\">";
    html += "<title>Nichrome burn test</title></head><body>";
    html += "<h1>Nichrome burn test (XIAO1)</h1>";
    html += "<pre>" + statusText() + "</pre>";
    html += "<p><a href=\"/fire?target=rocket\">FIRE ROCKET</a> "
            "(dur=" + String(gDurRocket) + "ms)</p>";
    html += "<p><a href=\"/fire?target=parachute\">FIRE PARACHUTE</a> "
            "(dur=" + String(gDurParachute) + "ms)</p>";
    html += "<p><a href=\"/stop\">STOP</a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "r") {
        startBurn(BurnTarget::ROCKET, PIN_ROCKET, gDurRocket);
    } else if (cmd == "p") {
        startBurn(BurnTarget::PARACHUTE, PIN_PARACHUTE, gDurParachute);
    } else if (cmd == "s") {
        stopBurn("manual stop");
    } else if (cmd == "?") {
        printStatus();
    } else if (cmd.startsWith("tr")) {
        gDurRocket = constrain((unsigned long)cmd.substring(2).toInt(), 0UL, MAX_BURN_MS);
        Serial.printf("[SET] rocket duration = %lums\n", gDurRocket);
    } else if (cmd.startsWith("tp")) {
        gDurParachute = constrain((unsigned long)cmd.substring(2).toInt(), 0UL, MAX_BURN_MS);
        Serial.printf("[SET] parachute duration = %lums\n", gDurParachute);
    } else {
        Serial.println("[ERR] unknown command (r/p/s/tr<ms>/tp<ms>/?)");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ROCKET, OUTPUT);
    pinMode(PIN_PARACHUTE, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_ROCKET, LOW);
    digitalWrite(PIN_PARACHUTE, LOW);
    digitalWrite(PIN_LED, LOW);

    Serial.println("[TEST] Nichrome burn test starting...");
    Serial.println("  r        ロケット分離線を通電");
    Serial.println("  p        パラシュート分離線を通電");
    Serial.println("  s        即座に停止");
    Serial.println("  tr<ms>   ロケット通電時間を設定（例: tr800）");
    Serial.println("  tp<ms>   パラシュート通電時間を設定（例: tp1200）");
    Serial.println("  ?        現在の設定・状態を表示");
    printStatus();

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[TEST] WiFi AP: %s  Open http://%s in a browser to fire remotely\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/",       HTTP_GET, handleWebRoot);
    server.on("/fire",   HTTP_GET, handleWebFire);
    server.on("/stop",   HTTP_GET, handleWebStop);
    server.on("/set",    HTTP_GET, handleWebSet);
    server.on("/status", HTTP_GET, handleWebStatus);
    server.begin();
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    server.handleClient();

    if (gBurning != BurnTarget::NONE && millis() - gBurnStartMs >= gBurnDurationMs) {
        stopBurn("duration elapsed");
    }
}
