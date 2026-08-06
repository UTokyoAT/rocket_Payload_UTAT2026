#include <Arduino.h>

// XIAO1実配線（lib/Deployer/Deployer.cppと同じピン配置）。
// ニクロム線でテグスを溶断するのに必要な通電時間を実機で調整するためのテスト。
// PlatformIOで env:test-nichrome を選択して書き込み、
// USBシリアル(115200bps)からコマンドで通電を操作する（XIAO1はWiFi非搭載のため）。
//
// コマンド（改行区切り）:
//   r        ロケット分離線(GPIO2)を現在の設定時間だけ通電
//   p        パラシュート分離線(GPIO4)を現在の設定時間だけ通電
//   s        通電中なら即座に停止
//   tr<ms>   ロケット分離線の通電時間を設定（例: tr800）
//   tp<ms>   パラシュート分離線の通電時間を設定（例: tp1200）
//   ?        現在の設定・状態を表示

static const int PIN_ROCKET    = 2;
static const int PIN_PARACHUTE = 4;
static const int PIN_LED       = 3;

// 誤操作で長時間通電し続けて焼き切りすぎたり発熱しすぎたりしないための上限
static const unsigned long MAX_BURN_MS = 15000;

enum class BurnTarget { NONE, ROCKET, PARACHUTE };

static BurnTarget    gBurning = BurnTarget::NONE;
static unsigned long gBurnStartMs = 0;
static unsigned long gBurnDurationMs = 0;

static unsigned long gDurRocket    = 10000; // とりあえず10秒流して様子を見るための暫定値
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
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    if (gBurning != BurnTarget::NONE && millis() - gBurnStartMs >= gBurnDurationMs) {
        stopBurn("duration elapsed");
    }
}
