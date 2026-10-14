#include <Arduino.h>
#include <Expander.h>
#include <Deployer.h>
#include <LED.h>

// ニクロム線でテグスを溶断するのに必要な通電時間を実機で調整するためのテスト。
// PlatformIOで env:test-nichrome を選択して書き込み、USBシリアル(115200bps)から操作する。
// ニクロム線・LEDのピン配置は src/Deployer, src/LED 側で定義している。
//
// シリアルコマンド（改行区切り）:
//   r        ロケット分離線を現在の設定時間だけ通電
//   p        パラシュート分離線を現在の設定時間だけ通電
//   s        通電中なら即座に停止
//   tr<ms>   ロケット分離線の通電時間を設定（例: tr800）
//   tp<ms>   パラシュート分離線の通電時間を設定（例: tp1200）
//   ?        現在の設定・状態を表示


// 誤操作で長時間通電し続けて焼き切りすぎたり発熱しすぎたりしないための上限
static const unsigned long MAX_BURN_MS = 15000;

static Expander expander;
static Deployer deployer;
static LED      led;

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
    deployer.setRocket(false);
    deployer.setParachute(false);
    led.off();
    if (gBurning != BurnTarget::NONE) {
        Serial.printf("[STOP] %s (elapsed=%lums)\n", reason, millis() - gBurnStartMs);
    }
    gBurning = BurnTarget::NONE;
}

static void startBurn(BurnTarget target, unsigned long durationMs) {
    if (gBurning != BurnTarget::NONE) {
        Serial.println("[WARN] already burning, ignoring request");
        return;
    }
    durationMs = constrain(durationMs, 0UL, MAX_BURN_MS);
    bool ok = (target == BurnTarget::ROCKET) ? deployer.setRocket(true) : deployer.setParachute(true);
    if (!ok) {
        Serial.println("[ERR] expander write failed, not burning");
        return;
    }
    gBurning = target;
    gBurnStartMs = millis();
    gBurnDurationMs = durationMs;
    led.on();
    Serial.printf("[START] target=%s duration=%lums\n",
                  target == BurnTarget::ROCKET ? "rocket" : "parachute", durationMs);
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "r") {
        startBurn(BurnTarget::ROCKET, gDurRocket);
    } else if (cmd == "p") {
        startBurn(BurnTarget::PARACHUTE, gDurParachute);
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
    delay(500);  // USB CDC安定待ち

    Serial.println("[TEST] Nichrome burn test starting...");
    if (!expander.setup()) {
        Serial.println("[TEST] Expander init failed (nichrome cannot be driven)");
    }
    // begin内のenableOutputでOUTPUT+LOWになる（起動直後に通電しない）
    deployer.begin(expander);
    led.begin(expander);

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
