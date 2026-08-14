#include <Arduino.h>
#include <Sensor.h>
#include <Deployer.h>
#include <SpiLinkMaster.h>
#include "spi_protocol.h"

// LAUNCH判定〜ロケット分離（ニクロム線通電）統合確認テスト（XIAO1側）。
// PlatformIOで env:test-launch-judge-xiao1 を選択してXIAO1へ書き込む。
//
// 目的: 本番のtask_mission.hのうちSETTING/DETACH以降を省き、
//   気圧センサ値取得 → XIAO2へのSPI送信 → LAUNCH判定 → Rocket分離ニクロム線通電
//   の一連の流れだけを単体で確認する。
// XIAO2側は対応する test_launch_judge_xiao2（SPI受信+WiFi中継のみ、モーターは使わない）を
// 書き込んでおくこと（配線は本番のXIAO1⇔XIAO2 SPI接続のまま）。
//
// LAUNCH判定のしきい値・確認tick数はtask_mission.hの値と揃えている
// （LAUNCH_ALT_THRESHOLD_M=3.0m、LAUNCH_CONFIRM_TICKS=5、周期は本テストも100Hz目安）。
//
// ベンチで気圧センサを手で覆う/開放する等して繰り返し試せるよう、シリアルから
// 'r' でLAUNCH判定状態をリセットできる（実機のように再起動しなくても再試行可能）。
//
// コマンド（USBシリアル、改行区切り）:
//   r    LAUNCH判定状態をリセットし、再度LAUNCH待ちに戻す（Rocketは再通電しない）
//   ?    現在の状態を表示

static const float LAUNCH_ALT_THRESHOLD_M = 3.0f;
static const int   LAUNCH_CONFIRM_TICKS   = 5;

static Sensor sensor;
static Deployer deployer;
static SpiLinkMaster spiLink;

static bool launched = false;
static int  launchConfirmCount = 0;

static uint32_t _lastPrintMs = 0;

static void printStatus(float alt) {
    Serial.printf("[STATUS] alt=%.2fm launched=%s confirmCount=%d/%d\n",
                  alt, launched ? "yes" : "no", launchConfirmCount, LAUNCH_CONFIRM_TICKS);
}

static void handleCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd == "r") {
        launched = false;
        launchConfirmCount = 0;
        Serial.println("[RESET] LAUNCH judge state reset. Waiting for LAUNCH again.");
    } else if (cmd == "?") {
        printStatus(sensor.getAltitude());
    } else {
        Serial.println("[ERR] unknown command (r/?)");
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Serial.println("[TEST] LAUNCH judge -> Rocket deploy integration check starting...");

    sensor.begin();
    Serial.printf("[TEST] BMP280=%s\n", sensor.isBmp280Ready() ? "OK" : "NG");

    deployer.begin();
    spiLink.begin();

    Serial.println("  r    LAUNCH判定状態をリセット（再度LAUNCH待ちに戻る）");
    Serial.println("  ?    現在の状態を表示");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }

    sensor.update();
    float alt = sensor.getAltitude();

    // task_mission.hのLAUNCH状態と同じロジック（高度がしきい値を一定tick連続で超えたら確定）
    if (!launched) {
        launchConfirmCount = (alt > LAUNCH_ALT_THRESHOLD_M) ? launchConfirmCount + 1 : 0;
        if (launchConfirmCount >= LAUNCH_CONFIRM_TICKS) {
            launched = true;
            Serial.println("[TEST] LAUNCH detected! Firing rocket separation nichrome...");
            deployer.deployRocket();
            Serial.println("[TEST] Rocket nichrome fired.");
        }
    }

    SpiFrameToXiao2 out{};
    out.timestamp_ms = millis();
    out.alt   = alt;
    out.roll  = sensor.getRoll();
    out.pitch = sensor.getPitch();
    out.yaw   = 0.0f;
    out.lat   = 0.0f;
    out.lon   = 0.0f;
    // このテストはDETACH以降を扱わないため、LAUNCH確定後は代替としてDETACHを送る
    // （XIAO2側は本テストではモータを動かさないため実害はない）
    out.mission_state   = static_cast<uint8_t>(launched ? MissionState::DETACH : MissionState::LAUNCH);
    out.pid_output       = 0.0f;
    out.destination_yaw  = 0.0f;

    spiLink.transfer(out);  // XIAO2への送信のみが目的。応答（goal）はこのテストでは使わない

    uint32_t now = millis();
    if (now - _lastPrintMs >= 500) {
        _lastPrintMs = now;
        printStatus(alt);
    }

    delay(10);  // 100Hz目安（task_mission.hのMISSION_TICK_MSに合わせる）
}
