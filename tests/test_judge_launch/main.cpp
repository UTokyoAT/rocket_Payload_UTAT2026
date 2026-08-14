#include <Arduino.h>
#include <Sensor.h>
#include <Deployer.h>
#include <Radio.h>
#include "spi_protocol.h"

// LAUNCH判定〜ロケット分離（ニクロム線通電）確認テスト（XIAO1単体完結版）。
// PlatformIOで env:test-judge-launch を選択してXIAO1へ書き込む。
//
// test_launch_judge_xiao1/xiao2はXIAO1⇔XIAO2のSPI連携込みで確認するテストだが、
// こちらはXIAO2を使わず、XIAO1だけで完結させる。WiFiテレメトリ配信もSPI経由の
// 中継ではなくXIAO1のRadioで直接行う（test_sensorと同じ構成）。
// 目的: 気圧センサ値取得 → LAUNCH判定 → Rocket分離ニクロム線通電 → WiFiでの状態確認、
//   の一連の流れをXIAO2の配線・書き込みなしで単体確認する。
//
// LAUNCH判定のしきい値・確認tick数はtask_mission.hの値と揃えている
// （LAUNCH_ALT_THRESHOLD_M=3.0m、LAUNCH_CONFIRM_TICKS=5、周期は本テストも100Hz目安）。
//
// 確認方法: CanSat-AP（パスワード cansat2026）に接続し、http://192.168.4.1 で
//   alt・mission_stateを確認する。mission_stateがLAUNCH(1)からDETACH(2)に変われば
//   LAUNCH判定＋Rocket通電が起きたことを意味する。
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
static Radio radio;

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

    Serial.println("[TEST] LAUNCH judge -> Rocket deploy check (XIAO1 standalone) starting...");

    sensor.begin();
    Serial.printf("[TEST] BMP280=%s\n", sensor.isBmp280Ready() ? "OK" : "NG");

    deployer.begin();

    radio.begin("CanSat-AP", "cansat2026");
    Serial.printf("[TEST] Radio ready. Connect to CanSat-AP and open http://%s\n",
                  radio.getIP().toString().c_str());

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
    out.mission_state   = static_cast<uint8_t>(launched ? MissionState::DETACH : MissionState::LAUNCH);
    out.pid_output       = 0.0f;
    out.destination_yaw  = 0.0f;

    radio.setData(out);
    radio.poll();

    uint32_t now = millis();
    if (now - _lastPrintMs >= 500) {
        _lastPrintMs = now;
        printStatus(alt);
    }

    delay(10);  // 100Hz目安（task_mission.hのMISSION_TICK_MSに合わせる）
}
