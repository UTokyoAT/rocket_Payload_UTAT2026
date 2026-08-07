#include <Arduino.h>
#include <Radio.h>
#include "spi_protocol.h"
#include <SpiLinkSlave.h>

// LAUNCH判定〜ロケット分離統合確認テスト（XIAO2側）。
// PlatformIOで env:test-launch-judge-xiao2 を選択してXIAO2へ書き込む。
//
// 目的: XIAO1（test_launch_judge_xiao1）からSPIで受信したフレームを
//   そのままWiFiテレメトリとして地上局へ中継する。本番のxiao2/main.cppと異なり、
//   このテストではモーター（Actuator）は使わない（LAUNCH判定はモータ走行と無関係のため）。
//
// 確認方法: XIAO1（test_launch_judge_xiao1）を書き込んで気圧センサ値を変化させ、
//   ブラウザで http://192.168.4.1 を開くとalt・mission_stateがリアルタイムに更新される。
//   mission_stateがLAUNCH(1)からDETACH(2)に変わればXIAO1側でLAUNCH判定＋Rocket通電が
//   起きたことを意味する。

static const char* AP_SSID = "CanSat-AP";
static const char* AP_PASS = "cansat2026";

static SpiLinkSlave spiLink;
static Radio radio;

// XIAO1から最後に受信したフレーム。新しいフレームが届かない間もこれを使い続ける
static SpiFrameToXiao2 lastFrame{};

static uint32_t lastSpiFrameMs = 0;
static const uint32_t SPI_LINK_TIMEOUT_MS = 5000;

static uint32_t _lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Serial.println("[TEST] LAUNCH judge integration check (XIAO2: SPI receive + WiFi relay) starting...");

    spiLink.begin();
    radio.begin(AP_SSID, AP_PASS);
    Serial.printf("[TEST] Radio ready. Connect to %s and open http://%s\n",
                  AP_SSID, radio.getIP().toString().c_str());
}

void loop() {
    if (spiLink.poll(lastFrame)) {
        // このテストでは目的地(goal)を使わないため常に無効値を返す
        SpiFrameFromXiao2 out{};
        out.goal_valid = 0;
        out.goal_lat   = 0.0f;
        out.goal_lon   = 0.0f;
        spiLink.setResponse(out);
        lastSpiFrameMs = millis();
    }

    radio.setData(lastFrame);
    radio.poll();

    uint32_t now = millis();
    if (now - _lastPrintMs >= 500) {
        _lastPrintMs = now;
        bool linkOk = (now - lastSpiFrameMs) <= SPI_LINK_TIMEOUT_MS;
        Serial.printf("[STATUS] SPI=%s alt=%.2fm mission_state=%d (last recv %lums ago)\n",
                      linkOk ? "OK" : "TIMEOUT", lastFrame.alt, lastFrame.mission_state,
                      (unsigned long)(now - lastSpiFrameMs));
    }
}
