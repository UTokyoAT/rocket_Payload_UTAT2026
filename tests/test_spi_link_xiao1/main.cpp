#include <Arduino.h>
#include <SpiLinkMaster.h>
#include "spi_protocol.h"

// SPI疎通確認テスト（XIAO1側・マスター）。
// PlatformIOで env:test-spi-link-xiao1 を選択してXIAO1へ書き込む。
//
// 目的: センサー・LAUNCH判定・WiFiを一切使わず、XIAO1⇔XIAO2間のSPI配線と
// SpiLinkMaster/SpiLinkSlaveの往復だけを確認する（test_launch_judge_xiao2で
// 常にSPI=TIMEOUTになる場合の切り分け用）。
// XIAO2側は対応する test_spi_link_xiao2 を書き込んでおくこと。
//
// 0.5秒ごとにカウンタを1つ増やしてSpiFrameToXiao2::altに載せて送信し、
// XIAO2からの応答（SpiFrameFromXiao2::goal_lat）に同じカウンタ値がエコーバック
// されて来ているかどうかをシリアルに表示する（全二重・1件キューのため、応答は
// 直前に送った値が1回分遅れて返る＝lagありで正常）。
//
// 原因切り分け用に以下も出す:
//   - setup()でこの機体のSPIピン配置（マスター側）を表示
//   - XIAO2から一度でも応答（goal_valid=1）が来た瞬間に[OK]を1回だけ表示
//     （これが出ない＝配線 or XIAO2側ファーム未書き込み/未起動の可能性が高い）
//   - goal_latの値が変化した最終時刻からの経過時間（msSinceChange）
//     （応答はあるが値が更新されない＝XIAO2側のloop()が止まっている可能性）
//   - 5秒ごとに送信数・応答数（goal_valid=1だった回数）のサマリ

static SpiLinkMaster spiLink;
static uint32_t counter = 0;

static bool     everResponded = false;
static float    lastGoalLat = -1.0f;
static uint32_t lastChangeMs = 0;

static uint32_t totalSent = 0;
static uint32_t totalResponses = 0;
static uint32_t _lastSummaryMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Serial.println("[TEST] SPI link check (XIAO1 master) starting...");
    Serial.printf("[PINS] SCK=GPIO%d MISO=GPIO%d MOSI=GPIO%d CS_MASTER=GPIO%d (XIAO2側CS_SLAVE=GPIO%d、GND共通必須)\n",
                  SpiPins::SCK, SpiPins::MISO, SpiPins::MOSI, SpiPins::CS_MASTER, SpiPins::CS_SLAVE);

    spiLink.begin();
    lastChangeMs = millis();
    _lastSummaryMs = millis();
}

void loop() {
    SpiFrameToXiao2 out{};
    out.timestamp_ms   = millis();
    out.alt             = static_cast<float>(counter);  // 疎通確認用にカウンタを載せる
    out.mission_state   = 0;

    uint32_t t0 = micros();
    SpiFrameFromXiao2 in = spiLink.transfer(out);
    uint32_t transferUs = micros() - t0;

    totalSent++;
    // goal_validはuint8_tで、XIAO2が正しく応答していれば必ず0か1にしかならない。
    // それ以外の値は配線未接続・XIAO2未起動時にMISOラインが浮いて拾うノイズなので
    // 「応答あり」として扱わない（!= 0 だけで判定すると誤検知する）
    if (in.goal_valid == 1) {
        totalResponses++;
        if (!everResponded) {
            everResponded = true;
            Serial.println("[OK] First response received from XIAO2! SPI link is up.");
        }
        if (in.goal_lat != lastGoalLat) {
            lastGoalLat = in.goal_lat;
            lastChangeMs = millis();
        }
    } else if (in.goal_valid != 0) {
        Serial.printf("[WARN] goal_valid=%d is neither 0 nor 1 -> likely floating/unconnected MISO, not a real response\n",
                      in.goal_valid);
    }

    uint32_t msSinceChange = millis() - lastChangeMs;
    Serial.printf("[SEND] counter=%lu (%luus)  ->  [RECV] goal_valid=%d goal_lat=%.1f  msSinceChange=%lu\n",
                  (unsigned long)counter, (unsigned long)transferUs,
                  in.goal_valid, in.goal_lat, (unsigned long)msSinceChange);

    uint32_t now = millis();
    if (now - _lastSummaryMs >= 5000) {
        _lastSummaryMs = now;
        Serial.printf("[SUMMARY] sent=%lu responded=%lu (%.0f%%) everResponded=%s\n",
                      (unsigned long)totalSent, (unsigned long)totalResponses,
                      totalSent ? (100.0f * totalResponses / totalSent) : 0.0f,
                      everResponded ? "yes" : "no");
        if (!everResponded) {
            Serial.println("[HINT] 一度も応答なし: 配線(SCK/MISO/MOSI/CS/GND)、"
                            "XIAO2側にtest_spi_link_xiao2が書き込まれ起動しているかを確認してください。");
        } else if (msSinceChange > 3000) {
            Serial.println("[HINT] 応答はあるが値が更新されていません: XIAO2側のloop()が"
                            "途中で止まっている可能性があります。");
        }
    }

    counter++;
    delay(500);
}
