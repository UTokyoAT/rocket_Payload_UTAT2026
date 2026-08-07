#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SpiLinkSlave.h>
#include "spi_protocol.h"

// SPI疎通確認テスト（XIAO2側・スレーブ）。
// PlatformIOで env:test-spi-link-xiao2 を選択してXIAO2へ書き込む。
//
// 目的: センサー・LAUNCH判定を一切使わず、XIAO1⇔XIAO2間のSPI配線と
// SpiLinkMaster/SpiLinkSlaveの往復だけを確認する。
// XIAO1側は対応する test_spi_link_xiao1 を書き込んでおくこと。
//
// XIAO2はベンチ試験中USBに繋がずSPI配線だけで動かすことが多いため、診断情報は
// シリアルではなくWiFi経由（ブラウザ）で見られるようにする。本番のRadioクラスは
// テレメトリ配信用の固定エンドポイント（/data等）しか持たないため使わず、
// このテスト専用にWiFiアクセスポイント＋簡易Webページを直接立てる。
//
// 使い方: CanSat-AP（パスワード cansat2026）に接続し、ブラウザで
// http://192.168.4.1 を開く（1秒ごとに自動更新）。
//   - everReceived=no のまま        : 配線(SCK/MISO/MOSI/CS/GND) or
//                                      XIAO1側ファーム未書き込み/未起動を疑う
//   - msSinceLastRecv が伸び続ける  : 一度は繋がったがリンクが途中で切れている

static const char* AP_SSID = "CanSat-AP";
static const char* AP_PASS = "cansat2026";

static WebServer server(80);
static SpiLinkSlave spiLink;
static SpiFrameToXiao2 lastFrame{};

static uint32_t recvCount = 0;
static uint32_t lastRecvMs = 0;
static bool     everReceived = false;

static void handleRoot() {
    uint32_t now = millis();
    uint32_t msSinceRecv = everReceived ? (now - lastRecvMs) : 0;

    String html;
    html += "<!doctype html><html><head><meta charset=\"utf-8\">";
    html += "<meta http-equiv=\"refresh\" content=\"1\">";
    html += "<title>SPI Link Debug (XIAO2)</title></head><body>";
    html += "<h1>SPI Link Debug (XIAO2 slave)</h1>";
    html += "<pre>";
    html += "uptime_ms       : " + String(now) + "\n";
    html += "everReceived    : " + String(everReceived ? "yes" : "no") + "\n";
    html += "recvCount       : " + String(recvCount) + "\n";
    html += "msSinceLastRecv : " + String(msSinceRecv) + "\n";
    html += "last counter    : " + String(lastFrame.alt, 0) + "\n";
    html += "last timestamp  : " + String(lastFrame.timestamp_ms) + "\n";
    html += "pins            : SCK=GPIO" + String(SpiPins::SCK) +
            " MISO=GPIO" + String(SpiPins::MISO) +
            " MOSI=GPIO" + String(SpiPins::MOSI) +
            " CS_SLAVE=GPIO" + String(SpiPins::CS_SLAVE) + "\n";
    html += "</pre>";

    if (!everReceived) {
        html += "<p style=\"color:red\">XIAO1から一度もフレームを受信していません。"
                "配線(SCK/MISO/MOSI/CS/GND)とXIAO1側にtest_spi_link_xiao1が"
                "書き込まれ起動しているか確認してください。</p>";
    } else if (msSinceRecv > 3000) {
        html += "<p style=\"color:orange\">直近3秒以上新しいフレームが届いていません"
                "（リンク断の可能性）。</p>";
    } else {
        html += "<p style=\"color:green\">受信中です。</p>";
    }

    html += "</body></html>";
    server.send(200, "text/html", html);
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち（USB未接続でも起動には影響しない）

    Serial.println("[TEST] SPI link check (XIAO2 slave, WiFi debug view) starting...");

    spiLink.begin();

    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[TEST] WiFi AP: %s  Open http://%s in a browser to view debug info\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    server.on("/", HTTP_GET, handleRoot);
    server.begin();
}

void loop() {
    if (spiLink.poll(lastFrame)) {
        recvCount++;
        lastRecvMs = millis();
        everReceived = true;

        // 受け取ったカウンタをそのまま送り返し、XIAO1側で往復一致を確認できるようにする
        SpiFrameFromXiao2 out{};
        out.goal_valid = 1;
        out.goal_lat   = lastFrame.alt;
        out.goal_lon   = 0.0f;
        spiLink.setResponse(out);
    }

    server.handleClient();
}
