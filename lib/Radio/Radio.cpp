#include "Radio.h"
#include "dashboard.h"
#include <WiFi.h>
#include <WebServer.h>

static WebServer _server(80);
static uint8_t    _frame[Radio::FRAME_SIZE] = {};
static size_t     _frameLen = Radio::FRAME_SIZE;

static int16_t  _motorCommand   = 0;
static uint32_t _motorCommandAt = 0;

void Radio::begin(const char* ssid, const char* password) {
    WiFi.softAP(ssid, password);
    Serial.printf("[Radio] SoftAP: %s  IP: %s\n",
                  ssid, WiFi.softAPIP().toString().c_str());

    // PC側・ブラウザ側ともにこのエンドポイントをGETでポーリングする（PULL方式）
    _server.on("/data", HTTP_GET, []() {
        _server.setContentLength(_frameLen);
        _server.send(200, "application/octet-stream", "");
        _server.sendContent((const char*)_frame, _frameLen);
    });

    // 地上局からのモーター手動制御コマンド（PC→機体のアウトバウンド通信なので
    // /data のPULLと同じ方向。ファイアウォールに阻まれやすいPUSHとは異なる）
    _server.on("/motor", HTTP_GET, []() {
        if (!_server.hasArg("value")) {
            _server.send(400, "text/plain", "missing value");
            return;
        }
        long v = _server.arg("value").toInt();
        v = constrain(v, -255, 255);
        _motorCommand   = static_cast<int16_t>(v);
        _motorCommandAt = millis();
        _server.send(200, "text/plain", "ok");
    });

    // ブラウザで 192.168.4.1 を開くとダッシュボードが表示される（/data をfetchポーリング）
    _server.on("/", HTTP_GET, []() {
        _server.send_P(200, "text/html", DASHBOARD_HTML);
    });

    _server.begin();
}

void Radio::setData(const SensorData& d, MissionState state) {
    size_t off = 0;
    auto put = [&](const void* src, size_t n) {
        memcpy(_frame + off, src, n);
        off += n;
    };

    put(&d.timestamp_ms, sizeof(d.timestamp_ms));
    put(&d.alt,          sizeof(d.alt));
    put(&d.roll,         sizeof(d.roll));
    put(&d.pitch,        sizeof(d.pitch));
    put(&d.yaw,           sizeof(d.yaw));

    float lat = static_cast<float>(d.lat);
    float lon = static_cast<float>(d.lon);
    put(&lat, sizeof(lat));
    put(&lon, sizeof(lon));

    uint8_t stateByte = static_cast<uint8_t>(state);
    put(&stateByte, sizeof(stateByte));

    put(&d.motorOutput, sizeof(d.motorOutput));
}

int16_t Radio::getMotorCommand() {
    if (millis() - _motorCommandAt > MOTOR_COMMAND_TIMEOUT_MS) return 0;
    return _motorCommand;
}

void Radio::poll() { _server.handleClient(); }

IPAddress Radio::getIP() { return WiFi.softAPIP(); }
