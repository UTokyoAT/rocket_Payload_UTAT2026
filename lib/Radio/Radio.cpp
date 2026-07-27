#include "Radio.h"
#include "dashboard.h"
#include <WiFi.h>
#include <WebServer.h>

static WebServer _server(80);
static uint8_t    _frame[Radio::FRAME_SIZE] = {};
static size_t     _frameLen = Radio::FRAME_SIZE;

static int16_t  _motorCommandLeft  = 0;
static int16_t  _motorCommandRight = 0;
static uint32_t _motorCommandAt    = 0;

static float _goalLat   = 0.0f;
static float _goalLon   = 0.0f;
static bool  _goalValid = false;

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
        if (!_server.hasArg("left") || !_server.hasArg("right")) {
            _server.send(400, "text/plain", "missing left/right");
            return;
        }
        long l = constrain(_server.arg("left").toInt(),  -255, 255);
        long r = constrain(_server.arg("right").toInt(), -255, 255);
        _motorCommandLeft  = static_cast<int16_t>(l);
        _motorCommandRight = static_cast<int16_t>(r);
        _motorCommandAt    = millis();
        _server.send(200, "text/plain", "ok");
    });

    // 地上局からの目的地座標設定（GET /motorと同じPULL方向の運用）。
    // 一度設定したらhasRecentMotorCommand()のようなタイムアウトはかけない
    // （目的地は明示的に変更されるまで保持し続けるべきで、通信断で失われては困るため）。
    _server.on("/goal", HTTP_GET, []() {
        if (!_server.hasArg("lat") || !_server.hasArg("lon")) {
            _server.send(400, "text/plain", "missing lat/lon");
            return;
        }
        _goalLat   = _server.arg("lat").toFloat();
        _goalLon   = _server.arg("lon").toFloat();
        _goalValid = true;
        _server.send(200, "text/plain", "ok");
    });

    // ブラウザで 192.168.4.1 を開くとダッシュボードが表示される（/data をfetchポーリング）
    _server.on("/", HTTP_GET, []() {
        _server.send_P(200, "text/html", DASHBOARD_HTML);
    });

    _server.begin();
}

void Radio::setData(const SpiFrameToXiao2& frame) {
    // SpiFrameToXiao2はpacked・パディング無しでワイヤーフレームと同一レイアウトなので
    // フィールドごとの詰め替えは不要
    memcpy(_frame, &frame, sizeof(frame));
}

int16_t Radio::getMotorCommandLeft() {
    if (millis() - _motorCommandAt > MOTOR_COMMAND_TIMEOUT_MS) return 0;
    return _motorCommandLeft;
}

int16_t Radio::getMotorCommandRight() {
    if (millis() - _motorCommandAt > MOTOR_COMMAND_TIMEOUT_MS) return 0;
    return _motorCommandRight;
}

bool Radio::hasRecentMotorCommand() {
    return (millis() - _motorCommandAt) <= MOTOR_COMMAND_TIMEOUT_MS;
}

float Radio::getGoalLat() { return _goalLat; }
float Radio::getGoalLon() { return _goalLon; }
bool  Radio::hasGoal()    { return _goalValid; }

void Radio::poll() { _server.handleClient(); }

IPAddress Radio::getIP() { return WiFi.softAPIP(); }
