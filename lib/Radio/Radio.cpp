#include "Radio.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

static AsyncWebServer _server(80);
static AsyncWebSocket _ws("/ws");

static void _onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client,
                       AwsEventType type, void*, uint8_t*, size_t) {
    if (type == WS_EVT_CONNECT)
        Serial.printf("[Radio] client #%u connected\n", client->id());
    else if (type == WS_EVT_DISCONNECT)
        Serial.printf("[Radio] client #%u disconnected\n", client->id());
}

void Radio::begin(const char* ssid, const char* password) {
    WiFi.softAP(ssid, password);
    Serial.printf("[Radio] SoftAP: %s  IP: %s\n",
                  ssid, WiFi.softAPIP().toString().c_str());

    _ws.onEvent(_onWsEvent);
    _server.addHandler(&_ws);
    _server.begin();
}

void Radio::broadcast(const SensorData& d, MissionState state) {
    if (_ws.count() == 0) return;

    char json[256];
    snprintf(json, sizeof(json),
        "{\"t\":%lu,\"alt\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,"
        "\"yaw\":%.2f,\"lat\":%.6f,\"lon\":%.6f,\"state\":%d}",
        d.timestamp_ms, d.alt, d.roll, d.pitch,
        d.yaw, d.lat, d.lon, static_cast<int>(state));

    _ws.textAll(json);
}

void Radio::cleanup()    { _ws.cleanupClients(); }
int  Radio::clientCount(){ return _ws.count(); }
