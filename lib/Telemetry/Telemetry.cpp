#include "Telemetry.h"
#include <WiFi.h>
#include <stdarg.h>
#include <string.h>

static const uint32_t WIFI_RETRY_MS = 10000;
static const uint32_t MQTT_RETRY_MS = 2000;
static const uint32_t MANAGE_TICK_MS = 200;

// MQTTトピックに使えない文字(/ + # 空白など)を _ に置き換える
static void sanitizeName(const char* in, char* out, size_t cap) {
    size_t n = 0;
    for (; in && *in && n + 1 < cap; ++in) {
        char c = *in;
        bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        out[n++] = ok ? c : '_';
    }
    if (n == 0 && cap > 8) { strcpy(out, "unnamed"); return; }
    out[n] = '\0';
}

// 内部タスクはWi-Fiスタックと同じCore 0に置き、リアルタイム系のCore 1を空けておく
Telemetry::Telemetry() : _mqtt(1, 0) {}

bool Telemetry::begin(const Config& cfg) {
    if (_started || !cfg.ssid || !cfg.mqttHost) return false;

    strlcpy(_ssid, cfg.ssid, sizeof(_ssid));
    strlcpy(_password, cfg.password ? cfg.password : "", sizeof(_password));
    strlcpy(_host, cfg.mqttHost, sizeof(_host));
    strlcpy(_clientId, cfg.clientId ? cfg.clientId : "xiao-esp32s3", sizeof(_clientId));
    strlcpy(_prefix, cfg.topicPrefix ? cfg.topicPrefix : "rocket", sizeof(_prefix));

    _mqtt.setServer(_host, cfg.mqttPort).setClientId(_clientId).setKeepAlive(15);
    _mqtt.onConnect([](bool) { Serial.println("[Telemetry] MQTT connected"); });
    _mqtt.onDisconnect([](espMqttClientTypes::DisconnectReason reason) {
        Serial.printf("[Telemetry] MQTT disconnected (reason=%d)\n", static_cast<int>(reason));
    });

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // モデムスリープを切る(10Hz送信の遅延ばらつきを避ける)
    WiFi.setAutoReconnect(true);
    WiFi.begin(_ssid, _password[0] ? _password : nullptr);

    _started = true;
    return xTaskCreate(&Telemetry::manageTask, "telemetry", 4096, this, 1, &_task) == pdPASS;
}

bool Telemetry::isConnected() const { return _mqtt.connected(); }

void Telemetry::manageTask(void* self) { static_cast<Telemetry*>(self)->manageLoop(); }

void Telemetry::manageLoop() {
    uint32_t lastWifiTry = millis();
    uint32_t lastMqttTry = 0;
    bool     wasWifiUp   = false;

    for (;;) {
        uint32_t now    = millis();
        bool     wifiUp = (WiFi.status() == WL_CONNECTED);

        if (wifiUp != wasWifiUp) {
            wasWifiUp = wifiUp;
            if (wifiUp) Serial.printf("[Telemetry] WiFi connected, IP=%s\n", WiFi.localIP().toString().c_str());
            else        Serial.println("[Telemetry] WiFi disconnected");
        }

        if (!wifiUp) {
            if (now - lastWifiTry >= WIFI_RETRY_MS) {
                lastWifiTry = now;
                WiFi.begin(_ssid, _password[0] ? _password : nullptr);
            }
        } else if (_mqtt.disconnected() && now - lastMqttTry >= MQTT_RETRY_MS) {
            lastMqttTry = now;
            _mqtt.connect();
        }

        vTaskDelay(pdMS_TO_TICKS(MANAGE_TICK_MS));
    }
}

bool Telemetry::publishRaw(const char* topic, uint8_t qos, const char* payload, size_t len) {
    return _mqtt.publish(topic, qos, false, reinterpret_cast<const uint8_t*>(payload), len) != 0;
}

bool Telemetry::publishRecord(const TelemetryRecord& record) {
    if (record.utc_ms == 0) return false;

    char payload[768];
    uint32_t seq = _seq.fetch_add(1);
    size_t   n   = telemetryFormatRecord(payload, sizeof(payload), record, seq);
    if (n == 0) return false;

    char topic[64];
    snprintf(topic, sizeof(topic), "%s/telemetry", _prefix);
    return publishRaw(topic, 1, payload, n);
}

bool Telemetry::sendTest(const char* name, std::initializer_list<TelemetryField> fields) {
    if (!_mqtt.connected()) return false;

    char safe[33];
    sanitizeName(name, safe, sizeof(safe));
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/test/%s", _prefix, safe);

    char payload[512];
    size_t n = telemetryFormatFields(payload, sizeof(payload), fields.begin(), fields.size(), millis());
    if (n == 0) return false;
    return publishRaw(topic, 0, payload, n);
}

void Telemetry::log(const char* fmt, ...) {
    char msg[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    char line[192];
    snprintf(line, sizeof(line), "[%lums] %s", static_cast<unsigned long>(millis()), msg);
    Serial.println(line);

    if (!_mqtt.connected()) return;
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/log", _prefix);
    publishRaw(topic, 0, line, strlen(line));
}
