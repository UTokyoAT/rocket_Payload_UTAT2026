#pragma once
#include <Arduino.h>
#include <espMqttClient.h>
#include <atomic>
#include <initializer_list>
#include "TelemetryFormat.h"

// Wi-Fi(STA) + MQTT で地上局(ground/server の Mosquitto)へデータを送る。
//
//   本番       publishRecord()  QoS1  <prefix>/telemetry        統合レコード(10Hz想定)
//   切り分け   sendTest()       QoS0  <prefix>/test/<name>      任意フィールドだけ
//   ログ       log()            QoS0  <prefix>/log              テキスト1行 (Serialにも出力)
//
// begin() は接続を待たずに戻る。Wi-Fi/MQTTの再接続は内部タスクが面倒を見るので、
// 呼び出し側は送信APIを呼ぶだけでよい。送信APIはどのタスクからでも呼べる。
class Telemetry {
public:
    struct Config {
        const char* ssid        = nullptr;
        const char* password    = nullptr;  // オープンAPなら nullptr
        const char* mqttHost    = nullptr;  // Mosquittoを動かしているPCのIP
        uint16_t    mqttPort    = 1883;
        const char* clientId    = "xiao-esp32s3";
        const char* topicPrefix = "rocket";
    };

    Telemetry();

    // 文字列は内部にコピーする。ssid/mqttHost が無ければ false。二重呼び出しも false。
    bool begin(const Config& cfg);

    // MQTTブローカーに接続済みか
    bool isConnected() const;

    // 本番用の統合レコードをQoS1で送る。seq はここで採番する。
    // record.utc_ms が0(GNSS時刻未取得)のときは何も送らず false を返す。
    // 切断中でも受け付けてRAM上のoutboxに積み、再接続後に再送する(空きヒープが
    // 16KBを切ると新規は拒否され false)。電源断・再起動では失われる。
    bool publishRecord(const TelemetryRecord& record);

    // 切り分けテスト用。渡したフィールドだけを <prefix>/test/<name> に送る。
    // 例: telemetry.sendTest("bmp280", {{"pressure_hpa", p}, {"alt_m", a}});
    // NANのフィールドは省略される。未接続なら捨てて false。
    bool sendTest(const char* name, std::initializer_list<TelemetryField> fields);

    // printf形式で1行を <prefix>/log に送る。Serialにも出す。未接続ならSerialだけ。
    void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

private:
    static void manageTask(void* self);
    void        manageLoop();
    bool        publishRaw(const char* topic, uint8_t qos, const char* payload, size_t len);

    espMqttClient _mqtt;
    TaskHandle_t  _task = nullptr;
    bool          _started = false;
    std::atomic<uint32_t> _seq{0};

    // espMqttClient は host/clientId をコピーせずポインタで保持するため、実体をここに持つ
    char _ssid[33]     = {};
    char _password[65] = {};
    char _host[64]     = {};
    char _clientId[32] = {};
    char _prefix[24]   = {};
};
