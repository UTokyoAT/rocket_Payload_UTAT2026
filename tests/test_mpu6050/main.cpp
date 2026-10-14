#include <Arduino.h>
#include <Sensor.h>
#include <Telemetry.h>
#include "secrets.h"

// MPU6050（6軸IMU）の動作確認。PlatformIO で env:test-mpu6050 を選択して書き込む。
// 事前に include/secrets.example.h を include/secrets.h にコピーして値を設定し、
// ground/server で `docker compose up -d` しておく。
//
// センサーは100Hzで読み、10Hz（本番の publishRecord と同じ周期）で地上局へ送る。
//
// 確認方法（InfluxDBのData Explorer / Grafana）:
//   rocket_test の test_name=mpu6050 … 100msごとの以下のフィールド
//     ready              MPU6050を検出できたか（1/0）
//     ax, ay, az         加速度 [m/s^2]（静止時は |a| ≒ 9.8）
//     gx, gy, gz         角速度 [rad/s]（静止時は ≒ 0）
//     a_norm             加速度の大きさ [m/s^2]
//   rocket_log                         … 1秒ごとの値の要約・接続状態

static Sensor    sensor;
static Telemetry telemetry;

static const uint32_t SAMPLE_PERIOD_MS = 10;    // 100Hz（Sensor::update()の想定周期）
static const uint32_t SEND_PERIOD_MS   = 100;   // 10Hz
static const uint32_t LOG_PERIOD_MS    = 1000;

static uint32_t _lastSendMs = 0;
static uint32_t _lastLogMs  = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    sensor.begin();
    // begin()直後はまだ未接続なのでこのログはSerialにしか出ない
    telemetry.log("[TEST] MPU6050 check starting... mpu=%s",
                  sensor.isMpu6050Ready() ? "OK" : "NG");
}

void loop() {
    uint32_t start = millis();
    sensor.update();

    bool  ready = sensor.isMpu6050Ready();
    float ax = sensor.getAccelX(), ay = sensor.getAccelY(), az = sensor.getAccelZ();
    float gx = sensor.getGyroX(),  gy = sensor.getGyroY(),  gz = sensor.getGyroZ();

    float aNorm = sqrtf(ax * ax + ay * ay + az * az);
    
    if (start - _lastSendMs >= SEND_PERIOD_MS) {
        _lastSendMs = start;
        if (ready) {
            telemetry.sendTest("mpu6050", {{"ready", true},
                                           {"ax", ax}, {"ay", ay}, {"az", az},
                                           {"gx", gx}, {"gy", gy}, {"gz", gz},
                                           {"a_norm", aNorm}});
        } else {
            // 未検出時は0埋めの値を送ると正常値と区別しにくいので ready だけ送る
            telemetry.sendTest("mpu6050", {{"ready", false}});
        }
    }

    if (start - _lastLogMs >= LOG_PERIOD_MS) {
        _lastLogMs = start;
        if (ready) {
            telemetry.log("[mpu OK] a[m/s^2]=%.2f,%.2f,%.2f |a|=%.2f g[rad/s]=%.3f,%.3f,%.3f mqtt=%s",
                          ax, ay, az, aNorm, gx, gy, gz,
                          telemetry.isConnected() ? "yes" : "no");
        } else {
            telemetry.log("[mpu NG] MPU6050 not detected. Check wiring (I2C: 0x68/0x69). mqtt=%s",
                          telemetry.isConnected() ? "yes" : "no");
        }
    }

    uint32_t elapsed = millis() - start;
    if (elapsed < SAMPLE_PERIOD_MS) delay(SAMPLE_PERIOD_MS - elapsed);
}
