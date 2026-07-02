#include <Arduino.h>
#include <Radio.h>
#include "shared.h"

// ダミーデータを /data で20Hz公開するテスト（PULL方式、PC側がGETしにくる）
// PlatformIO で env:test-wifi を選択して書き込む

static Radio radio;

// 高度：0〜100m をサイン波で往復
// 姿勢：ゆるやかに揺れる
// GPS ：定点から小さく円を描く
// state：10秒ごとに自動遷移
// motorOutputLeft/Right：地上局から /motor で受信した左右手動制御コマンドをそのまま折り返す
//              （GUIのスライダーを動かすとテレメトリに反映される様子を確認できる）

void setup() {
    Serial.begin(115200);
    Serial.println("[TEST] WiFi Dummy Sender starting...");
    radio.begin("CanSat-AP", "cansat2026");
    Serial.println("[TEST] Ready. Connect to CanSat-AP and open http://192.168.4.1");
}

void loop() {
    const float t = millis() / 1000.0f;

    SensorData d;
    d.timestamp_ms = millis();
    d.alt   = 50.0f + 45.0f * sinf(t * 0.4f);          // 5〜95m
    d.roll  = 25.0f * sinf(t * 1.1f);
    d.pitch = 12.0f * cosf(t * 0.9f);
    d.yaw   = fmodf(t * 36.0f, 360.0f);                 // 10秒で一周
    d.lat   = 35.681236 + 0.0002 * sinf(t * 0.2f);
    d.lon   = 139.767125 + 0.0002 * cosf(t * 0.2f);
    d.motorOutputLeft  = radio.getMotorCommandLeft();
    d.motorOutputRight = radio.getMotorCommandRight();

    // 10秒ごとに次のステートへ自動遷移（ループ）
    const int stateCount = 6;
    MissionState state = static_cast<MissionState>(
        (static_cast<int>(t) / 10) % stateCount
    );

    radio.setData(d, state);
    radio.poll();

    delay(50);  // 20Hz
}
