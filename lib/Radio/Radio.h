#pragma once
#include <Arduino.h>
#include "shared.h"

// PULL方式（PC側がHTTP GETで取りに来る）でテレメトリを配信する。
// XIAO ESP32S3のSoftAP + 標準WebServerを使用（WebSocketは使わない）。
// 理由: 機体→PCのPUSH（WebSocket/POST）はPC側ファイアウォールに
//       着信をブロックされる環境があり信頼できないため。
class Radio {
public:
    // ワイヤーフレームのバイトサイズ・レイアウト（リトルエンディアン）。
    // Python側 (ground/receiver.py) と共有する契約。変更する場合は両方直すこと。
    //
    // offset  size  type     field           note
    //   0      4    uint32   timestamp_ms
    //   4      4    float32  alt             [m]
    //   8      4    float32  roll            [deg]
    //  12      4    float32  pitch           [deg]
    //  16      4    float32  yaw             [deg]
    //  20      4    float32  lat             double→float32に縮小
    //  24      4    float32  lon             double→float32に縮小
    //  28      1    uint8    mission_state   MissionStateのenum値
    //  29      2    int16    motor_output_left   Actuator::setMotorLeft()相当値（-255〜255）
    //  31      2    int16    motor_output_right  Actuator::setMotorRight()相当値（-255〜255）
    static constexpr size_t FRAME_SIZE = 33;

    // 地上局からのモーター手動制御コマンド（GET /motor?left=N&right=M、各-255〜255）が
    // この期間 [ms] 以上更新されないと、getMotorCommandLeft/Right() は自動的に0を返す
    // （WiFi切断・地上局側クラッシュ時にモーターが暴走し続けないためのフェイルセイフ）。
    static constexpr uint32_t MOTOR_COMMAND_TIMEOUT_MS = 1000;

    // SoftAP起動 + HTTPサーバー起動（GET /data, GET /motor, GET / を登録）
    void begin(const char* ssid, const char* password);

    // 最新のセンサー値・ステートを内部バイナリフレームへ反映する。
    // taskWifi などから定期的（20Hz目安）に呼ぶ。
    void setData(const SensorData& data, MissionState state);

    // 地上局から最後に受信したモーター手動制御コマンド（-255〜255）を返す。
    // MOTOR_COMMAND_TIMEOUT_MS以上新しいコマンドが来ていなければ0（フェイルセイフ）。
    int16_t getMotorCommandLeft();
    int16_t getMotorCommandRight();

    // HTTPリクエストを処理する。loop()/タスク内で毎ティック呼ぶ必須
    // （呼ばないとGETに応答できない）。
    void poll();

    IPAddress getIP();
};
