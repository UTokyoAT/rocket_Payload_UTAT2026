#pragma once
#include <Arduino.h>
#include "spi_protocol.h"

// PULL方式（PC側がHTTP GETで取りに来る）でテレメトリを配信する。
// XIAO ESP32S3のSoftAP + 標準WebServerを使用（WebSocketは使わない）。
// 理由: 機体→PCのPUSH（WebSocket/POST）はPC側ファイアウォールに
//       着信をブロックされる環境があり信頼できないため。
//
// XIAO2がXIAO1からSPIで受信したSpiFrameToXiao2をそのままワイヤーフレームとして
// 中継する（フィールドレイアウトはspi_protocol.hのコメントを参照）。
// Python側 (ground/receiver.py) / dashboard.h と共有する契約なので、
// spi_protocol.hのSpiFrameToXiao2を変更する場合は両方直すこと。
class Radio {
public:
    static constexpr size_t FRAME_SIZE = sizeof(SpiFrameToXiao2);

    // 地上局からのモーター手動制御コマンド（GET /motor?left=N&right=M、各-255〜255）が
    // この期間 [ms] 以上更新されないと、getMotorCommandLeft/Right() は自動的に0を返す
    // （WiFi切断・地上局側クラッシュ時にモーターが暴走し続けないためのフェイルセイフ）。
    static constexpr uint32_t MOTOR_COMMAND_TIMEOUT_MS = 1000;

    // SoftAP起動 + HTTPサーバー起動（GET /data, GET /motor, GET /goal, GET / を登録）
    void begin(const char* ssid, const char* password);

    // 最新のSPIフレームを内部バイナリフレームへ反映する。
    // loop()などから定期的（100Hz目安）に呼ぶ。
    void setData(const SpiFrameToXiao2& frame);

    // 地上局から最後に受信したモーター手動制御コマンド（-255〜255）を返す。
    // MOTOR_COMMAND_TIMEOUT_MS以上新しいコマンドが来ていなければ0（フェイルセイフ）。
    int16_t getMotorCommandLeft();
    int16_t getMotorCommandRight();

    // MOTOR_COMMAND_TIMEOUT_MS以内に地上局から手動操作コマンドを受信していればtrue。
    // 呼び出し側はこれで手動/自律を切り替える（値が0か非0かでは判定できないため）。
    bool hasRecentMotorCommand();

    // 地上局から最後に受信した目的地座標（GET /goal?lat=..&lon=..）。
    // モーターコマンドと異なりタイムアウトで無効化されない
    // （通信が途切れたからといって目的地を見失わせるのは危険なため、明示的に
    //   上書きされるまで保持し続ける）。実際にXIAO1へ届けるのは
    //   taskSpiLinkがSpiFrameFromXiao2として送り返す応答フレーム経由。
    float getGoalLat();
    float getGoalLon();

    // 地上局が一度でもGET /goalで目的地を設定していればtrue。
    // falseの間、XIAO1側は起動時の既定値（Shared::goalLat/Lon）を使い続ける。
    bool hasGoal();

    // HTTPリクエストを処理する。loop()内で毎ティック呼ぶ必須
    // （呼ばないとGETに応答できない）。
    void poll();

    IPAddress getIP();
};
