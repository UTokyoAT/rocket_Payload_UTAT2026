#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>
#include "spi_protocol.h"  // MissionState enum

struct SensorData {
    float alt   = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    double lat  = 0.0;
    double lon  = 0.0;
    uint32_t timestamp_ms = 0;
    bool gpsValid = false;         // taskGPSが書き込む。fix取得前はtaskNavigationがPID出力を0に固定する
    int gpsSatellites = 0;         // taskGPSが書き込む捕捉中の衛星数。taskMissionのSETTING判定用
    uint32_t gpsFixSeq = 0;        // taskGPSが新規fixのたびインクリメント。taskMissionのNAVIGATE停止判定用
    float pidOutput      = 0.0f;  // taskNavigationが計算する誘導PIDの旋回量。taskSpiLinkがXIAO2へ転送する
    float destinationYaw = 0.0f;  // taskNavigationが計算する目的地への方位角 [deg]（磁北基準）
};

struct Shared {
    SemaphoreHandle_t mutex;
    SensorData latest;
    MissionState state = MissionState::SETTING;

    // 誘導PIDの目的地。SETTING完了時にtaskMissionがGPS取得座標（＝打ち上げ地点）で
    // 上書きする（return-to-launch方式）。この既定値はそれまでの間だけ使われる仮値。
    // 地上局がXIAO2へGET /goal?lat=..&lon=..を送ると、taskSpiLinkがSPI応答経由で
    // 受け取り、SETTING完了後であってもいつでも優先的に上書きする（taskNavigation.h参照）。
    double goalLat = 35.681236;
    double goalLon = 139.767125;
};
