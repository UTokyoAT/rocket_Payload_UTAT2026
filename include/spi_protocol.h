#pragma once
#include <Arduino.h>

// XIAO1（マスター）⇔ XIAO2（スレーブ）間のSPI通信契約。
// 変更する場合はSpiLinkMaster/SpiLinkSlave両方への影響を確認すること。

// TODO: 実際のピン番号に変更する（回路図のSPI_SCK/SPI_MISO/SPI_MOSI/SPI_CSに合わせる）
namespace SpiPins {
    constexpr int SCK  = 7;
    constexpr int MISO = 8;
    constexpr int MOSI = 9;
    constexpr int CS   = 10;
}

// XIAO1(マスター) → XIAO2(スレーブ)
// XIAO1側で計算済みの姿勢・位置・誘導PID出力を送る（誘導・PIDはXIAO1側で行う）。
// このバイナリレイアウトはXIAO2がWiFiテレメトリとしてそのまま地上局へ中継する
// ワイヤーフレームと共通（lib/Radio, ground/receiver.py, lib/Radio/dashboard.hも参照）。
// packed・パディング無しの37バイト、リトルエンディアン。
//
// offset  size  type     field            note
//   0      4    uint32   timestamp_ms
//   4      4    float32  alt              [m]
//   8      4    float32  roll             [deg]
//  12      4    float32  pitch            [deg]
//  16      4    float32  yaw              [deg]
//  20      4    float32  lat              doubleは送らずfloatに縮小
//  24      4    float32  lon
//  28      1    uint8    mission_state    MissionStateのenum値
//  29      4    float32  pid_output       誘導PIDの旋回量。XIAO2側でbase±turnとしてモータに反映
//  33      4    float32  destination_yaw  目的地への方位角 [deg]（磁北基準、表示用）
struct __attribute__((packed)) SpiFrameToXiao2 {
    uint32_t timestamp_ms;
    float alt;
    float roll;
    float pitch;
    float yaw;
    float lat;
    float lon;
    uint8_t mission_state;
    float pid_output;
    float destination_yaw;
};

// XIAO2(スレーブ) → XIAO1(マスター)
// XIAO1はモータ出力を消費しない（WiFiテレメトリはXIAO2側が担当するため）ので現状未使用。
// SPIは全二重のため転送自体は必要で、ダミーの応答バイトを返す。
struct __attribute__((packed)) SpiFrameFromXiao2 {
    uint8_t unused;
};

constexpr size_t SPI_FRAME_SIZE =
    (sizeof(SpiFrameToXiao2) > sizeof(SpiFrameFromXiao2))
        ? sizeof(SpiFrameToXiao2)
        : sizeof(SpiFrameFromXiao2);
