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
// XIAO1側で計算済みの姿勢・位置と、地上局からのWiFi手動操作コマンドを送る。
// 誘導（目標方位との誤差計算）・PIDはXIAO2側で行う。
struct __attribute__((packed)) SpiFrameToXiao2 {
    uint32_t timestamp_ms;
    float alt;
    float roll;
    float pitch;
    float yaw;
    float lat;  // doubleは送らずfloatに縮小（lib/Radioの33バイトフレームと同じ方針）
    float lon;
    uint8_t mission_state;      // MissionStateのenum値
    int16_t manual_motor_left;  // 地上局からの手動操作値（-255〜255、未受信時はRadio側で自動的に0）
    int16_t manual_motor_right;
    uint8_t manual_override;    // 1=手動操作を優先、0=XIAO2の自律PIDを優先
                                 // TODO: 手動/自律の調停ロジックは未実装。現状は常に0を送る
};

// XIAO2(スレーブ) → XIAO1(マスター)
// 実際にモータへ出力した値（手動 or PID後の値）。XIAO1がWiFiでテレメトリ配信する。
struct __attribute__((packed)) SpiFrameFromXiao2 {
    int16_t motor_output_left;
    int16_t motor_output_right;
};

constexpr size_t SPI_FRAME_SIZE =
    (sizeof(SpiFrameToXiao2) > sizeof(SpiFrameFromXiao2))
        ? sizeof(SpiFrameToXiao2)
        : sizeof(SpiFrameFromXiao2);
