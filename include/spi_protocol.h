#pragma once
#include <Arduino.h>

// XIAO1（マスター）⇔ XIAO2（スレーブ）間のSPI通信契約。
// 変更する場合はSpiLinkMaster/SpiLinkSlave両方への影響を確認すること。

// ミッションのシーケンス状態。XIAO1のtask_mission.hが遷移させ、SpiFrameToXiao2::mission_state
// としてXIAO2へ送る。XIAO2はNAVIGATE状態のときだけ自律PID出力をモータへ反映する（安全ゲート）ため、
// shared.h（XIAO1専用）ではなくXIAO1・XIAO2共通のこのヘッダで定義する。
enum class MissionState : uint8_t {
    SETTING,   // 起動直後。気圧ベースライン校正・GPS衛星捕捉待ち・目的地（打ち上げ地点）取得
    LAUNCH,    // 打ち上げ検知待ち（高度3m超を5tick連続検知）
    DETACH,    // ロケットから分離（ニクロム線）・パラシュート降下中の着地検知待ち
    UNFOLD,    // パラシュート分離（ニクロム線）・姿勢安定確認
    NAVIGATE,  // GNSS誘導で打ち上げ地点へ自律走行
    GOAL,      // 到達（停止）検知。回収支援のLED点滅
    ABORTED    // いずれかのタイムアウト・異常により中断。モータ停止・LED点滅
};

// Seeed XIAO ESP32S3のD番号 -> GPIO番号。SCK/MISO/MOSIはXIAO1・XIAO2共通配線（D8/D9/D10）。
// CSはXIAO1(マスター)側の出力ピンとXIAO2(スレーブ)側の入力ピンが異なるGPIO番号のため分けている。
namespace SpiPins {
    constexpr int SCK  = 7;   // D8
    constexpr int MISO = 8;   // D9
    constexpr int MOSI = 9;   // D10
    constexpr int CS_MASTER = 1;   // D0（XIAO1側）
    constexpr int CS_SLAVE  = 44;  // D7（XIAO2側）
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
// 地上局がXIAO2へGET /goal?lat=..&lon=..で設定した目的地座標を、XIAO1の誘導PIDへ
// 送り返す（XIAO1はWiFiを持たないため、目的地変更もこのSPI応答経由で受け取る）。
// packed・パディング無しの9バイト、リトルエンディアン。
//
// offset  size  type     field       note
//   0      1    uint8    goal_valid  地上局が一度でもGET /goalを送っていれば1
//   1      4    float32  goal_lat    goal_valid=0の間は不定値。XIAO1側は無視して既定値を使う
//   5      4    float32  goal_lon
struct __attribute__((packed)) SpiFrameFromXiao2 {
    uint8_t goal_valid;
    float goal_lat;
    float goal_lon;
};

constexpr size_t SPI_FRAME_SIZE =
    (sizeof(SpiFrameToXiao2) > sizeof(SpiFrameFromXiao2))
        ? sizeof(SpiFrameToXiao2)
        : sizeof(SpiFrameFromXiao2);
