#pragma once
#include <Arduino.h>

class GPS {
public:
    void begin(int rxPin, int txPin, uint32_t baud = 9600);
    void update();          // ループで頻繁に呼ぶ（NMEAをバイト単位で食わせる）

    double getLat();        // [deg]
    double getLon();        // [deg]
    float  getAltitude();   // [m] GPS高度（気圧高度と別）
    bool   isValid();

    float bearingTo(double lat, double lon);    // 現在地から指定座標への方位 [deg, 0-360, 北=0]
    float distanceTo(double lat, double lon);   // 現在地から指定座標への距離 [m]

    // --- デバッグ用診断情報 ---
    uint32_t charsProcessed();      // 受信して処理した総バイト数（増えない→配線/電源を疑う）
    uint32_t failedChecksumCount(); // チェックサム失敗数（増え続ける→ボーレート不一致/ノイズを疑う）
    int      satellites();          // 捕捉中の衛星数（Fix前でも取得可、0のまま→受信環境を疑う）
    float    hdop();                // 精度指標（値が出ていればFixが近い）
};
