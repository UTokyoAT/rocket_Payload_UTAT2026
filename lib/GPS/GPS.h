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

    // 前回呼び出し以降に新しい位置情報（NMEAセンテンス）を受信していればtrue。
    // getLat()/getLon()（内部で.lat()/.lng()を呼ぶ）より前に呼ぶこと
    // （呼ぶと内部の更新フラグが消費され、以後falseを返すようになるため）。
    bool locationUpdated();

    float bearingTo(double lat, double lon);    // 現在地から指定座標への方位 [deg, 0-360, 北=0]
    float distanceTo(double lat, double lon);   // 現在地から指定座標への距離 [m]

    // Course Over Ground（対地進行方向）。地磁気センサーが使えない環境向けの
    // ヘディング推定（Heading参照）にGPS側の基準として使う。GPRMC等から得られるため
    // 静止時・低速時はノイズが大きく信頼できない（getSpeedMps()と組み合わせて判断すること）。
    float getCourse();       // [deg] 0-360、真北基準
    bool  isCourseValid();
    float getSpeedMps();     // [m/s] 対地速度

    // --- デバッグ用診断情報 ---
    uint32_t charsProcessed();      // 受信して処理した総バイト数（増えない→配線/電源を疑う）
    uint32_t failedChecksumCount(); // チェックサム失敗数（増え続ける→ボーレート不一致/ノイズを疑う）
    int      satellites();          // 捕捉中の衛星数（Fix前でも取得可、0のまま→受信環境を疑う）
    float    hdop();                // 精度指標（値が出ていればFixが近い）
};
