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
};
