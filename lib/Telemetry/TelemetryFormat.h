#pragma once
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <type_traits>

// Arduino非依存のJSON生成部（PC上でも単体検証できるよう Telemetry 本体から分離している）。
// NAN を入れたフィールドは「未取得」として JSON から省略される。

struct TelemetryPos     { double lat = NAN; double lon = NAN; float alt = NAN; };
struct TelemetryImu     { float ax = NAN, ay = NAN, az = NAN, gx = NAN, gy = NAN, gz = NAN; };
struct TelemetryBaro    { float pressure_hpa = NAN; float alt_m = NAN; };
struct TelemetryEncoder { double right_rev = NAN; double left_rev = NAN; };
struct TelemetryGnss    { int fix = -1; float hdop = NAN; int sats = -1; };  // fix/sats は負値で未取得
struct TelemetryEkf     {
    float position_N = NAN, position_E = NAN, speed = NAN, azimuth = NAN;
    float bias_accel = NAN, bias_gyro = NAN;
};

// 本番用の統合レコード（要件書4.2）。seq と uptime_ms は Telemetry 側で付ける。
struct TelemetryRecord {
    TelemetryPos     pos;
    TelemetryImu     imu;
    TelemetryBaro    baro;
    TelemetryEncoder encoder;
    TelemetryGnss    gnss;
    TelemetryEkf     ekf;
};

// 切り分けテスト用の任意フィールド。数値なら何でも渡せる（{"pressure", p} のように書く）。
struct TelemetryField {
    const char* key;
    double      value;
    bool        isInt;    // 整数型（bool含む）は小数点なしで出力
    bool        isFloat;  // float は有効数字7桁、double は15桁で出力

    template <typename T,
              typename = typename std::enable_if<std::is_arithmetic<T>::value>::type>
    TelemetryField(const char* k, T v)
        : key(k),
          value(static_cast<double>(v)),
          isInt(std::is_integral<T>::value),
          isFloat(std::is_floating_point<T>::value && sizeof(T) == sizeof(float))
    {}
};

// 統合レコードをJSON化する。{"seq":N,"uptime_ms":M,"pos":{...},...}
// 戻り値は書き込んだ長さ（終端を除く）。バッファ不足のときは0を返す。
size_t telemetryFormatRecord(char* buf, size_t cap, const TelemetryRecord& r, uint32_t seq,
                             uint32_t uptime_ms);

// 切り分けテスト用JSON。{"uptime_ms":N,"key":value,...}。戻り値の意味は上と同じ。
size_t telemetryFormatFields(char* buf, size_t cap, const TelemetryField* fields, size_t count,
                             uint32_t uptime_ms);
