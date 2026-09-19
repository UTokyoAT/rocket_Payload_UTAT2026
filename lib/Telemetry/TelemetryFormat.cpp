#include "TelemetryFormat.h"
#include <stdarg.h>
#include <stdio.h>

namespace {

constexpr int FMT_FLOAT  = -1;  // %.7g
constexpr int FMT_DOUBLE = -2;  // %.15g

struct Writer {
    char*  buf;
    size_t cap;
    size_t len = 0;
    bool   ok  = true;

    Writer(char* b, size_t c) : buf(b), cap(c) {
        if (cap == 0) ok = false;
        else buf[0] = '\0';
    }

    void ch(char c) {
        if (!ok || len + 1 >= cap) { ok = false; return; }
        buf[len++] = c;
        buf[len]   = '\0';
    }

    void str(const char* s) { while (*s) ch(*s++); }

    __attribute__((format(printf, 2, 3))) void fmt(const char* f, ...) {
        if (!ok) return;
        va_list args;
        va_start(args, f);
        int n = vsnprintf(buf + len, cap - len, f, args);
        va_end(args);
        if (n < 0 || static_cast<size_t>(n) >= cap - len) { ok = false; return; }
        len += static_cast<size_t>(n);
    }
};

struct Num {
    const char* key;
    double      v;
    int         decimals;  // >=0: 固定小数桁数 / FMT_FLOAT / FMT_DOUBLE
};

void writeKey(Writer& w, const char* k) {
    w.ch('"');
    for (; *k; ++k) {
        unsigned char c = static_cast<unsigned char>(*k);
        if (c == '"' || c == '\\') { w.ch('\\'); w.ch(static_cast<char>(c)); }
        else if (c < 0x20)         { w.ch('_'); }
        else                       { w.ch(static_cast<char>(c)); }
    }
    w.str("\":");
}

void writeNumber(Writer& w, double v, int decimals) {
    if (decimals >= 0)              w.fmt("%.*f", decimals, v);
    else if (decimals == FMT_FLOAT) w.fmt("%.7g", v);
    else                            w.fmt("%.15g", v);
}

// 有限値が1つもなければグループごと省略する。先頭に常に ',' を付ける（seq/ts_utc の後ろに続けるため）。
void writeGroup(Writer& w, const char* name, const Num* nums, size_t n) {
    bool any = false;
    for (size_t i = 0; i < n; i++) {
        if (isfinite(nums[i].v)) { any = true; break; }
    }
    if (!any) return;

    w.ch(',');
    writeKey(w, name);
    w.ch('{');
    bool first = true;
    for (size_t i = 0; i < n; i++) {
        if (!isfinite(nums[i].v)) continue;
        if (!first) w.ch(',');
        first = false;
        writeKey(w, nums[i].key);
        writeNumber(w, nums[i].v, nums[i].decimals);
    }
    w.ch('}');
}

}  // namespace

void telemetryFormatUtc(char* buf, size_t cap, uint64_t utc_ms) {
    uint64_t secs = utc_ms / 1000;
    unsigned ms   = static_cast<unsigned>(utc_ms % 1000);
    uint64_t days = secs / 86400;
    unsigned rem  = static_cast<unsigned>(secs % 86400);

    // days since 1970-01-01 -> civil date (Howard Hinnant's algorithm)
    int64_t  z   = static_cast<int64_t>(days) + 719468;
    int64_t  era = z / 146097;
    unsigned doe = static_cast<unsigned>(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t  y   = static_cast<int64_t>(yoe) + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp  = (5 * doy + 2) / 153;
    unsigned d   = doy - (153 * mp + 2) / 5 + 1;
    unsigned m   = mp < 10 ? mp + 3 : mp - 9;
    if (m <= 2) y++;

    snprintf(buf, cap, "%04d-%02u-%02uT%02u:%02u:%02u.%03uZ",
             static_cast<int>(y), m, d, rem / 3600, (rem % 3600) / 60, rem % 60, ms);
}

size_t telemetryFormatRecord(char* buf, size_t cap, const TelemetryRecord& r, uint32_t seq) {
    if (r.utc_ms == 0) return 0;

    char ts[32];
    telemetryFormatUtc(ts, sizeof(ts), r.utc_ms);

    Writer w(buf, cap);
    w.ch('{');
    w.fmt("\"seq\":%lu,\"ts_utc\":\"%s\"", static_cast<unsigned long>(seq), ts);

    const Num pos[] = {
        {"lat", r.pos.lat, 7}, {"lon", r.pos.lon, 7}, {"alt", r.pos.alt, 2},
    };
    writeGroup(w, "pos", pos, 3);

    const Num imu[] = {
        {"ax", r.imu.ax, 4}, {"ay", r.imu.ay, 4}, {"az", r.imu.az, 4},
        {"gx", r.imu.gx, 4}, {"gy", r.imu.gy, 4}, {"gz", r.imu.gz, 4},
    };
    writeGroup(w, "imu", imu, 6);

    const Num baro[] = {
        {"pressure_hpa", r.baro.pressure_hpa, 2}, {"alt_m", r.baro.alt_m, 2},
    };
    writeGroup(w, "baro", baro, 2);

    const Num encoder[] = {
        {"right_rev", r.encoder.right_rev, 5}, {"left_rev", r.encoder.left_rev, 5},
    };
    writeGroup(w, "encoder", encoder, 2);

    const Num gnss[] = {
        {"fix",  r.gnss.fix  < 0 ? static_cast<double>(NAN) : r.gnss.fix,  0},
        {"hdop", r.gnss.hdop, 2},
        {"sats", r.gnss.sats < 0 ? static_cast<double>(NAN) : r.gnss.sats, 0},
    };
    writeGroup(w, "gnss", gnss, 3);

    const Num ekf[] = {
        {"position_N", r.ekf.position_N, 3}, {"position_E", r.ekf.position_E, 3},
        {"speed", r.ekf.speed, 3},           {"azimuth", r.ekf.azimuth, 2},
        {"bias_accel", r.ekf.bias_accel, 6}, {"bias_gyro", r.ekf.bias_gyro, 6},
    };
    writeGroup(w, "EKF", ekf, 6);

    w.ch('}');
    return w.ok ? w.len : 0;
}

size_t telemetryFormatFields(char* buf, size_t cap, const TelemetryField* fields, size_t count,
                             uint32_t uptime_ms) {
    Writer w(buf, cap);
    w.ch('{');
    w.fmt("\"uptime_ms\":%lu", static_cast<unsigned long>(uptime_ms));
    for (size_t i = 0; i < count; i++) {
        if (!isfinite(fields[i].value)) continue;
        w.ch(',');
        writeKey(w, fields[i].key);
        int decimals = fields[i].isInt ? 0 : (fields[i].isFloat ? FMT_FLOAT : FMT_DOUBLE);
        writeNumber(w, fields[i].value, decimals);
    }
    w.ch('}');
    return w.ok ? w.len : 0;
}
