#include "Heading.h"
#include <Arduino.h>

// GPSのcourseをこの対地速度以上のときだけ信頼する。低速・静止に近い状態ではcourse自体の
// ノイズが大きく、進行方向という概念自体が定義できないため。TODO: 実機でチューニング
static const float MIN_SPEED_FOR_COURSE_MPS = 0.5f;

// GPS courseによる補正の最小間隔。GPS fix自体は1Hz程度で来るが、fixごとに補正すると
// 短時間のcourseノイズをそのまま拾ってしまうため、この間隔でレート制限する。
// TODO: 実機でチューニング
static const float COURSE_CORRECTION_INTERVAL_S = 10.0f;

// GPS courseによる補正1回でheadingをcourseへ寄せる割合（相補フィルタの重み）。
// 補正間隔が長い（10秒に1回）分、1回あたりの重みは1Hzごとに小刻みに補正する場合より
// 大きめに設定している。1.0にするとGPS courseをそのまま採用、0だと補正なし。
// TODO: 実機でチューニング
static const float COURSE_CORRECTION_ALPHA = 0.5f;

static float normalizeAngle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void Heading::begin(float gyroBiasDegPerSec) {
    _gyroBiasDegPerSec = gyroBiasDegPerSec;
    _headingDeg = 0.0f;
    _initialized = false;
    _timeSinceCorrectionS = 0.0f;
}

void Heading::update(float gyroYawRateDegPerSec, float dt,
                      bool gpsFixUpdated, bool gpsCourseValid, float gpsCourseDeg, float gpsSpeedMps) {
    _headingDeg = normalizeAngle(_headingDeg + (gyroYawRateDegPerSec - _gyroBiasDegPerSec) * dt);
    _timeSinceCorrectionS += dt;

    bool courseUsable = gpsFixUpdated && gpsCourseValid && gpsSpeedMps >= MIN_SPEED_FOR_COURSE_MPS;
    bool dueForCorrection = !_initialized || _timeSinceCorrectionS >= COURSE_CORRECTION_INTERVAL_S;

    if (courseUsable && dueForCorrection) {
        if (!_initialized) {
            // 初回は積分の起点が不定（0スタート）なので、素直にGPS courseへスナップする
            _headingDeg = normalizeAngle(gpsCourseDeg);
            _initialized = true;
        } else {
            float courseError = normalizeAngle(gpsCourseDeg - _headingDeg);
            _headingDeg = normalizeAngle(_headingDeg + COURSE_CORRECTION_ALPHA * courseError);
        }
        _timeSinceCorrectionS = 0.0f;
    }
}

float Heading::get() const { return _headingDeg; }
