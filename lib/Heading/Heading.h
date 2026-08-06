#pragma once

// 地磁気センサーを使わない方位（ヘディング）推定。
// 実機ではBMM350の近くにモーター・バッテリー等の強い磁気源があり、地磁気の生値(magX/Y)が
// ほとんど動かない＝地磁気センサーとして使い物にならないことが判明した。その代替として、
// ジャイロのヨー角速度を積分した相対方位を、GPSのCourse Over Ground（対地進行方向）で
// 定期的に補正する相補フィルタで方位を推定する。
//
// GPSのcourseは真北基準のため、従来の磁北基準yaw（BMM350）と違い磁気偏角の補正が不要になる。
//
// センサー非依存のPIDと違い、ヨー軸ジャイロ・GPS courseというこのプロジェクト固有の
// 入力を前提にしているため、汎用クラスではなくプロジェクト固有の方位推定として置く。
class Heading {
public:
    // gyroBiasDegPerSec: 静止状態で計測したジャイロのヨー軸角速度バイアス（そのまま差し引く）。
    // 呼び出し側でSensor::getGyroX()を静止状態で数十〜百サンプル平均して求める
    // （Sensor::begin()のBMP280地上気圧較正と同じやり方）。
    void begin(float gyroBiasDegPerSec);

    // dt周期（想定100Hz）で毎回呼ぶ。
    // GPSに新しいfixが来た周期だけgpsFixUpdated=trueにしてcourse/speedを渡すこと
    // （GPSは1Hz程度の更新だが、GPS courseの補正自体はCOURSE_CORRECTION_INTERVAL_S
    //   （約10秒）に1回だけ内部でレート制限して適用する。毎fixごとに補正すると
    //   短時間のcourseノイズをそのまま拾ってしまうため）。
    //
    // 注意: gyroYawRateDegPerSecの符号（時計回り=正になっているか）は実機で要検証。
    // GPSが取得できる環境で機体を上から見て時計回りにゆっくり回し、get()の値が
    // 増加することを確認すること。逆に振れる場合は呼び出し側で符号を反転して渡す。
    void update(float gyroYawRateDegPerSec, float dt,
                bool gpsFixUpdated, bool gpsCourseValid, float gpsCourseDeg, float gpsSpeedMps);

    float get() const;  // [deg] -180〜180、真北基準

private:
    float _headingDeg = 0.0f;
    float _gyroBiasDegPerSec = 0.0f;
    bool  _initialized = false;  // 最初の有効なGPS courseでtrueにし、headingを真北基準へスナップする
    float _timeSinceCorrectionS = 0.0f;  // 前回のGPS course補正からの経過時間
};
