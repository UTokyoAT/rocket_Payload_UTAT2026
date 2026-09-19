#pragma once

// カルマンフィルタの状態変数クラスをまとめる。
class KarmanFliter {
public:
    // 出発地を原点とする現在地の北と東への移動距離（m)
    double position_N;
    double position_E;

    // 速さ（m/s）
    double speed;

    // 方位角、北を0度として時計回りが正（rad）
    float azimuth;

    // 加速度計のバイアス（m/s^2）
    float IMUbias;

    // ジャイロセンサのバイアス（rad/s）
    float gyrobias;

    //
};