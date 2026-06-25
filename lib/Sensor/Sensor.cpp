#include "Sensor.h"

// TODO: BMP388, ICM-42688, QMC5883 のドライバをインクルード

bool Sensor::begin() {
    // TODO: I2C初期化・各センサー初期化
    return true;
}

void Sensor::update() {
    // TODO: IMU読み取り → 姿勢フィルタ（相補フィルタ or カルマン）
    //       気圧読み取り → 高度計算
    //       地磁気読み取り → ヨー補正
}

float Sensor::getAltitude()  { return 0.0f; /* TODO */ }
float Sensor::getRoll()      { return 0.0f; /* TODO */ }
float Sensor::getPitch()     { return 0.0f; /* TODO */ }
float Sensor::getYaw()       { return 0.0f; /* TODO */ }
float Sensor::getAccelMag()  { return 9.8f; /* TODO */ }
