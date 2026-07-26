#include "Sensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <DFRobot_BMM350.h>

static Adafruit_BMP280 _bmp;
static Adafruit_MPU6050 _mpu;
static DFRobot_BMM350_I2C _bmm(&Wire, 0x14);

static bool _bmp280Ready = false;
static bool _mpu6050Ready = false;
static bool _bmm350Ready = false;

// 打ち上げ前の地上気圧をbegin()でキャリブレーションし、以後これを基準に高度を計算する
// （固定の標準大気圧1013.25hPaを使うと、その日の実際の海面気圧とのズレがそのまま
//   絶対高度の誤差になるため。この方式では「打ち上げ地点からの相対高度」になる）
static float _groundLevelHpa = 1013.25f;

static float _pressure = 0.0f;
static float _temperature = 0.0f;
static float _altitude = 0.0f;

static float _accelX = 0.0f, _accelY = 0.0f, _accelZ = 0.0f;
static float _gyroX = 0.0f, _gyroY = 0.0f, _gyroZ = 0.0f;
static float _magX = 0.0f, _magY = 0.0f, _magZ = 0.0f;  // 地磁気生値 [uT]
static float _roll = 0.0f, _pitch = 0.0f;
static float _yaw = 0.0f;

// 相補フィルタの重み（ジャイロ積分側の比率）。TODO: 実機でチューニング
static const float GRAVITY_FILTER_ALPHA = 0.98f;

static float _gravity[3] = {0.0f, 0.0f, 0.0f};  // 最初のupdate()でaccelから初期化する
static bool _gravityInitialized = false;
static uint32_t _lastUpdateUs = 0;

// v に対して R = Rz(thetaZ) * Ry(thetaY) * Rx(thetaX) を作用させる。
// 行列を明示合成せず Rx→Ry→Rz の順に逐次適用しても (Rz*Ry*Rx)*v と同じ結果になる。
static void rotateByGyro(float v[3], float thetaXDeg, float thetaYDeg, float thetaZDeg) {
    float rx = thetaXDeg * DEG_TO_RAD;
    float ry = thetaYDeg * DEG_TO_RAD;
    float rz = thetaZDeg * DEG_TO_RAD;

    float cx = cosf(rx), sx = sinf(rx);
    float x1 = v[0];
    float y1 = v[1] * cx - v[2] * sx;
    float z1 = v[1] * sx + v[2] * cx;

    float cy = cosf(ry), sy = sinf(ry);
    float x2 =  x1 * cy + z1 * sy;
    float y2 = y1;
    float z2 = -x1 * sy + z1 * cy;

    float cz = cosf(rz), sz = sinf(rz);
    float x3 = x2 * cz - y2 * sz;
    float y3 = x2 * sz + y2 * cz;
    float z3 = z2;

    v[0] = x3; v[1] = y3; v[2] = z3;
}

bool Sensor::begin() {
    Wire.begin();
    Wire.setClock(400000);  // 100kHz(既定)だとI2C読み取りだけで100Hz周期の大半を食うため400kHzに上げる

    _bmp280Ready = _bmp.begin(0x76) || _bmp.begin(0x77);
    if (!_bmp280Ready) {
        Serial.println("[Sensor] BMP280 not found");
    } else {
        // 地上（打ち上げ前）の気圧を基準点としてキャリブレーションする。
        // 複数回サンプリングして平均を取りノイズを減らす。
        float sum = 0.0f;
        const int N = 10;
        for (int i = 0; i < N; i++) {
            sum += _bmp.readPressure() / 100.0f;  // Pa -> hPa
            delay(20);
        }
        _groundLevelHpa = sum / N;
        Serial.printf("[Sensor] Ground level pressure calibrated: %.2f hPa\n", _groundLevelHpa);
    }

    _mpu6050Ready = _mpu.begin();
    if (!_mpu6050Ready) {
        // Adafruit_MPU6050::begin()はWHO_AM_Iが0x68固定でないと弾くが、
        // i2c_devの生成自体はそのチェックより前に行われているため、
        // MPU6050互換チップ（MPU6500/MPU9250系。ロット差でWHO_AM_Iが
        // 0x68以外になることがある）であれば以降のレジスタ操作は問題なく動く。
        // 生I2CでWHO_AM_Iを読み、互換範囲なら手動初期化にフォールバックする。
        Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
        Wire.write(0x75);  // WHO_AM_I register
        if (Wire.endTransmission(false) == 0 &&
            Wire.requestFrom((uint8_t)MPU6050_I2CADDR_DEFAULT, (uint8_t)1) == 1) {
            uint8_t whoAmI = Wire.read();
            if (whoAmI >= 0x68 && whoAmI <= 0x73) {
                _mpu.reset();
                // reset()直後はPWR_MGMT_1がデフォルト値(SLEEPビット=1)に戻り測定が止まった
                // ままになるため、クロックをPLL(Gyro X基準)に設定してスリープを解除する。
                Wire.beginTransmission(MPU6050_I2CADDR_DEFAULT);
                Wire.write(MPU6050_PWR_MGMT_1);
                Wire.write(0x01);
                Wire.endTransmission();
                delay(100);
                _mpu6050Ready = true;
                Serial.printf("[Sensor] MPU6050-compatible chip detected (WHO_AM_I=0x%02X), using manual init\n", whoAmI);
            }
        }
    }

    if (!_mpu6050Ready) {
        Serial.println("[Sensor] MPU6050 not found");
    } else {
        _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    _bmm350Ready = (_bmm.begin() == 0);  // DFRobot_BMM350::begin()は0が成功
    if (!_bmm350Ready) {
        Serial.println("[Sensor] BMM350 not found");
    } else {
        _bmm.setOperationMode(eBmm350NormalMode);
        // PID出力を100Hz周期で更新できるよう地磁気のODRも100Hzに引き上げる
        // （HIGHACCURACYプリセットのまま平均化回数8のみ据え置き）
        _bmm.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY, BMM350_DATA_RATE_100HZ);
        _bmm.setMeasurementXYZ();
    }

    return _bmp280Ready && _mpu6050Ready && _bmm350Ready;
}

void Sensor::update() {
    if (_bmp280Ready) {
        _pressure    = _bmp.readPressure() / 100.0f;  // Pa -> hPa
        _temperature = _bmp.readTemperature();
        _altitude    = _bmp.readAltitude(_groundLevelHpa);
    }

    uint32_t nowUs = micros();
    // 初回はdt=0にしてジャイロ積分をスキップし、accelから重力ベクトルを初期化する
    float dt = _gravityInitialized ? (nowUs - _lastUpdateUs) / 1000000.0f : 0.0f;
    _lastUpdateUs = nowUs;

    if (_mpu6050Ready) {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);

        _accelX = a.acceleration.x;
        _accelY = a.acceleration.y;
        _accelZ = a.acceleration.z;
        _gyroX  = g.gyro.x * RAD_TO_DEG;
        _gyroY  = g.gyro.y * RAD_TO_DEG;
        _gyroZ  = g.gyro.z * RAD_TO_DEG;

        // 実機のマウント方向: X=鉛直上向き, Z=進行方向前向き, Y=横方向
        float accelMag = sqrtf(_accelX * _accelX + _accelY * _accelY + _accelZ * _accelZ);
        float accelNorm[3] = {0.0f, 0.0f, 0.0f};
        if (accelMag > 0.0f) {
            accelNorm[0] = _accelX / accelMag;
            accelNorm[1] = _accelY / accelMag;
            accelNorm[2] = _accelZ / accelMag;
        }

        if (!_gravityInitialized) {
            _gravity[0] = accelNorm[0];
            _gravity[1] = accelNorm[1];
            _gravity[2] = accelNorm[2];
            _gravityInitialized = true;
        } else {
            // 1. 一個前の重力ベクトルをジャイロの回転量分だけ回転させる
            float rotated[3] = {_gravity[0], _gravity[1], _gravity[2]};
            rotateByGyro(rotated, _gyroX * dt, _gyroY * dt, _gyroZ * dt);

            // 相補フィルタでジャイロ側のドリフトをaccelで補正する
            _gravity[0] = GRAVITY_FILTER_ALPHA * rotated[0] + (1.0f - GRAVITY_FILTER_ALPHA) * accelNorm[0];
            _gravity[1] = GRAVITY_FILTER_ALPHA * rotated[1] + (1.0f - GRAVITY_FILTER_ALPHA) * accelNorm[1];
            _gravity[2] = GRAVITY_FILTER_ALPHA * rotated[2] + (1.0f - GRAVITY_FILTER_ALPHA) * accelNorm[2];
        }

        // 2. 重力ベクトルからroll/pitchを求める
        _roll  = atan2f(_gravity[2], _gravity[0]) * RAD_TO_DEG;
        _pitch = atan2f(_gravity[1], sqrtf(_gravity[0] * _gravity[0] + _gravity[2] * _gravity[2])) * RAD_TO_DEG;
    }

    if (_bmm350Ready) {
        sBmm350MagData_t mag = _bmm.getGeomagneticData();
        _magX = mag.float_x;
        _magY = mag.float_y;
        _magZ = mag.float_z;

        // 3. roll/pitchで地磁気を水平面に補正する（y軸周りに-roll、続けてz軸周りに-pitch）
        float rollRad  = _roll  * DEG_TO_RAD;
        float pitchRad = _pitch * DEG_TO_RAD;

        float x1 = _magX * cosf(rollRad) - _magZ * sinf(rollRad);
        float y1 = _magY;
        float z1 = _magX * sinf(rollRad) + _magZ * cosf(rollRad);

        float correctedX =  x1 * cosf(pitchRad) + y1 * sinf(pitchRad);
        float correctedY = -x1 * sinf(pitchRad) + y1 * cosf(pitchRad);
        float correctedZ =  z1;

        // 4. 方位角を求める（atan2で-180〜180の全範囲をカバー。北=0）
        _yaw = atan2f(-correctedY, correctedZ) * RAD_TO_DEG;
    }
}

float Sensor::getPressure()    { return _pressure; }
float Sensor::getTemperature() { return _temperature; }

float Sensor::getAccelX() { return _accelX; }
float Sensor::getAccelY() { return _accelY; }
float Sensor::getAccelZ() { return _accelZ; }
float Sensor::getGyroX()  { return _gyroX; }
float Sensor::getGyroY()  { return _gyroY; }
float Sensor::getGyroZ()  { return _gyroZ; }

float Sensor::getAltitude()      { return _altitude; }
float Sensor::getGroundLevelHpa() { return _groundLevelHpa; }
float Sensor::getRoll()      { return _roll; }
float Sensor::getPitch()     { return _pitch; }
float Sensor::getYaw()       { return _yaw; }
float Sensor::getAccelMag()  { return sqrtf(_accelX * _accelX + _accelY * _accelY + _accelZ * _accelZ); }

bool Sensor::isBmp280Ready()  { return _bmp280Ready; }
bool Sensor::isMpu6050Ready() { return _mpu6050Ready; }
bool Sensor::isBmm350Ready()  { return _bmm350Ready; }
