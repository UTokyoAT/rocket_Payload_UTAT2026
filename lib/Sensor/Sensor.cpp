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
static float _roll = 0.0f, _pitch = 0.0f;
static float _yaw = 0.0f;

bool Sensor::begin() {
    Wire.begin();

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
        _bmm.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY, BMM350_DATA_RATE_25HZ);
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

    if (_mpu6050Ready) {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);

        _accelX = a.acceleration.x;
        _accelY = a.acceleration.y;
        _accelZ = a.acceleration.z;
        _gyroX  = g.gyro.x * RAD_TO_DEG;
        _gyroY  = g.gyro.y * RAD_TO_DEG;
        _gyroZ  = g.gyro.z * RAD_TO_DEG;

        // 加速度のみによる簡易姿勢角（相補フィルタはTODO）
        // 実機のマウント方向: X=鉛直上向き, Z=進行方向前向き, Y=横方向
        _roll  = atan2f(_accelY, _accelX) * RAD_TO_DEG;
        _pitch = atan2f(-_accelZ, sqrtf(_accelY * _accelY + _accelX * _accelX)) * RAD_TO_DEG;
    }

    if (_bmm350Ready) {
        _yaw = _bmm.getCompassDegree();  // 0-360、北=0。TODO: roll/pitchによるチルト補正
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
