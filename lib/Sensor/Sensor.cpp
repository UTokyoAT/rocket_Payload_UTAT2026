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

static const float kSeaLevelHpa = 1013.25f;

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
    }

    _mpu6050Ready = _mpu.begin();
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
        _altitude    = _bmp.readAltitude(kSeaLevelHpa);
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
        _roll  = atan2f(_accelY, _accelZ) * RAD_TO_DEG;
        _pitch = atan2f(-_accelX, sqrtf(_accelY * _accelY + _accelZ * _accelZ)) * RAD_TO_DEG;
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

float Sensor::getAltitude()  { return _altitude; }
float Sensor::getRoll()      { return _roll; }
float Sensor::getPitch()     { return _pitch; }
float Sensor::getYaw()       { return _yaw; }
float Sensor::getAccelMag()  { return sqrtf(_accelX * _accelX + _accelY * _accelY + _accelZ * _accelZ); }

bool Sensor::isBmp280Ready()  { return _bmp280Ready; }
bool Sensor::isMpu6050Ready() { return _mpu6050Ready; }
bool Sensor::isBmm350Ready()  { return _bmm350Ready; }
