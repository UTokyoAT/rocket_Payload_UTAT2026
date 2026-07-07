#include <Arduino.h>
#include <Sensor.h>
#include <GPS.h>
#include <Actuator.h>
#include <PID.h>

// GPS方位（目標地点への方位）とBMM350 heading（現在の進行方向）の誤差をPIDで
// 補正し、左右モーター出力に反映する統合動作確認。
// PlatformIO で env:test-navigation を選択して書き込む。
// まずは実走行させず、車輪を浮かせた状態でSerial出力だけ確認すること
// （PIDゲイン・BASE_SPEEDは実機でのチューニングが必要）。

// TODO: 実際のゴール座標に置き換える
static const double GOAL_LAT = 35.681236;
static const double GOAL_LON = 139.767125;

// XIAO ESP32S3: D7(GPIO44)にGPSモジュールのTXを、D6(GPIO43)にGPSモジュールのRXを接続
static const int GPS_RX_PIN = 44;  // D7
static const int GPS_TX_PIN = 43;  // D6

static const int BASE_SPEED = 150;  // 直進基準速度（-255〜255）

static Sensor sensor;
static GPS gps;
static Actuator actuator;
static PID headingPid(2.0f, 0.0f, 0.5f, -255.0f, 255.0f);  // TODO: 実機でゲイン調整

static uint32_t _lastUpdateMs = 0;

static float normalizeAngle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] Navigation (GPS bearing + BMM350 heading -> PID -> motor) check starting...");
    sensor.begin();
    gps.begin(GPS_RX_PIN, GPS_TX_PIN);
    actuator.begin();

    _lastUpdateMs = millis();
}

void loop() {
    gps.update();
    sensor.update();

    uint32_t now = millis();
    float dt = (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    if (!gps.isValid()) {
        actuator.setMotorLeft(0);
        actuator.setMotorRight(0);
        Serial.println("[TEST] waiting for GPS fix...");
        delay(200);
        return;
    }

    float bearing  = gps.bearingTo(GOAL_LAT, GOAL_LON);
    float heading  = sensor.getYaw();
    float error    = normalizeAngle(bearing - heading);
    float distance = gps.distanceTo(GOAL_LAT, GOAL_LON);

    float turn = headingPid.update(error, dt);
    int left  = constrain((int)(BASE_SPEED - turn), -255, 255);
    int right = constrain((int)(BASE_SPEED + turn), -255, 255);

    actuator.setMotorLeft(left);
    actuator.setMotorRight(right);

    Serial.printf("bearing=%.1f heading=%.1f error=%.1f turn=%.1f left=%4d right=%4d dist=%.1fm\n",
                  bearing, heading, error, turn, left, right, distance);

    delay(100);  // 10Hz、dt=0.1s
}
