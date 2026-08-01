#include <Arduino.h>
#include <Wire.h>
#include <Sensor.h>
#include <Adafruit_BMP280.h>

// BMP280（気圧センサー）の動作確認
// PlatformIO で env:test-bmp280 を選択して書き込む
// デバッグ出力はUSBシリアルのみ（WiFi/DebugLogは使わない）。115200bpsでモニタすること。
//
// 注意: XIAO ESP32S3はネイティブUSB(ハードウェアCDC)で、ホストが本当に接続できているかの
// 判定(Serialのbool判定)が環境によっては信頼できず、while(!Serial)等で待つと
// 出力が永久に見えなくなることがある。そのため接続待ちはせず、診断ログを
// BMP280が見つかるまで一定間隔で繰り返し流し続ける方式にしている
// （モニタをいつ開いても、いずれ必ず出力が流れてくる）。

static Sensor sensor;

// Sensorクラスを介さず、Adafruit_BMP280::begin()を直接・毎回呼ぶための診断専用インスタンス。
// begin()が失敗する箇所（アドレス検出 or チップID比較）をsensorID()で切り分けるために使う
// （sensorID()はbegin()内部で読んだ生のチップID値を、成功/失敗に関わらず保持している）。
static Adafruit_BMP280 bmpDiag;

// BMP280のチップID(reg 0xD0)を生I2Cで読む。ライブラリのbegin()を介さずに
// 「バス上に応答する機器がいるか」「いるならBMP280として妥当なIDか」を切り分ける。
// 正常なBMP280は0x58を返す（0x60はBME280、0xFF/0x00は無応答＝配線/アドレス不良）。
static bool readChipId(uint8_t addr, uint8_t* outId) {
    Wire.beginTransmission(addr);
    Wire.write(0xD0);
    if (Wire.endTransmission(false) != 0) {
        return false;  // NACK: そのアドレスに機器がいない
    }
    if (Wire.requestFrom((uint8_t)addr, (uint8_t)1) != 1) {
        return false;
    }
    *outId = Wire.read();
    return true;
}

static void runDiagnostics() {
    Serial.println("[TEST] --- BMP280 diagnostics ---");

    // 1. I2Cバス全体をスキャンして、そもそも何かに応答があるか確認する
    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[TEST]   device found at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[TEST]   no device responded on the bus at all -> 配線(SDA/SCL/GND/VCC)を確認");
    }

    // 2. BMP280が使う候補アドレス(0x76/0x77)についてチップIDを直接読んでみる
    for (uint8_t addr : {0x76, 0x77}) {
        uint8_t chipId = 0;
        if (!readChipId(addr, &chipId)) {
            Serial.printf("[TEST] 0x%02X: no response (NACK)\n", addr);
        } else if (chipId == 0x58) {
            Serial.printf("[TEST] 0x%02X: chip ID = 0x%02X -> BMP280として妥当\n", addr, chipId);
        } else if (chipId == 0x60) {
            Serial.printf("[TEST] 0x%02X: chip ID = 0x%02X -> これはBME280（湿度センサ付き）。Adafruit_BMP280では正しく動かない\n", addr, chipId);
        } else {
            Serial.printf("[TEST] 0x%02X: chip ID = 0x%02X -> 想定外の値（別チップ or 読み取り不良）\n", addr, chipId);
        }
    }

    // 3. Adafruit_BMP280::begin()を毎回直接呼び、失敗する箇所を切り分ける。
    //    - i2c_dev->begin()（アドレス検出）で落ちていれば sensorID()==0 のまま
    //    - チップID比較で落ちていれば sensorID() に実際に読めた値が入る
    bool diagOk = bmpDiag.begin(0x76);
    Serial.printf("[TEST] Adafruit_BMP280::begin(0x76) => %s, internal sensorID=0x%02X\n",
                  diagOk ? "true" : "false", bmpDiag.sensorID());

    // 4. 実際にSensorクラス経由（本番コードと同じ経路）でも初期化を試す
    sensor.begin();
    if (!sensor.isBmp280Ready()) {
        Serial.println("[TEST] BMP280 not detected via Sensor::begin(). Check wiring (I2C: 0x76/0x77).");
    } else {
        // begin()時に地上気圧がキャリブレーションされている（Sensor.cppのコメント参照）。
        // alt はこの基準からの相対高度になるため、起動直後は0m付近になるはず。
        Serial.printf("[TEST] Ground level calibrated: %.2fhPa\n", sensor.getGroundLevelHpa());
    }
    Serial.println("[TEST] ----------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち（接続確認はしない。上記の理由を参照）

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("[TEST] BMP280 check starting...");
    runDiagnostics();
}

void loop() {
    static uint32_t lastDiagMs = 0;

    // BMP280が見つかっていない間は、診断ログを3秒おきに出し続ける
    // （モニタをいつ開いても必ず状況が確認できるようにするため）
    if (!sensor.isBmp280Ready() && millis() - lastDiagMs >= 3000) {
        lastDiagMs = millis();
        runDiagnostics();
    }

    sensor.update();

    Serial.printf("pressure=%.2fhPa  temp=%.1fC  alt(relative)=%.2fm\n",
                  sensor.getPressure(), sensor.getTemperature(), sensor.getAltitude());

    delay(500);
}
