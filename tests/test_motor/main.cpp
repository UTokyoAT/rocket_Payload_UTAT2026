#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Expander.h>
#include <Encoder.h>
#include <Actuator.h>
#include <Telemetry.h>
#include "secrets.h"

// 左右モーター（TB6612FNG）＋エンコーダの動作確認テスト。
// PlatformIOで env:test-motor を選択して書き込む。
// PCのブラウザからWiFi経由でPWM出力（左右デューティ・PWM周波数）を操作し、
// エンコーダの回転角・回転数・rpmをTelemetry経由で地上局（InfluxDB/Grafana）に送る。
//
// 事前に include/secrets.h を設定し、ground/server で `docker compose up -d` しておく。
// ボードは secrets.h のWi-Fiルーターに接続する（Telemetryと同じSTA接続）。
// 操作パネル: 接続後にシリアルと rocket_log に出る http://<ボードのIP>/ をPCのブラウザで開く。
// エンコーダ値: rocket_test の test_name=motor に 10Hz で送信
//   （left_pwm/right_pwm, left_rad/right_rad, left_rev/right_rev, left_rpm/right_rpm）。
//   操作パネルにも同じ値を表示する。
//
// エンドポイントはcurl等から直接叩くこともできる:
//   GET /motor?left=<-255..255>&right=<-255..255>   PWM出力を設定（片方だけでも可）
//   GET /freq?hz=<100..20000>                        PWM周波数を設定
//   GET /zero                                        エンコーダの角度表示を0に戻す
//   GET /status                                      現在の状態をJSONで返す
//
// 安全のため、ブラウザからの通信が CONTACT_TIMEOUT_MS 以上途絶えたら両モーターを止める
// （ページを開いている間は /status のポーリングが生存確認を兼ねる）。
// USBシリアル(115200bps)にも状態を定期出力する。
// ルーターに繋がらないとパネルが開けずモーターも回せない（通信途絶扱いで停止状態のまま）。
//
// 確認ポイント:
//   - 同じ符号で左右が前進方向に回るか（逆なら取り付け向きに合わせて符号反転が必要）
//   - 正のPWMでエンコーダの角度が増えるか（左右で符号がそろっているか）
//   - 回転数がそれらしい値か（JGA12-N10 1:210は6V無負荷で約100rpm）


static const unsigned long CONTACT_TIMEOUT_MS = 1500;  // 通信途絶でモーター停止するまでの時間
static const unsigned long RPM_PERIOD_MS      = 100;   // rpm算出周期
static const unsigned long SEND_PERIOD_MS     = 100;   // Telemetry送信周期（10Hz）
static const unsigned long PRINT_PERIOD_MS    = 500;   // シリアル出力周期

static Expander  expander;
static Encoder   encoder;
static Actuator  actuator;
static Telemetry telemetry;
static WebServer server(80);

static int      gLeftPwm  = 0;
static int      gRightPwm = 0;
static uint32_t gPwmFreq  = 1000;  // ESP32 Arduino 2.x の analogWrite 既定値

static double gLeftZero  = 0;  // /zero 時点の角度 [rad]
static double gRightZero = 0;
static double gLeftRpm   = 0;
static double gRightRpm  = 0;

static unsigned long gLastContactMs = 0;
static unsigned long gLastRpmMs     = 0;
static unsigned long gLastSendMs    = 0;
static unsigned long gLastPrintMs   = 0;
static bool          gAnnounced     = false;  // MQTT接続後にパネルのURLを1度だけ通知したか

static const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Motor / Encoder Test</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: sans-serif; padding: 16px; max-width: 640px; }
    h2 { margin-bottom: 16px; font-size: 1.1em; }
    h3 { margin: 20px 0 8px; font-size: 1em; color: #aaa; }
    .row { margin-bottom: 18px; }
    label { display: block; margin-bottom: 6px; }
    input[type=range] { width: 100%; }
    input[type=number] { width: 90px; font-size: 1em; padding: 4px; }
    .val { font-family: monospace; font-size: 1.2em; }
    button { font-size: 1em; padding: 8px 16px; margin: 6px 6px 0 0; }
    #stop { background: #a33; color: #fff; border: none; border-radius: 4px; padding: 12px 28px; }
    table { border-collapse: collapse; width: 100%; font-family: monospace; }
    th, td { border: 1px solid #333; padding: 6px 8px; text-align: right; }
    th:first-child, td:first-child { text-align: left; }
    #conn { font-size: 0.9em; }
  </style>
</head>
<body>
  <h2>Motor / Encoder Test <span id="conn"></span></h2>

  <div class="row">
    <label>LEFT PWM <span id="lVal" class="val">0</span></label>
    <input id="lSlider" type="range" min="-255" max="255" value="0">
  </div>
  <div class="row">
    <label>RIGHT PWM <span id="rVal" class="val">0</span></label>
    <input id="rSlider" type="range" min="-255" max="255" value="0">
  </div>
  <label><input id="link" type="checkbox"> 左右連動</label>
  <button id="stop">STOP</button>

  <h3>PWM周波数</h3>
  <input id="freq" type="number" min="100" max="20000" step="100" value="1000"> Hz
  <button id="freqBtn">設定</button>
  <span id="freqCur" class="val"></span>

  <h3>エンコーダ</h3>
  <table>
    <tr><th></th><th>LEFT</th><th>RIGHT</th></tr>
    <tr><td>PWM</td><td id="lp">-</td><td id="rp">-</td></tr>
    <tr><td>角度 [rad]</td><td id="la">-</td><td id="ra">-</td></tr>
    <tr><td>回転数 [rev]</td><td id="lr">-</td><td id="rr">-</td></tr>
    <tr><td>速度 [rpm]</td><td id="ls">-</td><td id="rs">-</td></tr>
  </table>
  <button id="zero">角度リセット</button>

  <script>
    const $ = id => document.getElementById(id);
    const lSlider = $('lSlider'), rSlider = $('rSlider');

    // スライダー操作中のリクエストが詰まらないよう、送信中は最新値だけ後で送る
    let pending = false, dirty = false;
    function send() {
      if (pending) { dirty = true; return; }
      pending = true;
      fetch(`/motor?left=${lSlider.value}&right=${rSlider.value}`, { cache: 'no-store' })
        .catch(() => {})
        .finally(() => { pending = false; if (dirty) { dirty = false; send(); } });
    }
    function onSlide(src, dst) {
      if ($('link').checked) dst.value = src.value;
      $('lVal').textContent = lSlider.value;
      $('rVal').textContent = rSlider.value;
      send();
    }
    lSlider.oninput = () => onSlide(lSlider, rSlider);
    rSlider.oninput = () => onSlide(rSlider, lSlider);
    $('stop').onclick = () => {
      lSlider.value = 0; rSlider.value = 0;
      $('lVal').textContent = 0; $('rVal').textContent = 0;
      send();
    };
    $('freqBtn').onclick = () => fetch(`/freq?hz=${$('freq').value}`, { cache: 'no-store' }).catch(() => {});
    $('zero').onclick = () => fetch('/zero', { cache: 'no-store' }).catch(() => {});

    // 200msごとに状態取得（ESP32側では通信途絶検知の生存確認も兼ねる）
    async function poll() {
      try {
        const s = await (await fetch('/status', { cache: 'no-store' })).json();
        $('lp').textContent = s.left.pwm;            $('rp').textContent = s.right.pwm;
        $('la').textContent = s.left.rad.toFixed(3); $('ra').textContent = s.right.rad.toFixed(3);
        $('lr').textContent = s.left.rev.toFixed(3); $('rr').textContent = s.right.rev.toFixed(3);
        $('ls').textContent = s.left.rpm.toFixed(1); $('rs').textContent = s.right.rpm.toFixed(1);
        $('freqCur').textContent = `(現在 ${s.freq} Hz)`;
        // 通信途絶で止められていたらスライダーも0に戻す
        if (s.left.pwm == 0 && s.right.pwm == 0 && !pending && (lSlider.value != 0 || rSlider.value != 0)) {
          lSlider.value = 0; rSlider.value = 0;
          $('lVal').textContent = 0; $('rVal').textContent = 0;
        }
        $('conn').textContent = '●'; $('conn').style.color = '#4c4';
      } catch (e) {
        $('conn').textContent = '● 切断'; $('conn').style.color = '#c44';
      }
      setTimeout(poll, 200);
    }
    poll();
  </script>
</body>
</html>
)rawhtml";

static void applyMotors() {
    actuator.setMotorLeft(gLeftPwm);
    actuator.setMotorRight(gRightPwm);
}

static void stopMotors(const char* reason) {
    if (gLeftPwm == 0 && gRightPwm == 0) return;
    gLeftPwm = 0;
    gRightPwm = 0;
    applyMotors();
    Serial.printf("[STOP] %s\n", reason);
}

static void handleMotor() {
    gLastContactMs = millis();
    if (server.hasArg("left"))  gLeftPwm  = constrain(server.arg("left").toInt(),  -255, 255);
    if (server.hasArg("right")) gRightPwm = constrain(server.arg("right").toInt(), -255, 255);
    applyMotors();

    char buf[32];
    snprintf(buf, sizeof(buf), "L=%d R=%d", gLeftPwm, gRightPwm);
    server.send(200, "text/plain", buf);
}

static void handleFreq() {
    gLastContactMs = millis();
    if (!server.hasArg("hz")) {
        server.send(400, "text/plain", "missing hz");
        return;
    }
    gPwmFreq = constrain(server.arg("hz").toInt(), 100, 20000);
    analogWriteFrequency(gPwmFreq);  // analogWriteで使用中の全ピンに反映される
    Serial.printf("[FREQ] %u Hz\n", gPwmFreq);

    char buf[32];
    snprintf(buf, sizeof(buf), "freq=%u", gPwmFreq);
    server.send(200, "text/plain", buf);
}

static void handleZero() {
    gLastContactMs = millis();
    gLeftZero  = encoder.getLeftRevolutions();
    gRightZero = encoder.getRightRevolutions();
    server.send(200, "text/plain", "zeroed");
}

static void handleStatus() {
    gLastContactMs = millis();
    double leftRad  = encoder.getLeftRevolutions()  - gLeftZero;
    double rightRad = encoder.getRightRevolutions() - gRightZero;

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"freq\":%u,"
             "\"left\":{\"pwm\":%d,\"rad\":%.4f,\"rev\":%.4f,\"rpm\":%.2f},"
             "\"right\":{\"pwm\":%d,\"rad\":%.4f,\"rev\":%.4f,\"rpm\":%.2f}}",
             gPwmFreq,
             gLeftPwm,  leftRad,  leftRad  / TWO_PI, gLeftRpm,
             gRightPwm, rightRad, rightRad / TWO_PI, gRightRpm);
    server.send(200, "application/json", buf);
}

// 一定周期ごとの角度差分からrpmを求める（getXxxDeltaAngleはここでだけ呼ぶ）
static void updateRpm() {
    unsigned long now = millis();
    unsigned long dt = now - gLastRpmMs;
    if (dt < RPM_PERIOD_MS) return;
    gLastRpmMs = now;

    double toRpm = 60.0 / TWO_PI / (dt / 1000.0);
    gLeftRpm  = encoder.getLeftDeltaAngle()  * toRpm;
    gRightRpm = encoder.getRightDeltaAngle() * toRpm;
}

static void sendEncoder() {
    if (millis() - gLastSendMs < SEND_PERIOD_MS) return;
    gLastSendMs = millis();

    double leftRad  = encoder.getLeftRevolutions()  - gLeftZero;
    double rightRad = encoder.getRightRevolutions() - gRightZero;
    telemetry.sendTest("motor", {
        {"left_pwm",  gLeftPwm},          {"right_pwm",  gRightPwm},
        {"left_rad",  leftRad},           {"right_rad",  rightRad},
        {"left_rev",  leftRad / TWO_PI},  {"right_rev",  rightRad / TWO_PI},
        {"left_rpm",  gLeftRpm},          {"right_rpm",  gRightRpm},
    });
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[TEST] Motor / Encoder test starting...");

    if (!expander.setup()) {
        Serial.println("[TEST] Expander init failed (motor direction pins will not work)");
    }
    actuator.begin(expander);
    encoder.begin();

    // Wi-Fi(STA)接続はTelemetryが行う。WebServerは接続前にbeginしておいて問題ない
    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE_HTML); });
    server.on("/motor",  HTTP_GET, handleMotor);
    server.on("/freq",   HTTP_GET, handleFreq);
    server.on("/zero",   HTTP_GET, handleZero);
    server.on("/status", HTTP_GET, handleStatus);
    server.begin();

    gLastRpmMs = millis();
}

void loop() {
    server.handleClient();
    updateRpm();
    sendEncoder();

    if (!gAnnounced && telemetry.isConnected()) {
        gAnnounced = true;
        telemetry.log("[TEST] motor panel: http://%s/", WiFi.localIP().toString().c_str());
    }

    if (millis() - gLastContactMs > CONTACT_TIMEOUT_MS) {
        stopMotors("contact timeout");
    }

    if (millis() - gLastPrintMs >= PRINT_PERIOD_MS) {
        gLastPrintMs = millis();
        double leftRad  = encoder.getLeftRevolutions()  - gLeftZero;
        double rightRad = encoder.getRightRevolutions() - gRightZero;
        Serial.printf("[ENC] L: pwm=%4d rad=%9.3f rpm=%7.1f | R: pwm=%4d rad=%9.3f rpm=%7.1f\n",
                      gLeftPwm, leftRad, gLeftRpm, gRightPwm, rightRad, gRightRpm);
    }
}
