#include <Arduino.h>
#include <WebServer.h>
#include <Actuator.h>
#include <DebugLog.h>

// 左右モーター（TB6612FNG、3ピン/モーター＋共有STBY方式）の手動操作テスト
// PlatformIO で env:test-motor を選択して書き込む
// PCのブラウザからWiFi経由でモーター出力をスライダーで直接操作できる。
// STBYはActuator::begin()内でソフト制御によりHIGHにする（ハード側の固定配線ではない）。
//
// CanSat-AP（パスワード: cansat2026）に接続して、
//   http://192.168.4.1:8080/  … モーター手動操作パネル
//   http://192.168.4.1/       … デバッグログ（DebugLogが提供、ポート80）
// をそれぞれ開く（USBシリアル不要）。

static Actuator actuator;
static DebugLog debug;
static WebServer controlServer(8080);

static int gLeftSpeed  = 0;
static int gRightSpeed = 0;

static const char CONTROL_PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Motor Manual Control</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: sans-serif; padding: 16px; }
    h2 { margin-bottom: 16px; font-size: 1.1em; }
    .row { margin-bottom: 20px; }
    label { display: block; margin-bottom: 6px; }
    input[type=range] { width: 100%; }
    .val { font-family: monospace; font-size: 1.2em; }
    button { font-size: 1em; padding: 10px 20px; margin-right: 8px; margin-top: 8px; }
    #stop { background: #a33; color: #fff; border: none; border-radius: 4px; }
  </style>
</head>
<body>
  <h2>Motor Manual Control</h2>
  <div class="row">
    <label>LEFT  <span id="lVal" class="val">0</span></label>
    <input id="lSlider" type="range" min="-255" max="255" value="0">
  </div>
  <div class="row">
    <label>RIGHT <span id="rVal" class="val">0</span></label>
    <input id="rSlider" type="range" min="-255" max="255" value="0">
  </div>
  <button id="stop">STOP</button>
  <script>
    const lSlider = document.getElementById('lSlider');
    const rSlider = document.getElementById('rSlider');
    const lVal = document.getElementById('lVal');
    const rVal = document.getElementById('rVal');

    let pending = false;
    let dirty = false;
    function send() {
      if (pending) { dirty = true; return; }
      pending = true;
      const l = lSlider.value, r = rSlider.value;
      fetch(`/motor?left=${l}&right=${r}`, { cache: 'no-store' })
        .catch(() => {})
        .finally(() => {
          pending = false;
          if (dirty) { dirty = false; send(); }
        });
    }
    lSlider.oninput = () => { lVal.textContent = lSlider.value; send(); };
    rSlider.oninput = () => { rVal.textContent = rSlider.value; send(); };
    document.getElementById('stop').onclick = () => {
      lSlider.value = 0; rSlider.value = 0;
      lVal.textContent = 0; rVal.textContent = 0;
      send();
    };
  </script>
</body>
</html>
)rawhtml";

static void handleMotorRequest() {
    if (controlServer.hasArg("left"))  gLeftSpeed  = constrain(controlServer.arg("left").toInt(),  -255, 255);
    if (controlServer.hasArg("right")) gRightSpeed = constrain(controlServer.arg("right").toInt(), -255, 255);

    actuator.setMotorLeft(gLeftSpeed);
    actuator.setMotorRight(gRightSpeed);

    debug.printf("[CTRL] left=%4d right=%4d", gLeftSpeed, gRightSpeed);

    char buf[32];
    snprintf(buf, sizeof(buf), "L=%d R=%d", gLeftSpeed, gRightSpeed);
    controlServer.send(200, "text/plain", buf);
}

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] Motor manual control starting...");
    actuator.begin();

    controlServer.on("/", HTTP_GET, []() {
        controlServer.send_P(200, "text/html", CONTROL_PAGE_HTML);
    });
    controlServer.on("/motor", HTTP_GET, handleMotorRequest);
    controlServer.begin();

    debug.printf("[TEST] Control panel: http://%s:8080/", debug.getIP().toString().c_str());
}

void loop() {
    debug.poll();
    controlServer.handleClient();
}
