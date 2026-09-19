#include "DebugLog.h"
#include <WiFi.h>
#include <WebServer.h>
#include <stdarg.h>

static const char* AP_SSID = "payload2026";
static const char* AP_PASS = "small";

static WebServer _server(80);

// 直近LINE_COUNT行だけ保持するリングバッファ
static const int LINE_COUNT = 60;
static String   _lines[LINE_COUNT];
static int      _nextLine   = 0;
static int      _totalLines = 0;

static const char PAGE_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CanSat Debug Log</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #111; color: #eee; font-family: monospace; padding: 12px; }
    h2 { margin-bottom: 10px; font-size: 1.1em; }
    #status.connected { color: #4fc; }
    #status.disconnected { color: #f44; }
    pre { white-space: pre-wrap; word-break: break-all; font-size: 0.85em; line-height: 1.4; }
  </style>
</head>
<body>
  <h2>CanSat Debug Log &nbsp;<span id="status" class="disconnected">● disconnected</span></h2>
  <pre id="log">(waiting for data...)</pre>
  <script>
    const POLL_MS = 300;
    const el = document.getElementById('status');
    const logEl = document.getElementById('log');
    async function poll() {
      try {
        const res = await fetch('/log', { cache: 'no-store' });
        if (!res.ok) throw new Error('HTTP ' + res.status);
        logEl.textContent = await res.text();
        window.scrollTo(0, document.body.scrollHeight);
        el.textContent = '● reachable';
        el.className = 'connected';
      } catch (e) {
        el.textContent = '● unreachable';
        el.className = 'disconnected';
      } finally {
        setTimeout(poll, POLL_MS);
      }
    }
    poll();
  </script>
</body>
</html>
)rawhtml";

void DebugLog::begin() {
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[DebugLog] SoftAP: %s  IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    _server.on("/log", HTTP_GET, []() {
        String out;
        int count = min(_totalLines, LINE_COUNT);
        int start = (_totalLines <= LINE_COUNT) ? 0 : _nextLine;
        for (int i = 0; i < count; i++) {
            out += _lines[(start + i) % LINE_COUNT];
            out += '\n';
        }
        _server.send(200, "text/plain", out);
    });

    _server.on("/", HTTP_GET, []() {
        _server.send_P(200, "text/html", PAGE_HTML);
    });

    _server.begin();
}

void DebugLog::printf(const char* fmt, ...) {
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.println(buf);

    _lines[_nextLine] = String(millis()) + "ms  " + buf;
    _nextLine = (_nextLine + 1) % LINE_COUNT;
    _totalLines++;
}

void DebugLog::poll() { _server.handleClient(); }

IPAddress DebugLog::getIP() { return WiFi.softAPIP(); }
