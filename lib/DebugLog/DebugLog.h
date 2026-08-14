#pragma once
#include <Arduino.h>

// WiFi経由でデバッグログを見るための軽量ロガー（tests/配下の単体動作確認コード用）。
// SoftAP + HTTPで動作し、USBシリアルを接続しなくてもブラウザ（http://192.168.4.1）や
// GET /log のポーリングでテスト出力を確認できる。SSID/パスワードは他のCanSat用WiFi
// （lib/Radio, tests/test_wifi_dummy）と同じ "CanSat-AP" 固定。
class DebugLog {
public:
    void begin();

    // printf形式で1行追加する。内部リングバッファ（直近60行）に保持し、
    // GET /log ・ GET / の両方から参照できる。Serialにも同時出力する
    // （ケーブル接続時のフォールバック用。無くても動作に支障はない）。
    void printf(const char* fmt, ...);

    // HTTPリクエストを処理する。loop()内で毎ティック呼ぶこと
    // （長いdelay()の途中で呼びたい場合はループを分割して呼ぶ）。
    void poll();

    IPAddress getIP();
};
