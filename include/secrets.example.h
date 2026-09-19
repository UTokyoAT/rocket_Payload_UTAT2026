#pragma once

// このファイルを include/secrets.h にコピーして値を書き換える（secrets.h は .gitignore 済み）。

// ロボットが接続するWi-Fiルーター
#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"

// Mosquittoを動かしているPC(ground/server の docker compose)のIPアドレス
#define MQTT_HOST     "192.168.0.10"
