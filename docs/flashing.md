# 書き込み方法

## 1. ボードをPCに接続

USB-Cケーブルで XIAO ESP32S3 を接続する。

デバイスマネージャー（またはターミナル）でCOMポートが認識されているか確認する。

```powershell
# Windows PowerShell で確認
[System.IO.Ports.SerialPort]::GetPortNames()
```

---

## 2. ビルドして書き込む

**VS Code の PlatformIO サイドバー（アリのアイコン）から操作する：**

```
PlatformIO サイドバー → PROJECT TASKS → esp32-s3
  ├─ Build       ← コンパイルのみ
  ├─ Upload      ← ビルド＋書き込み
  └─ Monitor     ← シリアルモニタ（115200 baud）
```

**ショートカット：**

| 操作 | ショートカット |
|---|---|
| ビルド | `Ctrl+Alt+B` |
| 書き込み | `Ctrl+Alt+U` |
| シリアルモニタ | `Ctrl+Alt+S` |

---

## 3. 書き込みに失敗する場合

XIAO ESP32S3 はネイティブUSBの自動リセットで通常は書き込みモードに入るが、失敗する場合は基板上の **BOOTボタン（Bと刻印）を押しながらUSBケーブルを挿す**（またはUploadを開始してから数秒BOOTを押し続ける）と強制的に書き込みモードになる。

```
Upload開始のログが出た直後に BOOTボタンを押す → 離す
```

`platformio.ini` でポートを固定したい場合：

```ini
upload_port = COM3   ; 自分の環境のCOMポートに変更
monitor_port = COM3
```
