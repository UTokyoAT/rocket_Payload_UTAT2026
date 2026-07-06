# 書き込み方法

機体には XIAO ESP32S3 が **2枚** 搭載されている（[architecture.md](architecture.md) 参照）。それぞれ別のファームウェアなので、両方とも個別に書き込む必要がある。

| 基板 | 役割 | PlatformIO env |
|---|---|---|
| XIAO1 | センサー・GPS・WiFi（マスター） | `xiao1` |
| XIAO2 | モータPID制御・誘導（スレーブ） | `xiao2` |

---

## 1. ボードをPCに接続

USB-Cケーブルで **書き込みたい方の** XIAO ESP32S3 を接続する（2枚同時にPCへ挿してもよいが、書き込み対象を間違えないようCOMポートを確認すること）。

デバイスマネージャー（またはターミナル）でCOMポートが認識されているか確認する。

```powershell
# Windows PowerShell で確認
[System.IO.Ports.SerialPort]::GetPortNames()
```

---

## 2. ビルドして書き込む

**VS Code の PlatformIO サイドバー（アリのアイコン）から操作する：**

```
PlatformIO サイドバー → PROJECT TASKS → xiao1（またはxiao2）
  ├─ Build       ← コンパイルのみ
  ├─ Upload      ← ビルド＋書き込み
  └─ Monitor     ← シリアルモニタ（115200 baud）
```

XIAO1用のファームウェアを書き込むときは `xiao1`、XIAO2用（モーター側）を書き込むときは `xiao2` を選ぶこと。VS Code右下のステータスバーで現在選択中の環境が確認できる。

**ショートカット（現在選択中の環境に対して実行される）：**

| 操作 | ショートカット |
|---|---|
| ビルド | `Ctrl+Alt+B` |
| 書き込み | `Ctrl+Alt+U` |
| シリアルモニタ | `Ctrl+Alt+S` |

XIAO2の初回ビルド時は `hideakitai/ESP32SPISlave` ライブラリが自動でダウンロードされる（ネット接続が必要）。

---

## 3. 書き込みに失敗する場合

XIAO ESP32S3 はネイティブUSBの自動リセットで通常は書き込みモードに入るが、失敗する場合は基板上の **BOOTボタン（Bと刻印）を押しながらUSBケーブルを挿す**（またはUploadを開始してから数秒BOOTを押し続ける）と強制的に書き込みモードになる。

```
Upload開始のログが出た直後に BOOTボタンを押す → 離す
```

`platformio.ini` の該当envでポートを固定したい場合：

```ini
[env:xiao1]
extends = common
build_src_filter = -<*> +<../src/xiao1/>
upload_port = COM3   ; 自分の環境のCOMポートに変更
monitor_port = COM3
```
