# rocket_Payload_UTAT2026

輸送機班2026年度の新人研修期間中の上級生での活動。CanSatを搭載したペットボトルロケットを開発するためのリポジトリ

---

## 使用マイコン

**ESP32-DevKitC（WROOM-32U WROVER）**

---

## 環境構築

### 1. 必要なソフトウェアのインストール

| ソフトウェア | 説明 |
|---|---|
| [VS Code](https://code.visualstudio.com/) | エディタ |
| [PlatformIO IDE（VS Code拡張）](https://platformio.org/install/ide?install=vscode) | ESP32への書き込み・ビルド環境 |

> PlatformIO IDE をインストールすると、必要なコンパイラ・ツールチェーンが自動でインストールされます。

### 2. USBドライバのインストール（Windows）

ESP32-DevKitC は **CP2102** または **CH340** チップを使ってUSBシリアル変換をしている。

- CP2102（シルクに"Silicon Labs"）→ [CP210x USB to UART Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- CH340（シルクに"CH340"）→ [CH340 Driver](https://www.wch-ic.com/downloads/CH341SER_EXE.html)

ボードを見て判断するか、デバイスマネージャーの「ポート」欄で確認する。

### 3. リポジトリのクローン

```bash
git clone <このリポジトリのURL>
cd rocket_Payload_UTAT2026
```

### 4. VS Code でプロジェクトを開く

```
VS Code → File → Open Folder → rocket_Payload_UTAT2026 フォルダを選択
```

PlatformIO が `platformio.ini` を検出し、自動でライブラリ・依存関係をダウンロードする。

---

## 書き込み方法

### 1. ボードをPCに接続

USBケーブルで ESP32-DevKitC を接続する。

デバイスマネージャー（またはターミナル）でCOMポートが認識されているか確認する。

```powershell
# Windows PowerShell で確認
[System.IO.Ports.SerialPort]::GetPortNames()
```

### 2. ビルドして書き込む

**VS Code の PlatformIO サイドバー（アリのアイコン）から操作する方法：**

```
PlatformIO サイドバー → PROJECT TASKS → esp32-wrover
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

### 3. 書き込みに失敗する場合

ESP32 は書き込み時に **BOOTボタンを押しながらENボタンを押す**（またはBOOTを押し続けてUpload開始）と強制的に書き込みモードになる。

```
書き込み開始のログが出た直後に BOOTボタンを押す → 離す
```

`platformio.ini` でポートを固定したい場合：

```ini
upload_port = COM3   ; 自分の環境のCOMポートに変更
monitor_port = COM3
```

---

## プロジェクト構成

```
rocket_Payload_UTAT2026/
├── platformio.ini   # ビルド・ボード設定
├── src/
│   └── main.cpp     # メインコード
├── include/         # ヘッダファイル置き場
└── lib/             # プロジェクト固有ライブラリ置き場
```
