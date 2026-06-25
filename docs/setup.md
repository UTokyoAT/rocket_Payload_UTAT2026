# 環境構築

## 使用マイコン

**ESP32-DevKitC（WROOM-32U WROVER）**

---

## 1. 必要なソフトウェアのインストール

| ソフトウェア | 説明 |
|---|---|
| [VS Code](https://code.visualstudio.com/) | エディタ |
| [PlatformIO IDE（VS Code拡張）](https://platformio.org/install/ide?install=vscode) | ESP32への書き込み・ビルド環境 |

> PlatformIO IDE をインストールすると、必要なコンパイラ・ツールチェーンが自動でインストールされます。

---

## 2. USBドライバのインストール（Windows）

ESP32-DevKitC は **CP2102** または **CH340** チップを使ってUSBシリアル変換をしている。

- CP2102（シルクに"Silicon Labs"）→ [CP210x USB to UART Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- CH340（シルクに"CH340"）→ [CH340 Driver](https://www.wch-ic.com/downloads/CH341SER_EXE.html)

ボードを見て判断するか、デバイスマネージャーの「ポート」欄で確認する。

---

## 3. リポジトリのクローン

```bash
git clone <このリポジトリのURL>
cd rocket_Payload_UTAT2026
```

---

## 4. VS Code でプロジェクトを開く

```
VS Code → File → Open Folder → rocket_Payload_UTAT2026 フォルダを選択
```

PlatformIO が `platformio.ini` を検出し、自動でライブラリ・依存関係をダウンロードする。

---

## 5. 書き込み方法

詳細は [flashing.md](flashing.md) を参照。
