# 環境構築

## 使用マイコン

**Seeed Studio XIAO ESP32S3**（ESP32-S3、8MB Flash / 8MB PSRAM）

---

## 1. 必要なソフトウェアのインストール

| ソフトウェア | 説明 |
|---|---|
| [VS Code](https://code.visualstudio.com/) | エディタ |
| [PlatformIO IDE（VS Code拡張）](https://platformio.org/install/ide?install=vscode) | ESP32への書き込み・ビルド環境 |

> PlatformIO IDE をインストールすると、必要なコンパイラ・ツールチェーンが自動でインストールされます。

---

## 2. USBドライバのインストール（Windows）

XIAO ESP32S3 は ESP32-S3 のネイティブUSB（USB CDC）でPCと直接通信するため、CP2102 / CH340 のような外付けドライバは**不要**。USB-Cケーブルで接続すればデバイスマネージャーの「ポート」欄にCOMポートとして認識される。

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
