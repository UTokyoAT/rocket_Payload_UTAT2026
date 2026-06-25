# rocket_Payload_UTAT2026

輸送機班2026年度の新人研修期間中の上級生での活動。CanSatを搭載したペットボトルロケットを開発するためのリポジトリ

---

## ドキュメント

- [環境構築](docs/setup.md)
- [書き込み方法](docs/flashing.md)
- [JLCPCB 発注手順](docs/pcb-ordering.md)

---

## プロジェクト構成

```
rocket_Payload_UTAT2026/
├── platformio.ini       # ビルド・ボード設定
├── src/
│   └── main.cpp         # メインコード
├── include/             # ヘッダファイル置き場
├── lib/                 # プロジェクト固有ライブラリ置き場
├── hardware/
│   ├── kicad/           # KiCad プロジェクトファイル
│   └── jlcpcb/
│       ├── gerbers/     # Gerberファイル
│       ├── bom/         # Bill of Materials
│       └── cpl/         # Component Placement List
└── docs/                # ドキュメント
```
