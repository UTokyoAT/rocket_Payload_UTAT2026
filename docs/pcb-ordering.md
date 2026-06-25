# JLCPCB 発注手順

## 必要なファイル

| ファイル | 説明 | 出力元 |
|---|---|---|
| Gerber（ZIP） | 基板データ本体 | KiCad → Plot |
| BOM（CSV） | 部品リスト（LCSC部品番号付き） | KiCad → BOM Plugin |
| CPL（CSV） | 部品実装座標リスト | KiCad → Fabrication Outputs |

出力ファイルは `hardware/jlcpcb/` に格納する。

---

## KiCad からのファイル出力手順

### Gerber

1. PCBエディタ → `File` → `Fabrication Outputs` → `Gerbers (.gbr)`
2. 出力先を `hardware/jlcpcb/gerbers/` に設定
3. 以下のレイヤーにチェックを入れる：
   - F.Cu / B.Cu（銅箔）
   - F.Silkscreen / B.Silkscreen（シルク）
   - F.Mask / B.Mask（ソルダーマスク）
   - Edge.Cuts（外形）
4. `Plot` → 続けて `Generate Drill Files`
5. `gerbers/` フォルダをZIPに圧縮してアップロード

### BOM（PCBA発注時のみ）

1. PCBエディタ → `File` → `Fabrication Outputs` → `BOM`
2. LCSC部品番号は部品プロパティの `LCSC` フィールドに記入しておく
3. 出力を `hardware/jlcpcb/bom/bom.csv` に保存

### CPL（PCBA発注時のみ）

1. PCBエディタ → `File` → `Fabrication Outputs` → `Component Placement (.pos)`
2. 形式: CSV、単位: mm、個別ファイル（表面・裏面）
3. 出力を `hardware/jlcpcb/cpl/` に保存

---

## JLCPCB サイトでの発注

1. [jlcpcb.com](https://jlcpcb.com/) にアクセス
2. `Order Now` → Gerber ZIPをアップロード
3. 基板仕様を設定（枚数・板厚・色など）
4. PCBA（実装）が必要な場合は `PCB Assembly` をONにしてBOM・CPLをアップロード
5. BOM確認画面でLCSC部品番号を照合して発注

---

## フォルダ構成

```
hardware/jlcpcb/
├── gerbers/    # Gerberファイル（ZIP化してアップロード）
├── bom/        # Bill of Materials（部品リスト）
└── cpl/        # Component Placement List（実装座標）
```
