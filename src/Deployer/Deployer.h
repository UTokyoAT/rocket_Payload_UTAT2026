#pragma once
#include <Arduino.h>
#include "Expander.h"

// エクスパンサ経由。

class Deployer {
public:
    void begin(Expander& expander);  // expanderはsetup()済みのものを渡す

    void deployRocket();  // ロケットからの分離（ニクロム線でテグス溶断）
    void deployParachute();  // パラシュートの分離（ニクロム線でテグス溶断）

    // 通電のON/OFFだけを切り替える（通電時間を呼び出し側で管理したい場合用）。書き込み失敗でfalse
    bool setRocket(bool on);
    bool setParachute(bool on);

private:
    Expander* _expander = nullptr;
};
