#pragma once
#include <Arduino.h>
#include "Expander.h"

// エクスパンサ経由。

class Deployer {
public:
    void begin(Expander& expander);  // expanderはsetup()済みのものを渡す

    void deployRocket();  // ロケットからの分離（ニクロム線でテグス溶断）
    void deployParachute();  // パラシュートの分離（ニクロム線でテグス溶断）

private:
    Expander* _expander = nullptr;
};
