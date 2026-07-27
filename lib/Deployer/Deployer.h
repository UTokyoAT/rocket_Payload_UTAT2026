#pragma once
#include <Arduino.h>

// XIAO1側。ロケットからの分離・パラシュートの分離はいずれもニクロム線でテグスを
// 溶断することで行う。回収支援用のLEDもXIAO1にのみGPIOの空きがあるためこちらに置く。
class Deployer {
public:
    void begin();

    void deployRocket();            // ロケットからの分離（ニクロム線でテグス溶断）
    void deployParachute();         // パラシュートの分離（ニクロム線でテグス溶断）

    void setLED(bool on);
    void beepPattern();             // 着地後の回収支援パターン（LED点滅）
};
