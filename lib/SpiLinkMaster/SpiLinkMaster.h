#pragma once
#include <SPI.h>
#include "spi_protocol.h"

// XIAO1側（マスター）。XIAO2へ姿勢/位置/手動操作コマンドを送りつつ、
// 同じトランザクションでXIAO2が出力した実モータ値を受け取る（全二重）。
class SpiLinkMaster {
public:
    void begin();
    SpiFrameFromXiao2 transfer(const SpiFrameToXiao2& out);
};
