#include "SpiLinkMaster.h"
#include <cstring>

void SpiLinkMaster::begin() {
    pinMode(SpiPins::CS_MASTER, OUTPUT);
    digitalWrite(SpiPins::CS_MASTER, HIGH);
    SPI.begin(SpiPins::SCK, SpiPins::MISO, SpiPins::MOSI, SpiPins::CS_MASTER);
}

SpiFrameFromXiao2 SpiLinkMaster::transfer(const SpiFrameToXiao2& out) {
    uint8_t txBuf[SPI_FRAME_SIZE] = {0};
    uint8_t rxBuf[SPI_FRAME_SIZE] = {0};
    memcpy(txBuf, &out, sizeof(out));

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(SpiPins::CS_MASTER, LOW);
    SPI.transferBytes(txBuf, rxBuf, SPI_FRAME_SIZE);
    digitalWrite(SpiPins::CS_MASTER, HIGH);
    SPI.endTransaction();

    SpiFrameFromXiao2 in{};
    memcpy(&in, rxBuf, sizeof(in));
    return in;
}
