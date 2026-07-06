#include "SpiLinkSlave.h"
#include <ESP32SPISlave.h>
#include <cstring>

SpiLinkSlave::SpiLinkSlave() : _slave(new ESP32SPISlave()) {}

SpiLinkSlave::~SpiLinkSlave() {
    delete _slave;
}

void SpiLinkSlave::begin() {
    _slave->setDataMode(SPI_MODE0);
    _slave->setQueueSize(1);
    _slave->begin(FSPI, SpiPins::SCK, SpiPins::MISO, SpiPins::MOSI, SpiPins::CS);
    _slave->queue(_txBuf, _rxBuf, SPI_FRAME_SIZE);
}

bool SpiLinkSlave::poll(SpiFrameToXiao2& out) {
    if (!_slave->hasTransactionsCompletedAndAllResultsReady(1)) {
        return false;
    }
    memcpy(&out, _rxBuf, sizeof(out));
    _slave->queue(_txBuf, _rxBuf, SPI_FRAME_SIZE);  // 次のトランザクションを予約
    return true;
}

void SpiLinkSlave::setResponse(const SpiFrameFromXiao2& in) {
    memcpy(_txBuf, &in, sizeof(in));
}
