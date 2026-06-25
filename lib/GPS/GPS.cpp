#include "GPS.h"
#include <TinyGPSPlus.h>

static TinyGPSPlus _gps;
static HardwareSerial _serial(1);

void GPS::begin(int rxPin, int txPin, uint32_t baud) {
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void GPS::update() {
    while (_serial.available())
        _gps.encode(_serial.read());
}

double GPS::getLat()      { return _gps.location.lat(); }
double GPS::getLon()      { return _gps.location.lng(); }
float  GPS::getAltitude() { return (float)_gps.altitude.meters(); }
bool   GPS::isValid()     { return _gps.location.isValid(); }
