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

float GPS::bearingTo(double lat, double lon) {
    return (float)TinyGPSPlus::courseTo(getLat(), getLon(), lat, lon);
}

float GPS::distanceTo(double lat, double lon) {
    return (float)TinyGPSPlus::distanceBetween(getLat(), getLon(), lat, lon);
}

uint32_t GPS::charsProcessed()      { return _gps.charsProcessed(); }
uint32_t GPS::failedChecksumCount() { return _gps.failedChecksum(); }
int      GPS::satellites()          { return _gps.satellites.isValid() ? _gps.satellites.value() : 0; }
float    GPS::hdop()                { return _gps.hdop.isValid() ? (float)_gps.hdop.hdop() : 0.0f; }
