#ifndef _BLEFINGERPRINT_
#define _BLEFINGERPRINT_

#include "GUI.h"
#include "rssi.h"
#include "strings.h"
#include <ArduinoJson.h>
#include <NimBLEAdvertisedDevice.h>
#include <NimBLEBeacon.h>
#include <NimBLEDevice.h>
#include <NimBLEEddystoneTLM.h>
#include <NimBLEEddystoneURL.h>
#include <SoftFilters.h>

#define NO_RSSI (-32768)

#define ID_TYPE_TX_POW short(1)
#define ID_TYPE_MISC_APPLE short(-5)

#define ID_TYPE_MAC short(0)
#define ID_TYPE_AD short(10)
#define ID_TYPE_SD short(15)
#define ID_TYPE_MD short(20)
#define ID_TYPE_MISC short(30)
#define ID_TYPE_NAME short(35)
#define ID_TYPE_PUBLIC_MAC short(50)
#define ID_TYPE_MSFT short(100)
#define ID_TYPE_SONOS short(105)
#define ID_TYPE_GARMIN short(107)
#define ID_TYPE_MITHERM short(110)
#define ID_TYPE_MIFIT short(115)
#define ID_TYPE_EXPOSURE short(120)
#define ID_TYPE_ITAG short(125)
#define ID_TYPE_TRACKR short(130)
#define ID_TYPE_TILE short( 135)
#define ID_TYPE_MEATER short(140)
#define ID_TYPE_VANMOOF short(145)
#define ID_TYPE_SMARTTAG short(146)
#define ID_TYPE_APPLE_NEARBY short(150)
#define ID_TYPE_QUERY_MODEL short(155)
#define ID_TYPE_QUERY_NAME short(160)
#define ID_TYPE_EBEACON short(165)
#define ID_TYPE_ABEACON short(170)
#define ID_TYPE_IBEACON short(175)
#define ID_TYPE_RM_ASST short(180)
#define ID_TYPE_KNOWN_MAC short(185)

class BleFingerprintCollection;

class BleFingerprint
{

public:
    BleFingerprint(const BleFingerprintCollection *parent, NimBLEAdvertisedDevice *advertisedDevice, float fcmin, float beta, float dcutoff);

    bool seen(BLEAdvertisedDevice *advertisedDevice);

    bool report(JsonDocument *doc);

    bool query();

    String getId() { return id; }

    bool setId(const String &newId, short int newIdType, const String &newName = "");

    void setInitial(int rssi, float distance);

    String getMac() const { return SMacf(address); }

    String const getDiscriminator() { return disc; }

    float getDistance() const { return output.value.position; }

    int getRssi() const { return rssi; }

    int getNewestRssi() const { return newest; }

    int get1mRssi() const;

    NimBLEAddress const getAddress() { return address; }

    unsigned long getMsSinceLastSeen() const { return millis() - lastSeenMillis; };

    unsigned long getMsSinceFirstSeen() const { return millis() - firstSeenMillis; };

    bool getAdded() const { return added; };

    bool getIgnore() const { return ignore; };

    bool getAllowQuery() const { return allowQuery; };

    bool getRmAsst() const { return rmAsst; };

    unsigned int getSeenCount()
    {
        auto sc = seenCount - lastSeenCount;
        lastSeenCount = seenCount;
        return sc;
    }

    bool shouldCount();

private:

    static bool shouldHide(const String &s);

    bool hasValue = false, added = false, close = false, reported = false, ignore = false, allowQuery = false, didQuery = false, rmAsst = false, hidden = false, connectable = false, countable = false, counting = false;
    NimBLEAddress address;
    String id, name, disc;
    short int idType = 0;
    int rssi = -100, calRssi = NO_RSSI, mdRssi = NO_RSSI, asRssi = NO_RSSI, newest = NO_RSSI, recent = NO_RSSI, oldest = NO_RSSI;
    unsigned int qryAttempts = 0, qryDelayMillis = 0;
    float raw = 0, lastReported = 0, temp = 0, humidity = 0;
    unsigned long firstSeenMillis, lastSeenMillis = 0, lastReportedMillis = 0, lastQryMillis = 0;
    unsigned long seenCount = 1, lastSeenCount = 0;
    uint16_t mv = 0;
    uint8_t battery = 0xFF;

    Reading<Differential<float>> output;

    OneEuroFilter<float, unsigned long> oneEuro;
    DifferentialFilter<float, unsigned long> diffFilter;

    bool filter();

    void fingerprint(NimBLEAdvertisedDevice *advertisedDevice);

    void fingerprintServiceAdvertisements(NimBLEAdvertisedDevice *advertisedDevice, size_t serviceAdvCount, bool haveTxPower, int8_t txPower);

    void fingerprintServiceData(NimBLEAdvertisedDevice *advertisedDevice, size_t serviceDataCount, bool haveTxPower, int8_t txPower);

    void fingerprintManufactureData(NimBLEAdvertisedDevice *advertisedDevice, bool haveTxPower, int8_t txPower);
};

#endif
