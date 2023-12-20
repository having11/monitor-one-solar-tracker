#include "Particle.h"
#include "edge.h"
#include "constants.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup();
);

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Edge::instance().init();

    TrackerLocation::instance().regLocGenCallback(myLocationGenerationCallback);

    Log.info("SENSORS INITIALIZED");
}

void loop()
{
    Edge::instance().loop();
}

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{

}

void sunAngles() {
    int daysSinceJan1 = 250;

    // TODO: -23.44 is in degrees but needs to be radians
    double t1 = (360.0 / 365.24) * (daysSinceJan1 - 2);
    double t2 = (360 / M_PI) * 0.0167 * sin(t1);
    double t3 = (360 / 365.24) * (daysSinceJan1 + 10);
    // TODO: convert to radians
    double t4 = cos(t3 + t2) * sin(-23.44);
    double declination = asin(t4);

    // https://solarsena.wpengine.com/solar-hour-angle-calculator-formula/
    constexpr double currentLocalTimeMins = 60 * 11 + 30;
    double solarHourAngle = (currentLocalTimeMins / 4) - 180;

    t1 = (327 - 1) + ((15.5 - 12) / 24);
    t2 = (2 * M_PI) / 365;
    double fractionalYearRadians = t1 * t2;

    t1 = -0.040849 * sin(2 * fractionalYearRadians);
    t2 = -0.014615 * cos(2 * fractionalYearRadians);
    t3 = -0.032077 * sin(fractionalYearRadians);
    t4 = 0.001868 * cos(fractionalYearRadians);
    double t5 = 0.000075 + t4;
    double timeEquation = 229.18 * t5;

    // longitude; west = negative
    double latitude = -118.24;
    // timezone offset from UTC
    double tzOffset = -8;
    double offset = timeEquation + (4 * (latitude - (15 * tzOffset)));

    double correctedLocalTimeHours = (currentLocalTimeMins / 60) + (offset / 60);
    double correctedSolarAngle = 15 * (correctedLocalTimeHours - 12);

    // https://solarsena.com/solar-elevation-angle-altitude/
    t1 = cos(latitude) * cos(declination) * cos(correctedSolarAngle);
    t2 = sin(latitude) * sin(declination);
    double solarElevationAngleRadians = asin(t1 + t2);

    // TODO: 90 is in degrees but needs to be in radians
    // Also the tilt angle
    double solarZenithAngle = 90 - solarElevationAngleRadians;

    double azimuth;
    if (solarElevationAngleRadians > 0) {
        t1 = cos(latitude) * sin(solarZenithAngle);
        t2 = sin(declination);
        t3 = sin(latitude) * cos(solarZenithAngle);
        t4 = acos((t3 - t2) / t1);
        // TODO: t4 MUST BE IN DEGREES instead of RADIANS
        azimuth = ((int)t4 + 180) % 360;
    } else {
        t1 = cos(latitude) * sin(solarZenithAngle);
        t2 = sin(latitude) * cos(solarZenithAngle);
        t3 = (t2 - sin(declination)) / t1;
        // TODO: convert acos(13) to degrees NOT CAST TO INT!
        azimuth = (540 - (int)acos(t3)) % 360;
    }
}