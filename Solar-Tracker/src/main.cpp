#include "Particle.h"
#include "edge.h"
#include "constants.h"
#include <time.h>

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#if EDGE_PRODUCT_NEEDED
PRODUCT_ID(EDGE_PRODUCT_ID);
#endif // EDGE_PRODUCT_NEEDED
PRODUCT_VERSION(EDGE_PRODUCT_VERSION);

STARTUP(
    Edge::startup();
);

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO, {
    { "app.gps.nmea", LOG_LEVEL_INFO },
    // { "app.gps.ubx",  LOG_LEVEL_INFO },
    { "ncp.at", LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
});

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);
void handleWakeCallback(EdgeSleepContext context);
void printCurrentSun(void);
void sendCommand(ControllerCommand_t*);
void requestData(ControllerReceivedData_t*, int);
void endTransmission(void);
void homeMotors(void);
ControllerReceivedData_t getCurrentAngles(double *tilt, double *roll);
/// @brief 
/// @param angles Min tilt, max tilt, min roll, max roll
ControllerReceivedData_t getMinMaxAngles(double angles[4]);
void moveToAngles(double tilt, double roll);
void sunAngles(double *zenithDeg, double *azimuthDeg);

double degToRad(double);
double radToDeg(double);
int daysSinceJan1(std::tm &currentTime);

static double currentTilt;
static double currentRoll;

static double minTilt;
static double maxTilt;
static double minRoll;
static double maxRoll;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Edge::instance().init();

    TrackerLocation::instance().regLocGenCallback(myLocationGenerationCallback);
    TrackerSleep::instance().registerWake(handleWakeCallback);

    Log.info("INITIALIZED");

    double angleConstraints[4];
    getMinMaxAngles(angleConstraints);
    minTilt = angleConstraints[0];
    maxTilt = angleConstraints[1];
    minRoll = angleConstraints[2];
    maxRoll = angleConstraints[3];
    getCurrentAngles(&currentTilt, &currentRoll);

    homeMotors();
}

void loop()
{
    Edge::instance().loop();
}

void handleWakeCallback(EdgeSleepContext context) {
    if (context.reason != EdgeSleepReason::WAKE) return;

    // Could move here too
    // Log.info("Moving to new angles");
    // double newTilt, newRoll;

    // sunAngles(&newTilt, &newRoll);

    // moveToAngles(newTilt, newRoll);

    // delay(3000);

    // getCurrentAngles(&currentTilt, &currentRoll);
    // Log.info("Now at tilt=%f, roll=%f", currentTilt, currentRoll);
}

void printCurrentSun() {
    Log.info("Getting new angles");
    double newTilt, newRoll;
    sunAngles(&newTilt, &newRoll);
}

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    double newTilt, newRoll;
    sunAngles(&newTilt, &newRoll);
    moveToAngles(newTilt, newRoll);
    delay(3000);

    getCurrentAngles(&currentTilt, &currentRoll);
    Log.info("Now at tilt=%f, roll=%f", currentTilt, currentRoll);

    writer.name("solar").beginObject();
    writer.name("current_tilt").value(currentTilt);
    writer.name("current_roll").value(currentRoll);
    writer.endObject();
}

void sendCommand(ControllerCommand_t *command) {
    Wire.beginTransmission(MotorControllerAddress);
    Wire.write((uint8_t*)command, sizeof(ControllerCommand_t));
}

void requestData(ControllerReceivedData_t *data, int len) {
    Wire.requestFrom(MotorControllerAddress, len);
    Wire.readBytes((char*)data, len);
}

void endTransmission() {
    Wire.endTransmission();
}

void homeMotors() {
    ControllerCommand_t cmd = {
        .type = ControllerCommandType::HomeAxes
    };

    sendCommand(&cmd);
    endTransmission();
}

ControllerReceivedData_t getCurrentAngles(double *tilt, double *roll) {
    ControllerCommand_t cmd = {
        .type = ControllerCommandType::GetAngles
    };
    ControllerReceivedData_t rec;

    sendCommand(&cmd);
    requestData(&rec, sizeof(rec.data.currentAngles));
    endTransmission();

    *tilt = rec.data.currentAngles[0];
    *roll = rec.data.currentAngles[1];

    return rec;
}

ControllerReceivedData_t getMinMaxAngles(double angles[4]) {
    ControllerCommand_t cmd = {
        .type = ControllerCommandType::GetMinMaxAngles
    };
    ControllerReceivedData_t rec;

    sendCommand(&cmd);
    requestData(&rec, sizeof(rec.data.minMaxAngles));
    endTransmission();

    memcpy(angles, &rec.data.minMaxAngles, sizeof(rec.data.minMaxAngles));

    return rec;
}

void moveToAngles(double tilt, double roll) {
    ControllerCommand_t cmd = {
        .type = ControllerCommandType::GoToAngles,
        .data = {
            .targetAngles = { tilt, roll }
        }
    };

    sendCommand(&cmd);
    endTransmission();
}

void sunAngles(double *zenithDeg, double *azimuthDeg) {
    // timezone offset from UTC
    int tzOffset = -6;

    LocationPoint loc;
    EdgeGnssAbstraction::instance().getLocation(loc);
    std::tm *currentTime = gmtime(&loc.epochTime);
    Log.info("Epoch current time=%lld", loc.epochTime);
    int jan1Days = daysSinceJan1(*currentTime);
    Log.info("Days since Jan 1: %d", jan1Days);

    double t1 = (360.0 / 365.24) * (jan1Days - 2);
    double t2 = (360 / M_PI) * 0.0167 * sin(t1);
    double t3 = (360 / 365.24) * (jan1Days + 10);
    double t4 = cos(t3 + t2) * sin(degToRad(-23.44));
    double declination = asin(t4);

    // https://solarsena.wpengine.com/solar-hour-angle-calculator-formula/
    Log.info("Local time hour=%d, min=%d", currentTime->tm_hour - 6, currentTime->tm_min);
    int currentLocalTimeMins = (60 * (currentTime->tm_hour - 6)) + currentTime->tm_min;
    // TODO: Handle when this value is negative
    Log.info("Local time mins=%d, with offset=%d", currentLocalTimeMins, currentTime->tm_hour - 6);
    double solarHourAngle = (currentLocalTimeMins / 4.0) - 180;
    Log.info("Solar hour angle=%f", solarHourAngle);

    t1 = (jan1Days - 1) + (((currentLocalTimeMins / 60.0) - 12) / 24);
    t2 = (2 * M_PI) / 365;
    double fractionalYearRadians = t1 * t2;

    t1 = -0.040849 * sin(2 * fractionalYearRadians);
    t2 = -0.014615 * cos(2 * fractionalYearRadians);
    t3 = -0.032077 * sin(fractionalYearRadians);
    t4 = 0.001868 * cos(fractionalYearRadians);
    double t5 = 0.000075 + t4;
    double timeEquation = 229.18 * t5;

    Log.info("Time equation=%f", timeEquation);

    // longitude; west = negative
    double longitude = loc.longitude;
    Log.info("Longitude=%f", longitude);
    double offset = timeEquation + (4 * (longitude - (15 * 6)));

    double correctedLocalTimeHours = (currentLocalTimeMins / 60.0) + (offset / 60);
    Log.info("Corrected local time hours=%f", correctedLocalTimeHours);
    double correctedSolarAngle = 15 * (correctedLocalTimeHours - 12);
    Log.info("Corrected solar angle=%f", correctedSolarAngle);

    // https://solarsena.com/solar-elevation-angle-altitude/
    double latitude = loc.latitude;
    t1 = cos(latitude) * cos(declination) * cos(correctedSolarAngle);
    t2 = sin(latitude) * sin(declination);
    double solarElevationAngleRadians = asin(t1 + t2);
    Log.info("Solar elevation angle degrees=%f", radToDeg(solarElevationAngleRadians));

    // Also the tilt angle
    double solarZenithAngle = degToRad(90) - solarElevationAngleRadians;
    Log.info("Solar zenith angle degrees=%f", radToDeg(solarZenithAngle));
    *zenithDeg = radToDeg(solarZenithAngle);

    // https://en.wikipedia.org/wiki/Solar_azimuth_angle#Conventional_Trigonometric_Formulas
    double azimuth;
    t1 = cos(degToRad(solarHourAngle)) * cos(declination) * sin(latitude);
    t2 = sin(declination) * cos(latitude);
    t3 = t2 - t1;
    t4 = t3 / sin(solarZenithAngle);
    azimuth = 180 - radToDeg(acos(t4));

    Log.info("Azimuth degrees=%f", azimuth);
    *azimuthDeg = azimuth;
}

int daysSinceJan1(std::tm &currentTime) {
    std::tm jan1 = { 0 };
    jan1.tm_year = currentTime.tm_year;
    jan1.tm_mon = 0;
    jan1.tm_mday = 1;

    std::time_t timeStart = std::mktime(&jan1);
    std::time_t timeEnd = std::mktime(&currentTime);

    constexpr int secPerDay = 60 * 60 * 24;
    double timeDif = std::difftime(timeEnd, timeStart) / secPerDay;
    return (int)timeDif;
}

double degToRad(double degrees) {
    return degrees * (M_PI / 180.0);
}

double radToDeg(double radians) {
    return radians * (180.0 / M_PI);
}