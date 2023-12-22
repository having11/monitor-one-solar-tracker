#include "Particle.h"
#include "edge.h"
#include "constants.h"
#include <time.h>
#include <chrono>

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
void handleWakeCallback(EdgeSleepContext context);
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

    Log.info("SENSORS INITIALIZED");

    homeMotors();

    double angleConstraints[4];
    getMinMaxAngles(angleConstraints);
    minTilt = angleConstraints[0];
    maxTilt = angleConstraints[1];
    minRoll = angleConstraints[2];
    maxRoll = angleConstraints[3];
    getCurrentAngles(&currentTilt, &currentRoll);
}

void loop()
{
    Edge::instance().loop();
}

void handleWakeCallback(EdgeSleepContext context) {
    Serial.printf("Moving to new angles\n");
    double newTilt, newRoll;

    sunAngles(&newTilt, &newRoll);

    moveToAngles(newTilt, newRoll);

    delay(3000);

    getCurrentAngles(&currentTilt, &currentRoll);
    Serial.printf("Now at tilt=%f, roll=%f\n", currentTilt, currentRoll);
}

void myLocationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
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
    constexpr double tzOffset = -5;

    LocationPoint loc;
    Edge::instance().locationService.getLocation(loc);
    std::tm currentTime = *gmtime(&loc.epochTime);
    int jan1Days = daysSinceJan1(currentTime);

    double t1 = (360.0 / 365.24) * (jan1Days - 2);
    double t2 = (360 / M_PI) * 0.0167 * sin(t1);
    double t3 = (360 / 365.24) * (jan1Days + 10);
    double t4 = cos(t3 + t2) * sin(degToRad(-23.44));
    double declination = asin(t4);

    // https://solarsena.wpengine.com/solar-hour-angle-calculator-formula/
    double currentLocalTimeMins = 60 * (currentTime.tm_hour - tzOffset) + currentTime.tm_min;
    // TODO: Handle when this value is negative
    Serial.printf("Local time mins=%f\n", currentLocalTimeMins);
    double solarHourAngle = (currentLocalTimeMins / 4) - 180;
    Serial.printf("Solar hour angle=%f\n", solarHourAngle);

    t1 = (327 - 1) + ((15.5 - 12) / 24);
    t2 = (2 * M_PI) / 365;
    double fractionalYearRadians = t1 * t2;

    t1 = -0.040849 * sin(2 * fractionalYearRadians);
    t2 = -0.014615 * cos(2 * fractionalYearRadians);
    t3 = -0.032077 * sin(fractionalYearRadians);
    t4 = 0.001868 * cos(fractionalYearRadians);
    double t5 = 0.000075 + t4;
    double timeEquation = 229.18 * t5;

    Serial.printf("Time equation=%f\n", timeEquation);

    // longitude; west = negative
    double latitude = loc.latitude;
    Serial.printf("Latitude=%f\n", latitude);
    double offset = timeEquation + (4 * (latitude - (15 * tzOffset)));

    double correctedLocalTimeHours = (currentLocalTimeMins / 60) + (offset / 60);
    Serial.printf("Corrected local time hours=%f\n", correctedLocalTimeHours);
    double correctedSolarAngle = 15 * (correctedLocalTimeHours - 12);
    Serial.printf("Corrected solar angle=%f\n", correctedSolarAngle);

    // https://solarsena.com/solar-elevation-angle-altitude/
    t1 = cos(latitude) * cos(declination) * cos(correctedSolarAngle);
    t2 = sin(latitude) * sin(declination);
    double solarElevationAngleRadians = asin(t1 + t2);
    Serial.printf("Solar elevation angle degrees=%f\n", radToDeg(solarElevationAngleRadians));

    // Also the tilt angle
    double solarZenithAngle = degToRad(90) - solarElevationAngleRadians;
    Serial.printf("Solar zenith angle degrees=%f\n", radToDeg(solarZenithAngle));
    *zenithDeg = radToDeg(solarZenithAngle);

    double azimuth;
    if (solarElevationAngleRadians > 0) {
        t1 = cos(latitude) * sin(solarZenithAngle);
        t2 = sin(declination);
        t3 = sin(latitude) * cos(solarZenithAngle);
        t4 = acos((t3 - t2) / t1);
        azimuth = ((int)radToDeg(t4) + 180) % 360;
    } else {
        t1 = cos(latitude) * sin(solarZenithAngle);
        t2 = sin(latitude) * cos(solarZenithAngle);
        t3 = (t2 - sin(declination)) / t1;
        azimuth = (540 - (int)radToDeg(acos(t3))) % 360;
    }

    Serial.printf("Azimuth degrees=%f\n", azimuth);
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