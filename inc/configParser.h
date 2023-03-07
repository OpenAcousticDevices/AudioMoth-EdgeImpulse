/****************************************************************************
 * configParser.h
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include <stdint.h>
#include <stdbool.h>

#define CP_MAX_START_STOP_PERIODS 5

/* Parsing state enum */

typedef enum {CP_WAITING, CP_PARSING, CP_CHARACTER_ERROR, CP_VALUE_ERROR, CP_SUCCESS} CP_parserStatus_t;

/* Configuration structure and enum */

typedef enum {NONE, BATTERY_LEVEL, NIMH_LIPO_BATTERY_VOLTAGE} CP_batteryLevelDisplayType_t;

typedef struct {
    uint16_t startMinutes;
    uint16_t stopMinutes;
} CP_startStopPeriod_t;

typedef struct {
    int8_t timezoneHours;
    int8_t timezoneMinutes;
    uint8_t enableLED : 1;
    uint8_t enableMagneticSwitch : 1;
    uint8_t enableEdgeImpulseModel : 1;
    uint8_t enableSleepRecordCycle : 1;
    uint8_t enableTimeSettingFromGPS : 1;
    CP_batteryLevelDisplayType_t batteryLevelDisplayType : 2;
    uint16_t sleepDuration;
    uint16_t recordDuration;
    uint8_t minimumTriggerDuration;
    uint8_t activeStartStopPeriods;
    CP_startStopPeriod_t startStopPeriods[CP_MAX_START_STOP_PERIODS];
    uint32_t earliestRecordingTime;
    uint32_t latestRecordingTime;
} CP_configSettings_t;

void ConfigParser_reset();

CP_parserStatus_t ConfigParser_parse(char c, CP_configSettings_t *result);

#endif /* CONFIGPARSER_H */
