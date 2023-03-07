/****************************************************************************
 * configParser.c
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#include <string.h>
#include <stdlib.h>

#include "configParser.h"

#define MAX_BUFFER_LENGTH 32

#define MAX(a,b) (((a) > (b)) ? (a) : (b))

#define MIN(a,b) (((a) < (b)) ? (a) : (b))

/* Structure to maintain state */

typedef struct {
    uint8_t state;
    uint8_t index;
    uint8_t count;
    char buffer[MAX_BUFFER_LENGTH];
    CP_parserStatus_t status;
} CP_parserState_t;

/* Time constants */

#define MINUTES_IN_HOUR                 60
#define MINUTES_IN_DAY                  1440
#define SECONDS_IN_DAY                  (1440 * 60)

#define MONTH_JAN                       1
#define MONTH_FEB                       2
#define MONTH_DEC                       12

#define DAYS_IN_YEAR                    365
#define UNIX_EPOCH_START                1970

#define MAXIMUM_TRIGGER_DURATION        60

#define MINIMUM_SLEEP_DURATION          5
#define MAXIMUM_SLEEP_DURATION          43200

#define MINIMUM_RECORD_DURATION         1
#define MAXIMUM_RECORD_DURATION         43200

/* Macro definitions for updating state */

#define INC_STATE state->state += 1
#define ZERO_STATE state->state = 0
#define SET_STATE(X) state->state = X
#define OFFSET_STATE(X) state->state += X

#define INDEX state->index
#define INC_INDEX state->index += 1
#define ZERO_INDEX state->index = 0

#define COUNT state->count
#define INC_COUNT state->count += 1
#define ZERO_COUNT state->count = 0

#define BUFFER state->buffer
#define ADD_TO_BUFFER if (state->count < MAX_BUFFER_LENGTH - 1) {state->buffer[state->count] = c; state->count += 1;}
#define CLEAR_BUFFER memset(state->buffer, 0, MAX_BUFFER_LENGTH); state->count = 0

#define CLEAR_STATE memset(state, 0, sizeof(CP_parserState_t))

#define SET_STATUS_SUCCESS state->status = CP_SUCCESS

/* Private macro definition for defining jump table functions */

#define _FUNCTION_START(NAME, NUMBER) \
void NAME ## NUMBER (char c, CP_parserState_t *state, CP_configSettings_t *configSettings) {if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c > 127) {} else

#define _FUNCTION_END(STATE, STATUS) \
{state->state = STATE; state->status = STATUS;} }

/* Macro definitions for defining jump table functions */

#define DEFINE_FUNCTION_INIT(NAME, NUMBER, CONDITION) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION) {CLEAR_STATE, state->state = 1; state->status = CP_PARSING;} else _FUNCTION_END(0, CP_WAITING)

#define DEFINE_FUNCTION_STEP(NAME, NUMBER, CONDITION, ACTION) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION) {ACTION;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_ELSE(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_CND3(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2, CONDITION3, ACTION3) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else if (CONDITION3) {ACTION3;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_CND4(NAME, NUMBER, CONDITION1, ACTION1, CONDITION2, ACTION2, CONDITION3, ACTION3, CONDITION4, ACTION4) \
_FUNCTION_START(NAME, NUMBER) if (CONDITION1) {ACTION1;} else if (CONDITION2) {ACTION2;} else if (CONDITION3) {ACTION3;} else if (CONDITION4) {ACTION4;} else _FUNCTION_END(0, CP_CHARACTER_ERROR)

#define DEFINE_FUNCTION_STRG(NAME, NUMBER, STRING, ACTION) \
_FUNCTION_START(NAME, NUMBER) {char* pattern = STRING; uint32_t length = strlen(pattern); if (c == pattern[COUNT]) {INC_COUNT; if (COUNT == length) {ACTION;}} else _FUNCTION_END(0, CP_CHARACTER_ERROR)}

/* Macro definitions for error cases */

#define VALUE_ERROR \
state->state = 0; state->status = CP_VALUE_ERROR

/* Macro definitions for various character combinations */

#define IS(X) (c == X)

#define VALUE (c - '0')

#define ISDIGIT ('0' <= c && c <= '9')

#define ISNUMBER (ISDIGIT || IS('-'))

/* Custom macro to handle configuration settings */

#define CHECK_BUFFER_MIN_MAX_AND_SET(destination, min, max, success) { \
    int value = atoi(BUFFER); \
    if (value < min || value > max) { \
        VALUE_ERROR; \
    } else { \
        destination = value; \
        success; \
    } \
}

/* Custom variables */

static uint32_t day, month, year, timestamp;

static const uint32_t daysInMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/* Custom function to handle configuration settings */

static inline bool checkStartStopPeriod(CP_configSettings_t *configSettings, uint32_t index) {

    if (configSettings->startStopPeriods[index].startMinutes > MINUTES_IN_DAY - 1) return false;

    if (configSettings->startStopPeriods[index].stopMinutes > MINUTES_IN_DAY) return false;

    if (configSettings->startStopPeriods[index].startMinutes >= configSettings->startStopPeriods[index].stopMinutes) return false;

    if (index > 0 && configSettings->startStopPeriods[index - 1].stopMinutes >= configSettings->startStopPeriods[index].startMinutes) return false;

    return true;

}

static inline bool isLeapYear(uint16_t year) {

    return (year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0);

}

static inline bool checkDate(uint32_t day, uint32_t month, uint32_t year) {

    timestamp = 0;

    if (month < MONTH_JAN || month > MONTH_DEC) return false;

    uint32_t dayInCurrentMonth =  daysInMonth[month - 1] + (month == MONTH_FEB && isLeapYear(year) ? 1 : 0);

    if (day == 0 || day > dayInCurrentMonth) return false;

    for (uint32_t y = UNIX_EPOCH_START; y < year; y += 1) timestamp += SECONDS_IN_DAY * (isLeapYear(y) ? DAYS_IN_YEAR + 1 : DAYS_IN_YEAR);

    for (uint32_t m = 0; m < month - 1; m += 1) timestamp += SECONDS_IN_DAY * daysInMonth[m];

    if (isLeapYear(year) && month > MONTH_FEB) timestamp += SECONDS_IN_DAY;

    timestamp += SECONDS_IN_DAY * (day - 1);

    return true;

}

/* Define jump table functions for configuration settings */
/* Must use CLEAR_BUFFER before DEFINE_FUNCTION_STRG or ADD_TO_BUFFER functionality */

DEFINE_FUNCTION_INIT(CP, 00, IS('{'))
DEFINE_FUNCTION_STRG(CP, 01, "enableLED:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 02, IS('0') || IS('1'), configSettings->enableLED = VALUE; INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 03, ",enableMagneticSwitch:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 04, IS('0') || IS('1'), configSettings->enableMagneticSwitch = VALUE; INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 05, ",enableTimeSettingFromGPS:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 06, IS('0') || IS('1'), configSettings->enableTimeSettingFromGPS = VALUE; INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 07, ",enableEdgeImpulseModel:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 08, IS('0') || IS('1'), configSettings->enableEdgeImpulseModel = VALUE; INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 09, ",batteryLevelDisplayType:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 10, IS('0') || IS('1') || IS('2'), configSettings->batteryLevelDisplayType = VALUE; INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 11, ",recordingPeriods:[", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 12, "{startTime:\"", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 13, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 14, ISDIGIT, ADD_TO_BUFFER; configSettings->startStopPeriods[INDEX].startMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 15, IS(':'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 16, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 17, ISDIGIT, ADD_TO_BUFFER; configSettings->startStopPeriods[INDEX].startMinutes += atoi(BUFFER); INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 18, "\",stopTime:\"", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 19, IS('0') || IS('1') || IS('2'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 20, ISDIGIT, ADD_TO_BUFFER; configSettings->startStopPeriods[INDEX].stopMinutes = MINUTES_IN_HOUR * atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 21, IS(':'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 22, IS('0') || IS('1') || IS('2') || IS('3') || IS('4') || IS('5'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 23, ISDIGIT, ADD_TO_BUFFER; configSettings->startStopPeriods[INDEX].stopMinutes += atoi(BUFFER); if (checkStartStopPeriod(configSettings, INDEX)) {INC_STATE; CLEAR_BUFFER;} else {VALUE_ERROR;})
DEFINE_FUNCTION_STRG(CP, 24, "\"}", INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 25, IS(',') && INDEX < (CP_MAX_START_STOP_PERIODS - 1), INC_INDEX; SET_STATE(12); CLEAR_BUFFER, IS(']'), configSettings->activeStartStopPeriods = INDEX + 1; INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 26, IS(','), INC_STATE, IS('}'), SET_STATUS_SUCCESS)
DEFINE_FUNCTION_CND4(CP, 27, IS('s'), INC_STATE; CLEAR_BUFFER, IS('m'), SET_STATE(34); CLEAR_BUFFER, IS('f'), SET_STATE(37); CLEAR_BUFFER, IS('l'), SET_STATE(49); CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 28, "leepRecordCycle:{sleepDuration:", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_ELSE(CP, 29, ISDIGIT, ADD_TO_BUFFER, IS(','), CHECK_BUFFER_MIN_MAX_AND_SET(configSettings->sleepDuration, MINIMUM_SLEEP_DURATION, MAXIMUM_SLEEP_DURATION, INC_STATE; CLEAR_BUFFER))
DEFINE_FUNCTION_STRG(CP, 30, "recordDuration:", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_ELSE(CP, 31, ISDIGIT, ADD_TO_BUFFER, IS('}'), CHECK_BUFFER_MIN_MAX_AND_SET(configSettings->recordDuration, MINIMUM_RECORD_DURATION, MAXIMUM_RECORD_DURATION, configSettings->enableSleepRecordCycle = true; INC_STATE; CLEAR_BUFFER))
DEFINE_FUNCTION_ELSE(CP, 32, IS(','), INC_STATE, IS('}'), SET_STATUS_SUCCESS)
DEFINE_FUNCTION_CND3(CP, 33, IS('m'), INC_STATE; CLEAR_BUFFER, IS('f'), SET_STATE(37); CLEAR_BUFFER, IS('l'), SET_STATE(49); CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 34, "inimumTriggerDuration:", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_CND3(CP, 35, ISDIGIT, ADD_TO_BUFFER, IS(','), CHECK_BUFFER_MIN_MAX_AND_SET(configSettings->minimumTriggerDuration, 0, MAXIMUM_TRIGGER_DURATION, INC_STATE; CLEAR_BUFFER), IS('}'), CHECK_BUFFER_MIN_MAX_AND_SET(configSettings->minimumTriggerDuration, 0, MAXIMUM_TRIGGER_DURATION, SET_STATUS_SUCCESS))
DEFINE_FUNCTION_ELSE(CP, 36, IS('f'), INC_STATE; CLEAR_BUFFER, IS('l'), SET_STATE(49); CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 37, "irstRecordingDate:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 38, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 39, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 40, ISDIGIT, ADD_TO_BUFFER; day = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 41, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 42, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 43, ISDIGIT, ADD_TO_BUFFER; month = atoi(BUFFER); INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 44, "/202", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 45, ISDIGIT, year = 2020 + VALUE; if (checkDate(day, month, year)) {configSettings->earliestRecordingTime = timestamp; INC_STATE;} else {VALUE_ERROR;})
DEFINE_FUNCTION_STEP(CP, 46, IS('\"'), INC_STATE)
DEFINE_FUNCTION_ELSE(CP, 47, IS(','), INC_STATE, IS('}'), SET_STATUS_SUCCESS)
DEFINE_FUNCTION_STEP(CP, 48, IS('l'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 49, "astRecordingDate:", INC_STATE)
DEFINE_FUNCTION_STEP(CP, 50, IS('\"'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 51, IS('0') || IS('1') || IS('2') || IS('3'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 52, ISDIGIT, ADD_TO_BUFFER; day = atoi(BUFFER); INC_STATE)
DEFINE_FUNCTION_STEP(CP, 53, IS('/'), INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 54, IS('0') || IS('1'), ADD_TO_BUFFER; INC_STATE)
DEFINE_FUNCTION_STEP(CP, 55, ISDIGIT, ADD_TO_BUFFER; month = atoi(BUFFER); INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STRG(CP, 56, "/202", INC_STATE; CLEAR_BUFFER)
DEFINE_FUNCTION_STEP(CP, 57, ISDIGIT, year = 2020 + VALUE; if (checkDate(day, month, year)) {configSettings->latestRecordingTime = timestamp + SECONDS_IN_DAY; INC_STATE;} else {VALUE_ERROR;})
DEFINE_FUNCTION_STEP(CP, 58, IS('\"'), INC_STATE)
DEFINE_FUNCTION_STEP(CP, 59, IS('}'), SET_STATUS_SUCCESS)

static void (*const CPfunctions[])(char, CP_parserState_t*, CP_configSettings_t*) = {CP00, CP01, CP02, CP03, CP04, CP05, CP06, CP07, \
                                                                                     CP08, CP09, CP10, CP11, CP12, CP13, CP14, CP15, \
                                                                                     CP16, CP17, CP18, CP19, CP20, CP21, CP22, CP23, \
                                                                                     CP24, CP25, CP26, CP27, CP28, CP29, CP30, CP31, \
                                                                                     CP32, CP33, CP34, CP35, CP36, CP37, CP38, CP39, \
                                                                                     CP40, CP41, CP42, CP43, CP44, CP45, CP46, CP47, \
                                                                                     CP48, CP49, CP50, CP51, CP52, CP53, CP54, CP55, \
                                                                                     CP56, CP57, CP58, CP59};

/* Define parser */

static CP_parserState_t parserState;

void ConfigParser_reset() {

    memset(&parserState, 0, sizeof(CP_parserState_t));

}

CP_parserStatus_t ConfigParser_parse(char c, CP_configSettings_t *settings) {

    CPfunctions[parserState.state](c, &parserState, settings);

    return parserState.status;

}