/****************************************************************************
 * main.c
 * openacousticdevices.info
 * March 2023
 *****************************************************************************/

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "audioMoth.h"
#include "configParser.h"
#include "digitalFilter.h"
#include "ei_main_loop.h"
#include "gps.h"

/* Edge Impulse constants */

#define KHZ_8                                   8000
#define KHZ_16                                  16000

#define EI_MODEL_FREQUENCY                      KHZ_16

#define DETECTION_THRESHOLD                     0.8f
#define EI_SIGNAL_LENGTH                        EI_MODEL_FREQUENCY

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                          (24 * SECONDS_IN_HOUR)

#define YEAR_OFFSET                             1900
#define MONTH_OFFSET                            1

/* Useful type constants */

#define BITS_PER_BYTE                           8
#define UINT32_SIZE_IN_BITS                     32
#define UINT32_SIZE_IN_BYTES                    4
#define UINT16_SIZE_IN_BYTES                    2

/* Sleep and LED constants */

#define LOW_BATTERY_LED_FLASHES                 10

#define SHORT_LED_FLASH_DURATION                100
#define LONG_LED_FLASH_DURATION                 500

#define WAITING_LED_FLASH_DURATION              10
#define WAITING_LED_FLASH_INTERVAL              2000

#define MINIMUM_LED_FLASH_INTERVAL              500

#define SHORT_WAIT_INTERVAL                     100
#define DEFAULT_WAIT_INTERVAL                   1000

/* SRAM buffer constants*/

#define NUMBER_OF_BUFFERS                       4
#define NUMBER_OF_BYTES_IN_SAMPLE               2

#define NUMBER_OF_SAMPLES_IN_BUFFER             (16 * 1024)
#define FREE_EXTERNAL_SRAM_START_ADDRESS        (AM_EXTERNAL_SRAM_START_ADDRESS + (NUMBER_OF_BYTES_IN_SAMPLE * NUMBER_OF_SAMPLES_IN_BUFFER * NUMBER_OF_BUFFERS))

/* DMA transfer constant */

#define MAXIMUM_SAMPLES_IN_DMA_TRANSFER         384

/* Compression constant */

#define COMPRESSION_BUFFER_SIZE_IN_BYTES        512

/* Temporary file buffer constant */

#define TEMP_FILE_BUFFER_SIZE_IN_BYTES          512

/* File size constants */

#define MAXIMUM_FILE_NAME_LENGTH                32

#define MAXIMUM_WAV_FILE_SIZE                   UINT32_MAX

/* WAV header constant */

#define PCM_FORMAT                              1
#define RIFF_ID_LENGTH                          4
#define LENGTH_OF_ARTIST                        32
#define LENGTH_OF_COMMENT                       384

/* Supply monitor constant */

#define MINIMUM_SUPPLY_VOLTAGE                  2800

/* GPS time setting constants */

#define GPS_MAXIMUM_MS_DIFFERENCE               (SECONDS_IN_HOUR * MILLISECONDS_IN_SECOND)
#define GPS_MAX_TIME_SETTING_PERIOD             300
#define GPS_MIN_TIME_SETTING_PERIOD             30
#define GPS_FREQUENCY_PRECISION                 1000
#define GPS_FILENAME                            "GPS.TXT"

/* Magnetic switch constants */

#define MAGNETIC_SWITCH_WAIT_MULTIPLIER         2
#define MAGNETIC_SWITCH_CHANGE_FLASHES          10

/* Recording preparation constants */

#define PREPARATION_PERIOD_INCREMENT            250
#define MINIMUM_PREPARATION_PERIOD              750
#define INITIAL_PREPARATION_PERIOD              2000
#define MAXIMUM_PREPARATION_PERIOD              30000

/* DC filter constants */

#define LOW_DC_BLOCKING_FREQ_GAIN               0.998432f
#define LOW_DC_BLOCKING_FREQ_CONSTANT           0.996863f

#define DEFAULT_DC_BLOCKING_FREQ_GAIN           0.990663f
#define DEFAULT_DC_BLOCKING_FREQ_CONSTANT       0.981326f

/* Configuration file constants */

#define CONFIG_MAX_FILE_READ_CHARACTERS         1024
#define CONFIG_FILE_READ_BUFFER_LENGTH          128

/* Recording and operation constants */

#define GAIN                                    AM_GAIN_MEDIUM
#define GAIN_RANGE                              AM_NORMAL_GAIN_RANGE

#define SAMPLE_RATE                             384000

#if EI_MODEL_FREQUENCY == KHZ_16
#define SAMPLE_RATE_DIVIDER                     24
#endif
#if EI_MODEL_FREQUENCY == KHZ_8
#define SAMPLE_RATE_DIVIDER                     48
#endif

#define ACQUISITION_SAMPLES                     16
#define OVER_SAMPLE_RATE                        1
#define CLOCK_DIVIDER                           4

#define ENABLE_LOW_VOLTAGE_CUTOFF               true
#define DISABLE_48HZ_DC_BLOCKING_FILTER         false

#define SHOW_GREEN_LED_DURING_COMPUTATION       true

/* Useful macros */

#define FLASH_LED(led, duration) { \
    AudioMoth_set ## led ## LED(true); \
    AudioMoth_delay(duration); \
    AudioMoth_set ## led ## LED(false); \
}

#define FLASH_REPEAT_LED(led, repeats, duration) { \
    for (uint32_t i = 0; i < repeats; i += 1) { \
        AudioMoth_set ## led ## LED(true); \
        AudioMoth_delay(duration); \
        AudioMoth_set ## led ## LED(false); \
        AudioMoth_delay(duration); \
    } \
}

#define FLASH_LED_AND_RETURN_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        FLASH_LED(Both, LONG_LED_FLASH_DURATION) \
        return SDCARD_WRITE_ERROR; \
    } \
}

#define RETURN_BOOL_ON_ERROR(fn) { \
    bool success = (fn); \
    if (success != true) { \
        return success; \
    } \
}

#define SAVE_SWITCH_POSITION_AND_POWER_DOWN(milliseconds) { \
    *previousSwitchPosition = switchPosition; \
    AudioMoth_powerDownAndWakeMilliseconds(milliseconds); \
}

#define SERIAL_NUMBER                           "%08X%08X"

#define FORMAT_SERIAL_NUMBER(src)               (unsigned int)*((uint32_t*)src + 1),  (unsigned int)*((uint32_t*)src)

#define ABS(a)                                  ((a) < (0) ? (-a) : (a))

#define MIN(a, b)                               ((a) < (b) ? (a) : (b))

#define MAX(a, b)                               ((a) > (b) ? (a) : (b))

#define ROUNDED_DIV(a, b)                       (((a) + (b/2)) / (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

#define ROUND_UP_TO_MULTIPLE(a, b)              (((a) + (b) - 1) & ~((b)-1))

/* Recording state enumeration */

typedef enum {RECORDING_OKAY, FILE_SIZE_LIMITED, SUPPLY_VOLTAGE_LOW, SWITCH_CHANGED, MICROPHONE_CHANGED, MAGNETIC_SWITCH, EI_MODEL_ERROR, SDCARD_WRITE_ERROR} AM_recordingState_t;

/* Filter type enumeration */

typedef enum {NO_FILTER, LOW_PASS_FILTER, BAND_PASS_FILTER, HIGH_PASS_FILTER} AM_filterType_t;

/* WAV header */

#pragma pack(push, 1)

typedef struct {
    char id[RIFF_ID_LENGTH];
    uint32_t size;
} chunk_t;

typedef struct {
    chunk_t icmt;
    char comment[LENGTH_OF_COMMENT];
} icmt_t;

typedef struct {
    chunk_t iart;
    char artist[LENGTH_OF_ARTIST];
} iart_t;

typedef struct {
    uint16_t format;
    uint16_t numberOfChannels;
    uint32_t samplesPerSecond;
    uint32_t bytesPerSecond;
    uint16_t bytesPerCapture;
    uint16_t bitsPerSample;
} wavFormat_t;

typedef struct {
    chunk_t riff;
    char format[RIFF_ID_LENGTH];
    chunk_t fmt;
    wavFormat_t wavFormat;
    chunk_t list;
    char info[RIFF_ID_LENGTH];
    icmt_t icmt;
    iart_t iart;
    chunk_t data;
} wavHeader_t;

#pragma pack(pop)

static const wavHeader_t wavHeader = {
    .riff = {.id = "RIFF", .size = 0},
    .format = "WAVE",
    .fmt = {.id = "fmt ", .size = sizeof(wavFormat_t)},
    .wavFormat = {.format = PCM_FORMAT, .numberOfChannels = 1, .samplesPerSecond = 0, .bytesPerSecond = 0, .bytesPerCapture = 2, .bitsPerSample = 16},
    .list = {.id = "LIST", .size = RIFF_ID_LENGTH + sizeof(icmt_t) + sizeof(iart_t)},
    .info = "INFO",
    .icmt = {.icmt.id = "ICMT", .icmt.size = LENGTH_OF_COMMENT, .comment = ""},
    .iart = {.iart.id = "IART", .iart.size = LENGTH_OF_ARTIST, .artist = ""},
    .data = {.id = "data", .size = 0}
};

/* Temporary buffers in SRAM */
typedef struct {
    wavHeader_t wavHeader;
    int16_t compressionBuffer[COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE];
    char filename[MAXIMUM_FILE_NAME_LENGTH];
    char foldername[MAXIMUM_FILE_NAME_LENGTH];
    char newFilename[MAXIMUM_FILE_NAME_LENGTH];
} tempBuffer_t;

/* Default configuration settings */

static CP_configSettings_t defaultConfigSettings;

/* Functions to set WAV header details and comment */

static void setHeaderDetails(wavHeader_t *wavHeader, uint32_t sampleRate, uint32_t numberOfSamples) {

    wavHeader->wavFormat.samplesPerSecond = sampleRate;
    wavHeader->wavFormat.bytesPerSecond = NUMBER_OF_BYTES_IN_SAMPLE * sampleRate;
    wavHeader->data.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples;
    wavHeader->riff.size = NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples + sizeof(wavHeader_t) - sizeof(chunk_t);

}

static void setHeaderComment(wavHeader_t *wavHeader, CP_configSettings_t *configSettings, uint32_t currentTime, uint8_t *serialNumber, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, bool externalMicrophone, AM_recordingState_t recordingState) {

    struct tm time;

    time_t rawTime = currentTime + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    gmtime_r(&rawTime, &time);

    /* Format artist field */

    char *artist = wavHeader->iart.artist;

    sprintf(artist, "AudioMoth " SERIAL_NUMBER, FORMAT_SERIAL_NUMBER(serialNumber));

    /* Format comment field */

    char *comment = wavHeader->icmt.comment;

    comment += sprintf(comment, "Recorded at %02d:%02d:%02d %02d/%02d/%04d (UTC", time.tm_hour, time.tm_min, time.tm_sec, time.tm_mday, MONTH_OFFSET + time.tm_mon, YEAR_OFFSET + time.tm_year);

    int8_t timezoneHours = configSettings->timezoneHours;

    int8_t timezoneMinutes = configSettings->timezoneMinutes;

    if (timezoneHours < 0) {

        comment += sprintf(comment, "%d", timezoneHours);

    } else if (timezoneHours > 0) {

        comment += sprintf(comment, "+%d", timezoneHours);

    } else {

        if (timezoneMinutes < 0) comment += sprintf(comment, "-%d", timezoneHours);

        if (timezoneMinutes > 0) comment += sprintf(comment, "+%d", timezoneHours);

    }

    if (timezoneMinutes < 0) comment += sprintf(comment, ":%02d", -timezoneMinutes);

    if (timezoneMinutes > 0) comment += sprintf(comment, ":%02d", timezoneMinutes);

    comment += sprintf(comment, ") by %s ", artist);

    if (externalMicrophone) {

        comment += sprintf(comment, "using external microphone ");

    }

    static char *gainSettings[5] = {"low", "low-medium", "medium", "medium-high", "high"};

    comment += sprintf(comment, "at %s gain while battery was ", gainSettings[GAIN]);

    if (extendedBatteryState == AM_EXT_BAT_LOW) {

        comment += sprintf(comment, "less than 2.5V");

    } else if (extendedBatteryState >= AM_EXT_BAT_FULL) {

        comment += sprintf(comment, "greater than 4.9V");

    } else {

        uint32_t batteryVoltage =  extendedBatteryState + AM_EXT_BAT_STATE_OFFSET / AM_BATTERY_STATE_INCREMENT;

        comment += sprintf(comment, "%01lu.%01luV", batteryVoltage / 10, batteryVoltage % 10);

    }

    char *sign = temperature < 0 ? "-" : "";

    uint32_t temperatureInDecidegrees = ROUNDED_DIV(ABS(temperature), 100);

    comment += sprintf(comment, " and temperature was %s%lu.%luC.", sign, temperatureInDecidegrees / 10, temperatureInDecidegrees % 10);
    
    if (configSettings->enableEdgeImpulseModel) {

        comment += sprintf(comment, " Edge Impulse model used with %us minimum trigger duration.", configSettings->minimumTriggerDuration);

    }

    if (recordingState != RECORDING_OKAY) {

        comment += sprintf(comment, " Recording stopped");

        if (recordingState == MICROPHONE_CHANGED) {

            comment += sprintf(comment, " due to microphone change.");

        } else if (recordingState == SWITCH_CHANGED) {

            comment += sprintf(comment, " due to switch position change.");

        } else if (recordingState == MAGNETIC_SWITCH) {
        
            comment += sprintf(comment, " by magnetic switch.");

        } else if (recordingState == SUPPLY_VOLTAGE_LOW) {

            comment += sprintf(comment, " due to low voltage.");

        } else if (recordingState == FILE_SIZE_LIMITED) {

            comment += sprintf(comment, " due to file size limit.");

        } else if (recordingState == EI_MODEL_ERROR) {

            comment += sprintf(comment, " due to Edge Impulse model error.");

        }
    }

}

/* Global variable to track original internal stack position - used to prevent Edge Impulse model from growing heap too large */

void *maxHeapAddress = (void *)0x1;

/* Global variable for the top of ERAM as defined in AudioMoth.ld */

const void *topERAM = (void *)(AM_EXTERNAL_SRAM_START_ADDRESS + AM_EXTERNAL_SRAM_SIZE_IN_BYTES);

/* Backup domain variables */

static uint32_t *previousSwitchPosition = (uint32_t*)AM_BACKUP_DOMAIN_START_ADDRESS;

static uint32_t *timeOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 4);

static uint32_t *durationOfNextRecording = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 8);

static uint32_t *timeOfNextGPSTimeSetting = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 12);

static uint32_t *writtenDeviceDataToFile = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 16);

static uint32_t *readyToMakeRecordings = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 20);

static uint32_t *shouldSetTimeFromGPS = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 24);

static uint32_t *recordingErrorHasOccurred = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 28);

static uint32_t *recordingPreparationPeriod = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 32);

static uint32_t *waitingForMagneticSwitch = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 36);

static uint32_t *poweredDownWithShortWaitInterval = (uint32_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 40);

static CP_configSettings_t *configSettings = (CP_configSettings_t*)(AM_BACKUP_DOMAIN_START_ADDRESS + 44);

/* DMA transfer variable */

static uint32_t numberOfRawSamplesInDMATransfer;

/* SRAM buffer variables */

static volatile uint32_t writeBuffer;

static volatile uint32_t writeBufferIndex;

static int16_t* buffers[NUMBER_OF_BUFFERS];

/* Flag to start processing DMA transfers */

static volatile uint32_t numberOfDMATransfers;

static volatile uint32_t numberOfDMATransfersToWait;

/* GPS fix variables */

static bool gpsEnableLED;

static bool gpsPPSEvent;

static bool gpsFixEvent;

static bool gpsMessageEvent;

static bool gpsFirstMessageReceived;

static uint32_t gpsTickEventCount = 1;

static uint32_t gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;

/* Recording state */

static volatile bool magneticSwitch;

static volatile bool microphoneChanged;

static volatile bool switchPositionChanged;

/* DMA buffers */

static int16_t primaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

static int16_t secondaryBuffer[MAXIMUM_SAMPLES_IN_DMA_TRANSFER];

/* Temporary file buffer */

static char *tempFileBuffer = (char*)primaryBuffer;

/* Firmware version and description */

static uint8_t firmwareVersion[AM_FIRMWARE_VERSION_LENGTH] = {1, 0, 0};

static uint8_t firmwareDescription[AM_FIRMWARE_DESCRIPTION_LENGTH] = "AudioMoth-NatGeoDeploy-v1";

/* Function prototypes */

static void flashLedToIndicateBatteryLife(void);

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *timeOfNextGPSTimeSetting);

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds);

/* Functions of copy to and from the backup domain */

static void copyToBackupDomain(uint32_t *dst, uint8_t *src, uint32_t length) {

    uint32_t value = 0;

    for (uint32_t i = 0; i < length / UINT32_SIZE_IN_BYTES; i += 1) {
        *(dst + i) = *((uint32_t*)src + i);
    }

    for (uint32_t i = 0; i < length % UINT32_SIZE_IN_BYTES; i += 1) {
        value = (value << BITS_PER_BYTE) + *(src + length - 1 - i);
    }

    if (length % UINT32_SIZE_IN_BYTES) *(dst + length / UINT32_SIZE_IN_BYTES) = value;

}

/* Function to write device data to file */

static bool writeDeviceDataToFile(uint8_t *firmwareDescription, uint8_t *firmwareVersion, uint8_t *serialNumber) {

    RETURN_BOOL_ON_ERROR(AudioMoth_openFile("DEVICE.TXT"));

    uint32_t length = sprintf(tempFileBuffer, "Device ID                       : " SERIAL_NUMBER "\n", FORMAT_SERIAL_NUMBER(serialNumber));

    length += sprintf(tempFileBuffer + length, "Firmware                        : %s (%u.%u.%u)\n", firmwareDescription, firmwareVersion[0], firmwareVersion[1], firmwareVersion[2]);

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(tempFileBuffer, length));

    RETURN_BOOL_ON_ERROR(AudioMoth_closeFile());

    return true;

}

/* GPS time setting functions */

static void writeGPSLogMessage(uint32_t currentTime, uint32_t currentMilliseconds, char *message) {

    struct tm time;

    time_t rawTime = currentTime;

    gmtime_r(&rawTime, &time);

    uint32_t length = snprintf(tempFileBuffer, MAXIMUM_SAMPLES_IN_DMA_TRANSFER, "%02d/%02d/%04d %02d:%02d:%02d.%03ld UTC: %s\n", time.tm_mday, MONTH_OFFSET + time.tm_mon, YEAR_OFFSET + time.tm_year, time.tm_hour, time.tm_min, time.tm_sec, currentMilliseconds, message);

    AudioMoth_writeToFile(tempFileBuffer, length);

}

static GPS_fixResult_t setTimeFromGPS(bool enableLED, uint32_t timeout) {

    uint32_t currentTime, currentMilliseconds;

    /* Enable GPS */

    GPS_powerUpGPS();

    GPS_enableGPSInterface();

    /* Enable GPS log file */

    bool success = AudioMoth_appendFile(GPS_FILENAME);

    /* Add power up message */

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    writeGPSLogMessage(currentTime, currentMilliseconds, "GPS powered up.");

    /* Set green LED and enter routine */

    gpsEnableLED = enableLED;

    if (gpsEnableLED) AudioMoth_setGreenLED(true);

    GPS_fixResult_t result = GPS_setTimeFromGPS(timeout);

    AudioMoth_setGreenLED(false);

    AudioMoth_setRedLED(false);

    /* Disable the GPS */

    GPS_disableGPSInterface();

    GPS_powerDownGPS();

    /* Add result message */

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    if (result == GPS_CANCELLED_BY_MAGNETIC_SWITCH) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Cancelled by magnetic switch.");
 
    if (result == GPS_CANCELLED_BY_SWITCH) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Cancelled by switch position change.");

    if (result == GPS_TIMEOUT) writeGPSLogMessage(currentTime, currentMilliseconds, "Time was not updated. Timed out.");

    writeGPSLogMessage(currentTime, currentMilliseconds, "GPS powered down.\n");

    if (success) AudioMoth_closeFile();

    return result;

}

/* Configuration reading functions */

static bool writeStatusMessage(char *message) {

    RETURN_BOOL_ON_ERROR(AudioMoth_openFile("STATUS.TXT"));

    RETURN_BOOL_ON_ERROR(AudioMoth_writeToFile(message, strlen(message)));

    RETURN_BOOL_ON_ERROR(AudioMoth_closeFile());

    return true;

}

static bool readConfigurationFromFile() {

    uint32_t count = 0;

    char character = ' ';

    CP_parserStatus_t status = CP_WAITING;

    while (count < CONFIG_MAX_FILE_READ_CHARACTERS) {

        if (count % CONFIG_FILE_READ_BUFFER_LENGTH == 0) {

            AudioMoth_readFile(tempFileBuffer, CONFIG_FILE_READ_BUFFER_LENGTH);

        }

        character = tempFileBuffer[count % CONFIG_FILE_READ_BUFFER_LENGTH];

        status = ConfigParser_parse(character, &defaultConfigSettings);

        if (status == CP_SUCCESS || status == CP_CHARACTER_ERROR) break;

        count += 1;

    }

    if (status == CP_SUCCESS) {

        writeStatusMessage("SUCCESS: Succeeded in reading the configuration.");

        copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(CP_configSettings_t));

        return true;

    }

    if (status == CP_WAITING || status == CP_PARSING) {
        
        char *error = status == CP_CHARACTER_ERROR ? "find start" : "complete read";

        sprintf(tempFileBuffer, "ERROR: Failed to %s of configuration before reaching position %lu.", error, count);

    } else {

        char *error = status == CP_CHARACTER_ERROR ? "Character error" : "Value error";

        sprintf(tempFileBuffer, "ERROR: %s occurred after reading character \'%c\' at position %lu.", error, character, count);

    }

    writeStatusMessage(tempFileBuffer);

    return false;

}

/* Magenetic switch wait functions */

static void startWaitingForMagneticSwitch() {

    /* Flash LED to indicate start of waiting for magnetic switch */

    FLASH_REPEAT_LED(Red, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

    /* Cancel any scheduled recording */

    *timeOfNextRecording = UINT32_MAX;

    *durationOfNextRecording = UINT32_MAX;

    *timeOfNextGPSTimeSetting = UINT32_MAX;

    *waitingForMagneticSwitch = true;

}

static void stopWaitingForMagneticSwitch(uint32_t *currentTime, uint32_t *currentMilliseconds) {

    /* Flash LED to indicate end of waiting for magnetic switch */

    FLASH_REPEAT_LED(Green, MAGNETIC_SWITCH_CHANGE_FLASHES, SHORT_LED_FLASH_DURATION);

    /* Schedule next recording */

    AudioMoth_getTime(currentTime, currentMilliseconds);

    uint32_t scheduleTime = *currentTime + ROUNDED_UP_DIV(*currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

    scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording, timeOfNextGPSTimeSetting);

    *waitingForMagneticSwitch = false;

}

/* Function to calculate the time to the next event */

static void calculateTimeToNextEvent(uint32_t currentTime, uint32_t currentMilliseconds, int64_t *timeUntilPreparationStart, int64_t *timeUntilNextGPSTimeSetting) {

    *timeUntilPreparationStart = (int64_t)*timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*recordingPreparationPeriod - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

    *timeUntilNextGPSTimeSetting = (int64_t)*timeOfNextGPSTimeSetting * MILLISECONDS_IN_SECOND - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

}

/* Main function */

int main(void) {

    /* Initialise device */

    AudioMoth_initialise();

    AM_switchPosition_t switchPosition = AudioMoth_getSwitchPosition();

    if (AudioMoth_isInitialPowerUp()) {
        
        /* Initialise recording schedule variables */

        *timeOfNextRecording = 0;

        *durationOfNextRecording = UINT32_MAX;

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        /* Initialise configuration writing variable */

        *writtenDeviceDataToFile = false;

        /* Initialise recording state variables */

        *previousSwitchPosition = AM_SWITCH_NONE;

        *shouldSetTimeFromGPS = false;

        *readyToMakeRecordings = false;

        *recordingErrorHasOccurred = false;

        *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

        /* Initialise magnetic switch state variables */

        *waitingForMagneticSwitch = false;

        /* Initialise the power down interval flag */

        *poweredDownWithShortWaitInterval = false;

        /* Copy default configuration to backup domain */

        copyToBackupDomain((uint32_t*)configSettings, (uint8_t*)&defaultConfigSettings, sizeof(CP_configSettings_t));

    }

    /* Handle the case that the switch is in USB position  */

    if (switchPosition == AM_SWITCH_USB) {

        if (configSettings->batteryLevelDisplayType != NONE && (*previousSwitchPosition == AM_SWITCH_DEFAULT || *previousSwitchPosition == AM_SWITCH_CUSTOM)) {

            flashLedToIndicateBatteryLife();

        }

        AudioMoth_handleUSB();

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Read the time */

    uint32_t currentTime;

    uint32_t currentMilliseconds;

    AudioMoth_getTime(&currentTime, &currentMilliseconds);

    /* Check if switch has just been moved to CUSTOM or DEFAULT */

    bool fileSystemEnabled = false;

    bool writtenDeviceDataToFileInThisSession = false;

    if (switchPosition != *previousSwitchPosition) {

        /* Reset the GPS flag */

        *shouldSetTimeFromGPS = false;

        /* Reset the power down interval flag */

        *poweredDownWithShortWaitInterval = false;

        /* Read the configuration from SD card */

        bool configurationIsOkay = false;

        fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

        if (fileSystemEnabled) {
            
            bool success = AudioMoth_openFileToRead("SETTINGS.TXT");

            if (success) {
                
                configurationIsOkay = readConfigurationFromFile();

                AudioMoth_closeFile();

            } 

        }

        /* Show double warning flash if configuration was not successfully read from the SD card */

        if (configurationIsOkay == false) {

            /* Write warning to file */

            writeStatusMessage("ERROR: Could not open configuration file.");

            /* Show show flash LED */

            FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            AudioMoth_delay(LONG_LED_FLASH_DURATION >> 1);

            FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            /* Reset switch position so it will try to read again */

            *previousSwitchPosition = AM_SWITCH_NONE;

            AudioMoth_powerDownAndWakeMilliseconds(DEFAULT_WAIT_INTERVAL);

        }

        /* Check that conditions to make a recording have been satisfied */
        
        *readyToMakeRecordings = switchPosition == AM_SWITCH_DEFAULT || (switchPosition == AM_SWITCH_CUSTOM && (AudioMoth_hasTimeBeenSet() || configSettings->enableTimeSettingFromGPS));

        /* Enable time setting from the GPS if it is not already set */

        *shouldSetTimeFromGPS = switchPosition == AM_SWITCH_CUSTOM && AudioMoth_hasTimeBeenSet() == false && configSettings->enableTimeSettingFromGPS;

        /* Calculate time of next recording if ready to make a recording */

        if (*readyToMakeRecordings) {

            /* Reset the error flag */

            *recordingErrorHasOccurred = false;

            /* Reset the recording preparation period to default */

            *recordingPreparationPeriod = INITIAL_PREPARATION_PERIOD;

            /* Reset persistent configuration write flag */

            *writtenDeviceDataToFile = false;

            /* Try to write configuration to file */

            writtenDeviceDataToFileInThisSession = writeDeviceDataToFile(firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS);

            /* Update the time and calculate earliest schedule start time */

            AudioMoth_getTime(&currentTime, &currentMilliseconds);

            uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

            /* Schedule the next recording */

            if (switchPosition == AM_SWITCH_CUSTOM) {

                *waitingForMagneticSwitch = configSettings->enableMagneticSwitch;

                if (configSettings->enableMagneticSwitch || *shouldSetTimeFromGPS) {

                    *timeOfNextRecording = UINT32_MAX;

                    *durationOfNextRecording = UINT32_MAX;

                    *timeOfNextGPSTimeSetting = UINT32_MAX;

                } else {

                    scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording, timeOfNextGPSTimeSetting);

                }

            }

            /* Set parameters to start recording now */

            if (switchPosition == AM_SWITCH_DEFAULT) {

                *timeOfNextRecording = scheduleTime;

                *durationOfNextRecording = UINT32_MAX;

                *timeOfNextGPSTimeSetting = UINT32_MAX;

            }

        }

    }
    
    /* If not ready to make a recording then flash LED and power down */

    if (*readyToMakeRecordings == false) {

        FLASH_LED(Both, SHORT_LED_FLASH_DURATION)

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Enable the magnetic switch */

    bool magneticSwitchEnabled = configSettings->enableMagneticSwitch && switchPosition == AM_SWITCH_CUSTOM;

    if (magneticSwitchEnabled) GPS_enableMagneticSwitch();

    /* Reset LED flags */

    bool enableLED = (switchPosition == AM_SWITCH_DEFAULT) || configSettings->enableLED;

    bool shouldSuppressLED = *poweredDownWithShortWaitInterval;

    *poweredDownWithShortWaitInterval = false;

    /* Calculate time until next activity */

    int64_t timeUntilPreparationStart;
    
    int64_t timeUntilNextGPSTimeSetting;

    calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

    /* If the GPS synchronisation window has passed then cancel it */

    int64_t timeSinceScheduledGPSTimeSetting = -timeUntilNextGPSTimeSetting;

    if (timeSinceScheduledGPSTimeSetting > GPS_MAX_TIME_SETTING_PERIOD * MILLISECONDS_IN_SECOND) {

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

    }

    /* Set the time from the GPS */

    if (*shouldSetTimeFromGPS && *waitingForMagneticSwitch == false) {

        /* Set the time from the GPS */

        if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

        GPS_fixResult_t fixResult = setTimeFromGPS(true, currentTime + GPS_MAX_TIME_SETTING_PERIOD);

        /* Update the schedule if successful */

        if (fixResult == GPS_SUCCESS) {

            /* Reset the flag */

            *shouldSetTimeFromGPS = false;

            /* Schedule the next recording */

            AudioMoth_getTime(&currentTime, &currentMilliseconds);

            uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

            scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording, timeOfNextGPSTimeSetting);
        }

        /* If time setting was cancelled with the magnet switch then start waiting for the magnetic switch */

        if (fixResult == GPS_CANCELLED_BY_MAGNETIC_SWITCH) startWaitingForMagneticSwitch();

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
    }

    /* Make a recording */
    
    if (timeUntilPreparationStart <= 0) {

        /* Write configuration if not already done so */

        if (writtenDeviceDataToFileInThisSession == false && *writtenDeviceDataToFile == false) {

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

            if (fileSystemEnabled) *writtenDeviceDataToFile = writeDeviceDataToFile(firmwareDescription, firmwareVersion, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS);

        }

        /* Make the recording */

        uint32_t fileOpenTime;

        uint32_t fileOpenMilliseconds;

        AM_recordingState_t recordingState = RECORDING_OKAY;

        /* Measure battery voltage */

        uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

        AM_extendedBatteryState_t extendedBatteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        /* Check if low voltage check is enabled and that the voltage is okay */

        bool okayToMakeRecording = true;

        if (ENABLE_LOW_VOLTAGE_CUTOFF) {

            AudioMoth_enableSupplyMonitor();

            AudioMoth_setSupplyMonitorThreshold(MINIMUM_SUPPLY_VOLTAGE);

            okayToMakeRecording = AudioMoth_isSupplyAboveThreshold();

        }

        /* Make recording if okay */

        if (okayToMakeRecording) {

            AudioMoth_enableTemperature();

            int32_t temperature = AudioMoth_getTemperature();

            AudioMoth_disableTemperature();

            if (!fileSystemEnabled) fileSystemEnabled = AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

            if (fileSystemEnabled)  {

                recordingState = makeRecording(*timeOfNextRecording, *durationOfNextRecording, enableLED, extendedBatteryState, temperature, &fileOpenTime, &fileOpenMilliseconds);

            } else {

                FLASH_LED(Both, LONG_LED_FLASH_DURATION);

                recordingState = SDCARD_WRITE_ERROR;

            }

        } else {

            if (enableLED) FLASH_LED(Both, LONG_LED_FLASH_DURATION);

            recordingState = SUPPLY_VOLTAGE_LOW;

        }

        /* Disable low voltage monitor if it was used */

        if (ENABLE_LOW_VOLTAGE_CUTOFF) AudioMoth_disableSupplyMonitor();

        /* Flash error if Edge Impulse Model has problems */

        if (recordingState == EI_MODEL_ERROR) {

            /* Show show flash LED */

            FLASH_LED(Both, SHORT_LED_FLASH_DURATION);

            AudioMoth_delay(SHORT_LED_FLASH_DURATION >> 1);

            FLASH_LED(Both, SHORT_LED_FLASH_DURATION);

            AudioMoth_delay(SHORT_LED_FLASH_DURATION >> 1);

            FLASH_LED(Both, SHORT_LED_FLASH_DURATION);

        }

        /* Enable the error warning flashes */

        if (switchPosition == AM_SWITCH_CUSTOM && (recordingState == SDCARD_WRITE_ERROR || recordingState == SUPPLY_VOLTAGE_LOW || recordingState == EI_MODEL_ERROR)) {

            *recordingErrorHasOccurred = true;

        }

        /* Update the preparation period */

        if (recordingState != SDCARD_WRITE_ERROR) {

            int64_t measuredPreparationPeriod = (int64_t)fileOpenTime * MILLISECONDS_IN_SECOND + (int64_t)fileOpenMilliseconds - (int64_t)currentTime * MILLISECONDS_IN_SECOND - (int64_t)currentMilliseconds;

            *recordingPreparationPeriod = MIN(MAXIMUM_PREPARATION_PERIOD, MAX(MINIMUM_PREPARATION_PERIOD, measuredPreparationPeriod + PREPARATION_PERIOD_INCREMENT));

        }

        /* Update the time and calculate earliest schedule start time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        uint32_t scheduleTime = currentTime + ROUNDED_UP_DIV(currentMilliseconds + *recordingPreparationPeriod, MILLISECONDS_IN_SECOND);

        /* Schedule the next recording */

        if (switchPosition == AM_SWITCH_CUSTOM) {

            /* Update schedule time as if the recording has ended correctly */

            if (recordingState == RECORDING_OKAY || recordingState == SUPPLY_VOLTAGE_LOW || recordingState == SDCARD_WRITE_ERROR || recordingState == EI_MODEL_ERROR) {

                scheduleTime = MAX(scheduleTime, *timeOfNextRecording + *durationOfNextRecording);

            }

            /* Calculate the next recording schedule */

            scheduleRecording(scheduleTime, timeOfNextRecording, durationOfNextRecording, timeOfNextGPSTimeSetting);

        }

        /* Set parameters to start recording now */

        if (switchPosition == AM_SWITCH_DEFAULT) {

            *timeOfNextRecording = scheduleTime;

            *durationOfNextRecording = UINT32_MAX;

            *timeOfNextGPSTimeSetting = UINT32_MAX;

        }

        /* If recording was cancelled with the magnetic switch then start waiting for the magnetic switch */

        if (recordingState == MAGNETIC_SWITCH) {
            
            startWaitingForMagneticSwitch();

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

         /* Power down with short interval if the next recording is due */

        if (switchPosition == AM_SWITCH_CUSTOM) {

            calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

            if (timeUntilPreparationStart < DEFAULT_WAIT_INTERVAL) {

                *poweredDownWithShortWaitInterval = true;

                SAVE_SWITCH_POSITION_AND_POWER_DOWN(SHORT_WAIT_INTERVAL);

            }
        
        }

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    }

    /* Update the time from the GPS */
    
    if (timeUntilNextGPSTimeSetting <= 0 && timeUntilPreparationStart > GPS_MIN_TIME_SETTING_PERIOD * MILLISECONDS_IN_SECOND) {

        /* Set the time from the GPS */

        AudioMoth_enableFileSystem(AM_SD_CARD_NORMAL_SPEED);

        GPS_fixResult_t fixResult = setTimeFromGPS(enableLED, *timeOfNextRecording - ROUNDED_UP_DIV(*recordingPreparationPeriod, MILLISECONDS_IN_SECOND));

        /* Update the next scheduled GPS fix time */

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        /* If time setting was cancelled with the magnet switch then start waiting for the magnetic switch */

        if (fixResult == GPS_CANCELLED_BY_MAGNETIC_SWITCH) startWaitingForMagneticSwitch();

        /* Power down */

        SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    } 

    /* Power down if switch position has changed */

    if (switchPosition != *previousSwitchPosition) SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

    /* Calculate the wait intervals */

    int64_t waitIntervalMilliseconds = WAITING_LED_FLASH_INTERVAL;

    uint32_t waitIntervalSeconds = WAITING_LED_FLASH_INTERVAL / MILLISECONDS_IN_SECOND;

    if (*waitingForMagneticSwitch) {
        
        waitIntervalMilliseconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

        waitIntervalSeconds *= MAGNETIC_SWITCH_WAIT_MULTIPLIER;

    }

    /* Wait for the next event whilst flashing the LED */

    bool startedRealTimeClock = false;

    while (true) {

        /* Update the time */

        AudioMoth_getTime(&currentTime, &currentMilliseconds);

        /* Handle magnetic switch event */

        bool magneticSwitchEvent = magneticSwitch || (magneticSwitchEnabled && GPS_isMagneticSwitchClosed());

        if (magneticSwitchEvent) {

            if (*waitingForMagneticSwitch) {

                stopWaitingForMagneticSwitch(&currentTime, &currentMilliseconds);

            } else {

                startWaitingForMagneticSwitch();

            }
            
            SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);

        }

        /* Handle switch position change */

        bool switchEvent = switchPositionChanged || AudioMoth_getSwitchPosition() != *previousSwitchPosition;
       
        if (switchEvent) SAVE_SWITCH_POSITION_AND_POWER_DOWN(DEFAULT_WAIT_INTERVAL);
        
        /* Calculate the time to the next event */
        
        calculateTimeToNextEvent(currentTime, currentMilliseconds, &timeUntilPreparationStart, &timeUntilNextGPSTimeSetting);

        int64_t timeToEarliestEvent = MIN(timeUntilPreparationStart, timeUntilNextGPSTimeSetting);

        /* Flash LED */

        bool shouldFlashLED = *waitingForMagneticSwitch || (enableLED && shouldSuppressLED == false && timeToEarliestEvent > MINIMUM_LED_FLASH_INTERVAL);

        if (shouldFlashLED) {

            if (*recordingErrorHasOccurred) {

                FLASH_LED(Both, WAITING_LED_FLASH_DURATION);

            } else {

                FLASH_LED(Green, WAITING_LED_FLASH_DURATION);

            }

        }

        /* Check there is time to sleep */

        if (timeToEarliestEvent < waitIntervalMilliseconds) {
            
            /* Calculate the remaining time to power down */

            uint32_t timeToWait = timeToEarliestEvent < 0 ? 0 : timeToEarliestEvent;

            SAVE_SWITCH_POSITION_AND_POWER_DOWN(timeToWait);

        }

        /* Start the real time clock if it isn't running */

        if (startedRealTimeClock == false) {

            AudioMoth_startRealTimeClock(waitIntervalSeconds);

            startedRealTimeClock = true;

        }

        /* Enter deep sleep */

        AudioMoth_deepSleep();

        /* Handle time overflow on awakening */

        AudioMoth_checkAndHandleTimeOverflow();

    }

}

/* Time zone handler */

inline void AudioMoth_timezoneRequested(int8_t *timezoneHours, int8_t *timezoneMinutes) {

    *timezoneHours = configSettings->timezoneHours;

    *timezoneMinutes = configSettings->timezoneMinutes;

}

/* GPS time handlers */

inline void GPS_handleSetTime(uint32_t time, uint32_t milliseconds, int64_t timeDifference, uint32_t measuredClockFrequency) {

    /* Re-use secondary DMA buffer to save space */ 

    char *setTimeBuffer = (char *)secondaryBuffer;

    /* Update the time if appropriate */

    if (!AudioMoth_hasTimeBeenSet()) {

        AudioMoth_setTime(time, milliseconds);

        writeGPSLogMessage(time, milliseconds, "Time was set from GPS.");

    } else {

        if (timeDifference == 0) {

            writeGPSLogMessage(time, milliseconds, "Time was not updated. The internal clock was correct.");

        } else if (timeDifference < -GPS_MAXIMUM_MS_DIFFERENCE || timeDifference > GPS_MAXIMUM_MS_DIFFERENCE) {

            writeGPSLogMessage(time, milliseconds, "Time was not updated. The discrepancy between the internal clock and the GPS was too large.");

        } else {

            AudioMoth_setTime(time, milliseconds);

            snprintf(setTimeBuffer, MAXIMUM_SAMPLES_IN_DMA_TRANSFER , "Time was updated. The internal clock was %ldms %s.", timeDifference > 0 ? (int32_t)timeDifference : -(int32_t)timeDifference, timeDifference > 0 ? "fast" : "slow");

            writeGPSLogMessage(time, milliseconds, setTimeBuffer);

        }

    }

    /* Calculate the actual sampling rate */

    uint32_t intendedClockFrequency = AudioMoth_getClockFrequency();

    uint32_t intendedSamplingRate = SAMPLE_RATE / SAMPLE_RATE_DIVIDER;

    uint32_t clockTicksPerSample = intendedClockFrequency / intendedSamplingRate;

    uint64_t actualSamplingRate = ROUNDED_DIV(GPS_FREQUENCY_PRECISION * (uint64_t)measuredClockFrequency, (uint64_t)clockTicksPerSample);

    uint32_t integerPart = actualSamplingRate / GPS_FREQUENCY_PRECISION;

    uint32_t fractionalPart = actualSamplingRate % GPS_FREQUENCY_PRECISION;

    sprintf(setTimeBuffer, "Actual sample rate will be %lu.%03lu Hz.", integerPart, fractionalPart);

    writeGPSLogMessage(time, milliseconds, setTimeBuffer);

}

inline void GPS_handleGetTime(uint32_t *time, uint32_t *milliseconds) {

    AudioMoth_getTime(time, milliseconds);

}

/* GPS interrupt handlers */

inline void GPS_handleTickEvent() {

    if (gpsTickEventCount == 0) {

        gpsTickEventModulo = GPS_TICK_EVENTS_PER_SECOND;
    
        if (gpsPPSEvent || gpsFixEvent) {

            gpsTickEventModulo /= 3;

        } else if (gpsMessageEvent) {

            gpsTickEventModulo /= 2;

        }

        gpsMessageEvent = false;

        gpsFixEvent = false;

        gpsPPSEvent = false;

    }

    if (gpsEnableLED) AudioMoth_setRedLED(gpsTickEventCount % gpsTickEventModulo == 0);

    gpsTickEventCount = (gpsTickEventCount + 1) % GPS_TICK_EVENTS_PER_SECOND;

}

inline void GPS_handlePPSEvent(uint32_t time, uint32_t milliseconds) {

    writeGPSLogMessage(time, milliseconds, "Received pulse per second signal.");

    gpsPPSEvent = true;

}

inline void GPS_handleFixEvent(uint32_t time, uint32_t milliseconds, GPS_fixTime_t *fixTime, GPS_fixPosition_t *fixPosition, char *message) {

    /* Use the secondary DMA buffer to save RAM */

    char *fixBuffer = (char *)secondaryBuffer;

    snprintf(fixBuffer, MAXIMUM_SAMPLES_IN_DMA_TRANSFER , "Received GPS fix - %02d°%02d.%04d'%c %03d°%02d.%04d'%c at %02d/%02d/%04d %02d:%02d:%02d.%03d UTC.", fixPosition->latitudeDegrees, fixPosition->latitudeMinutes, fixPosition->latitudeTenThousandths, fixPosition->latitudeDirection, fixPosition->longitudeDegrees, fixPosition->longitudeMinutes, fixPosition->longitudeTenThousandths, fixPosition->longitudeDirection, fixTime->day, fixTime->month, fixTime->year, fixTime->hours, fixTime->minutes, fixTime->seconds, fixTime->milliseconds);

    writeGPSLogMessage(time, milliseconds, fixBuffer);

    gpsFixEvent = true;

}

inline void GPS_handleMessageEvent(uint32_t time, uint32_t milliseconds, char *message) {

    if (!gpsFirstMessageReceived) {

        writeGPSLogMessage(time, milliseconds, "Received first GPS message.");

        gpsFirstMessageReceived = true;

    }

    gpsMessageEvent = true;

}

inline void GPS_handleMagneticSwitchInterrupt() {

    magneticSwitch = true;

    GPS_cancelTimeSetting(GPS_CANCEL_BY_MAGNETIC_SWITCH);

}

/* AudioMoth interrupt handlers */

inline void AudioMoth_handleMicrophoneChangeInterrupt() {

    microphoneChanged = true;

}

inline void AudioMoth_handleSwitchInterrupt() {

    switchPositionChanged = true;

    GPS_cancelTimeSetting(GPS_CANCEL_BY_SWITCH);

}

inline void AudioMoth_handleMicrophoneInterrupt(int16_t sample) { }

inline void AudioMoth_handleDirectMemoryAccessInterrupt(bool isPrimaryBuffer, int16_t **nextBuffer) {

    int16_t *source = secondaryBuffer;

    if (isPrimaryBuffer) source = primaryBuffer;

    /* Apply filter to samples */

    DigitalFilter_applyFilter(source, buffers[writeBuffer] + writeBufferIndex, SAMPLE_RATE_DIVIDER, numberOfRawSamplesInDMATransfer);

    numberOfDMATransfers += 1;

    /* Update the current buffer index and write buffer if wait period is over */

    if (numberOfDMATransfers > numberOfDMATransfersToWait) {

        writeBufferIndex += numberOfRawSamplesInDMATransfer / SAMPLE_RATE_DIVIDER;

        if (writeBufferIndex == NUMBER_OF_SAMPLES_IN_BUFFER) {

            writeBufferIndex = 0;

            writeBuffer = (writeBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

        }

    }

}

/* AudioMoth USB message handlers */

inline void AudioMoth_usbFirmwareVersionRequested(uint8_t **firmwareVersionPtr) {

    *firmwareVersionPtr = firmwareVersion;

}

inline void AudioMoth_usbFirmwareDescriptionRequested(uint8_t **firmwareDescriptionPtr) {

    *firmwareDescriptionPtr = firmwareDescription;

}

inline void AudioMoth_usbApplicationPacketRequested(uint32_t messageType, uint8_t *transmitBuffer, uint32_t size) { }

inline void AudioMoth_usbApplicationPacketReceived(uint32_t messageType, uint8_t* receiveBuffer, uint8_t *transmitBuffer, uint32_t size) { }

/* Clear and encode the compression buffer */

static void clearCompressionBuffer(int16_t *compressionBuffer) {

    for (uint32_t i = 0; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }
}

static void encodeCompressionBuffer(int16_t *compressionBuffer, uint32_t numberOfCompressedBuffers) {

    for (uint32_t i = 0; i < UINT32_SIZE_IN_BITS; i += 1) {

        compressionBuffer[i] = numberOfCompressedBuffers & 0x01 ? 1 : -1;

        numberOfCompressedBuffers >>= 1;

    }

    for (uint32_t i = UINT32_SIZE_IN_BITS; i < COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE; i += 1) {

        compressionBuffer[i] = 0;

    }
}

/* Generate foldername and filename from time */

static void generateFolderAndFilename(char *foldername, char *filename, uint32_t timestamp, bool triggeredRecording) {

    struct tm time;

    time_t rawTime = timestamp + configSettings->timezoneHours * SECONDS_IN_HOUR + configSettings->timezoneMinutes * SECONDS_IN_MINUTE;

    gmtime_r(&rawTime, &time);

    sprintf(foldername, "%04d%02d%02d", YEAR_OFFSET + time.tm_year, MONTH_OFFSET + time.tm_mon, time.tm_mday);

    uint32_t length = sprintf(filename, "%s/%02d%02d%02d", foldername, time.tm_hour, time.tm_min, time.tm_sec);

    char *extension = triggeredRecording ? "T.WAV" : ".WAV";

    strcpy(filename + length, extension);

}

/* Save recording to SD card */

static AM_recordingState_t makeRecording(uint32_t timeOfNextRecording, uint32_t recordDuration, bool enableLED, AM_extendedBatteryState_t extendedBatteryState, int32_t temperature, uint32_t *fileOpenTime, uint32_t *fileOpenMilliseconds) {

    /* Initialise buffers */

    writeBuffer = 0;

    writeBufferIndex = 0;

    buffers[0] = (int16_t*)AM_EXTERNAL_SRAM_START_ADDRESS;

    for (uint32_t i = 1; i < NUMBER_OF_BUFFERS; i += 1) {
        buffers[i] = buffers[i - 1] + NUMBER_OF_SAMPLES_IN_BUFFER;
    }

    int16_t *compressionBuffer = ((tempBuffer_t *)FREE_EXTERNAL_SRAM_START_ADDRESS)->compressionBuffer;

    /* Calculate effective sample rate */

    uint32_t effectiveSampleRate = SAMPLE_RATE / SAMPLE_RATE_DIVIDER;

    /* Set up the digital filter */

    float gain = DISABLE_48HZ_DC_BLOCKING_FILTER ? LOW_DC_BLOCKING_FREQ_GAIN : DEFAULT_DC_BLOCKING_FREQ_GAIN;

    float constant = DISABLE_48HZ_DC_BLOCKING_FILTER ? LOW_DC_BLOCKING_FREQ_CONSTANT : DEFAULT_DC_BLOCKING_FREQ_CONSTANT;

    DigitalFilter_setHighPassFilterConstants(gain, constant);

    /* Calculate the sample multiplier */

    float sampleMultiplier = 16.0f / (float)(OVER_SAMPLE_RATE * SAMPLE_RATE_DIVIDER);

    if (AudioMoth_hasInvertedOutput()) sampleMultiplier = -sampleMultiplier;

    DigitalFilter_setAdditionalGain(sampleMultiplier);

    /* Calculate the number of samples in each DMA transfer (while ensuring that number of samples written to the SRAM buffer on each DMA transfer is a power of two so each SRAM buffer is filled after an integer number of DMA transfers) */

    numberOfRawSamplesInDMATransfer = MAXIMUM_SAMPLES_IN_DMA_TRANSFER / SAMPLE_RATE_DIVIDER;

    while (numberOfRawSamplesInDMATransfer & (numberOfRawSamplesInDMATransfer - 1)) {

        numberOfRawSamplesInDMATransfer = numberOfRawSamplesInDMATransfer & (numberOfRawSamplesInDMATransfer - 1);

    }

    numberOfRawSamplesInDMATransfer *= SAMPLE_RATE_DIVIDER;

    /* Calculate the minimum amplitude threshold duration */

    uint32_t minimumNumberOfTriggeredBuffersToWrite = ROUNDED_UP_DIV(configSettings->minimumTriggerDuration * effectiveSampleRate, NUMBER_OF_SAMPLES_IN_BUFFER);

    /* Initialise termination conditions */

    microphoneChanged = false;

    bool supplyVoltageLow = false;

    bool modelError = false;

    /* Initialise microphone for recording */

    AudioMoth_enableExternalSRAM();

    bool externalMicrophone = AudioMoth_enableMicrophone(GAIN_RANGE, GAIN, CLOCK_DIVIDER, ACQUISITION_SAMPLES, OVER_SAMPLE_RATE);

    AudioMoth_initialiseDirectMemoryAccess(primaryBuffer, secondaryBuffer, numberOfRawSamplesInDMATransfer);

    /* Show LED for SD card activity */

    if (enableLED) AudioMoth_setRedLED(true);

    /* Open a file with the current local time as the name */

    char *filename = ((tempBuffer_t *)FREE_EXTERNAL_SRAM_START_ADDRESS)->filename;

    char *foldername = ((tempBuffer_t *)FREE_EXTERNAL_SRAM_START_ADDRESS)->foldername;

    generateFolderAndFilename(foldername, filename, timeOfNextRecording, configSettings->enableEdgeImpulseModel);

    bool directoryExists = AudioMoth_doesDirectoryExist(foldername);

    if (directoryExists == false) FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_makeDirectory(foldername));

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_openFile(filename));

    AudioMoth_setRedLED(false);

    /* Measure the time difference from the start time */

    AudioMoth_getTime(fileOpenTime, fileOpenMilliseconds);

    /* Calculate time correction for sample rate due to file header */

    uint32_t numberOfSamplesInHeader = sizeof(wavHeader_t) / NUMBER_OF_BYTES_IN_SAMPLE;

    int32_t sampleRateTimeOffset = ROUNDED_DIV(numberOfSamplesInHeader * MILLISECONDS_IN_SECOND, effectiveSampleRate);

    /* Calculate time until the recording should start */

    int64_t millisecondsUntilRecordingShouldStart = (int64_t)timeOfNextRecording * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenTime * MILLISECONDS_IN_SECOND - (int64_t)*fileOpenMilliseconds - (int64_t)sampleRateTimeOffset;

    /* Calculate the actual recording start time if the intended start has been missed */

    uint32_t timeOffset = millisecondsUntilRecordingShouldStart < 0 ? 1 - millisecondsUntilRecordingShouldStart / MILLISECONDS_IN_SECOND : 0;

    recordDuration = timeOffset >= recordDuration ? 0 : recordDuration - timeOffset;

    millisecondsUntilRecordingShouldStart += timeOffset * MILLISECONDS_IN_SECOND;

    /* Calculate the period to wait before starting the DMA transfers */

    uint32_t numberOfRawSamplesPerMillisecond = SAMPLE_RATE / MILLISECONDS_IN_SECOND;

    uint32_t numberOfRawSamplesToWait = millisecondsUntilRecordingShouldStart * numberOfRawSamplesPerMillisecond;

    numberOfDMATransfersToWait = numberOfRawSamplesToWait / numberOfRawSamplesInDMATransfer;

    uint32_t remainingNumberOfRawSamples = numberOfRawSamplesToWait % numberOfRawSamplesInDMATransfer;

    uint32_t remainingMillisecondsToWait = ROUNDED_DIV(remainingNumberOfRawSamples, numberOfRawSamplesPerMillisecond);

    /* Calculate updated recording parameters */

    uint32_t maximumNumberOfSeconds = (MAXIMUM_WAV_FILE_SIZE - sizeof(wavHeader_t)) / NUMBER_OF_BYTES_IN_SAMPLE / effectiveSampleRate;

    bool fileSizeLimited = (recordDuration > maximumNumberOfSeconds);

    uint32_t numberOfSamples = effectiveSampleRate * (fileSizeLimited ? maximumNumberOfSeconds : recordDuration);

    /* Initialise main loop variables */

    uint32_t readBuffer = 0;

    uint32_t samplesWritten = 0;

    uint32_t buffersProcessed = 0;

    uint32_t numberOfCompressedBuffers = 0;

    uint32_t totalNumberOfCompressedSamples = 0;

    uint32_t numberOfTriggeredBuffersWritten = 0;

    bool triggerHasOccurred = false;

    /* Start processing DMA transfers */

    numberOfDMATransfers = 0;

    AudioMoth_delay(remainingMillisecondsToWait);

    AudioMoth_startMicrophoneSamples(SAMPLE_RATE);

    /* Main recording loop */

    while (samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !magneticSwitch && !supplyVoltageLow && !modelError) {

        while (readBuffer != writeBuffer && samplesWritten < numberOfSamples + numberOfSamplesInHeader && !microphoneChanged && !switchPositionChanged && !magneticSwitch && !supplyVoltageLow && !modelError) {

            /* Determine the appropriate number of bytes to the SD card */

            uint32_t numberOfSamplesToWrite = MIN(numberOfSamples + numberOfSamplesInHeader - samplesWritten, NUMBER_OF_SAMPLES_IN_BUFFER);

            /* Perform Edge Impulse model classification of the buffer */

            bool writeIndicated = true;

            if (configSettings->enableEdgeImpulseModel) {

                if (SHOW_GREEN_LED_DURING_COMPUTATION && enableLED) AudioMoth_setGreenLED(true);

                if (EI_MODEL_FREQUENCY == KHZ_16) {

                    /* Run classifier on whole buffer */

                    float detectionProbability = external_ei_classify(buffers[readBuffer], EI_SIGNAL_LENGTH, &modelError);

                    writeIndicated = detectionProbability > DETECTION_THRESHOLD;

                } else if(EI_MODEL_FREQUENCY == KHZ_8) {

                    /* Run classifier on first half of buffer */

                    float detectionProbability = external_ei_classify(buffers[readBuffer], EI_SIGNAL_LENGTH, &modelError);

                    writeIndicated = detectionProbability > DETECTION_THRESHOLD;

                    if (!writeIndicated && !modelError) {

                        /* Save power by only running classifier on the second half if the first half returned false */

                        detectionProbability = external_ei_classify(buffers[readBuffer] + EI_SIGNAL_LENGTH, EI_SIGNAL_LENGTH, &modelError);

                        writeIndicated = detectionProbability > DETECTION_THRESHOLD;

                    }
                    
                }

                if (SHOW_GREEN_LED_DURING_COMPUTATION && enableLED) AudioMoth_setGreenLED(false);

            }

            /* Ensure the minimum number of buffers will be written */

            triggerHasOccurred |= writeIndicated;

            numberOfTriggeredBuffersWritten = writeIndicated ? 0 : numberOfTriggeredBuffersWritten + 1;

            bool shouldWriteThisSector = !modelError && (writeIndicated || (triggerHasOccurred && numberOfTriggeredBuffersWritten < minimumNumberOfTriggeredBuffersToWrite));

            /* Compress the buffer or write the buffer to SD card */

            if (shouldWriteThisSector == false && buffersProcessed > 0 && numberOfSamplesToWrite == NUMBER_OF_SAMPLES_IN_BUFFER) {

                numberOfCompressedBuffers += NUMBER_OF_BYTES_IN_SAMPLE * NUMBER_OF_SAMPLES_IN_BUFFER / COMPRESSION_BUFFER_SIZE_IN_BYTES;

            } else {

                /* Light LED during SD card write if appropriate */

                if (enableLED) AudioMoth_setRedLED(true);

                /* Encode and write compression buffer */

                if (numberOfCompressedBuffers > 0) {

                    encodeCompressionBuffer(compressionBuffer, numberOfCompressedBuffers);

                    totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;

                    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));

                    numberOfCompressedBuffers = 0;

                }

                /* Either write the buffer or write a blank buffer */

                if (shouldWriteThisSector) {

                    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(buffers[readBuffer], NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamplesToWrite));

                } else {

                    clearCompressionBuffer(compressionBuffer);

                    uint32_t numberOfBlankSamplesToWrite = numberOfSamplesToWrite;

                    while (numberOfBlankSamplesToWrite > 0) {

                        uint32_t numberOfSamples = MIN(numberOfBlankSamplesToWrite, COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE);

                        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, NUMBER_OF_BYTES_IN_SAMPLE * numberOfSamples));

                        numberOfBlankSamplesToWrite -= numberOfSamples;

                    }

                }

                /* Clear LED */

                AudioMoth_setRedLED(false);

            }

            /* Increment buffer counters */

            readBuffer = (readBuffer + 1) & (NUMBER_OF_BUFFERS - 1);

            samplesWritten += numberOfSamplesToWrite;

            buffersProcessed += 1;

        }

        /* Check the voltage level */

        if (ENABLE_LOW_VOLTAGE_CUTOFF && AudioMoth_isSupplyAboveThreshold() == false) {

            supplyVoltageLow = true;

        }

        /* Sleep until next DMA transfer is complete */

        AudioMoth_sleep();

    }

    /* Write the compression buffer files at the end */

    if (samplesWritten < numberOfSamples + numberOfSamplesInHeader && numberOfCompressedBuffers > 0) {

        /* Light LED during SD card write if appropriate */

        if (enableLED) AudioMoth_setRedLED(true);

        /* Encode and write compression buffer */

        encodeCompressionBuffer(compressionBuffer, numberOfCompressedBuffers);

        totalNumberOfCompressedSamples += (numberOfCompressedBuffers - 1) * COMPRESSION_BUFFER_SIZE_IN_BYTES / NUMBER_OF_BYTES_IN_SAMPLE;

        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(compressionBuffer, COMPRESSION_BUFFER_SIZE_IN_BYTES));

        /* Clear LED */

        AudioMoth_setRedLED(false);

    }

    /* Determine recording state */

    AM_recordingState_t recordingState = modelError ? EI_MODEL_ERROR :
                                         microphoneChanged ? MICROPHONE_CHANGED :
                                         switchPositionChanged ? SWITCH_CHANGED :
                                         magneticSwitch ? MAGNETIC_SWITCH :
                                         supplyVoltageLow ? SUPPLY_VOLTAGE_LOW :
                                         fileSizeLimited ? FILE_SIZE_LIMITED :
                                         RECORDING_OKAY;

    /* Initialise the WAV header */

    samplesWritten = MAX(numberOfSamplesInHeader, samplesWritten);

    /* Copy WAV header template from flash to ERAM */

    memcpy((void*)FREE_EXTERNAL_SRAM_START_ADDRESS, &wavHeader, sizeof(wavHeader_t));

    wavHeader_t *editableWavHeader = (wavHeader_t*)FREE_EXTERNAL_SRAM_START_ADDRESS;

    setHeaderDetails(editableWavHeader, effectiveSampleRate, samplesWritten - numberOfSamplesInHeader - totalNumberOfCompressedSamples);

    setHeaderComment(editableWavHeader, configSettings, timeOfNextRecording + timeOffset, (uint8_t*)AM_UNIQUE_ID_START_ADDRESS, extendedBatteryState, temperature, externalMicrophone, recordingState);

    /* Write the header */

    if (enableLED) AudioMoth_setRedLED(true);

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_seekInFile(0));

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_writeToFile(editableWavHeader, sizeof(wavHeader_t)));

    /* Close the file */

    FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_closeFile());

    AudioMoth_setRedLED(false);

    /* Rename the file if necessary */

    char *newFilename = ((tempBuffer_t *)FREE_EXTERNAL_SRAM_START_ADDRESS)->newFilename;

    if (timeOffset > 0) {

        generateFolderAndFilename(foldername, newFilename, timeOfNextRecording + timeOffset, configSettings->enableEdgeImpulseModel);

        if (enableLED) AudioMoth_setRedLED(true);

        FLASH_LED_AND_RETURN_ON_ERROR(AudioMoth_renameFile(filename, newFilename));

        AudioMoth_setRedLED(false);

    }

    /* Return recording state */

    return recordingState;
}

/* Schedule recordings */

static void scheduleRecording(uint32_t currentTime, uint32_t *timeOfNextRecording, uint32_t *durationOfNextRecording, uint32_t *timeOfNextGPSTimeSetting) {

    /* Check number of active state stop periods */

    uint32_t activeStartStopPeriods = MIN(configSettings->activeStartStopPeriods, CP_MAX_START_STOP_PERIODS);

    /* No active periods */

    if (activeStartStopPeriods == 0) {

        *timeOfNextRecording = UINT32_MAX;

        *durationOfNextRecording = 0;

        *timeOfNextGPSTimeSetting = UINT32_MAX;

        goto done;

    }

    /* Check if recording should be limited by earliest recording time */

    if (configSettings->earliestRecordingTime > 0) {

        currentTime = MAX(currentTime, configSettings->earliestRecordingTime);

    }

    /* Calculate the number of seconds of this day */

    struct tm time;

    time_t rawTime = currentTime;

    gmtime_r(&rawTime, &time);

    uint32_t currentSeconds = SECONDS_IN_HOUR * time.tm_hour + SECONDS_IN_MINUTE * time.tm_min + time.tm_sec;

    /* Check each active start stop period */

    for (uint32_t i = 0; i < activeStartStopPeriods; i += 1) {

        CP_startStopPeriod_t *period = configSettings->startStopPeriods + i;

        /* Calculate the start and stop time of the current period */

        uint32_t startSeconds = SECONDS_IN_MINUTE * period->startMinutes;

        uint32_t stopSeconds = SECONDS_IN_MINUTE * period->stopMinutes;

        uint32_t durationOfStartStopPeriod = stopSeconds - startSeconds;

        /* Calculate the start time of this start stop period */

        *timeOfNextGPSTimeSetting = configSettings->enableTimeSettingFromGPS ? currentTime + startSeconds - currentSeconds - GPS_MAX_TIME_SETTING_PERIOD : UINT32_MAX;

        /* Check if the start stop period has not yet started */

        if (currentSeconds <= startSeconds) {

            *timeOfNextRecording = currentTime + startSeconds - currentSeconds;

            if (configSettings->enableSleepRecordCycle == false) {

                *durationOfNextRecording = durationOfStartStopPeriod;

            } else {

                *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod);

            }

            goto done;

        }

        /* Check if currently inside a start stop period */

        if (currentSeconds < stopSeconds) {

            /* Handle case with no sleep record cycle */

            uint32_t secondsFromStartOfPeriod = currentSeconds - startSeconds;

            if (configSettings->enableSleepRecordCycle == false) {

                *timeOfNextRecording = currentTime;

                *durationOfNextRecording = durationOfStartStopPeriod - secondsFromStartOfPeriod;;

                goto done;

            }

            /* Check if recording should start immediately */

            uint32_t durationOfCycle = configSettings->recordDuration + configSettings->sleepDuration;

            uint32_t partialCycle = secondsFromStartOfPeriod % durationOfCycle;

            if (partialCycle < configSettings->recordDuration) {

                *timeOfNextRecording = currentTime;

                *durationOfNextRecording = MIN(configSettings->recordDuration - partialCycle, durationOfStartStopPeriod - secondsFromStartOfPeriod);

                goto done;

            }

            /* Wait for next cycle to begin */

            secondsFromStartOfPeriod += durationOfCycle - partialCycle;

            if (secondsFromStartOfPeriod < durationOfStartStopPeriod) {

                *timeOfNextRecording = currentTime + durationOfCycle - partialCycle;

                *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod - secondsFromStartOfPeriod);

                goto done;

            }

        }

    }

    /* Calculate time until first period tomorrow */

    CP_startStopPeriod_t *firstPeriod = configSettings->startStopPeriods;

    uint32_t startSeconds = SECONDS_IN_MINUTE * firstPeriod->startMinutes;

    uint32_t stopSeconds = SECONDS_IN_MINUTE * firstPeriod->stopMinutes;

    uint32_t durationOfStartStopPeriod = stopSeconds - startSeconds;

    *timeOfNextRecording = currentTime + (SECONDS_IN_DAY - currentSeconds) + startSeconds;

    if (configSettings->enableSleepRecordCycle == false) {

        *durationOfNextRecording = durationOfStartStopPeriod;

    } else {

        *durationOfNextRecording = MIN(configSettings->recordDuration, durationOfStartStopPeriod);

    }

    *timeOfNextGPSTimeSetting = configSettings->enableTimeSettingFromGPS ? *timeOfNextRecording - GPS_MAX_TIME_SETTING_PERIOD : UINT32_MAX;

done:

    /* Check if recording should be limited by last recording time */

    if (configSettings->latestRecordingTime > 0) {

        if (*timeOfNextRecording >= configSettings->latestRecordingTime) {

            *timeOfNextRecording = UINT32_MAX;

            *durationOfNextRecording = 0;

            *timeOfNextGPSTimeSetting = UINT32_MAX;

        } else {

            int64_t excessTime = (int64_t)*timeOfNextRecording + (int64_t)*durationOfNextRecording - (int64_t)configSettings->latestRecordingTime;

            if (excessTime > 0) {

                *durationOfNextRecording -= excessTime;

            }

        }

    }

}

/* Flash LED according to battery life */

static void flashLedToIndicateBatteryLife(void) {

    uint32_t numberOfFlashes = LOW_BATTERY_LED_FLASHES;

    uint32_t supplyVoltage = AudioMoth_getSupplyVoltage();

    if (configSettings->batteryLevelDisplayType == NIMH_LIPO_BATTERY_VOLTAGE) {

        /* Set number of flashes according to battery voltage */

        AM_extendedBatteryState_t batteryState = AudioMoth_getExtendedBatteryState(supplyVoltage);

        if (batteryState > AM_EXT_BAT_4V3) {

            numberOfFlashes = 1;

        } else if (batteryState > AM_EXT_BAT_3V5) {

            numberOfFlashes = AM_EXT_BAT_4V4 - batteryState;

        }

    } else {

        /* Set number of flashes according to battery state */

        AM_batteryState_t batteryState = AudioMoth_getBatteryState(supplyVoltage);

        if (batteryState > AM_BATTERY_LOW) {

            numberOfFlashes = (batteryState >= AM_BATTERY_4V6) ? 4 : (batteryState >= AM_BATTERY_4V4) ? 3 : (batteryState >= AM_BATTERY_4V0) ? 2 : 1;

        }

    }

    /* Flash LED */

    for (uint32_t i = 0; i < numberOfFlashes; i += 1) {

        FLASH_LED(Red, SHORT_LED_FLASH_DURATION)

        if (numberOfFlashes == LOW_BATTERY_LED_FLASHES) {

            AudioMoth_delay(SHORT_LED_FLASH_DURATION);

        } else {

            AudioMoth_delay(LONG_LED_FLASH_DURATION);

        }

    }

}
