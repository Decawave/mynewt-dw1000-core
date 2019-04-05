#include <stdlib.h>

#include "sysinit/sysinit.h"
#include <dw1000/dw1000_mac.h>
#include <dw1000/dw1000_hal.h>
#include "console/console.h"


#include <openthread/ot_common.h>
#include <openthread/platform/uart.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/logging.h>
#include <openthread/platform/radio.h>
#include <openthread/tasklet.h>
#include <hal/hal_flash.h>

struct PlatformDiagCommand
{
    const char *mName;
    void (*mCommand)(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen);
};

struct PlatformDiagMessage
{
    const char mMessageDescriptor[11];
    uint8_t mChannel;
    int16_t mID;
    uint32_t mCnt;
};

/**
 * Diagnostics mode variables.
 *
 */
static bool sDiagMode = false;
static bool sListen = false;
static bool sTransmitActive = false;
static uint8_t sChannel = 20;
static int8_t sTxPower = 0;
static uint32_t sTxPeriod = 1;
static int32_t sTxCount = 0;
static int32_t sTxRequestedCount = 1;
static int16_t sID = -1;
static struct PlatformDiagMessage sDiagMessage = {.mMessageDescriptor = "DiagMessage", .mChannel = 0, .mID = 0, .mCnt = 0};

static otError parseLong(char *argv, long *aValue)
{
    char *endptr;
    *aValue = strtol(argv, &endptr, 0);
    return (*endptr == '\0') ? OT_ERROR_NONE : OT_ERROR_PARSE;
}

static void appendErrorResult(otError aError, char *aOutput, size_t aOutputMaxLen)
{
    if (aError != OT_ERROR_NONE)
    {
        snprintf(aOutput, aOutputMaxLen, "failed\r\nstatus %#x\r\n", aError);
    }
}

static void processListen(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    (void) aInstance;
    otError error = OT_ERROR_NONE;

    if (argc == 0)
    {
        snprintf(aOutput, aOutputMaxLen, "listen: %s\r\n", sListen == true ? "yes" : "no");
    }
    else
    {
        long value;

        error = parseLong(argv[0], &value);
        //otEXPECT(error == OT_ERROR_NONE);
        sListen = (bool)(value);
        snprintf(aOutput, aOutputMaxLen, "set listen to %s\r\nstatus 0x%02x\r\n", sListen == true ? "yes" : "no", error);
    }


    appendErrorResult(error, aOutput, aOutputMaxLen);
}

static void processID(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    (void) aInstance;

    otError error = OT_ERROR_NONE;

    if (argc == 0)
    {
        snprintf(aOutput, aOutputMaxLen, "ID: %" PRId16 "\r\n", sID);
    }
    else
    {
        long value;

        error = parseLong(argv[0], &value);
        //otEXPECT(error == OT_ERROR_NONE);
        //otEXPECT_ACTION(value >= 0, error = OT_ERROR_INVALID_ARGS);
        sID = (int16_t)(value);
        snprintf(aOutput, aOutputMaxLen, "set ID to %" PRId16 "\r\nstatus 0x%02x\r\n", sID, error);
    }


    appendErrorResult(error, aOutput, aOutputMaxLen);
}

static void processTransmit(otInstance *aInstance, int argc, char *argv[], char *aOutput,
                            size_t aOutputMaxLen)
{
    otError error = OT_ERROR_NONE;

    if (argc == 0)
    {
        snprintf(aOutput, aOutputMaxLen, "transmit will send %ld diagnostic messages with %lu"
                  "ms interval\r\nstatus 0x%02x\r\n",
                 sTxRequestedCount, sTxPeriod, error);
    }
    else if (strcmp(argv[0], "stop") == 0)
    {
        otPlatAlarmMilliStop(aInstance);
        snprintf(aOutput, aOutputMaxLen, "diagnostic message transmission is stopped\r\nstatus 0x%02x\r\n", error);
        sTransmitActive = false;
    }
    else if (strcmp(argv[0], "start") == 0)
    {
        otPlatAlarmMilliStop(aInstance);
        sTransmitActive = true;
        sTxCount = sTxRequestedCount;
        uint32_t now = otPlatAlarmMilliGetNow();
        otPlatAlarmMilliStartAt(aInstance, now, sTxPeriod);

        snprintf(aOutput, aOutputMaxLen, "sending %ld diagnostic messages with %lu"
                 " ms interval\r\nstatus 0x%02x\r\n",
                 sTxRequestedCount, sTxPeriod, error);
    }
    else if (strcmp(argv[0], "interval") == 0)
    {
        long value;

        //otEXPECT_ACTION(argc == 2, error = OT_ERROR_INVALID_ARGS);

        error = parseLong(argv[1], &value);
        //otEXPECT(error == OT_ERROR_NONE);
        //otEXPECT_ACTION(value > 0, error = OT_ERROR_INVALID_ARGS);
        sTxPeriod = (uint32_t)(value);
        snprintf(aOutput, aOutputMaxLen, "set diagnostic messages interval to %lu ms\r\nstatus 0x%02x\r\n", sTxPeriod,
                 error);
    }
    else if (strcmp(argv[0], "count") == 0)
    {
        long value;

        //otEXPECT_ACTION(argc == 2, error = OT_ERROR_INVALID_ARGS);

        error = parseLong(argv[1], &value);
        //otEXPECT(error == OT_ERROR_NONE);
        //otEXPECT_ACTION((value > 0) || (value == -1), error = OT_ERROR_INVALID_ARGS);
        sTxRequestedCount = (uint32_t)(value);
        snprintf(aOutput, aOutputMaxLen, "set diagnostic messages count to %ld\r\nstatus 0x%02x\r\n", sTxRequestedCount,
                 error);
    }


    appendErrorResult(error, aOutput, aOutputMaxLen);
}

const struct PlatformDiagCommand sCommands[] =
{
    {"listen", &processListen },
    {"transmit", &processTransmit },
    {"id", &processID }
};

void otPlatDiagProcess(otInstance *aInstance, int argc, char *argv[], char *aOutput, size_t aOutputMaxLen)
{
    uint32_t i;

    for (i = 0; i < sizeof(sCommands) / sizeof(sCommands[0]); i++)
    {
        if (strcmp(argv[0], sCommands[i].mName) == 0)
        {
            sCommands[i].mCommand(aInstance, argc - 1, argc > 1 ? &argv[1] : NULL, aOutput, aOutputMaxLen);
            break;
        }
    }

    if (i == sizeof(sCommands) / sizeof(sCommands[0]))
    {
        snprintf(aOutput, aOutputMaxLen, "diag feature '%s' is not supported\r\n", argv[0]);
    }
}

void otPlatDiagModeSet(bool aMode)
{
    sDiagMode = aMode;
}

bool otPlatDiagModeGet()
{
    return sDiagMode;
}

void otPlatDiagChannelSet(uint8_t aChannel)
{
    sChannel = aChannel;
}

void otPlatDiagTxPowerSet(int8_t aTxPower)
{
    sTxPower = aTxPower;
}

void otPlatDiagRadioReceived(otInstance *aInstance, otRadioFrame *aFrame, otError aError)
{
    (void) aInstance;

    if (sListen && (aError == OT_ERROR_NONE))
    {
        if (aFrame->mLength == sizeof(struct PlatformDiagMessage))
        {
            struct PlatformDiagMessage *message = (struct PlatformDiagMessage *)aFrame->mPsdu;

            if (strncmp(message->mMessageDescriptor, "DiagMessage", 11) == 0)
            {
                printf(   "{\"Frame\":{"
                          "\"LocalChannel\":%u ,"
                          "\"RemoteChannel\":%u,"
                          "\"CNT\":%lu,"
                          "\"LocalID\":%" PRId16 ","
                          "\"RemoteID\":%" PRId16 ","
                          "\"RSSI\":%d"
                          "}}\r\n",
                          aFrame->mChannel,
                          message->mChannel,
                          message->mCnt,
                          sID,
                          message->mID,
                          aFrame->mInfo.mRxInfo.mRssi
                         );
            }
        }
    }
}

void otPlatDiagAlarmCallback(otInstance *aInstance)
{
    if (sTransmitActive)
    {
        if ((sTxCount > 0) || (sTxCount == -1))
        {
            otRadioFrame *sTxPacket = otPlatRadioGetTransmitBuffer(aInstance);

            sTxPacket->mLength = sizeof(struct PlatformDiagMessage);
            sTxPacket->mChannel = sChannel;
            sTxPacket->mInfo.mRxInfo.mRssi = sTxPower;

            sDiagMessage.mChannel = sTxPacket->mChannel;
            sDiagMessage.mID = sID;

            memcpy(sTxPacket->mPsdu, &sDiagMessage, sizeof(struct PlatformDiagMessage));
            otPlatRadioTransmit(aInstance, sTxPacket);

            sDiagMessage.mCnt++;

            if (sTxCount != -1)
            {
                sTxCount--;
            }

            uint32_t now = otPlatAlarmMilliGetNow();
            otPlatAlarmMilliStartAt(aInstance, now, sTxPeriod);
        }
        else
        {
            sTransmitActive = false;
            otPlatAlarmMilliStop(aInstance);
            printf("Transmit done \n");
        }
    }
}
