
/*
 * Test for TOF mini Waveshare
 */


#include "platform.h"

#if defined(USE_RANGEFINDER)

#include "build/debug.h"
#include "common/maths.h"
#include "common/time.h"
#include "drivers/time.h"
#include "drivers/serial.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"
#include "sensors/rangefinder.h"
#include "rangefinder_tofmini.h"

#define TOFMINI_FRAME_LEN                16
#define TOFMINI_HEADER                   0x57
#define TOFMINI_FUNC_ACTIVE_FRAME0       0x00

#define TOFMINI_STATUS_INVALID           0
#define TOFMINI_STATUS_VALID             1

#ifndef TOFMINI_BAUDRATE
#define TOFMINI_BAUDRATE                 115200
#endif

#ifndef TOFMINI_TIMEOUT_MS
#define TOFMINI_TIMEOUT_MS               200
#endif

typedef struct tofminiRuntime_s {
    serialPort_t *port;

    uint8_t buf[TOFMINI_FRAME_LEN];
    uint8_t index;

    int32_t distanceCm;
    bool healthy;
    timeMs_t lastFrameMs;
} tofminiRuntime_t;

static tofminiRuntime_t tofmini;

static uint8_t tofminiChecksum(const uint8_t *data, uint8_t len)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static uint32_t readU24LE(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16);
}

static void tofminiResetFrame(void)
{
    tofmini.index = 0;
}

static bool tofminiParseFrame(const uint8_t *frame, int32_t *distanceCmOut)
{
    // Waveshare NLink_TOFSense_Frame0:
    // [0]  header        0x57
    // [1]  function      0x00
    // [2]  reserved
    // [3]  id
    // [4]  system_time[0]
    // [5]  system_time[1]
    // [6]  system_time[2]
    // [7]  system_time[3]
    // [8]  dis*1000[0]
    // [9]  dis*1000[1]
    // [10] dis*1000[2]
    // [11] dis_status
    // [12] signal_strength[0]
    // [13] signal_strength[1]
    // [14] range_precision
    // [15] checksum

    if (frame[0] != TOFMINI_HEADER) {
        return false;
    }

    if (frame[1] != TOFMINI_FUNC_ACTIVE_FRAME0) {
        return false;
    }

    if (tofminiChecksum(frame, TOFMINI_FRAME_LEN - 1) != frame[TOFMINI_FRAME_LEN - 1]) {
        return false;
    }

    const uint8_t disStatus = frame[11];
    if (disStatus != TOFMINI_STATUS_VALID) {
        return false;
    }

    const uint32_t distanceMilliMeters = readU24LE(&frame[8]);

    // dis*1000 is meters * 1000 according to Waveshare docs.
    // Therefore numeric value equals millimeters.
    // Convert mm -> cm
    *distanceCmOut = (int32_t)((distanceMilliMeters + 5) / 10);

    return true;
}

static void tofminiConsumeByte(uint8_t c)
{
    if (tofmini.index == 0) {
        if (c != TOFMINI_HEADER) {
            return;
        }
        tofmini.buf[tofmini.index++] = c;
        return;
    }

    tofmini.buf[tofmini.index++] = c;

    if (tofmini.index < TOFMINI_FRAME_LEN) {
        return;
    }

    int32_t distanceCm;
    if (tofminiParseFrame(tofmini.buf, &distanceCm)) {
        tofmini.distanceCm = distanceCm;
        tofmini.healthy = true;
        tofmini.lastFrameMs = millis();
    }

    tofminiResetFrame();
}

bool tofminiRangefinderDetect(void)
{
    memset(&tofmini, 0, sizeof(tofmini));

    // Depending on your INAV branch, the identifier may differ.
    // Use the same serial-function lookup pattern as other UART rangefinder drivers.
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RANGEFINDER);
    if (!portConfig) {
        return false;
    }

    tofmini.port = openSerialPort(
        portConfig->identifier,
        FUNCTION_RANGEFINDER,
        NULL,
        NULL,
        TOFMINI_BAUDRATE,
        MODE_RX,
        SERIAL_NOT_INVERTED
    );

    if (!tofmini.port) {
        return false;
    }

    tofmini.distanceCm = RANGEFINDER_OUT_OF_RANGE;
    tofmini.healthy = false;
    tofmini.lastFrameMs = 0;
    tofmini.index = 0;

    return true;
}

void tofminiRangefinderInit(void)
{
    // Nothing extra needed for active UART streaming mode.
}

bool tofminiRangefinderUpdate(void)
{
    if (!tofmini.port) {
        tofmini.healthy = false;
        return false;
    }

    while (serialRxBytesWaiting(tofmini.port) > 0) {
        const uint8_t c = serialRead(tofmini.port);
        tofminiConsumeByte(c);
    }

    if ((millis() - tofmini.lastFrameMs) > TOFMINI_TIMEOUT_MS) {
        tofmini.healthy = false;
    }

    return tofmini.healthy;
}

int32_t tofminiRangefinderGetDistanceCm(void)
{
    if (!tofmini.healthy) {
        return RANGEFINDER_OUT_OF_RANGE;
    }

    return tofmini.distanceCm;
}

bool tofminiRangefinderIsHealthy(void)
{
    return tofmini.healthy;
}

#endif // USE_RANGEFINDER
