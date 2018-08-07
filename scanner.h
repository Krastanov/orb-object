#include "multilateration.h"

#include "app_util.h"

// TODO cleanup and documentation... certain uuids should be placed in a better, more central place


#define SCAN_INTERVAL   MSEC_TO_UNITS(40, UNIT_0_625_MS)
#define SCAN_WINDOW     MSEC_TO_UNITS(30, UNIT_0_625_MS) // The window was to be smaller than the interval in order to have reliable advertisements while scanning.

#define MULTILATERATION_BUFFER_SIZE 100
#define MULTILATERATION_INTERCEPT -64.19
#define MULTILATERATION_SLOPE -1.579
#define MULTILATERATION_MIN_DISTANCE 0.1
#define MULTILATERATION_MAX_DISTANCE 15

#define MULTILATERATION_BEACON_UUID {0x3a, 0xa8, 0x0a, 0x58, 0x9e, 0xba, 0x4c, 0xcd, 0xbc, 0x73, 0x1f, 0x16, 0x88, 0x00, 0xbb, 0x81}

void scanner_init();