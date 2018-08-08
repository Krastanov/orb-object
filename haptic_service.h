#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"


/* Macro for defining a haptic_service instance. */
#define HAPTIC_SERVICE_DEF(_name)                                                                   \
static haptic_service_t _name;                                                                      \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HIDS_BLE_OBSERVER_PRIO,                                                    \
                     haptic_service_on_ble_evt, &_name)

#define HAPTIC_SERVICE_UUID_BASE        {0x43, 0xD9, 0xB2, 0x37, 0xFA, 0x6E, 0x4A, 0x13, 0x8B, 0xDB, 0x9C, 0xE5, 0x0D, 0xC1, 0x32, 0x64}
         
#define HAPTIC_SERVICE_UUID_SERVICE     0x1523
#define HAPTIC_SERVICE_UUID_MOTOR_CHAR  0x1524


// Forward declaration of the haptic_service_t type.
typedef struct haptic_service_s haptic_service_t;

typedef void (*haptic_service_write_handler_t) (uint8_t motor_state);

struct haptic_service_s
{
    uint16_t                    service_handle;      /* Handle of Haptic Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    motor_char_handles;  /* Handles related to the Motor Characteristic. */
    uint8_t                     uuid_type;           /* UUID type for the Haptic Service. */
    haptic_service_write_handler_t write_handler;
};


/* Function for initializing the Haptic Service. */
uint32_t haptic_service_init(haptic_service_t * p_haptic_service);

void haptic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);