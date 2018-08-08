/* NeoPixel LEDs Service Server module. Used by the ws2812.h code.
 * The singleton instance is created statically there. */

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/*   Macro for defining a haptic_service instance. */
#define WS2812_SERVICE_DEF(_name)                                                                \
static ws2812_service_t _name;                                                                   \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                              \
                     BLE_HIDS_BLE_OBSERVER_PRIO,                                                 \
                     ws2812_service_on_ble_evt, &_name)

#define WS2812_SERVICE_UUID_BASE        {0x27, 0x2A, 0xD9, 0xB2, 0x2D, 0x7D, 0x4D, 0x56, 0xAC, 0x63, 0x38, 0xE8, 0xB9, 0x28, 0xF1, 0x0F}
#define WS2812_SERVICE_UUID_SERVICE         0x1223
#define WS2812_SERVICE_UUID_LED_CHAR        0x1224

// Forward declaration of the ws2812_service_t type.
typedef struct ws2812_service_s ws2812_service_t;

typedef void (*ws2812_service_leds_write_handler_t) (uint8_t const * rgb);

/* WS2812 Service structure. This structure contains various status information for the service. */
struct ws2812_service_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    leds_char_handles;
    uint8_t                     uuid_type;
    ws2812_service_leds_write_handler_t leds_write_handler;
};


/* Function for initializing the WS2812 Service. */
uint32_t ws2812_service_init(ws2812_service_t * p_ws2812_service);


void ws2812_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);