/** @file
 *
 * @defgroup ble_ws2812 WS2812 Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief NeoPixel LEDs Service Server module. Used by the ws2812.h code. The singleton instance is created statically there.
 *
 * @details ...
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_hids_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HIDS_BLE_OBSERVER_PRIO,
 *                                   ble_hids_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef WS2812_SERVICE_H__
#define WS2812_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a haptic_service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
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

/**@brief WS2812 Service structure. This structure contains various status information for the service. */
struct ws2812_service_s
{
    uint16_t                    service_handle;        /**< Handle of WS2812 Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    leds_char_handles;     /**< Handles related to the LEDs Characteristic. */
    uint8_t                     uuid_type;             /**< UUID type for the WS2812 Service. */
    ws2812_service_leds_write_handler_t leds_write_handler;  /**< Handler for write events to the LEDs. */
};

/**@brief Function for adding all the WS2812 characteristic.
 *
 * @param[in] p_ws2812_service      WS2812 Service structure.
 */
static uint32_t ws2812_char_add(ws2812_service_t * p_ws2812_service)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    ble_uuid.type = p_ws2812_service->uuid_type;
    ble_uuid.uuid = WS2812_SERVICE_UUID_LED_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 3;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 3;
    attr_char_value.p_value   = NULL;

    uint32_t   err_code;
    err_code = sd_ble_gatts_characteristic_add(p_ws2812_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_ws2812_service->leds_char_handles);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the WS2812 Service.
 *
 * @param[out] p_ws2812_service      WS2812 Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ws2812_service_init(ws2812_service_t * p_ws2812_service)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Add service.
    ble_uuid128_t base_uuid = {WS2812_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ws2812_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_ws2812_service->uuid_type;
    ble_uuid.uuid = WS2812_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ws2812_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    ws2812_char_add(p_ws2812_service);

    return NRF_SUCCESS;
}


/**@brief Function for handling the Write event.
 *
 * @param[in] p_ws2812_service      Haptic Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ws2812_service_t * p_ws2812_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if (   (p_evt_write->handle == p_ws2812_service->leds_char_handles.value_handle)
        && (p_evt_write->len == 3)
        && (p_ws2812_service->leds_write_handler != NULL))
    {
        p_ws2812_service->leds_write_handler(p_evt_write->data);
    }
}


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the WS2812 Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  WS2812 Service structure.
 */
void ws2812_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ws2812_service_t * p_ws2812_service = (ws2812_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_ws2812_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

#ifdef __cplusplus
}
#endif

#endif // WS2812_SERVICE_H__

/** @} */
