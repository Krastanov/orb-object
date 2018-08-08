#include "ws2812_service.h"

#include "sdk_macros.h" // TODO verify that appropriate error checking macros are used
#include "app_error.h" // TODO remove this header and APP_ERROR_CHECK and use VERIFY instead.

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