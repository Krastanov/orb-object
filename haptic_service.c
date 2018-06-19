#include "sdk_common.h"
#include "haptic_service.h"
#include "ble_srv_common.h"


/**@brief Function for handling the Write event.
 *
 * @param[in] p_haptic_service      Haptic Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(haptic_service_t * p_haptic_service, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_haptic_service->motor_char_handles.value_handle)
        && (p_evt_write->len == 1)
        && (p_haptic_service->motor_write_handler != NULL))
    {
        p_haptic_service->motor_write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_haptic_service, p_evt_write->data[0]);
    }
}


void haptic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    haptic_service_t * p_haptic_service = (haptic_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_haptic_service, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Motor Characteristic.
 *
 * @param[in] p_haptic_service      Haptic Service structure.
 * @param[in] p_haptic_service_init Haptic Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t motor_char_add(haptic_service_t * p_haptic_service, const haptic_service_init_t * p_haptic_service_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    ble_uuid.type = p_haptic_service->uuid_type;
    ble_uuid.uuid = HAPTIC_SERVICE_UUID_MOTOR_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_haptic_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_haptic_service->motor_char_handles);
}


/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_haptic_service      Haptic Service structure.
 * @param[in] p_haptic_service_init Haptic Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t button_char_add(haptic_service_t * p_haptic_service, const haptic_service_init_t * p_haptic_service_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_haptic_service->uuid_type;
    ble_uuid.uuid = HAPTIC_SERVICE_UUID_BUTTON_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_haptic_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_haptic_service->button_char_handles);
}


uint32_t haptic_service_init(haptic_service_t * p_haptic_service, const haptic_service_init_t * p_haptic_service_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_haptic_service->motor_write_handler = p_haptic_service_init->motor_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {HAPTIC_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_haptic_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_haptic_service->uuid_type;
    ble_uuid.uuid = HAPTIC_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_haptic_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = button_char_add(p_haptic_service, p_haptic_service_init);
    VERIFY_SUCCESS(err_code);

    err_code = motor_char_add(p_haptic_service, p_haptic_service_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


uint32_t haptic_service_on_button_change(uint16_t conn_handle, haptic_service_t * p_haptic_service, uint8_t button_state)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);

    memset(&params, 0, sizeof(params));
    params.type   = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_haptic_service->button_char_handles.value_handle;
    params.p_data = &button_state;
    params.p_len  = &len;

    return sd_ble_gatts_hvx(conn_handle, &params);
}
