#include "mpu_service.h"

#include "sdk_macros.h" // TODO verify that appropriate error checking macros are used
#include "app_error.h" // TODO remove this header and APP_ERROR_CHECK and use VERIFY instead.

#include "ble_conn_state.h"

#include "nrf_log.h"

static void mpu_char_add(mpu_service_t * p_mpu_service)
{
    uint32_t   err_code;

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

    ble_uuid.type = p_mpu_service->uuid_type;
    ble_uuid.uuid = MPU_SERVICE_UUID_QUAT_CHAR;

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
    attr_char_value.init_len  = 16; // 4*sizeof(float);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 16; // 4*sizeof(float);
    attr_char_value.p_value   = NULL;

    err_code = sd_ble_gatts_characteristic_add(p_mpu_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mpu_service->quat_char_handles);
    VERIFY_SUCCESS(err_code);


    ble_uuid.uuid = MPU_SERVICE_UUID_KINACCEL_CHAR;
    attr_char_value.init_len  = 12; // 3*sizeof(float);
    attr_char_value.max_len   = 12; // 3*sizeof(float);
    err_code = sd_ble_gatts_characteristic_add(p_mpu_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_mpu_service->kinaccel_char_handles);
    VERIFY_SUCCESS(err_code);
}


uint32_t mpu_service_init(mpu_service_t * p_mpu_service)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Add service.
    ble_uuid128_t base_uuid = {MPU_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_mpu_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_mpu_service->uuid_type;
    ble_uuid.uuid = MPU_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_mpu_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    mpu_char_add(p_mpu_service);

    return NRF_SUCCESS;
}


void mpu_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    switch (p_ble_evt->header.evt_id)
    {
        default:
            // No implementation needed.
            break;
    }
}


static uint32_t notify_all(ble_gatts_hvx_params_t * params)
{
    uint32_t err_code = NRF_SUCCESS;
    ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();
    // Try sending notifications to all valid connection handles.
    for (uint32_t i = 0; i < conn_handles.len; i++)
    {
        if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
        {
            if (err_code == NRF_SUCCESS)
            {
                err_code = sd_ble_gatts_hvx(conn_handles.conn_handles[i], params);
            }
            else
            {
                // Preserve the first non-zero error code
                UNUSED_RETURN_VALUE(sd_ble_gatts_hvx(conn_handles.conn_handles[i], params));
            }
        }
    }
    return err_code;
}


uint32_t mpu_service_on_orientation_change(mpu_service_t * p_mpu_service, float q0, float q1, float q2, float q3, float kax, float kay, float kaz)
{ // TODO This should be sending bytes, not floats. It is waste of resources to send 32bits per number.
    uint32_t   err_code;

    ble_gatts_hvx_params_t paramsq;
    uint16_t lenq = 16;
    static float bufferq[4];
    bufferq[0]=q0; bufferq[1]=q1; bufferq[2]=q2; bufferq[3]=q3;
    
    memset(&paramsq, 0, sizeof(paramsq));
    paramsq.type   = BLE_GATT_HVX_NOTIFICATION;
    paramsq.handle = p_mpu_service->quat_char_handles.value_handle;
    paramsq.p_data = (uint8_t*)&bufferq;
    paramsq.p_len  = &lenq;
    err_code = notify_all(&paramsq);
    if (err_code != NRF_SUCCESS &&
        err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != NRF_ERROR_INVALID_STATE &&
        err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        if (err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_DEBUG("Not enough resources to send q[]");
        } else
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    ble_gatts_hvx_params_t paramsk;
    uint16_t lenk = 12;
    static float bufferk[3];
    bufferk[0]=kax; bufferk[1]=kay; bufferk[2]=kaz;;
    
    memset(&paramsk, 0, sizeof(paramsk));
    paramsk.type   = BLE_GATT_HVX_NOTIFICATION;
    paramsk.handle = p_mpu_service->kinaccel_char_handles.value_handle;
    paramsk.p_data = (uint8_t*)&bufferk;
    paramsk.p_len  = &lenk;
    err_code = notify_all(&paramsk);
    if (err_code != NRF_SUCCESS &&
        err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
        err_code != NRF_ERROR_INVALID_STATE &&
        err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        if (err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_DEBUG("Not enough resources to send ka[]");
        } else {
            APP_ERROR_CHECK(err_code);
        }
    }
}