/** @file
 *
 * @defgroup ble_mpu MPU Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief Motion Service Server module. Used by the mpu.h code. The singleton instance is created statically there.
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

#ifndef MPU_SERVICE_H__
#define MPU_SERVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a mpu_service instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define MPU_SERVICE_DEF(_name)                                                                   \
static mpu_service_t _name;                                                                      \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                              \
                     BLE_HIDS_BLE_OBSERVER_PRIO,                                                 \
                     mpu_service_on_ble_evt, &_name)

#define MPU_SERVICE_UUID_BASE        {0x0A, 0x72, 0x1F, 0x75, 0x91, 0xDF, 0x45, 0x51, 0x84, 0x04, 0x0C, 0xAF, 0x06, 0xAE, 0xB9, 0x1C}
#define MPU_SERVICE_UUID_SERVICE         0x1623
#define MPU_SERVICE_UUID_QUAT_CHAR       0x1624
#define MPU_SERVICE_UUID_KINACCEL_CHAR   0x1625

// Forward declaration of the mpu_service_t type.
typedef struct mpu_service_s mpu_service_t;

/**@brief MPU Service structure. This structure contains various status information for the service. */
struct mpu_service_s
{
    uint16_t                    service_handle;        /**< Handle of MPU Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    quat_char_handles;     /**< Handles related to the Quaternion Characteristic. */
    ble_gatts_char_handles_t    kinaccel_char_handles; /**< Handles related to the Kinematic Acceleration Characteristic. */
    uint8_t                     uuid_type;             /**< UUID type for the MPU Service. */
};

/**@brief Function for adding all the MPU characteristic.
 *
 * @param[in] p_mpu_service      MPU Service structure.
 */
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

/**@brief Function for initializing the MPU Service.
 *
 * @param[out] p_mpu_service      MPU Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
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


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the MPU Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  MPU Service structure.
 */
void mpu_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    haptic_service_t * p_haptic_service = (haptic_service_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for sending an orientation notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_mpu_service         MPU Service structure.
 * @param[in] q[]      Quaternion components.
 * @param[in] ka[]     Kinematic acceleration component.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t mpu_service_on_orientation_change(uint16_t conn_handle, mpu_service_t * p_mpu_service, float q0, float q1, float q2, float q3, float kax, float kay, float kaz)
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
    err_code = sd_ble_gatts_hvx(conn_handle, &paramsq);
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
    err_code = sd_ble_gatts_hvx(conn_handle, &paramsk);
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


#ifdef __cplusplus
}
#endif

#endif // MPU_SERVICE_H__

/** @} */
