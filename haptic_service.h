/** @file
 *
 * @defgroup ble_haptic Haptic Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief Haptic Service Server module.
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

#ifndef HAPTIC_SERVICE_H__
#define HAPTIC_SERVICE_H__

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

/**@brief Haptic Service structure. This structure contains various status information for the service. */
struct haptic_service_s
{
    uint16_t                    service_handle;      /**< Handle of Haptic Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    motor_char_handles;  /**< Handles related to the Motor Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Haptic Service. */
    haptic_service_write_handler_t write_handler;
};


/**@brief Function for adding the Motor Characteristic.
 *
 * @param[in] p_haptic_service      Haptic Service structure.
 * @param[in] p_haptic_service_init Haptic Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t motor_char_add(haptic_service_t * p_haptic_service)
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

    ble_uuid.type = p_haptic_service->uuid_type;
    ble_uuid.uuid = HAPTIC_SERVICE_UUID_MOTOR_CHAR;

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
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = NULL;

    return sd_ble_gatts_characteristic_add(p_haptic_service->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_haptic_service->motor_char_handles);
}


/**@brief Function for initializing the Haptic Service.
 *
 * @param[out] p_haptic_service      Haptic Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_haptic_service_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t haptic_service_init(haptic_service_t * p_haptic_service)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Add service.
    ble_uuid128_t base_uuid = {HAPTIC_SERVICE_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_haptic_service->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_haptic_service->uuid_type;
    ble_uuid.uuid = HAPTIC_SERVICE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_haptic_service->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = motor_char_add(p_haptic_service);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Haptic Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Haptic Service structure.
 */
void haptic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    haptic_service_t * p_haptic_service = (haptic_service_t *)p_context;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            if (   (p_evt_write->handle == p_haptic_service->motor_char_handles.value_handle)
                && (p_evt_write->len == 1))
            {
                p_haptic_service->write_handler(p_evt_write->data[0]);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


#ifdef __cplusplus
}
#endif

#endif // HAPTIC_SERVICE_H__

/** @} */