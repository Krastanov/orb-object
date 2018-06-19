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
#define HAPTIC_SERVICE_UUID_BUTTON_CHAR 0x1524
#define HAPTIC_SERVICE_UUID_MOTOR_CHAR  0x1525


// Forward declaration of the haptic_service_t type.
typedef struct haptic_service_s haptic_service_t;

typedef void (*haptic_service_motor_write_handler_t) (uint16_t conn_handle, haptic_service_t * p_haptic_service, uint8_t new_state);

/** @brief Haptic Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    haptic_service_motor_write_handler_t motor_write_handler; /**< Event handler to be called when the Motor Characteristic is written. */
} haptic_service_init_t;

/**@brief Haptic Service structure. This structure contains various status information for the service. */
struct haptic_service_s
{
    uint16_t                    service_handle;      /**< Handle of Haptic Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    motor_char_handles;  /**< Handles related to the Motor Characteristic. */
    ble_gatts_char_handles_t    button_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the Haptic Service. */
    haptic_service_motor_write_handler_t motor_write_handler;   /**< Event handler to be called when the Motor Characteristic is written. */
};


/**@brief Function for initializing the Haptic Service.
 *
 * @param[out] p_haptic_service      Haptic Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_haptic_service_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t haptic_service_init(haptic_service_t * p_haptic_service, const haptic_service_init_t * p_haptic_service_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Haptic Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  Haptic Service structure.
 */
void haptic_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_haptic_service         Haptic Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t haptic_service_on_button_change(uint16_t conn_handle, haptic_service_t * p_haptic_service, uint8_t button_state);


#ifdef __cplusplus
}
#endif

#endif // HAPTIC_SERVICE_H__

/** @} */
