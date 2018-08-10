/* MPU reporting Service Server module. Used by the mpu.h code.
 * The singleton instance is created statically there. */

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/* Macro for defining a mpu_service instance. */
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

/* MPU Service structure. This structure contains various status information for the service. */
struct mpu_service_s
{
    uint16_t                    service_handle;        /**< Handle of MPU Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    quat_char_handles;     /**< Handles related to the Quaternion Characteristic. */
    ble_gatts_char_handles_t    kinaccel_char_handles; /**< Handles related to the Kinematic Acceleration Characteristic. */
    uint8_t                     uuid_type;             /**< UUID type for the MPU Service. */
};

/* Function for initializing the MPU Service. */
uint32_t mpu_service_init(mpu_service_t * p_mpu_service);

/* Function for handling the application's BLE stack events. */
void mpu_service_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

/* Function for sending an orientation notification.
 *
 * conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * p_mpu_service         MPU Service structure.
 * q[]      Quaternion components.
 * ka[]     Kinematic acceleration component.
 *
 * Returns NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t mpu_service_on_orientation_change(mpu_service_t * p_mpu_service, float q0, float q1, float q2, float q3, float kax, float kay, float kaz);