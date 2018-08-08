#include "battery_service.h"

#include "nrfx_saadc.h"
#include "ble_bas.h"
#include "app_timer.h"
#include "nrf_log.h"


#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(60000) /* Battery level measurement interval (ticks). */
#define BATTERY_420mV                   178

BLE_BAS_DEF(m_bas);                     /* Battery Service instance. */

APP_TIMER_DEF(m_battery_timer_id);      /* Battery measurement and BLE notification timer. */


static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    //NRF_LOG_DEBUG("ADC event");
}


static void saadc_init(uint8_t analog_in) // TODO sdk_config, call calibration routines (wait for done event), and doing it async
{
    ret_code_t err_code;
    nrfx_saadc_config_t config = NRFX_SAADC_DEFAULT_CONFIG;
    nrf_saadc_channel_config_t channel_config =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(analog_in);
    config.resolution = NRF_SAADC_RESOLUTION_8BIT;
    channel_config.acq_time = NRF_SAADC_ACQTIME_40US;

    err_code = nrfx_saadc_init(&config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_channel_init(0, &channel_config); // TODO channel 0 is kinda like a global variable
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_calibrate_offset();
    APP_ERROR_CHECK(err_code);
}


static void battery_level_update(void)
{
    ret_code_t err_code;
    nrf_saadc_value_t battery_measurement;
    uint16_t battery_mv, battery_level;

    err_code = nrfx_saadc_sample_convert(0, &battery_measurement); // TODO channel 0 is kinda like a global variable
    APP_ERROR_CHECK(err_code);
    battery_mv = battery_measurement*420/BATTERY_420mV;
    battery_level = MAX(battery_mv-360,0);
    battery_level = (battery_level*100)/60;
    battery_level = MIN(battery_level,100);
    battery_level /= 10;
    battery_level *= 10;
    NRF_LOG_DEBUG("Battery: raw %d, %dmV, %d%%", battery_measurement, battery_mv, battery_level);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


void battery_init(uint8_t analog_in)
{
    ret_code_t         err_code;
    ble_bas_init_t     bas_init;

    saadc_init(analog_in);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


void battery_start()
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}