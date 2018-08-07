#include "scanner.h"

#include <math.h>

#include "ble.h"
#include "ble_gap.h"
#include "nrf_sdh_ble.h"

#include "sdk_errors.h"

#include "app_error.h" // TODO remove this header and APP_ERROR_CHECK and use VERIFY instead.

#include "nrf_log.h"

// TODO cleanup and documentation... certain uuids should be placed in a better, more central place

#define SCAN_APP_BLE_OBSERVER_PRIO   3    /**< Application's BLE observer priority. You shouldn't need to modify this value. */


static ble_gap_scan_params_t scan_params = {
    .extended=0, .report_incomplete_evts=0, .active=0, .filter_policy=BLE_GAP_SCAN_FP_ACCEPT_ALL, .scan_phys=BLE_GAP_PHY_AUTO,
    .interval=SCAN_INTERVAL, .window=SCAN_WINDOW,
    .timeout=BLE_GAP_SCAN_TIMEOUT_UNLIMITED};
static uint8_t scan_buffer[200];
static ble_data_t scan_data = {.p_data = scan_buffer, .len = 100};

static multilateration_state_t m_multilatstate;


static uint8_t scanner_check_uuid(ble_data_t * adv_data)
{
    if (adv_data->len!=23) return 0;
    uint8_t beacon_uuid[] = MULTILATERATION_BEACON_UUID;
    for (uint8_t i=0; i<sizeof(beacon_uuid); i++)
    {
        if (beacon_uuid[i]!=adv_data->p_data[5+i]) return 0;
    }
    return 1;
}


static void scanner_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = sd_ble_gap_scan_stop();            
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = sd_ble_gap_scan_start(&scan_params,&scan_data);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * report = &p_ble_evt->evt.gap_evt.params.adv_report;
            if (scanner_check_uuid(&report->data))
            {
                //NRF_LOG_DEBUG("ADV EVENT MAC %x:%x RSSI %d LEN %d DATA %d %d",
                //              report->peer_addr.addr[5], report->peer_addr.addr[4],
                //              report->rssi, report->data.len, report->data.p_data[report->data.len-2], report->data.p_data[report->data.len-1]);
                float distance = (report->rssi-MULTILATERATION_INTERCEPT)/MULTILATERATION_SLOPE;
                if (distance<MULTILATERATION_MAX_DISTANCE) {
                    if (distance<MULTILATERATION_MIN_DISTANCE) distance=MULTILATERATION_MIN_DISTANCE;
                    multilateration_insert(&m_multilatstate,
                                           report->data.p_data[report->data.len-2],
                                           report->data.p_data[report->data.len-1],
                                           distance);
                    if (m_multilatstate.start==0 && m_multilatstate.stop==MULTILATERATION_BUFFER_SIZE-1) {
                        uint32_t retcode = multilateration(&m_multilatstate);
                        float X,Y,Xn,Yn;
                        uint32_t retcode1 = multilateration_getnearest(&m_multilatstate, &Xn, &Yn);
                        uint32_t retcode2 = multilateration_getXY(&m_multilatstate, &X, &Y);
                        NRF_LOG_DEBUG("error code %d %d %d", retcode, retcode1, retcode2);
                        NRF_LOG_DEBUG("estimated loc  : X " NRF_LOG_FLOAT_MARKER " Y " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(X), NRF_LOG_FLOAT(Y));
                        NRF_LOG_DEBUG("closest beacon : X " NRF_LOG_FLOAT_MARKER " Y " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(Xn), NRF_LOG_FLOAT(Yn));
                    }
                }

            }
            sd_ble_gap_scan_start(NULL, &scan_data);
        } break;

        default:
            break;
    }
}


void scanner_init()
{
    ret_code_t err_code;
    multilateration_init(&m_multilatstate, MULTILATERATION_BUFFER_SIZE); // TODO error check
    err_code = sd_ble_gap_scan_start(&scan_params,&scan_data);
    APP_ERROR_CHECK(err_code);
    NRF_SDH_BLE_OBSERVER(m_ble_scanner, SCAN_APP_BLE_OBSERVER_PRIO, scanner_ble_evt_handler, NULL);
}