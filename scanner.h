#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

static ble_gap_scan_params_t scan_params = {
    .extended=0, .report_incomplete_evts=0, .active=0, .filter_policy=BLE_GAP_SCAN_FP_ACCEPT_ALL, .scan_phys=BLE_GAP_PHY_AUTO,
    .interval=2000*10, .window=2000*10, .timeout=2000*10};
static uint8_t scan_buffer[200];
static ble_data_t scan_data = {.p_data = scan_buffer, .len = 100};

static void scanner_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * report = &p_ble_evt->evt.gap_evt.params.adv_report;
            NRF_LOG_DEBUG("ADV EVENT RSSI %d DATA %d %d", report->rssi, report->data.p_data[report->data.len-2], report->data.p_data[report->data.len-1]);
            sd_ble_gap_scan_start(NULL, &scan_data);
        } break;

        default:
            // No implementation needed.
            break;
    }
}

void scanner_init()
{
    NRF_SDH_BLE_OBSERVER(m_ble_scanner, APP_BLE_OBSERVER_PRIO, scanner_ble_evt_handler, NULL);
}