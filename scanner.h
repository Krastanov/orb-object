// TODO cleanup and documentation... certain uuids should be placed in a better, more central place

#define APP_BLE_OBSERVER_PRIO   3    /**< Application's BLE observer priority. You shouldn't need to modify this value. */

static ble_gap_scan_params_t scan_params = {
    .extended=0, .report_incomplete_evts=0, .active=0, .filter_policy=BLE_GAP_SCAN_FP_ACCEPT_ALL, .scan_phys=BLE_GAP_PHY_AUTO,
    .interval=2000*10, .window=2000*10, .timeout=2000*10};
static uint8_t scan_buffer[200];
static ble_data_t scan_data = {.p_data = scan_buffer, .len = 100};

static uint8_t scanner_check_uuid(ble_data_t * adv_data)
{
    if (adv_data->len!=24) return 0;
    uint8_t beacon_uuid[] = {0x3a, 0xa8, 0x0a, 0x58, 0x9e, 0xba, 0x4c, 0xcd, 0xbc, 0x73, 0x1f, 0x16, 0x88, 0x00, 0xbb, 0x81};
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
                NRF_LOG_DEBUG("ADV EVENT RSSI %d DATA %d %d", report->rssi, report->data.p_data[report->data.len-2], report->data.p_data[report->data.len-1]);
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
    err_code = sd_ble_gap_scan_start(&scan_params,&scan_data);
    APP_ERROR_CHECK(err_code);
    NRF_SDH_BLE_OBSERVER(m_ble_scanner, APP_BLE_OBSERVER_PRIO, scanner_ble_evt_handler, NULL);
}