/**
 * @brief Haptic driver.
 *
 * The PWM based haptic driver is implemented here. The BLE characteristic is also called from here - the code is tightly coupled and not particularly reusable. A singleton global variable is used, i.e. you can not run multiple instances of this driver.
 */

#include "nrfx_pwm.h"
#include "nrf_gpio.h"

#include "haptic_service.h"

static nrfx_pwm_t m_pwm = NRFX_PWM_INSTANCE(0);
HAPTIC_SERVICE_DEF(m_haptic_service);                                           /**< Haptic Service instance. */


static uint8_t motor_pin;


static void haptic_set_state(uint8_t motor_state)
{
    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static uint16_t /*const*/ seq1_values[9] = {256,  0,256,128,256,  0,128,256,256};
    static uint16_t /*const*/ seq2_values[9] = {256,  0,256,  0,256,  0,256,  0,256};
    static uint16_t /*const*/ seq3_values[9] = {256,128, 64, 64, 32, 32, 16, 16,  0};
    static uint16_t /*const*/ seq4_values[9] = {  0, 16, 32, 64,128,256,512,512,  0};

    uint16_t *seqs[] = {seq1_values, seq2_values, seq3_values, seq4_values};

    nrf_pwm_sequence_t seq =
    {
        .values.p_grouped = seqs[0],
        .length           = 9,
        .repeats          = 64*200,
        .end_delay        = 0
    };

    if (motor_state == 1)
    {
        nrf_gpio_pin_set(motor_pin);
    } else
    {
        nrf_gpio_pin_clear(motor_pin);
    }
    
    if (motor_state > 1 && motor_state < 6)
    {
        seq.values.p_grouped = seqs[motor_state-2];
        nrfx_pwm_simple_playback(&m_pwm, &seq, 1, NRFX_PWM_FLAG_STOP);
    } 
}


void haptic_init(uint8_t pin)
{
    uint32_t err_code;

    motor_pin = pin;
    nrf_gpio_cfg_output(motor_pin);
    nrf_gpio_pin_clear(motor_pin);

    m_haptic_service.write_handler = haptic_set_state;
    err_code = haptic_service_init(&m_haptic_service);
    APP_ERROR_CHECK(err_code);

    nrfx_pwm_config_t config =
    {
        .output_pins  = { pin,
                          NRFX_PWM_PIN_NOT_USED,
                          NRFX_PWM_PIN_NOT_USED,
                          NRFX_PWM_PIN_NOT_USED },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 256-1,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO,
    };
    nrfx_pwm_init(&m_pwm, &config, NULL);
    APP_ERROR_CHECK(err_code);
}