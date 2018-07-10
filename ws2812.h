/**
 * @brief WS2812B driver.
 *
 * The WS2812B/NeoPixel driver is implemented here. It is abusing the i2s hardware on the nrf52 chip. The BLE characteristic is also called from here - the code is tightly coupled and not particularly reusable.
 */

#include "nrfx_i2s.h"
#include "ws2812_service.h"

#define NLEDS 7  // TODO this should be accessible in the init function
#define RESET_BITS 6
#define I2S_BUFFER_SIZE 3*NLEDS + RESET_BITS

static uint32_t buffer_tx[I2S_BUFFER_SIZE]; // TODO should be allocated in the init function
static nrfx_i2s_buffers_t buffers = {.p_rx_buffer = NULL, .p_tx_buffer = buffer_tx};
WS2812_SERVICE_DEF(m_ws2812_service);

static void data_handler(nrfx_i2s_buffers_t const * p_released, uint32_t status)
{
    if (p_released->p_tx_buffer != NULL)// The tx buffer was just sent out
    {
        nrfx_i2s_stop();
    }
}

static uint32_t encode8to32(uint8_t level) // 1 is 1110, 0 is 1000
{
    uint32_t val = 0;

    if(level == 0)
    {
        val = 0x88888888;
    } else if (level == 255)
    {
        val = 0xeeeeeeee;
    } else
    {
        // apply 4-bit 0xe HIGH pattern wherever level bits are 1.
        val = 0x88888888;
        for (uint8_t i = 0; i < 8; i++) {
            if((1 << i) & level) {
                uint32_t mask = ~(0x0f << 4*i);
                uint32_t patt = (0x0e << 4*i);
                val = (val & mask) | patt;
            }
        }
        // swap 16 bits
        val = (val >> 16) | (val << 16);
    }
    return val;
}

void ws2812_set_color(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
    if (pixel>=NLEDS) return;
    buffer_tx[3*pixel]   = encode8to32(g);
    buffer_tx[3*pixel+1] = encode8to32(r);
    buffer_tx[3*pixel+2] = encode8to32(b);
}

void ws2812_clear_buffer()
{
    for (uint32_t i = 0; i<3*NLEDS; i++) buffer_tx[i] = 0x88888888;
}

uint32_t ws2812_display()
{
    return nrfx_i2s_start(&buffers, I2S_BUFFER_SIZE, 0);
}

void ws2812_set_uniform_color_and_display(uint8_t const * rgb)
{
    for (uint8_t i=0; i<NLEDS; i++)
    {
        buffer_tx[3*i]   = encode8to32(rgb[1]);
        buffer_tx[3*i+1] = encode8to32(rgb[0]);
        buffer_tx[3*i+2] = encode8to32(rgb[2]);
    }
    ws2812_display();
}

void ws2812_init(uint8_t outpin, uint8_t unused_sckpin, uint8_t unused_lrckpin) // TODO the unused_* pins are a very annoying leaky abstraction over this i2s hack
{
    ret_code_t err_code;

    nrfx_i2s_config_t config = NRFX_I2S_DEFAULT_CONFIG;
    config.sdin_pin  = NRFX_I2S_PIN_NOT_USED;
    config.sdout_pin = outpin;
    config.sck_pin   = unused_sckpin;
    config.lrck_pin  = unused_lrckpin;
    config.mck_setup = NRF_I2S_MCK_32MDIV10; ///< 32 MHz / 10 = 3.2 MHz.
    config.ratio     = NRF_I2S_RATIO_32X;    ///< LRCK = MCK / 32.
    config.channels  = NRF_I2S_CHANNELS_STEREO;

    err_code = nrfx_i2s_init(&config, data_handler);
    APP_ERROR_CHECK(err_code);

    m_ws2812_service.leds_write_handler = ws2812_set_uniform_color_and_display;
    err_code = ws2812_service_init(&m_ws2812_service);
    APP_ERROR_CHECK(err_code);
}

void ws2812_test()
{
    ws2812_clear_buffer();
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(0,50,50,50);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(1,50,0,0);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(2,0,50,0);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(3,0,0,50);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(4,50,0,50);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(5,50,50,0);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_set_color(6,0,50,50);
    ws2812_display();
    nrf_delay_ms(500);
    ws2812_clear_buffer();
    ws2812_display();
}