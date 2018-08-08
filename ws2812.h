/* WS2812B driver.
 *
 * The WS2812B/NeoPixel driver is implemented here.
 * It is abusing the i2s hardware on the nrf52 chip.
 * The BLE characteristic is also called from here - the code
 * is tightly coupled and not particularly reusable.
 */

#include <stdint.h>

#define NLEDS 7  // TODO this should be accessible in the init function


void ws2812_set_color(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);

void ws2812_clear_buffer();

uint32_t ws2812_display();

void ws2812_set_uniform_color_and_display(uint8_t const * rgb);

void ws2812_init(uint8_t outpin, uint8_t unused_sckpin, uint8_t unused_lrckpin);