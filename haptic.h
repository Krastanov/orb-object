/* Haptic driver.
 *
 * The PWM based haptic driver is implemented here. The BLE characteristic is
 * also called from here - the code is tightly coupled and not particularly
 * reusable. A singleton global variable is used, i.e. you can not run multiple
 * instances of this driver.
 */

#include <stdint.h>

void haptic_init(uint8_t pin);