/* MPU driver and BLE interface.
 *
 * The MPU driver is implemented here. Calls to the sensor fusion algorithms are done here. Call to BLE notification handlers are done here. The code is tightly coubled, not meant to be reusable.
 */

#include <stdint.h>

#define MAXMX  68.44 // TODO these should be programatically accessible
#define MINMX -28.77
#define MAXMY  31.98
#define MINMY -61.47
#define MAXMZ  58.05
#define MINMZ -36.86

enum Ascale_enum
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale_enum
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale_enum
{
  MFS_14BITS = 0, // 0.6 uT per LSB
  MFS_16BITS      // 0.15 uT per LSB
};

// Specify sensor full scale
static const uint8_t Gscale = GFS_250DPS;
static const uint8_t Ascale = AFS_2G;
static const uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution

// TODO verify these experimentally... there is definitely something wrong with the a (it is off by a factor of 2)
// TODO in the name of all that is holly and satanic, why is there a factor of two here!?
static const float Gres = 250.0/32768.0*3.1415/180; // in units of rad/s
static const float Ares = 2.0/32768.0*2; // in units of g
static const float Mres = 0.15; // in units of uTesla

static const uint8_t Mmode = 0x06; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

/* Initialize the MPU9250. It internally takes care of TWI initialization. The MPU is set to 200Hz and the MAG is set to 100Hz. */
void mpu_init(uint8_t scl, uint8_t sda, uint8_t irq, uint8_t * p_adv_handle);

// TODO make the following convenience functions

//  general read/write functions

//  set accel, gyro range

//  set mag precision

//  set filter

//  set sample rate

//  check self-test results for accel and gyro

//  check self-test results for magnetometer

//  set accel and gyro bias (self-calibration when charging as the orientation will be known)

//  set magnetometer bias

//  set FIFO (default off, but using it at appropriate frequency would make the predictions more stable as otherwise the code has strong timing requirements)
// register 26.6, 35

//  wake on motion
// register 31