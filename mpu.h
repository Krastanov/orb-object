#include "nrfx_twim.h"
#include "nrf_twi_sensor.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrfx_gpiote.h"
#include "MadgwickAHRS.h"

#define MPU_ADDR 0b1101001

#define MPU_WHO_ADDR 0x75 
#define MPU_WHO_9250 0x71
#define MPU_WHO_9255 0x73

#define MPU_PWR_MGMT_1 0x6B
#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_CONFIG2 0x1D

#define MPU_INT_PIN_CFG 0x37
#define MPU_INT_ENABLE 0x38

#define MPU_ACCEL_ADDR 0x3B // 59 to 64, 3*2 bytes
#define MPU_TEMP_ADDR 0x41  // 65 and 66, 2 bytes
#define MPU_GYRO_ADDR 0x43  // 67 to 72, 3*2 bytes 
#define MPU_MAG_ADDR 0x49   // 73 to 78, 3*2 bytes
#define MPU_SENS_ADDR 0x3B
#define MPU_ACCEL_OFFSET 0
#define MPU_TEMP_OFFSET 6
#define MPU_GYRO_OFFSET 8
#define MPU_MAG_OFFSET 14
#define MPU_MAG_OVERFLOW_OFFSET 20
#define MPU_SENS_SIZE 21

#define MAG_ADDR 0x0C
#define MAG_WHO_ADDR 0x00
#define MAG_WHO 0x48
#define MAG_CNTL 0x0A // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define MAG_ASAX 0x10 // Fuse ROM x-axis sensitivity adjustment value
#define MAG_ST1 0x02 // data ready status bit 0
#define MAG_XOUT_L 0x03 // data
#define MAG_XOUT_H 0x04
#define MAG_YOUT_L 0x05
#define MAG_YOUT_H 0x06
#define MAG_ZOUT_L 0x07
#define MAG_ZOUT_H 0x08
#define MAG_ST2 0x09 // Data overflow bit 3 and data read error status bit 2

enum Ascale
{
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale
{
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale
{
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
static const uint8_t Gscale = GFS_250DPS;
static const uint8_t Ascale = AFS_2G;
static const uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution

static const float gres = 250.0/32768.0; // in units of deg/s
static const float ares = 2.0/32768.0; // in units of g
static const float mres = 10.*4912/32768.0; // in units of mG

static const uint8_t Mmode = 0x06; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

static uint8_t sens_buffer[MPU_SENS_SIZE];  /**< Buffer for all sensor data (accel, temp, gyro, mag). */ // TODO explain why this is not volatile.
static float magcalibration[3]; // Calibration values for the magnetometer.

NRF_TWI_MNGR_DEF(m_twi_mngr, 4, 0);         /**< TWI transaction manager.*/
NRF_TWI_SENSOR_DEF(m_mpu, &m_twi_mngr, 30); /**< TWI Sensor instance.*/

void fusion_accel_gyro()
{
    float gx, gy, gz, ax, ay, az;
    gx = (float)(((int16_t)sens_buffer[ 0]<<8) | sens_buffer[ 1]) * gres;
    gy = (float)(((int16_t)sens_buffer[ 2]<<8) | sens_buffer[ 3]) * gres;
    gz = (float)(((int16_t)sens_buffer[ 4]<<8) | sens_buffer[ 5]) * gres;
    ax = (float)(((int16_t)sens_buffer[ 8]<<8) | sens_buffer[ 9]) * gres;
    ay = (float)(((int16_t)sens_buffer[10]<<8) | sens_buffer[11]) * gres;
    az = (float)(((int16_t)sens_buffer[12]<<8) | sens_buffer[13]) * gres;
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
}

void fusion_accel_gyro_mag()
{
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    gx = (float)(((int16_t)sens_buffer[ 0]<<8) | sens_buffer[ 1]) * gres;
    gy = (float)(((int16_t)sens_buffer[ 2]<<8) | sens_buffer[ 3]) * gres;
    gz = (float)(((int16_t)sens_buffer[ 4]<<8) | sens_buffer[ 5]) * gres;
    ax = (float)(((int16_t)sens_buffer[ 8]<<8) | sens_buffer[ 9]) * gres;
    ay = (float)(((int16_t)sens_buffer[10]<<8) | sens_buffer[11]) * gres;
    az = (float)(((int16_t)sens_buffer[12]<<8) | sens_buffer[13]) * gres;
    mx = (float)(((int16_t)sens_buffer[15]<<8) | sens_buffer[14]) * gres;
    my = (float)(((int16_t)sens_buffer[17]<<8) | sens_buffer[16]) * gres;
    mz = (float)(((int16_t)sens_buffer[19]<<8) | sens_buffer[18]) * gres;
    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
}

/**@brief Check the AK8963 readings. Call sensor fusion accordingly. To be set as callback from inside check_mag_ready_cb. If this is not fast, everything breaks.
 */
void read_mag_cb(ret_code_t result, void* p_register_data)
{
    APP_ERROR_CHECK(result);
    uint8_t* data = (uint8_t*)p_register_data;
    if(!(data[6] & 0x08))
    {
        fusion_accel_gyro_mag();
    } else
    {
        fusion_accel_gyro();
    }
}

/**@brief Check if the AK8963 said it is ready. Read it and call sensor fusion accordingly. To be set as callback from inside mpu_read_and_fuse_sensors. If this is not fast, everything breaks.
 */
void check_mag_ready_cb(ret_code_t result, void* p_register_data)
{
    APP_ERROR_CHECK(result);
    uint8_t status = ((uint8_t*)p_register_data)[0] & 0x01;
    if (status)
    {
        ret_code_t err_code;
        err_code = nrf_twi_sensor_reg_read(&m_mpu, MAG_ADDR, MAG_XOUT_L, read_mag_cb, sens_buffer+MPU_MAG_OFFSET, 6+1); //x, y, z, and st2 bit.
        APP_ERROR_CHECK(err_code);
    } else
    {
        fusion_accel_gyro();
    }
}

/**@brief Read all sensor data from the MPU9250. To be called from interrupt.
 */
void mpu_read_and_fuse_sensors()
{
    // read accel/gyro
    // check mag
    //   if available read and run accel/gyro/mag fusion
    //   else run accel/gyro fusion
    ret_code_t err_code;
    err_code = nrf_twi_sensor_reg_read(&m_mpu, MPU_ADDR, MPU_SENS_ADDR, NULL, sens_buffer, MPU_MAG_OFFSET);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_twi_sensor_reg_read(&m_mpu, MAG_ADDR, MAG_ST1, check_mag_ready_cb, sens_buffer+MPU_MAG_OFFSET, 1); // TODO Unless we are using the fancy i2c chaining, using the same buffer for MPU and MAG is not necessary. We are not using the fancy i2c chaining.
    APP_ERROR_CHECK(err_code);
}

void irq_callback(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    mpu_read_and_fuse_sensors();
}

// For use only inside mpu_init!
#define write_byte(addr, data) buffer[0]=addr; buffer[1]=data; APP_ERROR_CHECK(nrfx_twim_tx(&twi_master, MPU_ADDR, buffer, 2, false))
#define read_byte(addr) buffer[0]=addr; APP_ERROR_CHECK(nrfx_twim_tx(&twi_master, MPU_ADDR, buffer, 1, true)); APP_ERROR_CHECK(nrfx_twim_rx(&twi_master, MPU_ADDR, buffer, 1))
#define write_mag_byte(addr, data) buffer[0]=addr; buffer[1]=data; APP_ERROR_CHECK(nrfx_twim_tx(&twi_master, MAG_ADDR, buffer, 2, false))
#define read_mag_byte(addr) buffer[0]=addr; APP_ERROR_CHECK(nrfx_twim_tx(&twi_master, MAG_ADDR, buffer, 1, true)); APP_ERROR_CHECK(nrfx_twim_rx(&twi_master, MAG_ADDR, buffer, 1))

/**@brief Initialize the MPU9250. It internally takes care of TWI initialization. The MPU is set to 200Hz and the MAG is set to 100Hz.
 */
void mpu_init(uint8_t scl, uint8_t sda, uint8_t irq) // based on github.com/kriswiner/MPU9250
{
    // TODO use DPM and/or FIFO
    // TODO set low-pass filter
    // TODO self test for accel/gyro/mag
    // TODO calibrate biases for accel/gyro/mag

    ret_code_t err_code;
    uint8_t buffer[2];

    // Initialize twi without manager in blocking mode.
    const nrfx_twim_t twi_master = NRFX_TWIM_INSTANCE(0);
    nrfx_twim_config_t const config = {
        .scl                = scl,
        .sda                = sda,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
        .hold_bus_uninit    = true
    };
    nrfx_twim_init(&twi_master, &config, NULL, NULL);
    nrfx_twim_enable(&twi_master);



    // Check connection to MPU.
    read_byte(MPU_WHO_ADDR);
    if (buffer[0]!=MPU_WHO_9250 && buffer[0]!=MPU_WHO_9255)
    {
        NRF_LOG_DEBUG("MPU not recognized. WHO_AM_I is 0x%x", buffer[0]);
        APP_ERROR_CHECK(1); // TODO proper error code
    }
    NRF_LOG_DEBUG("MPU recognized.");

    // Wake up device.
    write_byte(MPU_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
    nrf_delay_ms(100); // Wait for all registers to reset 

    // Get stable time source.
    write_byte(MPU_PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    nrf_delay_ms(200);
  
    // Configure Gyro and Thermometer.
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    write_byte(MPU_CONFIG, 0x03);  

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV).
    write_byte(MPU_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                       // determined inset in CONFIG above
 
    // Set gyroscope full scale range.
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    read_byte(MPU_GYRO_CONFIG); // get current GYRO_CONFIG register value
    uint8_t c = buffer[0];
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x03; // Clear Fchoice bits [1:0] 
    c = c & ~0x18; // Clear GFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    write_byte(MPU_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register
  
    // Set accelerometer full-scale range configuration.
    read_byte(MPU_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    c = buffer[0];
    // c = c & ~0xE0; // Clear self-test bits [7:5] 
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer 
    write_byte(MPU_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration.
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = read_byte(MPU_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    write_byte(MPU_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
    // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting.

    // Configure Interrupts and Bypass Enable.
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on any read, and enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    write_byte(MPU_INT_PIN_CFG, 0b00110010);    
    write_byte(MPU_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    nrf_delay_ms(100);



    // Check connection to MAG.
    read_mag_byte(MAG_WHO_ADDR);
    if (buffer[0]!=MAG_WHO)
    {
        NRF_LOG_DEBUG("MAG not recognized. WHO_AM_I is 0x%x", buffer[0]);
        APP_ERROR_CHECK(1); // TODO proper error code
    }
    NRF_LOG_DEBUG("MAG recognized.");

    // First extract the factory calibration for each magnetometer axis.
    uint8_t rawData[3];  // x/y/z gyro calibration data stored here
    write_mag_byte(MAG_CNTL, 0x00); // Power down magnetometer  
    nrf_delay_ms(10);
    write_mag_byte(MAG_CNTL, 0x0F); // Enter Fuse ROM access mode
    nrf_delay_ms(10);
    read_mag_byte(MAG_ASAX);  // Read the x-, y-, and z-axis calibration values
    magcalibration[0] =  (float)(buffer[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
    read_mag_byte(MAG_ASAX+1);
    magcalibration[1] =  (float)(buffer[0] - 128)/256. + 1.;
    read_mag_byte(MAG_ASAX+2);
    magcalibration[2] =  (float)(buffer[0] - 128)/256. + 1.;
    write_mag_byte(MAG_CNTL, 0x00); // Power down magnetometer  
    nrf_delay_ms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    write_mag_byte(MAG_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    nrf_delay_ms(10);



    // Uninitialize TWI
    nrfx_twim_disable(&twi_master);
    nrfx_twim_uninit(&twi_master);

    // Reinitialize it in managed non-blocking mode.
    // Initialize the manager.
    nrf_drv_twi_config_t const config_async = { // TODO the config should move to the nrfx namespace instead of nrf_drv
        .scl                = scl,
        .sda                = sda,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,
        .clear_bus_init     = false
    };
    err_code = nrf_twi_mngr_init(&m_twi_mngr, &config_async);
    APP_ERROR_CHECK(err_code);

    // Initialize sensor.
    err_code = nrf_twi_sensor_init(&m_mpu);
    APP_ERROR_CHECK(err_code);



    // Set up interrupt handler.
    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    nrfx_gpiote_in_config_t irq_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    err_code = nrfx_gpiote_in_init(irq, &irq_config, irq_callback);
    APP_ERROR_CHECK(err_code);
    nrfx_gpiote_in_event_enable(irq, true);
    mpu_read_and_fuse_sensors();
    NRF_LOG_DEBUG("Done");
}

// TODO make the following convenience functions

// TODO general read/write functions

// TODO set accel, gyro range

// TODO set mag precision

// TODO set filter

// TODO set sample rate

// TODO check self-test results for accel and gyro

// TODO check self-test results for magnetometer

// TODO set accel and gyro bias (self-calibration when charging as the orientation will be known)

// TODO set magnetometer bias

// TODO set FIFO (default off, but using it at appropriate frequency would make the predictions more stable as otherwise the code has strong timing requirements)
// register 26.6, 35

// TODO wake on motion
// register 31