/**
 ************************************************************************
 * @file hal_imu.c
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Reads IMU driver sensors and updates BLE buffer for data transmission
 **********************************************************************
 * */

#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <stdio.h>
#include <math.h>

#include "hal_imu.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "imu_driver_defines.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "MPU9250_RegisterMap.h"
#include "mobile_connect.h"

LOG_MODULE_REGISTER(IMU_Driver, LOG_LEVEL_ERR);

#define MPU9250_ADDR 0x68
#define MPU9250_WHOAMI_ADDR 0x75

#define USER_CTRL_AD 0x6A
#define INT_BYPASS_CONFIG_AD 0x37

//MPU9250 is connected on i2c0 bus
#define I2C0 DT_LABEL(DT_NODELABEL(i2c0))

//Arduino Constraint
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

struct sensorSens
{
    float mSense;
    float aSense;
    float gSense;
};

struct sensorSens mySens = {
    .mSense = 6.665f, // Constant - 4915 / 32760
    .aSense = 0.0f,   // Updated after accel FSR is set
    .gSense = 0.0f    // Updated after gyro FSR is set
};

// Global Senser Vals
unsigned long stepCount = 0;
unsigned long stepTime = 0;
unsigned long lastStepCount = 0;

/**
 * @brief Configures I2C_0 as master
 * 
 * @return int 0 if success
 */
int hal_i2c_configX(void)
{
    /* Obtain pre-initialised device binding - see device_config.h */
    const struct device *dev = device_get_binding(I2C0);

    if (dev == NULL)
    {
        printk("Could not get pointer to I2C2 device struct.\n");
    }

    if (!dev)
    {
        /* Unable to retrive device structure */
        return -ENODEV;
    }
    else
    {
        /*Configure I2C using attained device struct, using the following parameters */
        return i2c_configure(dev, I2C_SPEED_STANDARD | I2C_MODE_MASTER);
    }
}

/**
 * @brief Read and validate WHOAMI register on the MPU9250
 * 
 * @param reg regaddr to read
 * @param buffer buffer to save data into
 * @param size size of data
 * @return int 
 */
int hal_mpu9250_read_regX(uint8_t reg, uint8_t *buffer, int size)
{
    const struct device *dev = device_get_binding(I2C0);

    if (i2c_burst_read(dev, MPU9250_ADDR, reg, buffer, size) == 0)
    {
        return 0;
    }
    return -EIO;
}

/**
 * @brief Set the MPU bypass mode.
 * 
 */
void set_MPU_bypass_mode(void)
{
    hal_i2c_configX();
    const struct device *dev = device_get_binding(I2C0);

    uint8_t buffer;

    if (hal_mpu9250_read_regX(MPU9250_WHOAMI_ADDR, &buffer, sizeof(buffer)) == 0)
    {
        //Read Complete
        if (buffer == 0x71)
        {
            printk("MPU OK\n");
        }
    }

    //Disabled I2C Master Mode
    i2c_reg_write_byte(dev, MPU9250_ADDR, USER_CTRL_AD, 0x00);
    //Turn Bypass MUX on
    i2c_reg_write_byte(dev, MPU9250_ADDR, INT_BYPASS_CONFIG_AD, 0x02);

    //Read AK89XX ACK In Bypass
    i2c_reg_read_byte(dev, 0x0C, 0x00, &buffer);

    if (buffer == 0x48)
    {
        printk("AK8963 OK\n");
    }

    if (hal_mpu9250_read_regX(MPU9250_WHOAMI_ADDR, &buffer, sizeof(buffer)) == 0)
    {
        //Read Complete
        if (buffer == 0x71)
        {
            printk("MPU OK\n");
        }
    }

    //Read AK89XX ACK In Bypass
    i2c_reg_read_byte(dev, 0x0C, 0x00, &buffer);

    if (buffer == 0x48)
    {
        printk("AK8963 OK\n");
    }
}

/**
 * @brief Main thread that handles communication with the IMU Driver.
 * 
 */
void thread_imu_rw(void)
{
    //Set MPU9250 slave bypass MUX
    set_MPU_bypass_mode();

    if (begin() != INV_SUCCESS)
    {
        printk("Error\n");
        return;
    }

    while (1)
    {
        if (dataReady())
        {

            update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
            printIMUData();
        }

        k_msleep(10);
    }
}

/**
 * @brief Initialise IMU
 * 
 * @return int ERRVAL
 */
int begin(void)
{

    struct int_param_s int_param;
    int result;
    //uint8_t buffer;

    hal_i2c_configX();

    // Place all slaves (including compass) on primary bus
    if (mpu_set_bypass(1) != 0)
    {
        printk("Slave Error\n");
    }

    k_msleep(10);

    result = mpu_init(&int_param);

    if (result)
    {
        printk("Init Fail\n");
        return result;
    }

    setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    mySens.gSense = getGyroSens();
    mySens.aSense = getAccelSens();
    printk("gSense: %f, aSense: %f\n", mySens.gSense, mySens.aSense);

    //Set Sensor Config
    // Use setGyroFSR() and setAccelFSR() to configure the
    // gyroscope and accelerometer full scale ranges.
    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    setGyroFSR(2000); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    setAccelFSR(2); // Set accel to +/-2g
    // Note: the MPU-9250's magnetometer FSR is set at
    // +/- 4912 uT (micro-tesla's)

    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    setLPF(5); // Set LPF corner frequency to 5Hz

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    setSampleRate(10); // Set sample rate to 10Hz

    // Likewise, the compass (magnetometer) sample rate can be
    // set using the setCompassSampleRate() function.
    // This value can range between: 1-100Hz
    setCompassSampleRate(10); // Set mag rate to 10H

    return result;
}

/**
 * @brief Sets IMU Data to buffer.
 * 
 */
void printIMUData(void)
{
    // float accelX = calcAccel(ax);
    // float accelY = calcAccel(ay);
    // float accelZ = calcAccel(az);
    // float gyroX = calcGyro(gx);
    // float gyroY = calcGyro(gy);
    // float gyroZ = calcGyro(gz);
    // float magX = calcMag(mx);
    // float magY = calcMag(my);
    // float magZ = calcMag(mz);

    imu_accel_raw[0] = ax;
    imu_accel_raw[1] = ay;
    imu_accel_raw[2] = az;

    imu_gyro_raw[0] = gx;
    imu_gyro_raw[1] = gy;
    imu_gyro_raw[2] = gz;

    //printk("aX: %d aY: %d aZ; %d", ax, ay, az);
    // printk("gX: %f gY: %f gZ; %f", gyroX, gyroY, gyroZ);
    //printk("mX: %f mY: %f mZ; %f", mx, my, mz);
}

/**
 * @brief Updates IMU sensor values
 * 
 * @param sensors sensors to select
 * @return int ERRVAL
 */
int update(unsigned char sensors)
{
    int aErr = INV_SUCCESS;
    int gErr = INV_SUCCESS;
    int mErr = INV_SUCCESS;
    int tErr = INV_SUCCESS;

    if (sensors & UPDATE_ACCEL)
        aErr = updateAccel();
    if (sensors & UPDATE_GYRO)
        gErr = updateGyro();
    if (sensors & UPDATE_COMPASS)
        mErr = updateCompass();
    if (sensors & UPDATE_TEMP)
        tErr = updateTemperature();

    return aErr | gErr | mErr | tErr;
}

/**
 * @brief Detect if new data is ready
 * 
 * @return true data ready
 * @return false data not ready
 */
bool dataReady(void)
{
    unsigned char intStatusReg;

    if (mpu_read_reg(MPU9250_INT_STATUS, &intStatusReg) == INV_SUCCESS)
    {
        return (intStatusReg & (1 << INT_STATUS_RAW_DATA_RDY_INT));
    }
    return false;
}

/**
 * @brief Set the Compass Sample Rate 
 * 
 * @param rate sample rate
 * @return int ERR
 */
int setCompassSampleRate(unsigned short rate)
{
    return mpu_set_compass_sample_rate(rate);
}

/**
 * @brief Set the MPU sample rate
 * 
 * @param rate sample rate
 * @return int ERR
 */
int setSampleRate(unsigned short rate)
{
    return mpu_set_sample_rate(rate);
}

/**
 * @brief Set MPU low pass filter
 * 
 * @param lpf lpf value
 * @return int ERR
 */
int setLPF(unsigned short lpf)
{
    return mpu_set_lpf(lpf);
}

/**
 * @brief Set the Accel FSR
 * 
 * @param fsr FSR val
 * @return int ERR
 */
int setAccelFSR(unsigned char fsr)
{
    int err;

    err = mpu_set_accel_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        mySens.aSense = getAccelSens();
    }
    return err;
}

/**
 * @brief Set the Gyro FSR
 * 
 * @param fsr fsr val
 * @return int ERR
 */
int setGyroFSR(unsigned short fsr)
{
    int err;

    err = mpu_set_gyro_fsr(fsr);
    if (err == INV_SUCCESS)
    {
        mySens.gSense = getGyroSens();
    }
    return err;
}

/**
 * @brief Set the Sensors on
 * 
 * @param sensors which sensors
 * @return int ERR
 */
int setSensors(unsigned char sensors)
{
    return mpu_set_sensors(sensors);
}

/**
 * @brief Get the Gyro Sens 
 * 
 * @return float sens
 */
float getGyroSens(void)
{
    float sens;
    if (mpu_get_gyro_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

/**
 * @brief Get the Accel Sens 
 * 
 * @return unsigned short sens
 */
unsigned short getAccelSens(void)
{
    unsigned short sens;

    if (mpu_get_accel_sens(&sens) == INV_SUCCESS)
    {
        return sens;
    }
    return 0;
}

/**
 * @brief Updata accel info
 * 
 * @return int ERR
 */
int updateAccel(void)
{
    short data[3];

    if (mpu_get_accel_reg(data, &time))
    {
        return INV_ERROR;
    }
    ax = data[X_AXIS];
    ay = data[Y_AXIS];
    az = data[Z_AXIS];
    return INV_SUCCESS;
}
/**
 * @brief Update Gyro info
 * 
 * @return int ERR
 */
int updateGyro(void)
{
    short data[3];

    if (mpu_get_gyro_reg(data, &time))
    {
        return INV_ERROR;
    }
    gx = data[X_AXIS];
    gy = data[Y_AXIS];
    gz = data[Z_AXIS];
    return INV_SUCCESS;
}

/**
 * @brief Update temp info
 * 
 * @return int ERR
 */
int updateTemperature(void)
{
    return mpu_get_temperature(&temperature, &time);
}

/**
 * @brief Update compass info
 * 
 * @return int ERR
 */
int updateCompass(void)
{
    short data[3];

    if (mpu_get_compass_reg(data, &time))
    {
        return INV_ERROR;
    }
    mx = data[X_AXIS];
    my = data[Y_AXIS];
    mz = data[Z_AXIS];
    return INV_SUCCESS;
}

//!SX150B POWER MPU9250 DEVICE ON AT BOOT------DON'T TOUCH----------------START
struct pwr_ctrl_cfg
{
    const char *port;
    uint32_t pin;
};

/**
 * @brief Generic power control function attained from
 *          thingy52/board.c
 * 
 * @param dev device handles
 * @return int return ERR val
 */
static int mpu_ctrl_init(const struct device *dev)
{
    const struct pwr_ctrl_cfg *cfg = dev->config;
    const struct device *gpio;

    gpio = device_get_binding(cfg->port);
    if (!gpio)
    {
        LOG_ERR("Could not bind device \"%s\"\n", cfg->port);
        return -ENODEV;
    }

    gpio_pin_configure(gpio, cfg->pin, GPIO_OUTPUT_HIGH);

    //Delay is required for the i2c rail to stabalize
    k_sleep(K_MSEC(100));

    return 0;
}

// Power up Thingy52 MPU9250 (analog switch) using SX1509B
#define MPU6050_VDD_PWR_CTRL_GPIO_PIN 8
static const struct pwr_ctrl_cfg mpu9250_vdd_pwr_ctrl_cfg = {
    .port = DT_LABEL(DT_INST(0, semtech_sx1509b)),
    .pin = MPU6050_VDD_PWR_CTRL_GPIO_PIN,
};

DEVICE_DEFINE(mpu9250_vdd_pwr_ctrl_initx, "", mpu_ctrl_init, NULL, NULL,
              &mpu9250_vdd_pwr_ctrl_cfg, POST_KERNEL,
              CONFIG_BOARD_CCS_VDD_PWR_CTRL_INIT_PRIORITY, NULL);

//!SX150B POWER MPU9250 DEVICE ON AT BOOT----------------------END