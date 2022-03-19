/**
 ************************************************************************
 * @file zephyr_i2c.c
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Contains helpher functions for the MPU Library
 **********************************************************************
 * */
#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <math.h>

//MPU9250 is connected on i2c0 bus
#define I2C0 DT_LABEL(DT_NODELABEL(i2c0))

/**
 * @brief Min function to determine the minimum value 
 * 
 * @param a value to compare
 * @param b value to compare
 * @return int min value
 */
int min(unsigned short a, unsigned short b)
{
    return (a > b) ? b : a;
}

/**
 * @brief Wrapper function to wrap i2c reads
 * 
 * @param slave_addr Slave dev addr
 * @param reg_addr Reg addr
 * @param length Length of data
 * @param data Data pool to save read data
 * @return ERRVAL
 */
int zephyr_i2c_read_wrapper(uint16_t slave_addr, uint8_t reg_addr,
                            uint32_t length, uint8_t *data)
{
    const struct device *dev = device_get_binding(I2C0);

    return i2c_burst_read(dev, slave_addr, reg_addr, data, length);
}

/**
 * @brief Wrapper function to wrap i2c write
 * 
 * @param slave_addr Slave dev addr
 * @param reg_addr Reg addr
 * @param length Length of data
 * @param data Data pool to write from
 * @return ERRVAL
 */
int zephyr_i2c_write_wrapper(uint16_t slave_addr, uint8_t reg_addr,
                             uint32_t length, uint8_t *data)
{
    const struct device *dev = device_get_binding(I2C0);

    return i2c_burst_write(dev, slave_addr, reg_addr, data, length);
}

/**
 * @brief Delays Ms
 * 
 * @param num_ms time ms to delay
 * @return int 0
 */
int zephyr_delay_wrapper(unsigned long num_ms)
{
    k_msleep((int32_t)num_ms);
    return 0;
}

/**
 * @brief Return systick in ms
 * 
 * @param count save ms to count var
 * @return int 0
 */
int zephyr_get_ms(unsigned long *count)
{
    *count = (int)k_cyc_to_ms_floor64(k_cycle_get_32());
    return 0;
}
