/**
 ************************************************************************
 * @file zephyr_i2c.h
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Contains helpher functions for the MPU Library
 **********************************************************************
 * */
#ifndef ZEPHYR_I2C_H
#define ZEPHYR_I2C_h

int zephyr_i2c_read_wrapper(unsigned char slave_addr, unsigned char reg_addr,
                            unsigned char length, unsigned char const *data);

int zephyr_i2c_write_wrapper(unsigned char slave_addr, unsigned char reg_addr,
                             unsigned char length, unsigned char *data);

int zephyr_delay_wrapper(unsigned long num_ms);

int zephyr_get_ms(unsigned long *count);

int min(unsigned short a, unsigned short b);

#endif