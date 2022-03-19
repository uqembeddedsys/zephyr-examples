/**
 ************************************************************************
 * @file main.c
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Spawns threads required by Base node.
 **********************************************************************
 **/

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <usb/usb_device.h>

#include "ble_base.h"

//START BLE BASE entry thread : Delayed Start (Wait for USB to be ready)
K_THREAD_DEFINE(ble_base, THREAD_BLE_BASE_STACK, thread_ble_base, NULL, NULL, NULL, THREAD_PRIORITY_BLE_BASE, 0, 0);
//Reads BLE Buffers and prints them to USB Console
K_THREAD_DEFINE(ble_read_out, THREAD_BLE_BASE_STACK, thread_ble_read_out, NULL, NULL, NULL, THREAD_PRIORITY_READ_BASE, 0, 0);
//Start BLE LED Thread
K_THREAD_DEFINE(ble_led, THREAD_BLE_LED_STACK, thread_ble_led, NULL, NULL, NULL, THREAD_PRIORITY_BLE_LED, 0, 0);

/**
 * @brief Enable USB Driver.
 * 
 */
void main(void)
{
	if (usb_enable(NULL))
		return;
}
