/**
 ************************************************************************
 * @file main.c
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Initialises mobile threads
 **********************************************************************
 * */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <data/json.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <init.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#include "mobile_node_ble.h"
#include "mobile_connect.h"
#include "hal_imu.h"
//
K_THREAD_DEFINE(ble_scan, STACK_SIZE_BLE_SCAN, thread_ble_scan, NULL, NULL, NULL, THREAD_PRIORITY_BLE_SCAN, 0, 0);
K_THREAD_DEFINE(ble_send, STACK_SIZE_BLE_SEND, thread_ble_send, NULL, NULL, NULL, THREAD_PRIORITY_BLE_SEND, 0, 10);

K_THREAD_DEFINE(ble_led, THREAD_BLE_LED_THREAD_STACK, thread_ble_led, NULL, NULL, NULL, THREAD_PRIORITY_BLE_LED_THREAD, 0, 50);
//BT Enable in ble_connect_thread
K_THREAD_DEFINE(ble_connect, THREAD_BLE_CONNECT_STACK, thread_ble_connect, NULL, NULL, NULL, THREAD_PRIORITY_BLE_CONNECT_THREAD, 0, 0);

//MPU9250 Thread
K_THREAD_DEFINE(imu, THREAD_IMU_RW_STACK, thread_imu_rw, NULL, NULL, NULL, THREAD_PRIORITY_IMU, 0, 100);
