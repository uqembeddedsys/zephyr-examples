/**
 ************************************************************************
 * @file mobile_node_ble.h
 * @author Aaron Helmore, Wilfred MK
 * @date 20.04.2021
 * @brief Contains required definitions by mobile_node_ble.c
 **********************************************************************
 **/

#ifndef MOBILE_NODE_BLE_H
#define MOBILE_NODE_BLE_H

#include <zephyr.h>

// Stack size for thread_ble_scan
#define STACK_SIZE_BLE_SCAN 1024

// Thread priority for thread_ble_scan
#define THREAD_PRIORITY_BLE_SCAN 3

// Stack size for thread_ble_send
#define STACK_SIZE_BLE_SEND 1024

// Thread priority for thread_ble_send
#define THREAD_PRIORITY_BLE_SEND 7

// Maximum possible BLE GAP name length
#define BLE_MAX_NAME_LEN 50

// Number of static nodes
#define STATIC_NODE_NUM 4

//Using 280ms period ensures packets aren't lost.
#define MOBILE_BASE_SCAN_PERIOD 280

// Holds all data relevant to a scanned node advertisement
struct NodeData
{
    char name[BLE_MAX_NAME_LEN];
    int8_t rssi;
    int16_t ultrasonic;
};

// Thread to scan for advertisements
void thread_ble_scan(void);

// Thread to display node data
void thread_ble_send(void);

#endif