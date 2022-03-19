/**
 ************************************************************************
 * @file ble_base.h
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Contain required globals and defines for ble_base.c
 **********************************************************************
 * */
#ifndef BLE_BASE_H
#define BLE_BASE_H

/* Debug Thread Stack size */
#define THREAD_BLE_LED_STACK 512
#define THREAD_BLE_BASE_STACK 4094
/* Debug Thread Priority */
#define THREAD_PRIORITY_BLE_LED 10
#define THREAD_PRIORITY_BLE_BASE -2
#define THREAD_PRIORITY_READ_BASE -10

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led1)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

/* The devicetree node identifier for the "led0" alias. */
#define LED1_NODE DT_ALIAS(led3)
#define LED1 DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1 DT_GPIO_FLAGS(LED1_NODE, gpios)

void thread_ble_led(void);

void thread_ble_base(void);

void thread_ble_read_out(void);

#endif