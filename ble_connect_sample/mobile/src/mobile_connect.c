/**
 ************************************************************************
 * @file mobile_connect.c
 * @author Wilfred MK, Aaron Helmore
 * @date 20.04.2021
 * @brief Mobile node will adv on BLE and await connection. Sets up 
 *          GATT attritubes for the base to read required data.
 **********************************************************************
 **/
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "mobile_node_ble.h"
#include "mobile_connect.h"

/* 1000 msec = 1 sec */
#define BLE_DISC_SLEEP_MS 250
#define BLE_CONN_SLEEP_MS 1000
#define SHORT_SLEEP_MS 50

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                  0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb),
};

//Keeps Track of BLE connection within APP
bool ble_conencted = false;

//!XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//!XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//TEST SERVIES N STUFF HERE
/* Custom Service Variables */
static struct bt_uuid_128 mobile_uuid = BT_UUID_INIT_128(
    0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 node_ultra_uuid = BT_UUID_INIT_128(
    0xd1, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 node_rssi_uuid = BT_UUID_INIT_128(
    0xd2, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imu_accel_uuid = BT_UUID_INIT_128(
    0xd3, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imu_gyro_uuid = BT_UUID_INIT_128(
    0xd4, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

static struct bt_uuid_128 imu_mag_uuid = BT_UUID_INIT_128(
    0xd5, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
    0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb);

//GATT CHARACTERISTIC VALUES
uint16_t node_rssi[] = {0x00, 0x00, 0x00, 0x00};
uint16_t node_ultra[] = {0x00, 0x00, 0x00, 0x00};

int16_t imu_accel_raw[] = {0x00, 0x00, 0x00};
int16_t imu_gyro_raw[] = {0x00, 0x00, 0x00};
int16_t imu_mag_raw[] = {0x00, 0x00, 0x00};

/**
 * @brief Callback funtion to read RSSI buffer.
 * 
 * @param conn connection handler
 * @param attr Attribute data/user data
 * @param buf Buffer storing the value
 * @param len Length of data
 * @param offset Data offset for multiple reads
 * @return ssize_t 0, to stop continous reads. 
 */
static ssize_t read_rssi(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset)
{
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(node_rssi));
}

/**
 * @brief Callback funtion to read Ultra buffer.
 * 
 * @param conn connection handler
 * @param attr Attribute data/user data
 * @param buf Buffer storing the value
 * @param len Length of data
 * @param offset Data offset for multiple reads
 * @return ssize_t 0, to stop continous reads. 
 */
static ssize_t read_ultra(struct bt_conn *conn,
                          const struct bt_gatt_attr *attr, void *buf,
                          uint16_t len, uint16_t offset)
{
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(node_ultra));
}

/**
 * @brief Callback funtion to read sensor array data
 * 
 * @param conn connection handler
 * @param attr Attribute data/user data
 * @param buf Buffer storing the value
 * @param len Length of data
 * @param offset Data offset for multiple reads
 * @return ssize_t 0, to stop continous reads. 
 */
static ssize_t read_sensor_array(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, void *buf,
                                 uint16_t len, uint16_t offset)
{
    const int16_t *value = attr->user_data;

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
                             sizeof(imu_accel_raw));
}

//Helper Macro to define BLE Gatt Attributes based on CX UUIDS
BT_GATT_SERVICE_DEFINE(mobile_svc,
                       BT_GATT_PRIMARY_SERVICE(&mobile_uuid),

                       BT_GATT_CHARACTERISTIC(&node_ultra_uuid.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_ultra, NULL, &node_ultra),

                       BT_GATT_CHARACTERISTIC(&node_rssi_uuid.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_rssi, NULL, &node_rssi),

                       BT_GATT_CHARACTERISTIC(&imu_accel_uuid.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_sensor_array, NULL, &imu_accel_raw),

                       BT_GATT_CHARACTERISTIC(&imu_gyro_uuid.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_sensor_array, NULL, &imu_gyro_raw),

                       BT_GATT_CHARACTERISTIC(&imu_mag_uuid.uuid,
                                              BT_GATT_CHRC_READ,
                                              BT_GATT_PERM_READ,
                                              read_sensor_array, NULL, &imu_mag_raw), );

/**
 * @brief Connection call back
 * 
 * @param conn conenction handler
 * @param err err val
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
    {
        printk("Connection failed (err 0x%02x)\n", err);
        ble_conencted = false;
    }
    else
    {
        printk("BLE Connected to Device\n");
        ble_conencted = true;
        struct bt_le_conn_param *param = BT_LE_CONN_PARAM(6, 6, 0, 400);

        if (bt_conn_le_param_update(conn, param) < 0)
        {
            while (1)
            {
                printk("Connection Update Error\n");
                k_msleep(10);
            }
        }
    }
}

/**
 * @brief Disconnect Callback, used to keep track of connection status 
 *          in the application layer
 * 
 * @param conn connection handler
 * @param reason disconnect reason.
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
    ble_conencted = false;
}

/**
 * @brief Passcode handler for accessing encrypted data
 * 
 * @param conn connection handler
 * @param passkey passkey
 */
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Passkey for %s: %06u\n", addr, passkey);
}

/**
 * @brief Authorisation cancelled handler
 * 
 * @param conn conenction handler
 */
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Pairing cancelled: %s\n", addr);
}

/**
 * @brief Conn callback data structs, holds
 *          function pointers.
 * 
 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};
/**
 * @brief Conn AUTH callback data structs, holds
 *          function pointers.
 * 
 */
static struct bt_conn_auth_cb auth_cb_display = {
    .passkey_display = auth_passkey_display,
    .passkey_entry = NULL,
    .cancel = auth_cancel,
};

/**
 * @brief Initialises bluetooth, and begins advertising data
 *            on BLE.
 * 
 */
static void bt_ready(void)
{
    int err;

    err = bt_enable(NULL);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    if (IS_ENABLED(CONFIG_SETTINGS))
    {
        //settings_load();
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);

    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    //bt_passkey_set(0xAA289);
    printk("Advertising successfully started\n");
}

/**
 * @brief Enabled bluetooth, and sets connection callback handler, awaits
 *          central to connect to peripheral (mobile)
 * 
 */
void thread_ble_connect(void)
{
    bt_ready();

    bt_conn_cb_register(&conn_callbacks);
    bt_conn_auth_cb_register(&auth_cb_display);

    while (1)
    {
        k_msleep(SHORT_SLEEP_MS);
    }
}

/**
 * @brief Enable GPIO led pins
 * 
 */
void led_gpio_enable(void)
{
    int ret = gpio_pin_configure(device_get_binding(LED0), PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (ret < 0)
    {
        return;
    }
}

/**
 * @brief Debug blink led thread, blinks LED based on conenction status.
 * 
 */
void thread_ble_led(void)
{
    bool led_is_on = true;
    led_gpio_enable();

    while (1)
    {
        led_is_on = !led_is_on;
        gpio_pin_set(device_get_binding(LED0), PIN, (int)led_is_on);

        if (ble_conencted)
        {
            k_msleep(BLE_CONN_SLEEP_MS);
        }
        else
        {
            k_msleep(BLE_DISC_SLEEP_MS);
        }
    }
}
