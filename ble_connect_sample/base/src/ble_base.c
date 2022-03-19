/**
 ************************************************************************
 * @file ble_base.c
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Base device driver that will connect to the mobile node and
 *          send required data to serial via USB console.
 **********************************************************************
 * */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#include "ble_base.h"

//TODO BUG: Opening cosole instantly locks on bluetooth connect, otherwise takes a bit longer (~12s backend priority issue with USB and Console). Caused by (CONFIG_USB_UART_CONSOLE)
static void start_scan(void);

static struct bt_conn *default_conn;
bool ble_connected;

/* Custom UUIDs For Mobile and it's GATT Attributes */
#define UUID_BUFFER_SIZE 16
//Used to as a key to test against scanned UUIDs
uint16_t mobile_uuid[] = {0xd0, 0x92, 0x67, 0x35, 0x78, 0x16, 0x21, 0x91,
                          0x26, 0x49, 0x60, 0xeb, 0x06, 0xa7, 0xca, 0xcb};

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

//RSSI RX BUFFER
int16_t rx_rssi[] = {0x00, 0x00, 0x00, 0x00};
//ULTRA RX BUFFER
int16_t rx_ultra[] = {0x00, 0x00, 0x00, 0x00};
//IMT Data Array
int16_t rx_imu_accel_raw[] = {0x00, 0x00, 0x00};
int16_t rx_imu_gyro_raw[] = {0x00, 0x00, 0x00};
int16_t rx_imu_mag_raw[] = {0x00, 0x00, 0x00};

//Accel Range/32767.5
float accel_scale = 0.0002441;
//Gyro DPS/32767.5
float gyro_scale = 0.01397917;
//These values were read from the calibration registers on the Mag.
float mag_scale[3] = {0.176835, 0.177421, 0.170980};
/**
 * @brief Callback for BLE scanning, checks weather the returned 
 *          UUID matches the custom UUID of the mobile device.
 *        If matched, attempt to connect to device.
 * 
 * @param data Callback data from scanning
 * @param user_data Device User data
 * @return true 
 * @return false 
 */
static bool parse_device(struct bt_data *data, void *user_data)
{
    bt_addr_le_t *addr = user_data;
    int i;
    int matchedCount = 0;

    printk("[AD]: %u data_len %u\n", data->type, data->data_len);

    if (data->type == BT_DATA_UUID128_ALL)
    {

        uint16_t temp = 0;
        for (i = 0; i < data->data_len; i++)
        {
            temp = data->data[i];
            if (temp == mobile_uuid[i])
            {
                matchedCount++;
            }
        }

        if (matchedCount == UUID_BUFFER_SIZE)
        {
            //MOBILE UUID MATCHED
            printk("Mobile UUID Found, attempting to connect\n");

            int err = bt_le_scan_stop();
            k_msleep(10);

            if (err)
            {
                printk("Stop LE scan failed (err %d)\n", err);
                return true;
            }

            struct bt_le_conn_param *param = BT_LE_CONN_PARAM_DEFAULT;

            err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                                    param, &default_conn);
            if (err)
            {
                printk("Create conn failed (err %d)\n", err);
                start_scan();
            }

            return false;
        }
    }
    return true;
}

/**
 * @brief Callback function for when scan detects device, scanned devices
 *          are filtered by their connectibilty and scan data is parsed.
 * 
 * @param addr Device Address
 * @param rssi RSSI 
 * @param type Device Type
 * @param ad Adv Data
 */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{

    if (default_conn)
    {
        return;
    }

    /* We're only interested in connectable events */
    if (type == BT_GAP_ADV_TYPE_ADV_IND ||
        type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND)
    {
        bt_data_parse(ad, parse_device, (void *)addr);
    }
}

/**
 * @brief Starts passive BLE scanning for nearby
 *          devices.
 */
static void start_scan(void)
{
    int err;

    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err)
    {
        printk("Scanning failed to start (err %d)\n", err);
        return;
    }

    printk("Scanning successfully started\n");
}

/**
 * @brief Callback for when reading RSSI Gatt atrribute data
 *          from the mobile device. The data read is saved into    
 *          and internal rx buffer. 
 * 
 * @param conn ble connection handler
 * @param err  ble ATT error val
 * @param params Read params
 * @param data Data read from GATT attribute
 * @param length Len of Data
 * @return uint8_t retVal (custom)
 */
uint8_t read_rssi_from_mobile(struct bt_conn *conn, uint8_t err,
                              struct bt_gatt_read_params *params,
                              const void *data, uint16_t length)
{
    memcpy(&rx_rssi, data, sizeof(rx_rssi));
    //printk("RSSI: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
    return 0;
}

/**
 * @brief Callback for when reading RSSI Gatt atrribute data
 *          from the mobile device. The data read is saved into    
 *          and internal rx buffer. 
 * 
 * @param conn ble connection handler
 * @param err  ble ATT error val
 * @param params Read params
 * @param data Data read from GATT attribute
 * @param length Len of Data
 * @return uint8_t retVal (custom)
 */
uint8_t read_ultra_from_mobile(struct bt_conn *conn, uint8_t err,
                               struct bt_gatt_read_params *params,
                               const void *data, uint16_t length)
{

    memcpy(&rx_ultra, data, sizeof(rx_ultra));
    //printk("ULTRA: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_ultra[0], rx_ultra[1], rx_ultra[2], rx_ultra[3]);
    return 0;
}

/**
 * @brief Callback for when reading IMU sensor data from mobile device, data read is saved
 *          into internal sensor rx buffer.
 * 
 * @param conn ble connection handler
 * @param err  ble ATT error val
 * @param params Read params
 * @param data Data read from GATT attribute
 * @param length Len of Data
 * @return uint8_t retVal (custom)
 */
uint8_t read_sensor_array_from_mobile(struct bt_conn *conn, uint8_t err,
                                      struct bt_gatt_read_params *params,
                                      const void *data, uint16_t length)
{

    if (bt_uuid_cmp(params->by_uuid.uuid, &imu_accel_uuid.uuid) == 0)
    {
        memcpy(rx_imu_accel_raw, data, sizeof(rx_imu_accel_raw));
        //printk("AX %f, AY %f, AZ %f\n", rx_imu_accel_raw[0] * accel_scale, rx_imu_accel_raw[1] * accel_scale, rx_imu_accel_raw[2] * accel_scale);
    }

    if (bt_uuid_cmp(params->by_uuid.uuid, &imu_gyro_uuid.uuid) == 0)
    {
        memcpy(rx_imu_gyro_raw, data, sizeof(rx_imu_accel_raw));
        //printk("gX %f, gY %f, gZ %f\n", rx_imu_gyro_raw[0] * gyro_scale, rx_imu_gyro_raw[1] * gyro_scale, rx_imu_gyro_raw[2] * gyro_scale);
    }

    if (bt_uuid_cmp(params->by_uuid.uuid, &imu_mag_uuid.uuid) == 0)
    {
        memcpy(rx_imu_mag_raw, data, sizeof(rx_imu_accel_raw));
        //printk("mX %f, mY %f, mZ %f\n", rx_imu_mag_raw[0] * mag_scale[0], rx_imu_mag_raw[1] * mag_scale[1], rx_imu_mag_raw[2] * mag_scale[2]);
    }

    return BT_GATT_ITER_STOP;
}

/**
 * @brief BLE Device connected callback function. If an error is detected
 *          scan is restarted. Else, the app can establish that the 
 *          devices are now conneted using flag ble_connected;
 * 
 * @param conn Connection handler
 * @param err BLE ERR
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err)
    {
        printk("Failed to connect to %s (%u)\n", addr, err);

        bt_conn_unref(default_conn);
        default_conn = NULL;

        start_scan();
        return;
    }

    if (conn != default_conn)
    {
        return;
    }
    ble_connected = true;
    printk("Connected: %s\n", addr);
}

/**
 * @brief BLE Disconnected callback, when disconnected, restarts BLE scanning
 *          and the app can detect that BLE has been disconnected by referring to
 *          ble_connected flag.
 * 
 * @param conn Connection handler.
 * @param reason Disconnect reason (ERR VAL).
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (conn != default_conn)
    {
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);

    bt_conn_unref(default_conn);
    default_conn = NULL;
    ble_connected = false;
    start_scan();
}

//#define BT_ATT_FIRST_ATTTRIBUTE_HANDLE 0x0001
//#define BT_ATT_LAST_ATTTRIBUTE_HANDLE 0xffff

/**
 * @brief Connection callback struct, required to set conn/disconn
 *          function pointers.
 */
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

void thread_ble_read_out(void)
{
    static struct bt_gatt_read_params read_params_rssi = {
        .func = read_rssi_from_mobile,
        .handle_count = 0,
        .by_uuid.uuid = &node_rssi_uuid.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    static struct bt_gatt_read_params read_param_ultra = {
        .func = read_ultra_from_mobile,
        .handle_count = 0,
        .by_uuid.uuid = &node_ultra_uuid.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    static struct bt_gatt_read_params read_param_accel = {
        .func = read_sensor_array_from_mobile,
        .handle_count = 0,
        .by_uuid.uuid = &imu_accel_uuid.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    static struct bt_gatt_read_params read_param_gyro = {
        .func = read_sensor_array_from_mobile,
        .handle_count = 0,
        .by_uuid.uuid = &imu_gyro_uuid.uuid,
        .by_uuid.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .by_uuid.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    /* Disabled following feature, due to spec scope reduction. Maybe re-enabled just to flex? */
    /*
    static struct bt_gatt_read_params read_param_mag = {
         .func = read_sensor_array_from_mobile,
         .handle_count = 0,
         .by_uuid.uuid = &imu_mag_uuid.uuid,
         .by_uuid.start_handle = BT_ATT_FIRST_ATTTRIBUTE_HANDLE,
         .by_uuid.end_handle = BT_ATT_LAST_ATTTRIBUTE_HANDLE,
     };
     */
    int timeStamp = 0;

    while (1)
    {

	/* NOTE: The commented out code implements additional features that were deprecated by the spec */
        if (ble_connected)
        {
            timeStamp = k_cyc_to_ms_floor64(k_cycle_get_32());
            //Read Node RSSI data from mobile
            bt_gatt_read(default_conn, &read_params_rssi);

            //Read Node Ultra Values from mobile
            bt_gatt_read(default_conn, &read_param_ultra);

            //Read IMU Values from mobile
            bt_gatt_read(default_conn, &read_param_accel);
            bt_gatt_read(default_conn, &read_param_gyro);
            // bt_gatt_read(default_conn, &read_param_mag);

            //printk("Delay Time: %d\n", (int)k_cyc_to_ms_floor64(k_cycle_get_32()) - timeStamp);

            // printk("RSSI: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
            // printk("ULTRA: N1:%d, N2:%d, N3:%d, N4:%d\n", rx_ultra[0], rx_ultra[1], rx_ultra[2], rx_ultra[3]);

            // printk("aX %f, aY %f, aZ %f\n", rx_imu_accel_raw[0] * accel_scale, rx_imu_accel_raw[1] * accel_scale, rx_imu_accel_raw[2] * accel_scale);
            // printk("gX %f, gY %f, gZ %f\n", rx_imu_gyro_raw[0] * gyro_scale, rx_imu_gyro_raw[1] * gyro_scale, rx_imu_gyro_raw[2] * gyro_scale);
            // printk("mX %f, mY %f, mZ %f\n", rx_imu_mag_raw[0] * mag_scale[0], rx_imu_mag_raw[1] * mag_scale[1], rx_imu_mag_raw[2] * mag_scale[2]);

            printk("%d,%d,%d,%d,%d,", (int)k_cyc_to_ms_floor64(k_cycle_get_32()), rx_rssi[0], rx_rssi[1], rx_rssi[2], rx_rssi[3]);
            printk("%d,%d,", rx_ultra[0], rx_ultra[1]);
            printk("%d,%d,%d,", rx_imu_accel_raw[0], rx_imu_accel_raw[1], rx_imu_accel_raw[2]);
            printk("%d,%d,%d\n", rx_imu_gyro_raw[0], rx_imu_gyro_raw[1], rx_imu_gyro_raw[2]);
        }

        k_usleep(100);
    }
}

/**
 * @brief BLE Base entry thread, starts initial ble scanning.
 *          When a valid mobile device is connected.
 */
void thread_ble_base(void)
{
    int err;

    err = bt_enable(NULL);
    default_conn = NULL;

    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_conn_cb_register(&conn_callbacks);

    start_scan();

    //Should not reach here
    printk("Debug_1\n");
}

/**
 * @brief Super important thread that will ensure le
 * 			led is blinking. Like i said, this is super important.
 * 
 */
void thread_ble_led(void)
{

    ble_connected = false;
    bool led_is_on = true;
    gpio_pin_configure(device_get_binding(LED0), PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    gpio_pin_configure(device_get_binding(LED1), PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);

    while (1)
    {
        led_is_on = !led_is_on;

        if (ble_connected)
        {
            gpio_pin_set(device_get_binding(LED1), PIN1, (int)led_is_on);
            gpio_pin_set(device_get_binding(LED0), PIN, (int)false);
            k_msleep(BLE_CONN_SLEEP_MS);
        }
        else
        {
            gpio_pin_set(device_get_binding(LED1), PIN1, (int)led_is_on);
            gpio_pin_set(device_get_binding(LED0), PIN, (int)led_is_on);
            k_msleep(BLE_DISC_SLEEP_MS);
        }
    }
    //Still very important...
}
