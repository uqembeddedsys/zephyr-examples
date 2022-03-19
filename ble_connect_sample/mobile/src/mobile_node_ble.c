/**
 ************************************************************************
 * @file mobile_node_ble.c
 * @author Aaron Helmore, Wilfred MK
 * @date 20.04.2021
 * @brief Read adv data broadcast (BLE) by static nodes and parses them 
 *          appropriately, works together with mobile_connect to update 
 *          internal buffer so that base can read the most up to data
 *          static node information.
 **********************************************************************
 **/

#include <zephyr.h>
#include <stddef.h>

#include <stdbool.h>
#include <string.h>

#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include "mobile_node_ble.h"
#include "mobile_connect.h"

// Holds received advertisement data from each node
static struct NodeData nodeData[STATIC_NODE_NUM];

// Semaphores to signal that advertisement was received from node at that index
static struct k_sem scannedSems[STATIC_NODE_NUM];

// Semaphore to signal that a scan has been completed
static struct k_sem scanDoneSem;

// Static node GAP names according to index
const char *nodeNames[] = {
    "Leonardo_U",
    "Raphael_U",
    "Michelangelo",
    "Donatello"};

/**
 * @brief Initialise all semaphores used in BLE advertisement scanning
 */
static void scanned_sem_init(void)
{
    k_sem_init(&scanDoneSem, 0, 1);

    for (int i = 0; i < STATIC_NODE_NUM; ++i)
    {

        k_sem_init(&scannedSems[i], 0, 1);
    }
}

/**
 * @brief Callback function to extract GAP name and ultrasonic data from BLE
 *          advertisements
 * 
 * @param data : Data to parse
 * @param user_data : Container to save parsed data to
 * @return true : If parsing should continue
 * @return false : If parsing should stop
 */
static bool parse_advertising_data(struct bt_data *data, void *user_data)
{
    struct NodeData *parsedData = user_data;

    // Check if the data is the advertiser's name
    if (data->type == BT_DATA_NAME_COMPLETE)
    {

        // If the name length is valid, save it and stop parsing
        if (data->data_len < BLE_MAX_NAME_LEN)
        {

            memcpy(parsedData->name, data->data, data->data_len);
            parsedData->name[data->data_len] = '\0';

            return true;
        }
    }
    else if (data->type == BT_DATA_MANUFACTURER_DATA)
    {

        // If data is correct length, try save it as ultrasonic data
        if (data->data_len == 2 * sizeof(uint16_t))
        {

            parsedData->ultrasonic = ((int16_t)(data->data[2]) << 8) | data->data[3];

            return false;
        }
    }

    return true;
}

/**
 * @brief Callback for when an advertisement has been found
 * 
 * @param addr : BLE address of the advertiser
 * @param rssi : Received signal strength indicator of the advertisement
 * @param advType : Type of advertisement
 * @param ad : The scanned advertisement
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t advType, struct net_buf_simple *ad)
{
    struct NodeData adData = {0};

    bt_data_parse(ad, parse_advertising_data, &adData);

    for (int i = 0; i < STATIC_NODE_NUM; ++i)
    {

        // See if a valid node; if so, save data
        if (strcmp(adData.name, nodeNames[i]) == 0)
        {

            nodeData[i].rssi = rssi;

            //
            // Check if the node is an ultrasonic node, if so save data
            nodeData[i].ultrasonic = (adData.name[strlen(adData.name) - 1] == 'U') ? adData.ultrasonic : -1;

            k_sem_give(&scannedSems[i]);
        }
    }
}

/**
 * @brief Thread to scan for BLE advertisements
 */
void thread_ble_scan(void)
{
    // Setup passive scan with 40ms interval 40ms window (constant scan)
    struct bt_le_scan_param scanParams = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0040,
        .window = 0x0040};
    int err;

    // Initialise all semaphores used in scanning routine
    scanned_sem_init();

    err = bt_le_scan_start(&scanParams, scan_cb);
    if (err)
    {

        printk("Starting scanning failed (err %d)\n", err);
        return;
    }

    while (1)
    {

        // Wait for the period time then signal scan complete
        k_msleep(MOBILE_BASE_SCAN_PERIOD);

        k_sem_give(&scanDoneSem);
    }
}

/**
 * @brief Thread to print static node ads to console
 * NOTE: THIS WILL BE DEPRECIATED
 */
void thread_ble_send(void)
{
    int err;
    int total = 0, missed = 0;

    while (1)
    {

        k_sem_take(&scanDoneSem, K_FOREVER);

        printk("Scan done\n");

        ++total;

        for (int i = 0; i < STATIC_NODE_NUM; ++i)
        {

            // Wait max 10ms before deciding that nodes ad was missed
            err = k_sem_take(&scannedSems[i], K_MSEC(10));
            if (err == -EAGAIN)
            {

                if (i == 0)
                {
                    ++missed;
                }
                // sem_take timed out, data was therefore missed
                printk("Node %d ad was missed\n", i + 1);
            }
            else
            {

                printk("rssi: %d, ultra: %d\n", nodeData[i].rssi, nodeData[i].ultrasonic);
                //Update GATT Characteristic Buffers
                node_rssi[i] = nodeData[i].rssi;
                node_ultra[i] = nodeData[i].ultrasonic;
            }
        }
    }
}