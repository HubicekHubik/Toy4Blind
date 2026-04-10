#include "ble_modul.h"
#include "toy_utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BLE_modul, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/nus/inst.h>
#include <bluetooth/services/nus_client.h>

#define BUT2BLE_Q_LENGHT 32
K_MSGQ_DEFINE(but2ble_q, sizeof(struct but_ev), BUT2BLE_Q_LENGHT, 4);

#define BT_UUID_NUS_VAL \
	BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca9e)

static struct bt_uuid_128 nus_uuid = BT_UUID_INIT_128(BT_UUID_NUS_VAL);
#define BT_UUID_NUS &nus_uuid.uuid

#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(CONFIG_BT_DEVICE_NAME) - 1)

#define BT_THREAD_PRIORITY 8
#define BT_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(BT_stack_area, BT_THREAD_STACK_SIZE);
static struct k_thread BT_thread_data;

static void BT_thread(void *arg1, void *arg2, void *arg3);

static struct bt_conn *connected_toys[CONFIG_BT_MAX_CONN];
uint8_t connected_count = 0;
uint8_t act_toy_idx = 0;

// 1. Scanning Parameters
static bool data_cb(struct bt_data *data, void *user_data);
static struct bt_le_scan_param scan_param = {
    .type       = BT_LE_SCAN_TYPE_ACTIVE,
    .options    = BT_LE_SCAN_OPT_NONE,
    .interval   = BT_GAP_SCAN_FAST_INTERVAL,
    .window     = BT_GAP_SCAN_FAST_WINDOW,
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad);
// 2. Connection Callbacks
static struct bt_nus_client nus_client[CONFIG_BT_MAX_CONN] = {NULL};

static uint8_t nus_client_received_cb(struct bt_nus_client *inst,
					const uint8_t *data, uint16_t len);

static struct bt_nus_client_cb nus_client_cb = {
	.received = nus_client_received_cb,
	.sent = NULL,             // Můžeš přidat funkci pro potvrzení odeslání
	.unsubscribed = NULL,
};

static void discovery_cb(struct bt_gatt_dm *dm, void *context);
static struct bt_gatt_dm_cb discovery_cb_data = {
    .completed         = discovery_cb,        // Tvoje funkce
    .service_not_found = NULL,                // Můžeš přidat hlášku "Nenalezeno"
    .error_found       = NULL,
};

void connected(struct bt_conn *conn, uint8_t err);

void disconnected(struct bt_conn *conn, uint8_t reason);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

uint8_t get_free_con_idx();

int ble_modul_init(void){
    k_thread_create(&BT_thread_data, BT_stack_area,
                        K_THREAD_STACK_SIZEOF(BT_stack_area),
                        BT_thread,
                        NULL, NULL, NULL,
                        BT_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

static void BT_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("Bluetooth theread has started");
	bt_enable(NULL);
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		struct bt_nus_client_init_param init_param = { .cb = &nus_client_cb };
		int err = bt_nus_client_init(&nus_client[i], &init_param);
		if (err) {
			printk("Global NUS init failed for idx %d (err %d)\n", i, err);
		}
	}
	struct but_ev but_info;
	while(1) {
		if (k_msgq_get(&but2ble_q, &but_info, K_FOREVER) == 0) {
			switch (but_info.cmd) {
			case MSG_TYPE_TOY_SWITCH:
				if (connected_count < CONFIG_BT_MAX_CONN) {
						bt_le_scan_start(&scan_param, device_found);
				} else {
					LOG_INF("All slots for toys occupied");
				}
				continue;
			case MT_VOL_UP:
				LOG_INF("Recieved VOL_UP");
				break;
			case MT_VOL_DOWN:
				LOG_INF("Recieved VOL_DOWN");
				break;
			case MSG_TYPE_SOUNDSET:
				LOG_INF("Recieved SOUNDSET");
				break;
			case MSG_TYPE_TURNOFF:
				LOG_INF("Recieved TURNOFF");
				break;
			case MT_BT_DISCONNECT:
				LOG_INF("Recieved BT_SWITCH");
			default:
				break;
			}
			if(nus_client[act_toy_idx].conn != NULL) {
				int err = bt_nus_client_send(&nus_client[act_toy_idx], (const uint8_t *)&but_info, sizeof(but_info));
				if (err) {
					LOG_INF("Chyba odesilani: %d (mozna discovery jeste neskoncilo?)", err);
				} else {
					LOG_INF("Prikaz odeslan do hracky!");
				}
			} else {
				LOG_INF("No active NUS client please connect a device");
			}
		}
	}
}

static bool data_cb(struct bt_data *data, void *user_data)
{
    char *name = user_data;
    switch (data->type) {
    case BT_DATA_NAME_SHORTENED:
    case BT_DATA_NAME_COMPLETE:
        // Copy the name found in the data to our buffer
        size_t len = MIN(data->data_len, 30 - 1); // 30 is our name buffer size
        memcpy(name, data->data, len);
        name[len] = '\0';
        return false; // Stop parsing once name is found
    default:
        return true; // Continue parsing
    }
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
char addr_str[BT_ADDR_LE_STR_LEN];
    char name[30] = "Unknown"; // Default name
	char name_toy[] = "Toy_BLE";
    // 1. Convert the binary address to a readable string (e.g. AA:BB:CC:DD:EE:FF)
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    // 2. Parse the advertising data to find the name
    bt_data_parse(ad, data_cb, name);

    // 3. Print everything to the console
    printk("Scan Result: [Addr: %s] [RSSI: %d] [Name: %s]\n", 
           addr_str, rssi, name);
	int err;
	if(strcmp(name,name_toy) == 0) {
		printk("Conection begin");
		bt_le_scan_stop();
		uint8_t free_con_idx = get_free_con_idx();
		LOG_INF("Free idx %d", free_con_idx);
		err = bt_conn_le_create(addr, // The pointer to the address you found
					BT_CONN_LE_CREATE_CONN, // Default connection type
					BT_LE_CONN_PARAM_DEFAULT, // Default timing/power params
					&connected_toys[free_con_idx]); // Pointer to store the connection handle	
		if (err) {
			LOG_INF("something went wrong during conection %d\n", err);
		}
		
	}	
}

static uint8_t nus_client_received_cb(struct bt_nus_client *inst,
					const uint8_t *data, uint16_t len)
{
	printk("Remote prijal data z hracky: %d bytu\n", len);
	return BT_GATT_ITER_CONTINUE;
}

static void discovery_cb(struct bt_gatt_dm *dm, void *context)
{
	int err;

	printk("Service discovery hotovo, hledam NUS charakteristiky...\n");
	uint8_t conn_idx = bt_conn_index(bt_gatt_dm_conn_get(dm));
	LOG_INF("Assigning handles for idx %d", conn_idx);
	// Přiřadíme nalezené kanály našemu nus_clientovi
	err = bt_nus_handles_assign(dm, &nus_client[conn_idx]);
	if (err) {
		printk("Nepodarilo se najit NUS charakteristiky (err %d)\n", err);
	} else {
		printk("NUS nalezen! Ted muzeme odesilat data.\n");
		// Volitelně: automaticky se přihlásit k odběru dat z hračky
		//err = bt_nus_subscribe_receive(&nus_client[conn_idx]);
		if (err) {
			printk("Subscribe failed (err %d)\n", err);
		}
	}

	// Uvolníme paměť po vyhledávání
	bt_gatt_dm_data_release(dm);
}

void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

	uint8_t idx = bt_conn_index(conn);
    if (idx >= CONFIG_BT_MAX_CONN) {
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return;
    }

	connected_toys[idx] = bt_conn_ref(conn);
	connected_count++;
	LOG_INF("Connected toy n. %d", idx);
	bt_conn_unref(conn);
    if (err) return;

    err = bt_gatt_dm_start(conn, BT_UUID_NUS, &discovery_cb_data, NULL);
    if (err) {
        LOG_ERR("Discovery start failed (err %d)", err);
    }
}

void disconnected(struct bt_conn *conn, uint8_t reason) {
	uint8_t conn_idx = bt_conn_index(conn);
	LOG_INF("Disconnected toy n.%d (reason %u)", conn_idx,reason);
	
	bt_conn_unref(connected_toys[conn_idx]);
	connected_toys[conn_idx] = NULL;
	nus_client[conn_idx].conn = NULL;

	if (connected_count > 0) {
        connected_count--;
    }
}

uint8_t get_free_con_idx(){
	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_toys[i] == NULL) return i;
	}
	return -1;
}