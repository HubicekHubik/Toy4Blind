#include "ble_modul.h"
#include "toy_utils.h"
#include "app_state.h"

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
    .options    = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval   = BT_GAP_SCAN_FAST_INTERVAL,
    .window     = BT_GAP_SCAN_FAST_WINDOW,
};

void stop_scan_handler();

int is_connected(const bt_addr_le_t *addr);

K_TIMER_DEFINE(scan_limit_timer, stop_scan_handler, NULL);
// 2. Connection Callbacks
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad);

static struct bt_nus_client nus_client[CONFIG_BT_MAX_CONN];

static uint8_t nus_client_received_cb(struct bt_nus_client *inst,
					const uint8_t *data, uint16_t len);

struct bt_nus_client_init_param init_param = { 
	.cb.received = nus_client_received_cb,
	.cb.sent = NULL,
	.cb.unsubscribed = NULL,
};

static void discovery_cb(struct bt_gatt_dm *dm, void *context);
static struct bt_gatt_dm_cb discovery_cb_data = {
    .completed         = discovery_cb,
    .service_not_found = NULL,
    .error_found       = NULL,
};

void connected(struct bt_conn *conn, uint8_t err);

void disconnected(struct bt_conn *conn, uint8_t reason);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

int get_free_con_idx();

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
		int err = bt_nus_client_init(&nus_client[i], &init_param);
		if (err) {
			LOG_INF("Global NUS init failed for idx %d (err %d)\n", i, err);
		}
	}
	struct but_ev but_info;
	while(1) {
		if (k_msgq_get(&but2ble_q, &but_info, K_FOREVER) == 0) {
			switch (but_info.cmd) {
			case MT_CONNECT_TOY:
				if (connected_count < CONFIG_BT_MAX_CONN) {
					int err = bt_le_scan_start(&scan_param, device_found);
					if (err) {
						LOG_ERR("Scan start failed (err %d)", err);
					} else {
						LOG_INF("Searching toys for 20s...");
						k_timer_start(&scan_limit_timer, K_SECONDS(20), K_NO_WAIT);
					}
				} else {
					LOG_INF("All slots for toys occupied");
				}
				break;
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
				LOG_INF("Recieved MT_BT_DISCONNECT");
				if (nus_client[act_toy_idx].conn != NULL) {
					int err = bt_conn_disconnect(nus_client[act_toy_idx].conn, 
												BT_HCI_ERR_REMOTE_USER_TERM_CONN);
					if (err) {
						LOG_ERR("Error during disconnect: %d", err);
					}
				}
				break;
			case MT_GEST:
				LOG_INF("Recieved MT_GEST");
				break;
			case MT_SWITCH_TOY:
				LOG_INF("Recieved MT_SWITCB_TOY");
				break;
			default:
				break;
			}
			if(nus_client[act_toy_idx].conn != NULL) {
				int err = bt_nus_client_send(&nus_client[act_toy_idx], (const uint8_t *)&but_info, sizeof(but_info));
				if (err) {
					LOG_INF("Error while transmiting: %d maby not end of discovery", err);
				} else {
					LOG_INF("Comand send to toy!");
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
    char name[30] = "Unknown";
	char name_toy[] = "Toy_BLE";
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    bt_data_parse(ad, data_cb, name);

	if(strcmp(name, name_toy) != 0) {
        return;
    }

    LOG_INF("Scan Result: [Addr: %s] [RSSI: %d] [Name: %s]\n", 
           addr_str, rssi, name);
	int err;

	LOG_INF("Conection begin");

	if(is_connected(addr)){
		return;
	}

	int free_con_idx = get_free_con_idx();
	if (free_con_idx == -1)	{
		bt_le_scan_stop();
        k_timer_stop(&scan_limit_timer);
        return;
	}

	bt_le_scan_stop();

	LOG_INF("Free idx %d", free_con_idx);
	err = bt_conn_le_create(addr, // The pointer to the address you found
				BT_CONN_LE_CREATE_CONN, // Default connection type
				BT_LE_CONN_PARAM_DEFAULT, // Default timing/power params
				&connected_toys[free_con_idx]); // Pointer to store the connection handle	
	if (err) {
		LOG_INF("something went wrong during conection %d\n", err);
	}
}

static uint8_t nus_client_received_cb(struct bt_nus_client *inst,
					const uint8_t *data, uint16_t len) {
	uint8_t conn_idx = inst - nus_client;
	uint8_t other_conn = (conn_idx + 1) % CONFIG_BT_MAX_CONN;
    LOG_INF("MSG from toy with index: %d sending data to toy with index %d", conn_idx, other_conn);
	
	static uint8_t mirror_first_gest = 0xFF;
	static uint8_t mirror_first_toy = 0xFF;
	static int64_t mirror_timestamp = 0;
	
	struct toy_events *rx_event = (struct toy_events *)data;

	int err;
	switch (rx_event->type) {
		case MT_GEST:
			uint8_t gest_id = rx_event->payload.game.gest_id;
			uint8_t game_mode = rx_event->payload.game.mode;

			struct toy_events tx_gest = { 
				.type = MT_GEST,
				.len = sizeof(struct gest_ev),
				.payload.gest.type = gest_id,
			}; 
			if(game_mode == GAME_MODE_SWAPPED) {
				err = bt_nus_client_send(&nus_client[other_conn], &tx_gest, len);
				if(err) {
					LOG_INF("Wasnt able to send data to the other toy %d",err);
				}
			}

			if (game_mode == GAME_MODE_MIRROR) {
					int64_t now = k_uptime_get();

					if (mirror_first_gest != 0xFF && (now - mirror_timestamp > 5000)) {
						LOG_INF("Mirror timeout expired, resetting...");
						mirror_first_gest = 0xFF;
					}

					if (mirror_first_gest == 0xFF) {
						mirror_first_gest = gest_id;
						mirror_first_toy = conn_idx;
						mirror_timestamp = now;
						LOG_INF("First mirror gesture (%d) from toy %d. Waiting 5s...", gest_id, conn_idx);
					} 
					else {
						if (mirror_first_toy != conn_idx && mirror_first_gest == gest_id) {
							LOG_INF("Mirror MATCH! Sending sound to both toys.");
							
							bt_nus_client_send(&nus_client[conn_idx], &tx_gest, sizeof(tx_gest));
							bt_nus_client_send(&nus_client[other_conn], &tx_gest, sizeof(tx_gest));

							mirror_first_gest = 0xFF;
						} else if (mirror_first_toy == conn_idx) {
							mirror_timestamp = now; 
						} else {
							LOG_INF("Mirror MISMATCH (gest %d vs %d), still waiting...", mirror_first_gest, gest_id);
						}
					}
				}
			
			break;
		
		default:
			LOG_WRN("Unknown comand throwing away 0x%2x.\n", rx_event->type);
			break;
	}
	return BT_GATT_ITER_CONTINUE;
}

static void discovery_cb(struct bt_gatt_dm *dm, void *context) {
	int err;

	LOG_INF("Service discovery complete, looking for NUS charakteristic...\n");
	uint8_t conn_idx = bt_conn_index(bt_gatt_dm_conn_get(dm));
	LOG_INF("Assigning handles for idx %d", conn_idx);

	err = bt_nus_handles_assign(dm, &nus_client[conn_idx]);
	if (err) {
		LOG_INF("Unable to find NUS charakteristic (err %d)\n", err);
	} else {
		LOG_INF("NUS FOUND!\n");
		err = bt_nus_subscribe_receive(&nus_client[conn_idx]);
		if (err) {
			LOG_INF("Subscribe failed (err %d)\n", err);
		} else {
			LOG_INF("Subscribe was succesfull");
		}
	}
	if (connected_count < CONFIG_BT_MAX_CONN) {
        LOG_INF("Scanning for next toy...");
        bt_le_scan_start(&scan_param, device_found);
    }
	bt_gatt_dm_data_release(dm);
}

void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
		if (connected_count < CONFIG_BT_MAX_CONN) {
            LOG_INF("Restarting scan after failed attempt...");
            bt_le_scan_start(&scan_param, device_found);
        }
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
	err = bt_gatt_dm_start(conn, BT_UUID_NUS, &discovery_cb_data, NULL);
    if (err) {
        LOG_ERR("Discovery start failed (err %d)", err);
    }

	bt_conn_unref(conn);
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

int get_free_con_idx(){
	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_toys[i] == NULL) return i;
	}
	return -1;
}

void stop_scan_handler() {
	bt_le_scan_stop();
    LOG_INF("Scand automaticli terminated after 20s.");
}

int is_connected(const bt_addr_le_t *addr) {
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (connected_toys[i] != NULL) {
			if (bt_addr_le_cmp(bt_conn_get_dst(connected_toys[i]), addr) == 0) {
				LOG_INF("Already connected to this adress");
				return 1; 
			}
		}
    }
	return 0;
}