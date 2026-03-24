#include "ble_modul.h"
#include "app_state.h"
#include "toy_utils.h"
#include "sd_modul.h"

#include "audio_modul.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(BLE_modul, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/bluetooth/services/nus/inst.h>
#define DEVICE_NAME		CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(CONFIG_BT_DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

void connected(struct bt_conn *conn, uint8_t err);

void disconnected(struct bt_conn *conn, uint8_t reason);

static void bt_receive_cb(struct bt_conn *conn, const void *data, uint16_t len, void *ctx);

static void le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    LOG_INF("DLE REALITA: TX_len: %u, RX_len: %u", info->tx_max_len, info->rx_max_len);
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("TIMING REALITA: Interval %.2f ms", interval * 1.25);
}

static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    LOG_INF("--- PHY UPDATE POTVRZEN ---");
    LOG_INF("Aktuální TX PHY: %s", (param->tx_phy == BT_GAP_LE_PHY_2M) ? "2 Mbps" : "1 Mbps");
    LOG_INF("Aktuální RX PHY: %s", (param->rx_phy == BT_GAP_LE_PHY_2M) ? "2 Mbps" : "1 Mbps");
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
	.le_data_len_updated = le_data_len_updated,
	.le_param_updated = le_param_updated,
	.le_phy_updated =le_phy_updated
};

static struct bt_nus_cb nus_callbacks = {
    .received = bt_receive_cb,
};

static struct bt_gatt_exchange_params exchange_params;

static void update_data_len_mtu_timing_phy(struct bt_conn *conn);
static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params);

#define BT_THREAD_PRIORITY 8
#define BT_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(BT_stack_area, BT_THREAD_STACK_SIZE);
static struct k_thread BT_thread_data;

static void BT_thread(void *arg1, void *arg2, void *arg3);

int ble_modul_init(void){
    k_thread_create(&BT_thread_data, BT_stack_area,
                        K_THREAD_STACK_SIZEOF(BT_stack_area),
                        BT_thread,
                        NULL, NULL, NULL,
                        BT_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

static void BT_thread(void *arg1, void *arg2, void *arg3) {
    bt_enable(NULL);
    // Tady se NUS "narodí" jen jednou a správně
	int err = bt_nus_cb_register(&nus_callbacks, NULL);
	if (err) {
		printk("Failed to register CONN callback: %d\n", err);
		return;
	}
    bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    while (1) {
		k_sleep(K_SECONDS(1));
	}
}

void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    master_conn = bt_conn_ref(conn); // Save the connection
    LOG_INF("Phone Connected!");
	
	k_sleep(K_MSEC(1));
    struct toy_events tx_conn_e = {
        .type = MT_BT_CONNECTED,
        .payload.lmr.effect = CONNECTED_EFFECT
    };
	k_msgq_put(&aud_event_msgq, &tx_conn_e, K_NO_WAIT);
    update_data_len_mtu_timing_phy(conn);
}

void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
    
    struct toy_events tx_conn_e = {
    .type = MT_BT_DISCONNECT,
    .payload.lmr.effect = MT_BT_DISCONNECT
    };
    k_msgq_put(&aud_event_msgq, &tx_conn_e, K_NO_WAIT);

    if (master_conn) {
        bt_conn_unref(master_conn);
        master_conn = NULL;
    }

	bt_le_adv_stop();

	int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1 , ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Could not start advertising (err %d)\n", err);
    } else {
        printk("Advertising started.\n");
    }
}

static void bt_receive_cb(struct bt_conn *conn, const void *data, uint16_t len, void *ctx) {
	const uint8_t *data8 = (const uint8_t *)data;
    uint8_t cmd = data8[0];
	switch (cmd) {
            case MT_CHANGE_CATEGORY:
            case MSG_TYPE_TURNOFF:
            case MT_LSM6DSL_OFF:
            case MT_LSM6DSL_ON:
            case MT_VOL_UP:
            case MT_VOL_DOWN:
            case MT_REQUEST_SD_DATA:
                LOG_INF("Recieved comand form remote :%d",cmd);
                k_msgq_put(&sysfb_msgq, &cmd, K_NO_WAIT);
                break;
            case MT_FILE:
            case MT_FILE_TRANSFER:
            case MT_DELETE_SD_FILE:
            case MT_DELETE_SD_CATEGORY:
            case MT_RENAME_SD_FILE:
            case MT_RENAME_SD_FOLDER:
            case MT_RENAME_SD_CATEGORY:
            case MT_ADD_SD_DIR:
                {
                    struct file_data fd; // Definujeme jen jednu strukturu pro všechny
                    fd.file_msg_type = cmd;
                    fd.data_len = len - 1;

                    // Bezpečnostní kontrola, abychom nepřetekli buffer v struct file_data
                    if (fd.data_len > sizeof(fd.data)) {
                        fd.data_len = sizeof(fd.data);
                    }

                    memcpy(fd.data, &data8[1], fd.data_len);

                    // Přidání nulového terminátoru pro řetězce (cesty, názvy souborů)
                    if (fd.data_len < sizeof(fd.data)) {
                        fd.data[fd.data_len] = '\0';
                    }
                    k_msgq_put(&aud_dataq, &fd, K_NO_WAIT);
                }
                break;
            case MT_BT_DISCONNECT:
                //k_msgq_put(&sysfb_msgq, data, K_NO_WAIT); // Pošlu do systému
                LOG_INF("Recieved info to switch off device");
				int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
				if (err) {
					printk("Chyba pri odpojovani: %d\n", err);
				}
				break;
            case MSG_TYPE_SOUNDSET:
                //k_msgq_put(&sysfb_msgq, data, K_NO_WAIT); // Pošlu k motorům
                LOG_INF("Recieved info to switch sound set");
				break;
			default:
                LOG_WRN("Unknown comand throwing away 0x%2x.\n", data8[0]);
                break;
        }
}

static void exchange_func(struct bt_conn *conn, uint8_t err,
                          struct bt_gatt_exchange_params *params)
{
    if (err) {
        printk("MTU exchange failed (err %u)\n", err);
    } else {
        printk("MTU updated: %u\n", bt_gatt_get_mtu(conn));
    }
}

static void update_data_len_mtu_timing_phy(struct bt_conn *conn)
{
    int err;

    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        printk("data_len_update failed (err %d)", err);
    }

    exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        printk("bt_gatt_exchange_mtu failed (err %d)", err);
    }
	k_sleep(K_MSEC(20));
    const struct bt_conn_le_phy_param phy_param = {
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    err = bt_conn_le_phy_update(conn, &phy_param);
    if (!err) {
        LOG_INF("PHY update initiated successfully");
    } else {
    	LOG_ERR("Failed to initiate PHY update (err %d)", err);
	}
	k_sleep(K_MSEC(20));
	struct bt_le_conn_param *fast_params = BT_LE_CONN_PARAM(12, 12, 0, 400); 
	bt_conn_le_param_update(master_conn, fast_params);
}