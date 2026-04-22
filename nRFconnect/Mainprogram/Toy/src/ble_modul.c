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

void con_discon_form_adress(struct bt_conn *conn, bool connect);

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
static void update_data_len_mtu(struct bt_conn *conn);
static void update_timing_phy(struct bt_conn *conn, uint8_t cmd);
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
    con_discon_form_adress(conn, true);

    //master_conn = bt_conn_ref(conn); // Save the connection
    LOG_INF("Device Connected!");
	
	k_sleep(K_MSEC(1));
    struct toy_events tx_conn_e = {.type = MT_BT_CONNECTED};
	k_msgq_put(&sysfb_msgq, &tx_conn_e, K_NO_WAIT);

    update_data_len_mtu(conn);
    update_timing_phy(conn, MT_DEC_SPEED);
    
    if ((master_conn == NULL) || (remote_conn == NULL)) {
        int adv_err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
        if (adv_err) {
            LOG_ERR("Unable to reset advertising (err %d)", adv_err);
        } else {
            LOG_INF("Advertising reset...");
        }
    }
}

void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Disconnected (reason %u)", reason);
    
    struct toy_events tx_conn_e = {.type = MT_BT_DISCONNECT};
    k_msgq_put(&sysfb_msgq, &tx_conn_e, K_NO_WAIT);

    con_discon_form_adress(conn, false);

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
            case MT_GEST:
            	if (device_running) {
                    k_msgq_put(&aud_event_msgq, data, K_NO_WAIT);
                }
                break;
            case MT_G_MODE_CHANGE:
            case MT_CHANGE_CATEGORY:
            case MSG_TYPE_TURNOFF:
            case MT_LSM6DSL_OFF:
            case MT_LSM6DSL_ON:
            case MT_VOL_UP:
            case MT_VOL_DOWN:
            case MT_REQUEST_SD_DATA:
            case MT_SWITCH_TOY:
            case MT_REQ_LASTBAT:
                LOG_INF("Recieved comand form remote : 0x%2x",cmd);
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
                    struct file_data fd;
                    fd.file_msg_type = cmd;
                    fd.data_len = len - 1;

                    if (fd.data_len > sizeof(fd.data)) {
                        fd.data_len = sizeof(fd.data);
                    }

                    memcpy(fd.data, &data8[1], fd.data_len);

                    if (fd.data_len < sizeof(fd.data)) {
                        fd.data[fd.data_len] = '\0';
                    }
                    k_msgq_put(&aud_dataq, &fd, K_NO_WAIT);
                }
                break;
            case MT_INC_SPEED:
                LOG_INF("Recieved msg to increase BLE speed");
                update_timing_phy(conn, cmd);
                break;
            case MT_DEC_SPEED:
                LOG_INF("Recieved msg to decrease BLE speed");
                update_timing_phy(conn, cmd);
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
static void update_data_len_mtu(struct bt_conn *conn) {
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
}

static void update_timing_phy(struct bt_conn *conn, uint8_t cmd) {
    int err;
    
    if (cmd == MT_INC_SPEED) {
        LOG_INF("Setting BLE to FAST mode");

        const struct bt_conn_le_phy_param phy_param_fast = {
            .pref_tx_phy = BT_GAP_LE_PHY_2M,
            .pref_rx_phy = BT_GAP_LE_PHY_2M,
        };
        bt_conn_le_phy_update(conn, &phy_param_fast);

        k_sleep(K_MSEC(50));

        struct bt_le_conn_param *fast_params = BT_LE_CONN_PARAM(12, 12, 5, 400); 
        err = bt_conn_le_param_update(conn, fast_params);
        
    } else {
        LOG_INF("Setting BLE to SLOW mode");

        const struct bt_conn_le_phy_param phy_param_slow = {
            .pref_tx_phy = BT_GAP_LE_PHY_1M,
            .pref_rx_phy = BT_GAP_LE_PHY_1M,
        };
        bt_conn_le_phy_update(conn, &phy_param_slow);

        k_sleep(K_MSEC(50));

        // Interval 45-60ms - šetří baterii, stačí pro povely z ovladače
        struct bt_le_conn_param *slow_params = BT_LE_CONN_PARAM(36, 40, 5, 400); 
        err = bt_conn_le_param_update(conn, slow_params);
    }

    if (!err) {
        LOG_INF("Link parameters update initiated");
    } else {
        LOG_ERR("Link parameters update failed (err %d)", err);
    }
}

void con_discon_form_adress(struct bt_conn *conn, bool connect) {
    const bt_addr_le_t *addr = bt_conn_get_dst(conn);
    if (connect){
        if ((addr->a.val[5] & 0xc0) == 0x40) {
            if (!master_conn) {
                master_conn = bt_conn_ref(conn);
                LOG_INF("Conected to App (based on adress type))");
            }
        } else {
            if (!remote_conn) {
                remote_conn = bt_conn_ref(conn);
                LOG_INF("Conected to Remote (based on adress type))");
            }
        }
    } else {
        if ((addr->a.val[5] & 0xc0) == 0x40) {
            if (master_conn) {
                bt_conn_unref(master_conn);
                master_conn = NULL;
                LOG_INF("Disconected form App (based on adress type))");
            }
        } else {
            if (remote_conn) {
                bt_conn_unref(remote_conn);
                remote_conn = NULL;
                LOG_INF("Disconected from Remote (based on adress type))");
            }
        }
    }
}