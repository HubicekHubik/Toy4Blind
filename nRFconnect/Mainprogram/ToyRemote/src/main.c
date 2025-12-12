/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/random/random.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
/*modules for the Buttons*/
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec button3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback button1_cb_data;
static struct gpio_callback button2_cb_data;
static struct gpio_callback button3_cb_data;

#define DEBOUNCE_MS 1

void button_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#define BUTTONS_PRIORITY 12
#define BUTTONS_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(buttons_stack_area, BUTTONS_THREAD_STACK_SIZE);
static struct k_thread buttons_thread_data;

volatile bool btn_pressed[3] = {false, false, false};

/*moduls for SD card support*/
#include <stdlib.h>
#include <zephyr/sys/util.h>

/*end of moduls for SD card support*/

/*Packet ID */
#define PKT_GEST 0xB1
#define PKT_PING 0xB2
#define PKT_STAUDIO 0xAB
#define PKT_AUDIO 0xAD
#define PKT_ENDAUDIO 0xAE
#define PKT_TURNOFF 0x0F
#define PKT_SYSINF 0x1F
#define PKT_SYSCHNG 0x8C

enum GestureID {
	G_DROP = 0,
    G_IDLE = 1,
    G_Roll = 2,
    G_SHAKE = 3,
};

const char *const g_names[] = {
	"G_DROP",
	"G_IDLE",
	"G_Roll",
	"G_SHAKE",
};

struct sys_ev {
	uint8_t identifier;
	uint8_t device_on;
	uint8_t device_on_chg;
	uint8_t device_charged;
	uint16_t V_bat;
	uint16_t V_ched;
	uint16_t V_ching;
};

#define MSGQ_LENGTH 64

struct gest_msg {
	uint8_t cmd;
	uint8_t cmd_id;
	uint8_t pipeprefix;
	uint8_t rsv;
};

uint8_t addr_prefix[2] = {223, 21};
#define DEVICE_COUNT ARRAY_SIZE(addr_prefix)
int dev_sel = 0;

K_MSGQ_DEFINE(msg_queue, sizeof(struct gest_msg), MSGQ_LENGTH, 4);

#define SEND_AUDIO_PRIORITY 1
#define SEND_AUDIO_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(sendaudio_stack_area, SEND_AUDIO_THREAD_STACK_SIZE);
static struct k_thread sendaudio_thread_data;

/**/
LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)
	 
struct esb_payload rx;

void event_handler(struct esb_evt const *event) {

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		//LOG_DBG("TX SUCCESS EVENT");
		break;

	case ESB_EVENT_TX_FAILED:
		//LOG_DBG("TX FAILED EVENT");
		break;

	case ESB_EVENT_RX_RECEIVED:
		//LOG_DBG("RX RECIEVED EVENT");
		if (esb_read_rx_payload(&rx) == 0) {
			uint8_t PC_ID = rx.data[0];
			switch (PC_ID) {
				case PKT_PING:
					LOG_INF("PING PACKET RECIEVED");
					break;

				case PKT_GEST:
						sizeof(struct gest_msg);
						struct gest_msg *g = (struct gest_msg *)rx.data;
						printf("G_id(\x1b[34m%s\x1b[0m) prob: \x1b[32m%d\x1b[0m device %u\n", g_names[g->cmd_id], g->rsv,g->cmd);
					break;
				case PKT_SYSINF:
				struct sys_ev *dbg = (struct sys_ev *)rx.data;
				/*printk("---- DEBUG RX SYS_EV ----\n");
				printk("PKT_IDENTIFIER   : 0x%0x\n", dbg->identifier);
				printk("Device charged  : %d\n", dbg->device_charged);
				printk("Device ON        : %d\n", dbg->device_on);
				printk("Device on charger: %d\n", dbg->device_on_chg);
				printk("Battery voltage  : %d mV\n", dbg->V_bat);
				printk("Charging pin voltage  : %d mV\n", dbg->V_ching);
				printk("Charged pin voltage  : %d mV\n", dbg->V_ched);
				printk("-------------------------\n");
				*/
				break;
				default:
					break;
			}
		}
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	LOG_DBG("HF clock started");
	return 0;
}
#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */


/*Start of moduls used for sandig audio data*/

#define PING_INTERVAL_MS  1000
#define PING_PAUSE_MS     20
#define PING_PKT_LEN      1

/*End of moduls for SD card initialization*/

int esb_initialize(void);

void read_toy_info(void *arg1, void *arg2, void *arg3);

void read_buttons(void *arg1, void *arg2, void *arg3);

int main(void)
{
	int err;

	err = clocks_start();
	if (err) {
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	k_thread_create(&sendaudio_thread_data, sendaudio_stack_area,
		K_THREAD_STACK_SIZEOF(sendaudio_stack_area),
		read_toy_info,
		NULL, NULL, NULL,
		SEND_AUDIO_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&buttons_thread_data, buttons_stack_area,
		K_THREAD_STACK_SIZEOF(buttons_stack_area),
		read_buttons,
		NULL, NULL, NULL,
		BUTTONS_PRIORITY, 0, K_NO_WAIT);

	LOG_INF("Initialization complete");

}

int esb_initialize(void)
{
	int err;
	/* These aren't arbitrary default addresses any more. (In end user products
	 * different addresses should be used for each set of devices.)
	 */
	uint8_t base_addr_0[4] = {0xF1, 0xF1, 0xF1, 0xF1};
	struct esb_config config = ESB_DEFAULT_CONFIG;
	
	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.tx_output_power = 8;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;

	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		LOG_ERR("esb_init failed: %d", err);
		return err;
	}

	// Set channel to 80 ( that means 2480 MHz)
	err = esb_set_rf_channel(79);
	if (err) {
		LOG_ERR("Set RF channel failed");
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	//err = esb_set_tx_power(27);
	
	if (err) {
		LOG_ERR("Set TX Power failed");
		return err;
	}
	return 0;
}

void read_toy_info(void *arg1, void *arg2, void *arg3) {

	int ret;

	LOG_INF("Send audio thread init");
	
	struct esb_payload ping = {
		.pipe = 0,
		.noack = false,
		.length = PING_PKT_LEN,
		.data   = {PKT_PING}
	};

	struct gest_msg g;

	while (1) {

		bool gesture_received = false;	//ping until gesture is detected

		while (!gesture_received) {
			esb_flush_tx(); // maby not necceserry (more like veary necceserry!!!!!)
			for (uint8_t i = 0; i < ARRAY_SIZE(addr_prefix); i++) {
				//printk("pinging prefix (\x1b[32m%u\x1b[0m)\t", addr_prefix[i]);
                esb_update_prefix(0,addr_prefix[i]);
                ret = esb_write_payload(&ping);
                k_sleep(K_MSEC(20));// wait for ack which should contain gesture
			}
			k_sleep(K_MSEC(PING_INTERVAL_MS));
		}

		/* === Gesture recievd send audio === */
		if (g.cmd_id < 255) {
			if (g.cmd_id == G_IDLE) {
				g.cmd_id = sys_rand8_get() % 255;
				LOG_INF("random gesture %d Device prefix 0x%02X", g.cmd_id, g.pipeprefix);
			}
		} else {
			LOG_ERR("Unknown gesture ID: %d", g.cmd_id);
		}
		k_sleep(K_MSEC(100)); //wait utnill all packets that might be neccessery to arrive
	}
}

void read_buttons(void *arg1, void *arg2, void *arg3) {
    int ret;

    if (!device_is_ready(button1.port) || !device_is_ready(button2.port)) {
        LOG_ERR("GPIO device not ready");
        return 0;
    }

    // Configure buttons as input with pull-up
    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&button2, GPIO_INPUT);
    if (ret) return ret;

    ret = gpio_pin_configure_dt(&button3, GPIO_INPUT);
    if (ret) return ret;

    // Configure interrupts on falling edge (button to GND)
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    ret = gpio_pin_interrupt_configure_dt(&button3, GPIO_INT_EDGE_BOTH);
    if (ret) return ret;

    // Set callback handlers
    gpio_init_callback(&button1_cb_data, button_changed, BIT(button1.pin));
    gpio_add_callback(button1.port, &button1_cb_data);

    gpio_init_callback(&button2_cb_data, button_changed, BIT(button2.pin));
    gpio_add_callback(button2.port, &button2_cb_data);

    gpio_init_callback(&button3_cb_data, button_changed, BIT(button3.pin));
    gpio_add_callback(button3.port, &button3_cb_data);

    LOG_INF("Buttons initialized");

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

void button_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    int index;

    // Zjistí, které tlačítko vyvolalo interrupt
    if (pins & BIT(button1.pin)) {
        index = 0;
    } else if (pins & BIT(button2.pin)) {
        index = 1;
    } else if (pins & BIT(button3.pin)) {
        index = 2;
	} else {
		return;
	}


    static int64_t last_time[3] = {0, 0, 0};
    static int64_t press_start[3] = {0, 0, 0};

    int64_t now = k_uptime_get();
    if (now - last_time[index] < DEBOUNCE_MS) return;
    last_time[index] = now;

    // Zjistí aktuální stav
	bool pressed=false;

	switch (index) {
		case 0: pressed = gpio_pin_get(dev, button1.pin);
			break;
		case 1: pressed = gpio_pin_get(dev, button2.pin);
			break;
		case 2: pressed = gpio_pin_get(dev, button3.pin);
			break;
		default:
			break;
	}

	btn_pressed[index] = pressed;

	struct gest_msg msg_vol = {
		.cmd = PKT_SYSCHNG,
		.cmd_id = 0,
	};

	struct esb_payload tx = {
		.pipe = 0,
		.noack = false,
		.length = sizeof(struct gest_msg),
	};

	if (index == 2) {

        if (pressed && press_start[index] == 0) {
            // just pressed
            press_start[index] = now;
        }

        if (!pressed) {
			int time = now - press_start[index];
            if (time >= 3000) {
                printk("LONG PRESS detected on button 3 (3s)\n");
				struct gest_msg msg;
				msg.cmd = PKT_TURNOFF;
				memcpy(tx.data, &msg, sizeof(msg));
				esb_update_prefix(0, addr_prefix[dev_sel]);
				esb_write_payload(&tx);
				printk("Button %d released\n", index + 1);
				btn_pressed[index] = false;
				press_start[index] = 0;
				return;
            }
		    press_start[index] = 0;
        }
    }

    if (!pressed) {
        printk("Button %d released\n", index + 1);
		switch (index)
		{
		case 0:
			msg_vol.cmd_id=1;
			break;
		case 1:
			msg_vol.cmd_id=0;
			break;
		case 2:
			dev_sel++;
			dev_sel%=DEVICE_COUNT;
			printk("Operating device with id: %d\n", addr_prefix[dev_sel]);
			break;
		default:
			break;
		}
		memcpy(tx.data, &msg_vol, sizeof(msg_vol));
		esb_update_prefix(0,addr_prefix[dev_sel]);
		int ret = esb_write_payload(&tx);
		if (ret) {
			printk("Error writting esb payload ret:%d\n",ret);
		}
		btn_pressed[index] = false;
    } else {
        printk("Button %d pressed\n", index + 1);
    }
}