/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <nrf.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <math.h>
/**/
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Toy_Remote, CONFIG_LOG_DEFAULT_LEVEL);

#include "app_state.h"
#include "toy_utils.h"
#include "ble_modul.h"
#include "bat_measures_modul.h"
#include "haptic_modul.h"
#include "but_modul.h"

#define SYSSOUND_THREAD_PRIORITY 11
#define SYSSOUND_THREAD_STACK_SIZE 4092

#define SYSFBQ_LENGTH 16

K_THREAD_STACK_DEFINE(syssound_stack_area, SYSSOUND_THREAD_STACK_SIZE);
static struct k_thread syssound_thread_data;

K_MSGQ_DEFINE(sysfb_Q, sizeof(struct sys_ev), SYSFBQ_LENGTH, 4);

static void system_feedback_thread(void *arg1, void *arg2, void *arg3);

/*moduls for SD card support*/
#include <stdlib.h>
#include <zephyr/sys/util.h>

/*end of moduls for SD card support*/

enum GestureID {
	G_ROLL = 3,
	G_CIRCLE = 0,
	G_DROP = 1,
	G_IDLE = 2,
	G_SHAKE = 5,
	G_SPIN = 4,
};

const char *const g_names[] = {
	"G_CIRCLE",
	"G_DROP",
	"G_IDLE",
	"G_ROLL",
	"G_TAP",
	"G_SHAKE",
};

#define READ_INFO_PRIORITY 1
#define READ_INFO_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(readinfo_stack_area, READ_INFO_THREAD_STACK_SIZE);
static struct k_thread readinfo_thread_data;

void read_toy_info(void *arg1, void *arg2, void *arg3);

int main(void)
{
	int err;

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	k_thread_create(&readinfo_thread_data, readinfo_stack_area,
		K_THREAD_STACK_SIZEOF(readinfo_stack_area),
		read_toy_info,
		NULL, NULL, NULL,
		READ_INFO_PRIORITY, 0, K_NO_WAIT);
	
	err = but_modul_init();
	if (err) {
		LOG_ERR("Button modul initialization failed, err %d", err);
		return err;
	}

	err = haptic_modul_init();
	if (err) {
		LOG_ERR("Audio Haptic modul initialization failed, err %d", err);
		return err;
	}
	
	err = bat_measure_modul_init();
	if (err) {
		LOG_ERR("Baterry measure modul initialization failed, err %d", err);
		return err;
	}
	
	k_thread_create(&syssound_thread_data, syssound_stack_area,
		K_THREAD_STACK_SIZEOF(syssound_stack_area),
		system_feedback_thread,
		NULL, NULL, NULL,
		SYSSOUND_THREAD_PRIORITY, 0, K_NO_WAIT);

	err = ble_modul_init();
	if (err) {
		LOG_ERR("BLE modul initialization failed, err %d", err);
		return 0;
	}

	LOG_INF("Initialization complete");
}

void read_toy_info(void *arg1, void *arg2, void *arg3) {

	//int ret;

	LOG_INF("Send audio thread init");
	
	/*struct esb_payload ping = {
		.pipe = 0,
		.noack = false,
		.length = PING_PKT_LEN,
		.data   = {PKT_PING}
	};*/

	//struct gest_ev g;

	/*while (1) {

		bool gesture_received = false;	//ping until gesture is detected

		while (!gesture_received) {
			esb_flush_tx(); // maby not necceserry (more like veary necceserry!!!!!)
			for (uint8_t i = 0; i < ARRAY_SIZE(addr_prefix); i++) {
				//LOG_INF("pinging prefix (\x1b[32m%u\x1b[0m)\t", addr_prefix[i]);
                esb_update_prefix(0,addr_prefix[i]);
                ret = esb_write_payload(&ping);
                k_sleep(K_MSEC(20));// wait for ack which should contain gesture
			}
			k_sleep(K_MSEC(PING_INTERVAL_MS));
		}

		// === Gesture recievd send audio
		if (g.cmd_id < 255) {
			LOG_INF("Recieved gesture %d Device prefix 0x%02X", g.cmd_id, g.pipeprefix);
		} else {
			LOG_ERR("Unknown gesture ID: %d", g.cmd_id);
		}
		k_sleep(K_MSEC(100)); //wait utnill all packets that might be neccessery to arrive
	}*/
}

static void system_feedback_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("System feedback thread started");
	struct toy_events rx_event;
    while (true) {
        if(k_msgq_get(&sysfb_msgq, &rx_event, K_FOREVER) == 0) {
			switch (rx_event.type) {
				case MT_SYSINF:
					LOG_INF("Recieved something");
					break;
				default:
					LOG_WRN("Unknown system feedback message received: %d", rx_event.type);
					break;
			}		
		}
    }
}