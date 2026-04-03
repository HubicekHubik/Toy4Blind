/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Toy, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
//#include <zephyr/sys/reboot.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <nrf.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>

/*modules that uses Toy and Remote together*/
#include "toy_utils.h"
/*Modules for nvs*/
#include "nvs_modul.h"
/*Modules for global variables of app state*/
#include "app_state.h"
/*Modules for sd operations*/
#include "sd_modul.h"
/*Modules for imu and edge impulse wrapper*/
#include "ei_imu_modul.h"
/*Modules for audio response*/
#include "audio_modul.h"
/*Modules for haptic response*/
#include "haptic_modul.h"
/*Modules for battery and charging measurements*/
#include "bat_measures_modul.h"
/*Modules for BLE comunication*/
#include "ble_modul.h"

#define SYSSOUND_THREAD_PRIORITY 11
#define SYSSOUND_THREAD_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(syssound_stack_area, SYSSOUND_THREAD_STACK_SIZE);
static struct k_thread syssound_thread_data;

static void system_feedback_thread(void *arg1, void *arg2, void *arg3);

int main(void)
{
	int err;
	/*nvs storage init*/
	err = nvs_init_datarec(&volume, &current_category, &device_running);
	if (err) {
		LOG_ERR("NVS initialization failed, err %d", err);
	}
	/*end of nvs storage init*/

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	err = sd_modul_init();
	if (err) {
		LOG_ERR("SD modul initialization failed, err %d", err);
		return err;
	}

	err = imu_ei_init();
	if (err) {
		LOG_ERR("Edge Impulse & IMU modul initialization failed, err %d", err);
		return err;
	}
	err = audio_modul_init();
	if (err) {
		LOG_ERR("Audio Haptic modul initialization failed, err %d", err);
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
		return err;
	}
	return 0;
}

static void system_feedback_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("System feedback thread started");
	struct toy_events rx_event;
    while (true) {
        if(k_msgq_get(&sysfb_msgq, &rx_event, K_FOREVER) == 0) {
			switch (rx_event.type) {
				case MT_BT_CONNECTED:
					k_msgq_put(&aud_event_msgq, &rx_event,K_NO_WAIT);
				case MT_REQUEST_SD_DATA:
					struct file_data data_req = {
						.file_msg_type = MT_REQUEST_SD_DATA
					};
					k_msgq_put(&aud_dataq, &data_req, K_NO_WAIT); // Pošlu do vlákna pro přenos souborů
					break;
				case MT_LSM6DSL_ON:
				case MT_LSM6DSL_OFF:
					if(!device_running){
						device_running = true;
						nvs_save_power_state(device_running);
						lsm6dsl_ei_wake();
						LOG_INF("Toy turned ON");
					} else {
						lsm6dsl_sleep();
						device_running = false;
						LOG_INF("Toy turned OFF");
						nvs_save_power_state(device_running);
					}
					break;
				case MT_VOL_UP:
					if(volume <= MAX_VOLUME) {
						volume += VOLUME_STEP;
						nvs_save_volume(volume);
						LOG_INF("Volume has increased to %d", volume);
					} else {
						LOG_WRN("Volume is at maximum %d", volume);
					}
					break;
				case MT_VOL_DOWN:
					if(volume >= MIN_VOLUME) {
						volume -= VOLUME_STEP;
						nvs_save_volume(volume);
						LOG_INF("Volume has decreased to %d", volume);
					} else {
						LOG_WRN("Volume is at minimum %d", volume);
					}
					break;
				case MSG_TYPE_TURNOFF:
					if (device_running) {
						LOG_INF("Turning off device");
						device_running = false;
						lsm6dsl_sleep();
					} else {
						LOG_INF("Turning on device");
						device_running = true;
						lsm6dsl_ei_wake();
					}
					break;
				case MT_GEST:
					struct toy_events tx_lmr = {
						.type = MT_LMR,
						.payload.lmr.effect = rx_event.payload.gest.type
					};
					k_msgq_put(&lmr_msgq, &tx_lmr, K_NO_WAIT);
					k_msgq_put(&aud_event_msgq, &rx_event, K_NO_WAIT);
					break;
				case MT_CHANGE_CATEGORY:
					current_category= (current_category + 1) % actual_categories_count;
					nvs_save_categ(current_category);
					LOG_INF("Current category changed to %d", current_category);
				break;
				default:
					LOG_WRN("Unknown system feedback message received: %d", rx_event.type);
					break;
			}
		}
    }
}