/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Main, CONFIG_LOG_DEFAULT_LEVEL);

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

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

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
/*Modules for Game mods*/
#include "game_modul.h"

#define SYSSOUND_THREAD_PRIORITY 11
#define SYSSOUND_THREAD_STACK_SIZE 2048

K_THREAD_STACK_DEFINE(syssound_stack_area, SYSSOUND_THREAD_STACK_SIZE);
static struct k_thread syssound_thread_data;

static void system_feedback_thread(void *arg1, void *arg2, void *arg3);

int main(void)
{
	int err;
	/*nvs storage init*/
	err = nvs_init_datarec(&volume, &current_category, &device_running, &current_game);
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
	
	
	err = game_modul_init();
	if (err) {
		LOG_ERR("GAME modul initialization failed, err %d", err);
		return err;
	}

	return 0;
}

static void system_feedback_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("System feedback thread started");
	struct toy_events rx_event;
	struct toy_events last_bat = {0};
	struct toy_events tx_audio = {0};
	struct toy_events tx_lra = {.type = MT_LRA, .len = sizeof(struct lra_ev)};

    while (true) {
        if(k_msgq_get(&sysfb_msgq, &rx_event, K_FOREVER) == 0) {
			bool send = false;
			bool sendaudio = false;
			bool sendlra = false;

			switch (rx_event.type) {
				case MT_REQUEST_SD_DATA:
					struct file_data data_req = {
						.file_msg_type = MT_REQUEST_SD_DATA
					};
					k_msgq_put(&aud_dataq, &data_req, K_NO_WAIT); // Pošlu do vlákna pro přenos souborů
					break;
				case MT_LSM6DSL_ON:
				case MT_LSM6DSL_OFF:
					sendaudio = true;
					if(!device_running){
						device_running = true;
						nvs_save_power_state(device_running);
						lsm6dsl_ei_wake();
						tx_audio.type = MT_LSM6DSL_ON;
						LOG_INF("Toy turned ON");
					} else {
						lsm6dsl_sleep();
						device_running = false;
						tx_audio.type = MT_LSM6DSL_OFF;
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
					if(volume > MIN_VOLUME) {
						volume -= VOLUME_STEP;
						nvs_save_volume(volume);
						LOG_INF("Volume has decreased to %d", volume);
					} else {
						LOG_WRN("Volume is at minimum %d", volume);
					}
					break;
				case MT_GEST:
					if (current_game == GAME_MODE_SWAPPED && remote_conn != NULL) {
						char send_buffer[120];
						int len = sizeof(rx_event.payload);
						memcpy(&send_buffer[1], &rx_event.payload, len);
						bt_nus_send(remote_conn, (const uint8_t *)send_buffer, len + 1);
						break;
					}
					if (current_game == GAME_MODE_CHAINS) {
						k_msgq_put(&game_ev_msgq, &rx_event, K_NO_WAIT);
						break;
					}
					sendaudio = true;
					sendlra = true;
					sendlra = true;
					tx_lra.payload.lra.effect = rx_event.payload.gest.type;
					tx_audio.type = MT_GEST;
					tx_audio.payload.audio.id = rx_event.payload.gest.type;
					break;
				case MT_CHANGE_CATEGORY:
					if(current_game == GAME_MODE_CHAINS) {
						LOG_INF("current_game is CHAINS to change category change the game mode");
						break;
					}
					current_category= (current_category + 1) % actual_categories_count;
					nvs_save_categ(current_category);
					sendaudio = true;
					tx_audio.type = MT_CHANGE_CATEGORY;
					tx_audio.payload.audio.id = (uint8_t *)categories[current_category].categ_id;
					LOG_INF("Current category changed to %d", current_category);
					break;
				case MT_LOW_BATERY:
					sendaudio = true;
					tx_audio.type = MT_LOW_BATERY;
					break;
				case MT_REQ_LASTBAT:
					memcpy(&rx_event.payload.power, &last_bat.payload.power, sizeof(rx_event.payload.power));
					last_bat.payload.power.deviceOn = device_running;
                    send = true; 
                    break;
				case MT_CHARGING:
				case MT_CHARGED:
					sendlra = true;
					tx_lra.payload.lra.effect = CHARGING_EFFECT;
				case MT_BAT_INF:
					memcpy(&last_bat.payload.power, &rx_event.payload.power, sizeof(rx_event.payload.power));
					send = true;
					break;
				case MT_G_MODE_CHANGE:
					current_game= (current_game + 1) % GAME_MODE_CNT;
					nvs_save_gameM(current_game);
					LOG_INF("Current game mode changed to %d", current_game);
					sendaudio = true;
					tx_audio.type = MT_G_MODE_CHANGE;
					tx_audio.payload.gest.type = current_game;
					if(current_game == GAME_MODE_CHAINS) {
						current_category = ctg_piano_idx;
						nvs_save_categ(current_category);
					} 
					break;
				case MT_SWITCH_TOY:
					sendlra = true;
					tx_lra.payload.lra.effect = CONNECTED_EFFECT;
					break;
				case MT_BT_CONNECTED:
					sendaudio = true;
					tx_audio.type = MT_BT_CONNECTED;
					break;
				case MT_BT_DISCONNECT:
					sendaudio = true;
					tx_audio.type = MT_BT_DISCONNECT;
					break;
				default:
					LOG_WRN("Unknown system feedback message received: %d", rx_event.type);
					break;
			}
			if (sendlra) {
				k_msgq_put(&lra_msgq, &tx_lra, K_NO_WAIT);
			}
			
			if(sendaudio) {
				k_msgq_put(&aud_event_msgq, &tx_audio, K_NO_WAIT);
			}
			if(send && master_conn != NULL) {
                char send_buffer[128];
				int len = sizeof(rx_event.payload.power);
                send_buffer[0] = MT_RECV_POW_DATA;
                memcpy(&send_buffer[1], &rx_event.payload.power, len);

                bt_nus_send(master_conn, (const uint8_t *)send_buffer, len + 1);
                
                LOG_INF("BT Send -> V_bat: %d, Charging: %d, Charged: %d, DeviceON: %d",
                        rx_event.payload.power.V_bat,
                        rx_event.payload.power.charging,
						rx_event.payload.power.charged,
                        rx_event.payload.power.deviceOn);
            }
		}
    }
}