/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/reboot.h>

/*Modules used for I2S and PWM*/
#include <stdio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include "pwmeffects.h"

#define AUDIO_CHUNK_SIZE 1024 //working fine 192
#define AUDIO_SD_CHUNK AUDIO_CHUNK_SIZE //working fine 3072
uint8_t audio_buffer[AUDIO_SD_CHUNK];
/*end of moduls for SD card support*/

#define PRIORITY_PWM 4
#define LMRPWM_THREAD_STACK_SIZE 1024

#define AUDIOSTREAM_MAX_LENGHT (10 * 1000)

/*Modules used for Edge inpuls*/
#include <ei_wrapper.h>
#define HISTORY_LEN 7
#define NUM_CLASSES 9
#define MIN_DETECTION_COUNT 6
#define STRAIGHT_IDX 5
#define DEBUG_RESULT_CB //coment this to use the old function of result_ready

uint8_t hystor_index;
float label_history[HISTORY_LEN][NUM_CLASSES];
float labels_value[NUM_CLASSES];
uint8_t idlefollowup = 0;

//-----------------------------------//
bool audioEventStart = false;
bool audioStreamDone = false;
bool audioStreamPlay = false;	//bool used to control audio stream logic

/*Modules used for LSM6DSL*/
#include <zephyr/sys/util.h>
#define IMU_THREAD_STACK_SIZE 10120
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static void lsm6dsl_trigger_handler() {
	while(1) {
		int err;
		float sample_buf[3] = {0.0 , 0.0 , 8.9};

		printk("sensor values %f %f %f\n", sample_buf[0], sample_buf[1], sample_buf[2]);
		err = ei_wrapper_add_data(sample_buf, 3); //uset to be controled by audio stream bool trying to chnage taht
		k_sleep(K_MSEC(20));
	}
 }

/*End of modules used for LSM6DSL*/

#define STACK_SIZE 2048
#define PRIORITY 2

#define MSGQ_LENGTH 1

struct gest_msg {
	uint8_t cmd;
	uint8_t gesture_id;
	uint8_t pipeprefix;
    uint8_t rsv;
};

enum GestureID {
	G_IDLE = 0,
    G_CIRCLE1 = 1,
    G_DROP = 2,
    G_NOISE = 3,
    G_SQUARE = 4,
    G_STRAIGHT = 5,
};

/*Packet ID */
#define PKT_GEST 0xB1
#define PKT_PING 0xB2
#define PKT_STAUDIO 0xAB
#define PKT_AUDIO 0xAD
#define PKT_ENDAUDIO 0xAE

K_MSGQ_DEFINE(msg_queue, sizeof(struct gest_msg), MSGQ_LENGTH, 4);

static volatile bool esb_tx_done = true;   // bool used for controling TX logic of gest


#ifdef CONFIG_NOCACHE_MEMORY
	#define MEM_SLAB_CACHE_ATTR __nocache
#else
	#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

#define BLOCK_SIZE AUDIO_CHUNK_SIZE
#define SLAB_COUNT 4

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32)) 
	 _k_mem_slab_buf_tx_0_mem_slab[(SLAB_COUNT) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	 Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
							 WB_UP(BLOCK_SIZE), SLAB_COUNT);

void check_bitrate_monitoring(size_t new_packet_length_bits);
/*bitrate tracking*/

#define PWM0_NODE DT_NODELABEL(pwm0)

K_THREAD_STACK_DEFINE(lmrPwm_stack_area, LMRPWM_THREAD_STACK_SIZE);
static struct k_thread lmrPwm_thread_data;

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

#define PING_TIMEOUT_MS (300 * 1000) // 1 min

const int8_t DEVICE_PREFIX = 0xA2;

int clocks_start(void);

static void imu_trigger(void *arg1, void *arg2, void *arg3);

static void result_ready_cb(int err);

K_THREAD_STACK_DEFINE(audio_stack_area, STACK_SIZE);

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

	LOG_INF("Initialization complete");
	LOG_INF("Setting up for packet receiption");

	k_thread_create(&lmrPwm_thread_data, lmrPwm_stack_area,
	K_THREAD_STACK_SIZEOF(lmrPwm_stack_area),
	lsm6dsl_trigger_handler,
	NULL, NULL, NULL,
	3, 0, K_NO_WAIT);

	k_thread_create(&imu_thread_data, imu_stack_area,
		K_THREAD_STACK_SIZEOF(imu_stack_area),
		imu_trigger,
		NULL, NULL, NULL,
		IMU_THREAD_PRIORITY, 0, K_NO_WAIT);

	/* return to idle thread */
	return 0;
}

int clocks_start(void) {

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

	LOG_DBG("HF clock started");
	return 0;
}

static void result_ready_cb(int err) {
    if (err) {
        printk("Result error: %d\n", err);
        return;
    }

    /* 1. Vynuluj aktuální řádek a odečti staré hodnoty */
    for (int c = 0; c < NUM_CLASSES; c++) {
        labels_value[c] -= label_history[hystor_index][c];
        label_history[hystor_index][c] = 0.0f;
    }

    /* 2. Přečti klasifikační výsledky */
    const char *label;
    float prob;
    size_t idx;

    while (!ei_wrapper_get_next_classification_result(&label, &prob, &idx)) {
        if (idx < NUM_CLASSES) {
            label_history[hystor_index][idx] = prob;
            labels_value[idx] += prob;
        }
    }

    /* 4. Posuň index v historii */
    hystor_index = (hystor_index + 1) % HISTORY_LEN;

    /* 5. Najdi třídu s nejvyšší pravděpodobností */
    int max_i = 0;
    float max_v = labels_value[0];
    for (int c = 1; c < NUM_CLASSES; c++) {
        if (labels_value[c] > max_v) {
            max_v = labels_value[c];
            max_i = c;
        }
    }

    /* 6. Spočítej "hlasy" (kolikrát se daná třída opakovala) */
    int votes = 0;
    for (int k = 0; k < HISTORY_LEN; k++) {
        if (label_history[k][max_i] > 0.55f) {
            votes++;
        }
    }

    //printk("Label '%s' (%d) votes: %d\n",
    //     ei_wrapper_get_classifier_label(max_i), max_i, votes);

    if (max_i == 0) {
        ++idlefollowup;
    } else {
        idlefollowup = 0;
    }

    /* 7. Detekce gesta */
    if (votes >= MIN_DETECTION_COUNT &&
        max_i != STRAIGHT_IDX && max_i != 3 &&
        ((max_i != 0) || (idlefollowup == 230))) {

        struct gest_msg g = {
            .cmd        = PKT_GEST,
            .gesture_id = max_i,
            .pipeprefix = DEVICE_PREFIX
        };

        if (!audioEventStart) {
            printk("Gesture detected: %s\n",
                   ei_wrapper_get_classifier_label(max_i));
            k_msgq_put(&msg_queue, &g, K_NO_WAIT);
        }

        memset(label_history, 0, sizeof(label_history));
        memset(labels_value, 0, sizeof(labels_value));
        hystor_index = 0;
        idlefollowup = 0;
    }

    /* 8. Spusť další inference */
    ei_wrapper_start_prediction(0, 3);
}

static void imu_trigger(void *arg1, void *arg2, void *arg3) {

	int err;

	err = ei_wrapper_init(result_ready_cb);
	if (err) {
		LOG_ERR("Edge Impulse wrapper failed to initialize (err: %d)\n",err);
		//return 0;
	}

	err = ei_wrapper_start_prediction(0, 0);
	LOG_ERR("valueof err:%d", err);

	if (err) {
		LOG_ERR("Cannot start prediction (err: %d)\n", err);
	} else {
		LOG_INF("Prediction started...\n");
	}	
	while (1) {
        k_sleep(K_FOREVER);   /* uspi vlákno – jen aby se neukončilo */
    }
}
