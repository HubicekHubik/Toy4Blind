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

#define PRIORITY_PWM 4
#define LMRPWM_THREAD_STACK_SIZE 1024

#define AUDIOSTREAM_MAX_LENGHT (10 * 1000)

/*Modules used for Edge inpuls*/
#include <ei_wrapper.h>
#define HISTORY_LEN 7
#define NUM_CLASSES 6
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
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>
#define IMU_THREAD_STACK_SIZE 2024
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static int lsm6dsl_trig_cnt;

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig) {
	int err;
	struct sensor_value accel_x, accel_y, accel_z;
	float sample_buf[3];

	lsm6dsl_trig_cnt++;

	/* Fetch latest accelerometer data */
	err = sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);

	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* Convert to float and push to Edge Impulse */
	sample_buf[0] = sensor_value_to_double(&accel_x);
	sample_buf[1] = sensor_value_to_double(&accel_y);
	sample_buf[2] = sensor_value_to_double(&accel_z);
	err = ei_wrapper_add_data(sample_buf, 3); //uset to be controled by audio stream bool trying to chnage taht
 }
#endif

/*End of modules used for LSM6DSL*/
/*Begining of moduls used for baterry messurement*/
#include <math.h>
#include <stdlib.h>

#include "battery.h"
#define BATTERY_MEASURE_STACK_SIZE 1024
#define BATTERY_MEASURE_PRIORITY   7  
#define BAT_MEASURE_PERIOD 60000 //wait for 1 min between battery measurements

K_THREAD_STACK_DEFINE(battery_measure_stack, BATTERY_MEASURE_STACK_SIZE);
static struct k_thread battery_measure_thread_data;

static const struct battery_level_point levels[] = {
	/* "Curve" here eyeballed from captured data for the [Adafruit
	 * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
	 * under full load that started with a charge of 3.96 V and
	 * dropped about linearly to 3.58 V over 15 hours.  It then
	 * dropped rapidly to 3.10 V over one hour, at which point it
	 * stopped transmitting.
	 *
	 * Based on eyeball comparisons we'll say that 15/16 of life
	 * goes between 3.95 and 3.55 V, and 1/16 goes between 3.55 V
	 * and 3.1 V.
	 */

	{ 10000, 3950 },
	{ 625, 3550 },
	{ 0, 3100 },
};

/*End of moduls used for baterry messurement*/
#define STACK_SIZE 2048
#define PRIORITY 2

#define Q_BLOCK_SIZE (CONFIG_ESB_MAX_PAYLOAD_LENGTH-1)
#define Q_LENGTH 512
#define SILENCE_TIMEOUT_MS 300

K_MSGQ_DEFINE(audio_queue, Q_BLOCK_SIZE, Q_LENGTH, 4);

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

#define GESTURE_TX_STACK_SIZE 2024
#define GESTURE_TX_PRIORITY   3          /* mezi audio (4) a PWM (4) */

K_THREAD_STACK_DEFINE(gesture_tx_stack, GESTURE_TX_STACK_SIZE);
static struct k_thread gesture_tx_thread_data;

static volatile bool esb_tx_done = true;   // bool used for controling TX logic of gest


#ifdef CONFIG_NOCACHE_MEMORY
	#define MEM_SLAB_CACHE_ATTR __nocache
#else
	#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

#define BLOCK_SIZE 384
#define SLAB_COUNT 10

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32)) 
	 _k_mem_slab_buf_tx_0_mem_slab[(SLAB_COUNT) * WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	 Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
							 WB_UP(BLOCK_SIZE), SLAB_COUNT);

/*bitrate tracking*/
static size_t total_bits_received = 0;
static int64_t last_bitrate_check_time = 0;

void check_bitrate_monitoring(size_t new_packet_length_bits);
/*bitrate tracking*/

#define PWM_PERIOD_USEC 20000  // 20 ms perioda (50 Hz)
#define PWM0_NODE DT_NODELABEL(pwm0)

K_THREAD_STACK_DEFINE(lmrPwm_stack_area, LMRPWM_THREAD_STACK_SIZE);
static struct k_thread lmrPwm_thread_data;

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

static struct esb_payload rx_payload;

#define PM_THREAD_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(Pm_stack_area, PM_THREAD_STACK_SIZE);
static struct k_thread Pm_thread_data;

#define PING_TIMEOUT_MS (60 * 1000) // 1 min

static int64_t last_ping_time = 0;

const int8_t DEVICE_PREFIX = 0xA2;

void print_audio_data(uint8_t *data, size_t size); //prints the audio data recieved serves for debuging

void playaudio(const struct device *dev_i2s); // plays the recieved audio data using i2s checks bitrate

int i2s_initialize(const struct device *dev_i2s); //init of i2s for playing audio

void event_handler(struct esb_evt const *event); //controls comunication via ESB

int clocks_start(void);

int esb_initialize(); //init of ESB

void audio_playback_thread(void *arg1, void *arg2, void *arg3);

void LMR_control_thread(void *arg1, void *arg2, void *arg3);

void effect_fade(const struct device *pwm_dev);

void effect_alternate(const struct device *pwm_dev);

void effect_pulse_both(const struct device *pwm_dev);

void effect_fade_left(const struct device *pwm_dev);

void effect_fade_right(const struct device *pwm_dev);

static void imu_trigger(void *arg1, void *arg2, void *arg3);

static void result_ready_cb(int err);

static void gesture_tx_thread(void *arg1, void *arg2, void *arg3);

static void power_manager_thread(void *arg1, void *arg2, void *arg3);

static void battery_measure_thread(void *arg1, void *arg2, void *arg3);

K_THREAD_STACK_DEFINE(audio_stack_area, STACK_SIZE);

static struct k_thread audio_thread_data;
				
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

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}


	LOG_INF("Initialization complete");
	LOG_INF("Setting up for packet receiption");

	k_thread_create(&lmrPwm_thread_data, lmrPwm_stack_area,
		K_THREAD_STACK_SIZEOF(lmrPwm_stack_area),
		LMR_control_thread,
		NULL, NULL, NULL,
		PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&audio_thread_data, audio_stack_area,
		K_THREAD_STACK_SIZEOF(audio_stack_area),
		audio_playback_thread,
		NULL, NULL, NULL,
		PRIORITY_PWM, 0, K_NO_WAIT);

	k_thread_create(&imu_thread_data, imu_stack_area,
		K_THREAD_STACK_SIZEOF(imu_stack_area),
		imu_trigger,
		NULL, NULL, NULL,
		IMU_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&gesture_tx_thread_data, gesture_tx_stack,
		K_THREAD_STACK_SIZEOF(gesture_tx_stack),
		gesture_tx_thread,
		NULL, NULL, NULL,
		GESTURE_TX_PRIORITY, 0, K_NO_WAIT);
	
	//This is wrong aproach you have to change this
	k_thread_create(&Pm_thread_data, Pm_stack_area,
		K_THREAD_STACK_SIZEOF(Pm_stack_area),
		power_manager_thread,
		NULL, NULL, NULL,
		GESTURE_TX_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&battery_measure_thread_data, battery_measure_stack,
		K_THREAD_STACK_SIZEOF(battery_measure_stack),
		battery_measure_thread,
		NULL,NULL,NULL,
		BATTERY_MEASURE_PRIORITY, 0,K_NO_WAIT);

	/* return to idle thread */
	return 0;
}

int esb_initialize() {
	int err;
	/* These are nonarbitrary default addresses.*/
	
	uint8_t base_addr_0[4] = {0xF1, 0xF1, 0xF1, 0xF1};
	uint8_t addr_prefix[1] = {DEVICE_PREFIX};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.tx_mode  = ESB_TXMODE_AUTO;
	config.tx_output_power = 8;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PRX;
	config.selective_auto_ack = true;


	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
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

	return err;
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

void event_handler(struct esb_evt const *event) {
	
	switch (event->evt_id) {
		case ESB_EVENT_RX_RECEIVED:
			int8_t rssi_dbm = -((int8_t)NRF_RADIO->RSSISAMPLE);
			if (esb_read_rx_payload(&rx_payload) == 0) {
				uint8_t PC_CMD = rx_payload.data[0];
				/*if(rx_payload.data[1] != DEVICE_PREFIX) {
					//LOG_INF("The other ball is tring to hiject me bot not now");
					break;
				}*/
				//LOG_INF("what is the value of PC_CMD between 2 RX %02X",PC_CMD);
				switch (PC_CMD) {
					case PKT_GEST:
						break;

					case PKT_STAUDIO:
							audioEventStart =true;
							LOG_INF("Recieved packet to start audio stream with RSSI: %3d dBm", rssi_dbm);
						break;

					case PKT_AUDIO:
							/*if (k_msgq_num_used_get(&audio_queue) > (Q_LENGTH * 3 / 4)) {
								LOG_WRN("Audio queue > 75%% full!");
							}*/
							LOG_INF("Recieved audio data packet with RSSI: %3d dBm", rssi_dbm);
							if (k_msgq_put(&audio_queue, &rx_payload.data[1], K_NO_WAIT) != 0) {
								LOG_ERR("Audio queue full! Dropping packet!");
							}
						break;
					case PKT_ENDAUDIO:
							audioStreamDone = true; //start audio stream
							LOG_INF("Recieved packet to END audio stream with RSSI: %3d dBm", rssi_dbm);
						break;

					case PKT_PING:
						LOG_INF("Recieved PING packet with RSSI: %3d dBm", rssi_dbm);
    					last_ping_time = k_uptime_get();
						break;

					default:
						break;
				}
			} else {
				LOG_ERR("Error while reading rx packet");
			}
			break;

		case ESB_EVENT_TX_SUCCESS:
			esb_tx_done = true;
			LOG_DBG("TX SUCCESS EVENT");
			break;

		case ESB_EVENT_TX_FAILED:
			LOG_DBG("TX FAILED EVENT");
			esb_tx_done = true;
			break;
		}
}


void audio_playback_thread(void *arg1, void *arg2, void *arg3) {
	int ret;
	    
	LOG_INF("Audio thread starting...\n");

	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2stx));

	ret = i2s_initialize(dev_i2s);
	if (ret){
		LOG_ERR("Init failed");
	}else {
		LOG_INF("Initilize of i2s was succesful");
	}
	while (1) {
		if (audioStreamDone) {
			audioStreamDone = false;
			playaudio(dev_i2s);
		}
		k_sleep(K_MSEC(50));
	}		
}

void LMR_control_thread(void *arg1, void *arg2, void *arg3) {
	const struct device *pwm0_dev = DEVICE_DT_GET(PWM0_NODE);
	int function;
	if (!device_is_ready(pwm0_dev)) {
		LOG_ERR("PWM0 device not ready\n");
	} else
	{
		LOG_INF("LMR si ready\n");
	}
	while (1)
	{
		k_sleep(K_MSEC(50));
		if (audioStreamPlay) {
			function = 0;//sys_rand8_get() % 5;
			switch (function) {
				case 0:
					effect_alternate(pwm0_dev);
					break;
				case 1:
					effect_pulse_both(pwm0_dev);
					break;
				case 2:
					effect_fade(pwm0_dev);
					break;
				case 3:
					effect_fade_right(pwm0_dev);
					break;
				case 4:
					effect_fade_left(pwm0_dev);
					break;
				default:
					break;
			}
		}
	}
}

static void imu_trigger(void *arg1, void *arg2, void *arg3) {

	int err;
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	err = ei_wrapper_init(result_ready_cb);
	if (err) {
		LOG_ERR("Edge Impulse wrapper failed to initialize (err: %d)\n",err);
		//return 0;
	}


	if (!device_is_ready(lsm6dsl_dev)) {
		LOG_ERR("sensor: device not ready.\n");
	}
	
	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		LOG_ERR("Cannot set sampling frequency for accelerometer.\n");
	}

	#ifdef CONFIG_LSM6DSL_TRIGGER
		struct sensor_trigger trig;

		trig.type = SENSOR_TRIG_DATA_READY;
		trig.chan = SENSOR_CHAN_ACCEL_XYZ;

		if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
			printk("Could not set sensor type and channel\n");
		}
	#endif

	err = ei_wrapper_start_prediction(0, 0);
	if (err) {
		LOG_ERR("Cannot start prediction (err: %d)\n", err);
	} else {
		LOG_INF("Prediction started...\n");
	}	
	while (1) {
        k_sleep(K_FOREVER);   /* uspi vlákno – jen aby se neukončilo */
    }
}

#ifdef DEBUG_RESULT_CB
static void result_ready_cb(int err) {
    if (err) {
        printk("Result error: %d\n", err);
        return;
    }

    /* 1. Vynuluj řádek, který právě přepisujeme
       a odečti jeho příspěvek ze součtového pole */
    for (int c = 0; c < NUM_CLASSES; c++) {
        labels_value[c] -= label_history[hystor_index][c];
        label_history[hystor_index][c] = 0.0f;
    }

    /* 2. Získej všechny výsledky pro aktuální okno */
    const char *dummy_label;          
    float      prob;
    size_t     lbl_idx;               

	static size_t last_lbl_idx = SIZE_MAX;  
    static char last_label[32] = {0};       

	bool same_label = true;
	//printk("\nClassification results\n");
	//printk("======================\n");
    while (!ei_wrapper_get_next_classification_result(&dummy_label, &prob, &lbl_idx)) {
        if (lbl_idx < NUM_CLASSES) {
            label_history[hystor_index][lbl_idx] = prob;
            labels_value[lbl_idx] += prob;
        }

        if (lbl_idx != last_lbl_idx || strcmp(dummy_label, last_label) != 0) {
            same_label = false;
            last_lbl_idx = lbl_idx;
            strncpy(last_label, dummy_label, sizeof(last_label));
            last_label[sizeof(last_label) - 1] = '\0';
        }
    }
    
    /* 3. Posuň index v kruhovém bufferu */
    hystor_index = (hystor_index + 1) % HISTORY_LEN;

    /* 4. Najdi třídu s největším součtem pravděpodobností */
    int   max_i = 0;
    float max_v = labels_value[0];
    for (int c = 1; c < NUM_CLASSES; c++) {
        if (labels_value[c] > max_v) {
            max_v = labels_value[c];
            max_i = c;
        }
    }

    /* 5. Přepočítej na „počty hlasů“ */
    int votes = 0;
    for (int k = 0; k < HISTORY_LEN; k++) {
        if (label_history[k][max_i] > 0.55f) {
            votes++;
        }
    }

    // --- nový výpis do konzole ---
    /*printk("Label '%s' (index %d) has %d votes in sliding window\n",
           ei_wrapper_get_classifier_label(max_i), max_i, votes);
	*/
	if (max_i == 0) {
		++idlefollowup;
	} else {
		idlefollowup = 0;
	}
	
    /* 6. Vyhlášení gesta */
    if (votes >= MIN_DETECTION_COUNT && max_i != STRAIGHT_IDX && max_i != 3 && ((max_i != 0) || (idlefollowup == 230))) {
        struct gest_msg g = {
            .cmd        = PKT_GEST,
            .gesture_id = max_i,
            .pipeprefix = DEVICE_PREFIX
        };
		
		if (!audioEventStart) {
			printk("Gesture detected: %s\n", ei_wrapper_get_classifier_label(max_i));
        	k_msgq_put(&msg_queue, &g, K_NO_WAIT);
		}
		
        /* vyčisti buffer */
        memset(label_history, 0, sizeof(label_history));
        memset(labels_value,  0, sizeof(labels_value));
        hystor_index = 0;
		idlefollowup = 0;
    }

    /* 7. Spusť další sliding-window inferenci */
    ei_wrapper_start_prediction(0, 3);
}
#else
static void result_ready_cb(int err) {
    if (err) {
        printk("Result error: %d\n", err);
        return;
    }

    /* 1. Vynuluj řádek, který právě přepisujeme
       a odečti jeho příspěvek ze součtového pole */
    for (int c = 0; c < NUM_CLASSES; c++) {
        labels_value[c] -= label_history[hystor_index][c];
        label_history[hystor_index][c] = 0.0f;
    }

    /* 2. Získej všechny výsledky pro aktuální okno */
    const char *dummy_label;          /* řetězec ignorujeme */
    float      prob;
    size_t     lbl_idx;               /* ← tady dostaneme číselný index!   */

    while (!ei_wrapper_get_next_classification_result(&dummy_label, &prob, &lbl_idx)) {
        if (lbl_idx < NUM_CLASSES) {
            label_history[hystor_index][lbl_idx] = prob;
            labels_value[lbl_idx] += prob;      /* inkrementální součet */
        }
    }

    /* 3. Posuň index v kruhovém bufferu */
    hystor_index = (hystor_index + 1) % HISTORY_LEN;

    /* 4. Najdi třídu s největším součtem pravděpodobností */
    int   max_i = 0;
    float max_v = labels_value[0];
    for (int c = 1; c < NUM_CLASSES; c++) {
        if (labels_value[c] > max_v) {
            max_v = labels_value[c];
            max_i = c;
        }
    }

    /* 5. Přepočítej na „počty hlasů“ (≈ jestli průměr > 0.5) */
    int votes = 0;
    for (int k = 0; k < HISTORY_LEN; k++) {
        if (label_history[k][max_i] > 0.55f) {   /* práh pro „hlas“ */
            votes++;
        }
    }

	if (max_i == 0)
	{
		++idlefollowup;
		//printk("Increasing folllowup to %d\n", idlefollowup);
	} else {
		idlefollowup = 0;
	}
	
    /* 6. Vyhlášení gesta – ignorujeme 'straight' */
    if (votes >= MIN_DETECTION_COUNT && max_i != STRAIGHT_IDX && max_i != 3 && ((max_i != 0) || (idlefollowup == 230))) {
        struct gest_msg g = {
            .cmd        = PKT_GEST,
            .gesture_id = max_i,          /* posíláme číslo labelu */
            .pipeprefix = DEVICE_PREFIX
        };
		
		if (!audioEventStart) //maby there is a need of tracking state of audiostream
		{
			printk("Gesture detected: %s\n", ei_wrapper_get_classifier_label(max_i));
        	k_msgq_put(&msg_queue, &g, K_NO_WAIT);
		}
		
        /* vyčisti buffer, ať se gesto nehlásí znovu hned za chvíli */
        memset(label_history, 0, sizeof(label_history));
        memset(labels_value,  0, sizeof(labels_value));
        hystor_index = 0;
		idlefollowup = 0;
    }

    /* 7. Spusť další sliding-window inferenci */
    ei_wrapper_start_prediction(0, 9);
}
#endif

int i2s_initialize(const struct device *dev_i2s) {

	int ret;

	struct i2s_config i2s_cfg;

	if (!device_is_ready(dev_i2s)) {
		LOG_ERR("I2S device not ready\n");
		ret = -ENODEV;
		return ret;
	} else {
		LOG_INF("I2S device is ready\n");
	}

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 1U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 11025;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
    i2s_cfg.mem_slab = &tx_0_mem_slab;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);

	if (ret < 0) {
		LOG_ERR("Failed to configure I2S stream");
		return ret;
	}

	return 0;
}

static void gesture_tx_thread(void *arg1, void *arg2, void *arg3) {

	int ret;
	LOG_INF("Gesture thread comunication started");
    struct gest_msg g;
    struct esb_payload tx = {
        .noack  = false,
        .length = sizeof(struct gest_msg)
    };

    while (1) {
        /* 1. počkej na gesto z fronty */
        ret = k_msgq_get(&msg_queue, &g, K_FOREVER);
		if (ret == 0) {
			LOG_INF("Sending Gesture packet with cmd %02X Gesture ID %d Device prefix %02X", g.cmd , g.gesture_id, g.pipeprefix);
		}

        memcpy(tx.data, &g, sizeof(g));

		//for (size_t i = 0; i < 4; i++){LOG_INF("data of payload 0x%02X", tx.data[i]);}
		esb_tx_done = false;
		//esb_flush_tx();
		ret = esb_write_payload(&tx);
		LOG_INF("esb_write_payload returned %d", ret);

        if (ret) {
            LOG_ERR("esb_write_payload failed");
        } else {
            // wait till packet is succesfully send to TX
			while (!esb_tx_done) {
                k_sleep(K_USEC(500));
				/*esb_flush_tx();
				ret = esb_write_payload(&tx);
				if (ret) {
					LOG_ERR("esb_write_payload failed");
				}*/
            }
        }
    }
}

void playaudio(const struct device *dev_i2s) {
	//Not neccesery i gess
	void *tx_block;
	int ret;
	audioStreamPlay = true;
	LOG_INF("Playaudio function begin");
	//int64_t start = k_uptime_get(); // aktuální čas v ms

	// i2s warm up
	for (int j = 0; j < 1; j++) {
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("Failed to alloc mem_slab block");
			continue;
		}

   		memset(tx_block, 0, BLOCK_SIZE);

	
		ret = i2s_write(dev_i2s, tx_block, BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("Could not write TX buffer: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &tx_block);
		}
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Could not start I2S TX error: %d",ret);
		return;
	}

	//while (audiostream ) {
	while (k_msgq_num_used_get(&audio_queue) > 0) {

		//int64_t now = k_uptime_get(); // aktuální čas v ms
		// Alokuj nový blok
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block, K_NO_WAIT);
		if (ret != 0) {
			LOG_ERR("TX block allocation failed %d", ret);
			continue;
		}
	
		// Naplň blok
		for (int i = 0; i < BLOCK_SIZE / Q_BLOCK_SIZE; i++) {
			uint8_t *dst = ((uint8_t *)tx_block) + (i * Q_BLOCK_SIZE);
			ret = k_msgq_get(&audio_queue, dst, K_NO_WAIT);
			if (ret != 0) {
				//LOG_WRN("Queue underrun, padding with zeros");
				memset(dst, 0, Q_BLOCK_SIZE);
			}
		}

		// Odeslat blok
		ret = i2s_write(dev_i2s, (uint8_t *)tx_block, BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("I2S write error: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &tx_block);
		}
		//check_bitrate_monitoring(BLOCK_SIZE * 8);

		/*if (now - start > AUDIOSTREAM_MAX_LENGHT) {
			audiostream = false;
			LOG_INF("Audio stream terminated due to unreasonable stream time");
			break;
		}*/
		
	}
	
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);

	// free all unused slab blocks
	//k_mem_slab_free(&tx_0_mem_slab, NULL);

	uint8_t dummy[Q_BLOCK_SIZE];
	while (k_msgq_get(&audio_queue, dummy, K_NO_WAIT) == 0) {
		// jen vybíráme, dokud něco je
	}
	audioStreamPlay = false;
	audioEventStart = false;
}

void print_audio_data(uint8_t *data, size_t size) {
    LOG_INF("Audio data (in hex):\n");
    for (size_t i = 0; i < size; i++) {
        LOG_INF("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            LOG_INF("\n");  // Vytvoří nový řádek každých 16 bajtů
        }
    }
    LOG_INF("\n");
}

void check_bitrate_monitoring(size_t new_packet_length_bits)
{
	int64_t now = k_uptime_get(); // aktuální čas v ms

	total_bits_received += new_packet_length_bits;

	if ((now - last_bitrate_check_time) >= 1000) { // každou sekundu
		uint32_t bitrate_kbps = total_bits_received / 1000;
		LOG_INF("Bitrate: %u kbps", bitrate_kbps);

		total_bits_received = 0;
		last_bitrate_check_time = now;
	}
}

void effect_alternate(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while (audioStreamPlay && (k_uptime_get() - start < 3000)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 0 ON
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);                    // Motor 1 OFF
		k_sleep(K_MSEC(300));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);                    // Motor 0 OFF
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 1 ON
		k_sleep(K_MSEC(300));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_pulse_both(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while (audioStreamPlay && (k_uptime_get() - start < 3000)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		k_sleep(K_MSEC(150));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
		k_sleep(K_MSEC(100));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	uint32_t duty;
	while (audioStreamPlay && (k_uptime_get() - start < 3000)) {
		// Fade in
		for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
		// Fade out
		for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade_left(const struct device *pwm_dev)
{
    int64_t start = k_uptime_get();
    uint32_t duty;

    while (audioStreamPlay && (k_uptime_get() - start < 3000)) {

        /* Fade-in */
        for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
            pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }

        /* Fade-out */
        for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
            pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }
    }

    /* stop – obě strany ticho */
    pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade_right(const struct device *pwm_dev)
{
    int64_t start = k_uptime_get();
    uint32_t duty;

    while (audioStreamPlay && (k_uptime_get() - start < 3000)) {

        /* Fade-in */
        for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
            pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }

        /* Fade-out */
        for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
            pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }
    }

    /* stop – obě strany ticho */
    pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
}

static void power_manager_thread(void *arg1, void *arg2, void *arg3) {
    while (1) {
        int64_t now = k_uptime_get();

        if (now - last_ping_time > PING_TIMEOUT_MS) {
            LOG_INF("No PING received in 5 minutes. Rebooting...");
            sys_reboot(SYS_REBOOT_COLD);  // restartuj zařízení
        }

        k_sleep(K_SECONDS(10)); // kontroluj každých 10 sekund
    }
}

static void battery_measure_thread(void *arg1, void *arg2, void *arg3) {
	int rc = battery_measure_enable(true);

	if (rc != 0) {
		printk("Failed initialize battery measurement: %d\n", rc);
		return 0;
	}

	while (true) {
		int batt_mV = battery_sample();

		if (batt_mV < 0) {
			printk("Failed to read battery voltage: %d\n",
			       batt_mV);
			break;
		}

		unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

		printk("%d mV; %u pptt\n",batt_mV, batt_pptt);

		/* Burn battery so you can see that this works over time */
		k_sleep(K_MSEC(BAT_MEASURE_PERIOD));
	}
	printk("Disable: %d\n", battery_measure_enable(false));
	return 0;
}