/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Toy, CONFIG_LOG_DEFAULT_LEVEL);
static struct esb_payload rx_payload;
bool device_running = true;

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

/*Power/Battery modules*/
#include <math.h>
#include <stdlib.h>
#include "battery.h"
#include <zephyr/drivers/adc.h>

K_SEM_DEFINE(run_sem, 1, 1); // zařízení začíná "zapnuté"

static const struct battery_level_point levels[] = {
	{10000, 3950},
	{625, 3550},
	{0, 3100},
};

bool charging = false;
bool charged = false;

/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)

#define SYSSOUND_THREAD_PRIORITY 11
#define SYSSOUND_THREAD_STACK_SIZE 4092

K_THREAD_STACK_DEFINE(syssound_stack_area, SYSSOUND_THREAD_STACK_SIZE);
static struct k_thread syssound_thread_data;

#define SYSFBQ_LENGTH 16

struct sys_ev {
	uint8_t identifier;
	uint8_t device_on;
	uint8_t device_on_chg;
	uint8_t device_charged;
	uint16_t V_bat;
	uint16_t V_ched;
	uint16_t V_ching;
};

K_MSGQ_DEFINE(sysfb_Q, sizeof(struct sys_ev), SYSFBQ_LENGTH, 4);

static void system_feedback_thread(void *arg1, void *arg2, void *arg3);

/*Esb modules*/
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>

#define ID0 NRF_FICR->DEVICEID[0]
#define PREFIX (uint8_t)(ID0 & 0xFF)

#define ESBINF 0
/*moduls for SD card support*/
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

/*Start of moduls for SD card initialization*/
#include <zephyr/random/random.h>
#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/" DISK_DRIVE_NAME ":"
#define MAX_PATH 128

const char *draw_audio_path(int gesture_id);

static const char *disk_mount_pt = DISK_MOUNT_PT;

static FATFS fat_fs;

/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

#define AUDIO_CHUNK_SIZE 1024
#define AUDIO_SD_CHUNK AUDIO_CHUNK_SIZE
uint8_t audio_buffer[AUDIO_SD_CHUNK];

const char *aduio_horse[] = {
	"SD:/horse/h1.wav",
	"SD:/horse/h2.wav",
	"SD:/horse/h3.wav",
	"SD:/horse/h4.wav",
	"SD:/horse/h5.wav",
	"SD:/horse/h6.wav",
};

const char *audio_cat[] = {
	"SD:/cat/c1.wav",
	"SD:/cat/c2.wav",
	"SD:/cat/c3.wav",
	"SD:/cat/c4.wav",
};


const char *audio_dog[] = {
	"SD:/dog/d1.wav",
	"SD:/dog/d2.wav",
	"SD:/dog/d3.wav",
	"SD:/dog/d4.wav",
	"SD:/dog/d5.wav",
};

const char *audio_sheep[] = {
	"SD:/sheep/s1.wav",
	"SD:/sheep/s2.wav",
	"SD:/sheep/s3.wav",
	"SD:/sheep/s4.wav",
};

const char *audio_chickens[] = {
	"SD:/chicken/c1.wav",
	"SD:/chicken/c2.wav",
	"SD:/chicken/c3.wav",
	"SD:/chicken/c4.wav",
};

const char *audio_birds[] = {
	"SD:/bird/b1.wav",
	"SD:/bird/b2.wav",
	"SD:/bird/b3.wav",
	"SD:/bird/b4.wav",
};

const char *audio_pig[] = {
	"SD:/pig/p1.wav",
	"SD:/pig/p2.wav",
	"SD:/pig/p3.wav",
	"SD:/pig/p4.wav",
};

const char *audio_cow[] = {
	"SD:/cow/c1.wav",
	"SD:/cow/c2.wav",
	"SD:/cow/c3.wav",
};

#define AUDIO_LIB_HORESE_SIZE ARRAY_SIZE(aduio_horse)
#define AUDIO_LIB_CAT_SIZE ARRAY_SIZE(audio_cat)
#define AUDIO_LIB_DOG_SIZE ARRAY_SIZE(audio_dog)
#define AUDIO_LIB_SHEEP_SIZE ARRAY_SIZE(audio_sheep)
#define AUDIO_LIB_CHICKENS_SIZE ARRAY_SIZE(audio_chickens)
#define AUDIO_LIB_BIRDS_SIZE ARRAY_SIZE(audio_birds)
#define AUDIO_LIB_PIG_SIZE ARRAY_SIZE(audio_pig)
#define AUDIO_LIB_COW_SIZE ARRAY_SIZE(audio_cow)

const char **audio_lib[] = {
	aduio_horse,
	audio_cat,
	audio_dog,
	audio_sheep,
	audio_chickens,
	audio_birds,
	audio_pig,
	audio_cow,
};

struct audio_grupe {
	const uint8_t *dir_indexis;
    size_t dir_count;	
	const uint8_t *dir_sizes;
};

struct audio_grupe audio_groups[] = {
	{.dir_indexis = (const uint8_t[]){0, 2, 5},
	.dir_count = 3,
	 .dir_sizes = (const uint8_t[]){AUDIO_LIB_HORESE_SIZE, AUDIO_LIB_DOG_SIZE, AUDIO_LIB_BIRDS_SIZE}},
	{.dir_indexis = (const uint8_t[]){0, 1, 2, 3, 4, 5, 6, 7},
	 .dir_count = 8,
	 .dir_sizes = (const uint8_t[]){AUDIO_LIB_HORESE_SIZE, AUDIO_LIB_CAT_SIZE, AUDIO_LIB_DOG_SIZE,
								AUDIO_LIB_SHEEP_SIZE, AUDIO_LIB_CHICKENS_SIZE, AUDIO_LIB_BIRDS_SIZE,
								AUDIO_LIB_PIG_SIZE, AUDIO_LIB_COW_SIZE}},
	{.dir_indexis = (const uint8_t[]){1, 3, 4},
	 .dir_count = 3,
	 .dir_sizes = (const uint8_t[]){AUDIO_LIB_CAT_SIZE, AUDIO_LIB_SHEEP_SIZE, AUDIO_LIB_CHICKENS_SIZE}},
	{.dir_indexis = (const uint8_t[]){6, 7},
	 .dir_count = 2,
	 .dir_sizes = (const uint8_t[]){AUDIO_LIB_PIG_SIZE, AUDIO_LIB_COW_SIZE}},	
};

#ifdef CONFIG_NOCACHE_MEMORY
#define MEM_SLAB_CACHE_ATTR __nocache
#else
#define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */

#define BLOCK_SIZE AUDIO_CHUNK_SIZE
#define SLAB_COUNT 4

static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
	_k_mem_slab_buf_tx_0_mem_slab[(SLAB_COUNT)*WB_UP(BLOCK_SIZE)];

static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
	Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
						   WB_UP(BLOCK_SIZE), SLAB_COUNT);

float volume = 0.5f;

int i2s_initialize(const struct device *dev_i2s); // init of i2s for playing audio

int SD_initialize(void);

struct gest_msg {
	uint8_t cmd;
	uint8_t cmd_id;
	uint8_t pipeprefix;
	uint8_t rsv;
};

bool audioEventStart = false;
bool audioStreamDone = false;
bool audioStreamPlay = false;

/*Packet ID */
#define PKT_GEST 0xB1
#define PKT_PING 0xB2
#define PKT_STAUDIO 0xAB
#define PKT_AUDIO 0xAD
#define PKT_ENDAUDIO 0xAE
#define PKT_TURNOFF 0x0F
#define PKT_SYSINF 0x1F
#define PKT_SYSCHNG 0x8C

#define MSGQ_LENGTH 16

K_MSGQ_DEFINE(msg_queue, sizeof(struct gest_msg), MSGQ_LENGTH, 4);

#define AUDIO_THREAD_STACK_SIZE 10240
#define AUDIO_THREAD_PRIORITY 3
K_THREAD_STACK_DEFINE(audio_stack_area, AUDIO_THREAD_STACK_SIZE);
static struct k_thread audio_thread_data;

/*Modules used for I2S and PWM*/
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include "pwmeffects.h"

#define PWM0_NODE DT_NODELABEL(pwm0)
#define PWM_THREAD_PRIORITY 11
#define LMRPWM_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(lmrPwm_stack_area, LMRPWM_THREAD_STACK_SIZE);
static struct k_thread lmrPwm_thread_data;

static struct sensor_value accel_xyz_out[3];

struct lmr_ev {
	uint8_t effect;	//number to identify effect that should be used
	uint8_t ms_duration; //not implemented yet
};

K_MSGQ_DEFINE(lmr_msq, sizeof(struct lmr_ev), 32, 4);

K_MSGQ_DEFINE(imu_msgq, sizeof(int32_t) * 3 * 2, 150, 4);

void lsm6dsl_sleep();
void lsm6dsl_ei_wake();
static void lsm6dsl_trigger_handler(const struct device *dev,
									const struct sensor_trigger *trig);

#define IMU_THREAD_STACK_SIZE 1024
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static void imu_ini_data_forw(void *arg1, void *arg2, void *arg3);

#include <ei_wrapper.h>

#define HISTORY_LEN 14
#define NUM_CLASSES 5
#define MIN_DETECTION_COUNT 6
#define STRAIGHT_IDX 5
#define DEBUG_RESULT_CB // coment this to use the old function of result_ready

uint8_t hystor_index;
float label_history[HISTORY_LEN][NUM_CLASSES];
float labels_value[NUM_CLASSES];
uint8_t idlefollowup = 0;

static void result_ready_cb(int err);

void event_handler(struct esb_evt const *event);

int esb_initialize();

int clocks_start();

void audio_playback_thread(void *arg1, void *arg2, void *arg3);

int playaudio(const struct device *dev_i2s, const char *path);

void LMR_control_thread(void *arg1, void *arg2, void *arg3);

static int lsdir(const char *path);

#define BATTERY_THREAD_STACK_SIZE 1024
#define BATTERY_THREAD_PRIORITY 10

K_THREAD_STACK_DEFINE(battery_stack_area, BATTERY_THREAD_STACK_SIZE);
static struct k_thread battery_stack_data;
static void battery_manager_thread(void *arg1, void *arg2, void *arg3);

int main(void)
{
	int err;
	err = clocks_start();
	if (err)
	{
		return 0;
	}

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("LEDs initialization failed, err %d", err);
		return 0;
	}

	err = SD_initialize();
	if (err)
	{
		LOG_ERR("SD initialization failed, err %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err)
	{
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}

	err = esb_start_rx();
	if (err)
	{
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}
	LOG_INF("Initialization complete");
	LOG_INF("Setting up for packet receiption");

	k_thread_create(&imu_thread_data, imu_stack_area,
					K_THREAD_STACK_SIZEOF(imu_stack_area),
					imu_ini_data_forw,
					NULL, NULL, NULL,
					IMU_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&audio_thread_data, audio_stack_area,
					K_THREAD_STACK_SIZEOF(audio_stack_area),
					audio_playback_thread,
					NULL, NULL, NULL,
					AUDIO_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&lmrPwm_thread_data, lmrPwm_stack_area,
					K_THREAD_STACK_SIZEOF(lmrPwm_stack_area),
					LMR_control_thread,
					NULL, NULL, NULL,
					PWM_THREAD_PRIORITY, 0, K_NO_WAIT);

	k_thread_create(&battery_stack_data, battery_stack_area,
					K_THREAD_STACK_SIZEOF(battery_stack_area),
					battery_manager_thread,
					NULL, NULL, NULL,
					BATTERY_THREAD_PRIORITY, 0, K_NO_WAIT);
	
	k_thread_create(&syssound_thread_data, syssound_stack_area,
					K_THREAD_STACK_SIZEOF(syssound_stack_area),
					system_feedback_thread,
					NULL, NULL, NULL,
					SYSSOUND_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

static void imu_ini_data_forw(void *arg1, void *arg2, void *arg3) {
	int err;
	err = ei_wrapper_init(result_ready_cb);

	if (err) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n",
			   err);
		return 0;
	};
	lsm6dsl_ei_wake();

	float f_val_buffer[3];
	struct sensor_value acc_buffer[3];
	while (1) {
		k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
		k_sem_give(&run_sem);
	
		k_msgq_get(&imu_msgq, acc_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (150 * 3 / 4))
		{
			printk("Audio queue > 75%% full!\n");
		}
		f_val_buffer[0] = sensor_value_to_float(&acc_buffer[0]);
		f_val_buffer[1] = sensor_value_to_float(&acc_buffer[1]);
		f_val_buffer[2] = sensor_value_to_float(&acc_buffer[2]);
		// printk("This are the walues form sensor Ax:%f Ay:%f Az:%f \n",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2]);
		if (!audioStreamPlay)
		{
			err = ei_wrapper_add_data(f_val_buffer, 3);
			if (err)
			{
				printk("Cannot provide input data (err: %d)\n", err);
				printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
			}
		}
		else
		{
		}
	}
}

void LMR_control_thread(void *arg1, void *arg2, void *arg3) {
	const struct device *pwm0_dev = DEVICE_DT_GET(PWM0_NODE);
	
	struct lmr_ev lmr_event = {
		.effect = 0,
		.ms_duration = 0
	};

	if (!device_is_ready(pwm0_dev))
	{
		LOG_ERR("PWM0 device not ready\n");
	}
	else
	{
		LOG_INF("LMR si ready\n");
	}
	while (1) {
		k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
    	k_sem_give(&run_sem);

		if (k_msgq_get(&lmr_msq, &lmr_event, K_FOREVER) == 0) {
			LOG_INF("LMR effect recieved: %d and time is %d", lmr_event.effect, lmr_event.ms_duration);
			switch (lmr_event.effect+1) {
				case 1:
					effect_alternate(pwm0_dev);
					break;
				case 2:
					effect_pulse_both(pwm0_dev);
					break;
				case 3:
					effect_fade(pwm0_dev);
					break;
				case 4:
					effect_fade_right(pwm0_dev);
					break;
				case 5:
					effect_fade_left(pwm0_dev);
					break;
				case 101:
					effect_docked(pwm0_dev);
					break;
				default:
					break;
			}
		}
	}
}

static void lsm6dsl_trigger_handler(const struct device *dev,
									const struct sensor_trigger *trig)
{

	static struct sensor_value accel_x, accel_y, accel_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
	accel_xyz_out[0] = accel_x;
	accel_xyz_out[1] = accel_y;
	accel_xyz_out[2] = accel_z;
	int err = k_msgq_put(&imu_msgq, accel_xyz_out, K_NO_WAIT);
	if (err)
	{
		printk("IMU queue full dropping packet:%d\n", err);
	}
}

void audio_playback_thread(void *arg1, void *arg2, void *arg3)
{
	int ret;
	struct gest_msg g;
	g.cmd_id = PKT_GEST;
	struct esb_payload tx = {
		.noack = false,
		.length = sizeof(struct gest_msg)};
	struct lmr_ev lmr_event = {0};

	LOG_INF("Audio thread starting...");

	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2stx));

	ret = i2s_initialize(dev_i2s);
	if (ret)
	{
		LOG_ERR("Init failed");
	}
	else
	{
		LOG_INF("Initilize of i2s was succesful");
	}

	while (1) {
		k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
    	k_sem_give(&run_sem);

		if (k_msgq_get(&msg_queue, &g, K_FOREVER) == 0)
		{
			LOG_INF("Sending Gesture packet with cmd %02X Gesture ID %d with prob %f Device prefix %u", g.cmd, g.cmd_id, g.rsv, g.pipeprefix);
			memcpy(tx.data, &g, sizeof(g));
			ret = esb_write_payload(&tx);
			LOG_INF("esb_write_payload returned %d", ret);
			lmr_event.effect = g.cmd_id;
			k_msgq_put(&lmr_msq, &lmr_event, K_NO_WAIT);
			playaudio(dev_i2s, draw_audio_path(g.cmd_id));
		}
		k_sleep(K_MSEC(50));
	}
}

int playaudio(const struct device *dev_i2s, const char *path)
{
	audioStreamPlay = true;
	void *audio_block;
	int ret;

	static uint8_t audio_buf[AUDIO_SD_CHUNK];

	UINT bytes_read;
	FRESULT fr;

	FIL wav;
	fr = f_open(&wav, path, FA_READ);
	if (fr)
	{
		LOG_ERR("open %s failed (%d)", path, fr);
		return -EIO;
	}
	else
	{
		LOG_INF("Opened file and skipping header");
		f_lseek(&wav, 44); // skip WAV header
	}

	LOG_INF("Playaudio function begin");

	// i2s warm up
	for (int j = 0; j < 1; j++)
	{
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0)
		{
			LOG_ERR("Failed to alloc mem_slab block");
			continue;
		}

		memset(audio_block, 0, BLOCK_SIZE);

		ret = i2s_write(dev_i2s, audio_block, BLOCK_SIZE);
		if (ret != 0)
		{
			LOG_ERR("Could not write TX buffer: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &audio_block);
		}
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0)
	{
		LOG_ERR("Could not start I2S TX error: %d", ret);
		return ret;
	}

	while ((fr = f_read(&wav, audio_buf, AUDIO_SD_CHUNK, &bytes_read)) == FR_OK && bytes_read > 0)
	{

		int16_t *samples = (int16_t *)audio_buf;
		size_t count = bytes_read / 2; // 16bit PCM

		for (size_t i = 0; i < count; i++)
		{
			float scaled = samples[i] * volume;

			// přetečení – clamp
			if (scaled > 32767.0f)
				scaled = 32767.0f;
			if (scaled < -32768.0f)
				scaled = -32768.0f;

			samples[i] = (int16_t)scaled;
		}

		// Alokace bloku
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0)
		{
			LOG_ERR("TX block allocation failed %d", ret);
			continue;
		}

		memcpy(audio_block, audio_buf, bytes_read);

		// Odeslat blok
		ret = i2s_write(dev_i2s, (uint8_t *)audio_block, BLOCK_SIZE);
		if (ret != 0)
		{
			LOG_ERR("I2S write error: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &audio_block);
		}
	}

	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);

	fr = f_close(&wav);
	if (fr != FR_OK)
	{
		LOG_ERR("f_close failed: %d", fr);
		return -EIO;
	}

	audioStreamPlay = false;
	audioEventStart = false;
	return 0;
}

int i2s_initialize(const struct device *dev_i2s)
{

	int ret;

	struct i2s_config i2s_cfg;

	if (!device_is_ready(dev_i2s))
	{
		LOG_ERR("I2S device not ready\n");
		ret = -ENODEV;
		return ret;
	}
	else
	{
		LOG_INF("I2S device is ready\n");
	}

	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 1U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 1000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);

	if (ret < 0)
	{
		LOG_ERR("Failed to configure I2S stream");
		return ret;
	}

	return 0;
}

int esb_initialize()
{
	int err;
	/* These are nonarbitrary default addresses.*/
	uint8_t base_addr_0[4] = {0xF1, 0xF1, 0xF1, 0xF1};
	uint8_t addr_prefix[1] = {PREFIX};
	LOG_INF("Tis is the prefix of this Xiao: %u\n", PREFIX);
	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.tx_mode = ESB_TXMODE_AUTO;
	config.tx_output_power = 8;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PRX;
	config.selective_auto_ack = true;

	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING))
	{
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err)
	{
		return err;
	}

	// Set channel to 80 ( that means 2480 MHz)
	err = esb_set_rf_channel(79);
	if (err)
	{
		LOG_ERR("Set RF channel failed");
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return err;
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_RX_RECEIVED:
		int8_t rssi_dbm = -((int8_t)NRF_RADIO->RSSISAMPLE);
		if (esb_read_rx_payload(&rx_payload) == 0)
		{
			uint8_t PC_CMD = rx_payload.data[0];
			switch (PC_CMD)
			{
			case PKT_GEST:
				break;
			case PKT_TURNOFF:
				if (device_running) {
					device_running = false;
					k_sem_take(&run_sem, K_NO_WAIT);     // ZASTAVÍ všechna vlákna
					lsm6dsl_sleep();
				} else {
					device_running = true;
					k_sem_give(&run_sem);                // PROBUDÍ vlákna
					lsm6dsl_ei_wake();
				}
				break;
			case PKT_AUDIO:
				break;
			case PKT_ENDAUDIO:
				break;
			case PKT_PING:
				// LOG_INF("Recieved PING packet with RSSI: %3d dBm", rssi_dbm);
				break;
			case PKT_SYSCHNG:
				if (rx_payload.data[1] && (volume <= 1.5f))
				{
					LOG_INF("Volume has increased");
					volume += 0.1f;
				}
				else if (!rx_payload.data[1] && (volume >= 0.0f))
				{
					LOG_INF("Volume has decreased");
					volume -= 0.1f;
				}
				break;
			default:
				break;
			}
		}
		else
		{
			LOG_ERR("Error while reading rx packet");
		}
		break;

	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;

	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

int SD_initialize(void)
{
	/*
	 * Init of SD card open and list
	 * raw disk i/o
	 */
	static const char *disk_pdrv = DISK_DRIVE_NAME;
	int ret = 0;

	uint32_t block_count = 0;
	uint32_t block_size = 0;

	// Inicializace disku
	ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_INIT, NULL);
	if (ret != 0)
	{
		LOG_ERR("Storage init ERROR!");
		return ret; // návrat při chybě
	}

	// Získání počtu sektorů
	ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
	if (ret != 0)
	{
		LOG_ERR("Unable to get sector count");
		return ret;
	}
	LOG_INF("Block count %u", block_count);

	// Získání velikosti sektoru
	ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
	if (ret != 0)
	{
		LOG_ERR("Unable to get sector size");
		return ret;
	}
	LOG_INF("Sector size %u", block_size);

	uint64_t memory_size_mb = (uint64_t)block_count * block_size;
	LOG_INF("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));

	// Mount filesystem
	mp.mnt_point = disk_mount_pt;

	ret = fs_mount(&mp);
	if (ret != FR_OK)
	{
		LOG_INF("Error mounting disk.");
		return ret;
	}

	LOG_INF("Disk mounted.");

	// Projít obsah adresáře
	if (lsdir(disk_mount_pt) == 0)
	{
		LOG_INF("SD is empty.");
	}
	else
	{
		LOG_INF("SD and file configuration was successful.");
	}

	/*End of init of SD card*/
	return 0;
}

/* List dir entry by path
 *
 * @param path Absolute path to list
 *
 * @return Negative errno code on error, number of listed entries on
 *         success.
 */
static int lsdir(const char *path)
{

	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res)
	{
		LOG_ERR("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;)
	{
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0)
		{
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR)
		{
			printk("[DIR ] %s\n", entry.name);
		}
		else
		{
			printk("[FILE] %s (size = %zu)\n",
				   entry.name, entry.size);
		}

		count++;
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0)
	{
		res = count;
	}

	return res;
}

#ifdef DEBUG_RESULT_CB
static void result_ready_cb(int err)
{
	if (err)
	{
		printk("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	float temp_value;
	float values[4];
	float anomaly;
	int idx;

	// printk("\nClassification results\n");
	// printk("======================\n");

	while (true) {    
		err = ei_wrapper_get_next_classification_result(&label, &temp_value, &idx);
		values[idx] = temp_value;

		if (err)
		{
			if (err == -ENOENT)
			{
				err = 0;
			}
			break;
		}

		printk("Value: %.2f\tLabel: %s\n", (double)temp_value, label);
	}

	if (err)
	{
		printk("Cannot get classification results (err: %d)", err);
	}
	else
	{
		if (ei_wrapper_classifier_has_anomaly())
		{
			err = ei_wrapper_get_anomaly(&anomaly);
			if (err)
			{
				printk("Cannot get anomaly (err: %d)\n", err);
			}
			else
			{
				printk("Anomaly: %.2f\n", (double)anomaly);
			}
		}
	}

	for (size_t i = 0; i < 4; i++)
	{

		if ((values[i] > 0.90) &&
			((i != 1) || (idlefollowup >= 20)))
		{
			printk("Values index %d val: %f\n", i, values[i]);
			sizeof(struct gest_msg);
			struct gest_msg g = {
				.cmd = PKT_GEST,
				.cmd_id = i,
				.pipeprefix = PREFIX,
				.rsv = (uint8_t)(values[i] * 100.0f)
			};

			if (!audioEventStart)
			{
				printk("Gesture detected: %s %d\n",
					   ei_wrapper_get_classifier_label(i), i);
				
				k_msgq_put(&msg_queue, &g, K_NO_WAIT);
			}

			idlefollowup = 0;
		}

		if (i == 1)
		{
			if (values[i] >= 0.90)
			{
				idlefollowup += 1;
				printk("idlefollowup increased to %d\n", idlefollowup);
			}
			else
			{
				idlefollowup = 0;
				printk("idlefollowup voided se to 0\n");
			}
		}
	}

	err = ei_wrapper_start_prediction(1, 0);
	if (err)
	{
		printk("Cannot restart prediction (err: %d)\n", err);
	}
	else
	{
		printk("Prediction restarted...\n");
	}
}
#else
static void result_ready_cb(int err)
{
	if (err)
	{
		printk("Result error: %d\n", err);
		return;
	}

	/* 1. Vynuluj řádek, který právě přepisujeme
	   a odečti jeho příspěvek ze součtového pole */
	for (int c = 0; c < NUM_CLASSES; c++)
	{
		labels_value[c] -= label_history[hystor_index][c];
		label_history[hystor_index][c] = 0.0f;
	}

	/* 2. Získej všechny výsledky pro aktuální okno */
	const char *dummy_label; /* řetězec ignorujeme */
	float prob;
	size_t lbl_idx; /* ← tady dostaneme číselný index!   */

	while (!ei_wrapper_get_next_classification_result(&dummy_label, &prob, &lbl_idx))
	{
		if (lbl_idx < NUM_CLASSES)
		{
			label_history[hystor_index][lbl_idx] = prob;
			labels_value[lbl_idx] += prob; /* inkrementální součet */
		}
	}

	/* 3. Posuň index v kruhovém bufferu */
	hystor_index = (hystor_index + 1) % HISTORY_LEN;

	/* 4. Najdi třídu s největším součtem pravděpodobností */
	int max_i = 0;
	float max_v = labels_value[0];
	for (int c = 1; c < NUM_CLASSES; c++)
	{
		if (labels_value[c] > max_v)
		{
			max_v = labels_value[c];
			max_i = c;
		}
	}

	/* 5. Přepočítej na „počty hlasů“ (≈ jestli průměr > 0.5) */
	int votes = 0;
	for (int k = 0; k < HISTORY_LEN; k++)
	{
		if (label_history[k][max_i] > 0.55f)
		{ /* práh pro „hlas“ */
			votes++;
		}
	}

	if (max_i == 0)
	{
		++idlefollowup;
		// printk("Increasing folllowup to %d\n", idlefollowup);
	}
	else
	{
		idlefollowup = 0;
	}

	/* 6. Vyhlášení gesta – ignorujeme 'straight' */
	if (votes >= MIN_DETECTION_COUNT && max_i != STRAIGHT_IDX && max_i != 3 && ((max_i != 0) || (idlefollowup == 230)))
	{
		struct gest_msg g = {
			.cmd = PKT_GEST,
			.gesture_id = max_i, /* posíláme číslo labelu */
			.pipeprefix = DEVICE_PREFIX};

		if (!audioEventStart) // maby there is a need of tracking state of audiostream
		{
			printk("Gesture detected: %s\n", ei_wrapper_get_classifier_label(max_i));
			k_msgq_put(&msg_queue, &g, K_NO_WAIT);
		}

		/* vyčisti buffer, ať se gesto nehlásí znovu hned za chvíli */
		memset(label_history, 0, sizeof(label_history));
		memset(labels_value, 0, sizeof(labels_value));
		hystor_index = 0;
		idlefollowup = 0;
	}

	/* 7. Spusť další sliding-window inferenci */
	ei_wrapper_start_prediction(0, 9);
}
#endif

static void battery_manager_thread(void *arg1, void *arg2, void *arg3)
{	
	int err;
	/*charger state reading setup*/
	uint16_t sample_buffer;

	struct adc_sequence seq = {
		.buffer      = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution  = 12,
		.oversampling = 8
	};

	int ret;
	for (size_t i = 0; i < CHANNEL_COUNT; i++)
	{
		if (channel_cfgs[i].channel_id != 7) {
			LOG_INF("Setting up ADC with channel %u!\n", channel_cfgs[i].channel_id);
			ret = adc_channel_setup(adc, &channel_cfgs[i]);
				if (ret < 0) {
					printk("ADC channel setup failed: %d\n", ret);
					return;
			}
		}
	}
	/*Battery reading*/
	LOG_INF("Battery mesure thread started");

	ret = battery_measure_enable(true);
	if (ret != 0) {
		LOG_INF("Failed initialize battery measurement: %d", ret);
		return 0;
	}

	struct sys_ev pow_inf = {
		.identifier = PKT_SYSINF
	};

	while (true) {
		k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
    	k_sem_give(&run_sem);

		/*charging state reading*/
		for (size_t i = 0; i < CHANNEL_COUNT - 1; i++) {
			seq.channels = BIT(channel_cfgs[i].channel_id);
			ret = adc_read(adc, &seq);
			if (ret == 0) {
				int32_t mv0 = sample_buffer;
				adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_3, 12, &mv0);
				if(i == 0){
					pow_inf.V_ched = mv0;
				} else {
					pow_inf.V_ching = mv0;
				} 
			} else {
				printk("ADC read failed: %d\n", ret);
			}
		}
		/*battery state reading*/
		uint16_t batt_mV = battery_sample();
		pow_inf.V_bat = batt_mV;
		if (batt_mV < 0) {
			LOG_INF("Failed to read battery voltage: %d", batt_mV);
			break;
		}

		unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);
		pow_inf.device_on = device_running;
		//LOG_INF("V_bat is:%d mV charge:%u %%", batt_mV, batt_pptt);
		int err = k_msgq_put(&sysfb_Q, &pow_inf, K_NO_WAIT);
		k_sleep(K_MSEC(10));
	}
	printk("Disable: %d\n", battery_measure_enable(false));
}

void lsm6dsl_sleep() {
	int err;
    const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

    // 1) Zastavit trigger
    sensor_trigger_set(lsm6dsl_dev, NULL, NULL);

    // 2) Vypnout ODR → IMU přestane posílat data
    struct sensor_value off_odr = {0, 0};
    sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &off_odr);

    // 3) Vyčistit queue
    k_msgq_purge(&imu_msgq);
	bool canceld;
	err = ei_wrapper_clear_data(&canceld);
    	printk("tohle je hodnota canceled:%d\n", canceld);
    	printk("tohle je hodnota err:%d\n", err);
	if(!err && canceld) {
    	printk("Wrapper data cleared and prediction canceled\n");
	}
    printk("IMU stopped & queue cleared, system ready for sleep\n");
}

void lsm6dsl_ei_wake() {
	int err;
	printk("Machine learning model sampling frequency: %zu\n",
		   ei_wrapper_get_classifier_frequency());
	printk("Labels assigned by the model:\n");
	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		printk("- %s\n", ei_wrapper_get_classifier_label(i));
	}
	printk("\n");

	err = ei_wrapper_start_prediction(0, 0);
	if (err) {
		printk("Cannot start prediction (err: %d)\n", err);
	}
	else {
		printk("Prediction started...\n");
	}
	/* set accel/gyro sampling frequency to 52 Hz */
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
	
	if (!device_is_ready(lsm6dsl_dev)) {
		printk("IMU not ready\n");
	}	

	struct sensor_value odr_attr;
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev,
						SENSOR_CHAN_ACCEL_XYZ,
						SENSOR_ATTR_SAMPLING_FREQUENCY,
						&odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
	}
}

static void system_feedback_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("System feedback thread started");

	int ret;

	struct esb_payload tx_inf = {
		.noack = false,
		.length = sizeof(struct sys_ev)
	};

	struct sys_ev pow_inf = {0};

	struct lmr_ev lmr_event = {
		.effect = 0,
		.ms_duration = 0
	};
	
    while (1) {
        if(k_msgq_get(&sysfb_Q, &pow_inf, K_FOREVER) == 0) {
			if ((pow_inf.V_ching == 0) && (pow_inf.V_ched == 0)) {
				//LOG_INF("Device not on the charger");
				charging = false;
				pow_inf.device_on_chg = 0;
				pow_inf.device_charged = 0;
				charged = false;
			}
			if ((pow_inf.V_ching > 500) && !charging) {
				//LOG_INF("Device was placed on charger");
				charging = true;
				pow_inf.device_on_chg = 1;
				lmr_event.effect = 101;
				k_msgq_put(&lmr_msq,&lmr_event,K_NO_WAIT);
			}
			if ((pow_inf.V_ched > 600) && (pow_inf.V_ching == 0)) {
				//LOG_INF("Device is charged");
				if (!charged) {
					lmr_event.effect = 101;
					k_msgq_put(&lmr_msq,&lmr_event,K_NO_WAIT);
					charged = true;
				}
			}

			if(ESBINF) {
				memcpy(&tx_inf.data, &pow_inf, sizeof(struct sys_ev));
				ret = esb_write_payload(&tx_inf);
			}
		}
    }
}

const char *draw_audio_path(int gesture_id){
	const uint8_t *g_dirs = audio_groups[gesture_id].dir_indexis;
	const uint8_t *dir_count = audio_groups[gesture_id].dir_sizes;
	int rand_dir = sys_rand8_get() % audio_groups[gesture_id].dir_count;
	int select_audio = sys_rand8_get() % dir_count[rand_dir];

	const char *path_for_gesture = audio_lib[rand_dir][select_audio];
	LOG_INF("Selected audio path: %s", path_for_gesture);
	return path_for_gesture;
}