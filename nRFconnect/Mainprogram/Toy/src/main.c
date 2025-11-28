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

/*Battery modules*/
#include <math.h>
#include <stdlib.h>
#include "battery.h"

static const struct battery_level_point levels[] = {
	{ 10000, 3950 },
	{ 625, 3550 },
	{ 0, 3100 },
};


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

const int8_t DEVICE_PREFIX = 0xA2;

/*moduls for SD card support*/
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

/*Start of moduls for SD card initialization*/
#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/"DISK_DRIVE_NAME":"
#define MAX_PATH 128

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

const char *audio_lib[] = {
    "SD:/Msheep.wav",
    "SD:/Dog.wav",
	"SD:/Cat.wav",
	"SD:/Horse.wav",
    "SD:/Elephant.wav",
	"SD:/Monkey.wav",
	"SD:/Owl.wav"
};

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

int i2s_initialize(const struct device *dev_i2s); //init of i2s for playing audio

int SD_initialize(void);

struct gest_msg {
	uint8_t cmd;
	uint8_t gesture_id;
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
#define PRIORITY_PWM 11
#define LMRPWM_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(lmrPwm_stack_area, LMRPWM_THREAD_STACK_SIZE);
static struct k_thread lmrPwm_thread_data;

static struct sensor_value accel_xyz_out[3];

K_MSGQ_DEFINE(imu_msgq, sizeof(int32_t) * 3 * 2, 150, 4);

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig);

#define IMU_THREAD_STACK_SIZE 1024
#define IMU_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(imu_stack_area, IMU_THREAD_STACK_SIZE);
static struct k_thread imu_thread_data;

static void imu_trigger(void *arg1, void *arg2, void *arg3);

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

int main(void) {
	printk("MAIN START\n");

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

	err = SD_initialize();
	if (err) {
		LOG_ERR("SD initialization failed, err %d", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return 0;
	}
	k_thread_create(&imu_thread_data, imu_stack_area,
		K_THREAD_STACK_SIZEOF(imu_stack_area),
		imu_trigger,
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
		PRIORITY_PWM, 0, K_NO_WAIT);

	k_thread_create(&battery_stack_data, battery_stack_area,
		K_THREAD_STACK_SIZEOF(battery_stack_area),
		battery_manager_thread,
		NULL, NULL, NULL,
		BATTERY_THREAD_PRIORITY, 0, K_NO_WAIT);
	
	return 0;
}

static void imu_trigger(void *arg1, void *arg2, void *arg3) {

	int err;
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	err = ei_wrapper_init(result_ready_cb);

	if (err) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n",
		       err);
		return 0;
	};

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
	} else {
		printk("Prediction started...\n");
	}

		if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
	}
	
	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
	}
	
	float f_val_buffer[3];
	struct sensor_value acc_buffer[3];
	while (1) {
		k_msgq_get(&imu_msgq, acc_buffer, K_FOREVER);
		if (k_msgq_num_used_get(&imu_msgq) > (150 * 3 / 4)) {
								printk("Audio queue > 75%% full!\n");
		}
		f_val_buffer[0]= sensor_value_to_float(&acc_buffer[0]);
		f_val_buffer[1]= sensor_value_to_float(&acc_buffer[1]);
		f_val_buffer[2]= sensor_value_to_float(&acc_buffer[2]);
		//printk("This are the walues form sensor Ax:%f Ay:%f Az:%f \n",f_val_buffer[0], f_val_buffer[1], f_val_buffer[2]);
		err = ei_wrapper_add_data(f_val_buffer,3);
		if (err) {
			printk("Cannot provide input data (err: %d)\n", err);
			printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
		}
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
	while (0)
	{
		k_sleep(K_MSEC(50));
		if (1) {
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

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig) {

	static struct sensor_value accel_x, accel_y, accel_z;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
	accel_xyz_out[0]=accel_x;
	accel_xyz_out[1]=accel_y;
	accel_xyz_out[2]=accel_z;
	int err = k_msgq_put(&imu_msgq, accel_xyz_out, K_NO_WAIT);
	if(err){
		printk("Couldn t send packet err:%d\n", err);
	}
}

void audio_playback_thread(void *arg1, void *arg2, void *arg3) {
	int ret;
	struct gest_msg g;

	LOG_INF("Audio thread starting...");

	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2stx));

	ret = i2s_initialize(dev_i2s);
	if (ret){
		LOG_ERR("Init failed");
	}else {
		LOG_INF("Initilize of i2s was succesful");
	}
	while (1) {
		if (k_msgq_get(&msg_queue, &g, K_FOREVER) == 0) {
			playaudio(dev_i2s, audio_lib[1]);
		}
		k_sleep(K_MSEC(50));
	}		
}

int playaudio(const struct device *dev_i2s, const char *path) {
	//Not neccesery i gess
	void *audio_block;
	int ret;

	static uint8_t audio_buf[AUDIO_SD_CHUNK];

    UINT bytes_read;
    FRESULT fr;

	FIL wav;
    fr = f_open(&wav, path, FA_READ);
    if (fr) {
        LOG_ERR("open %s failed (%d)", path, fr);
        return -EIO;
    } else {
        LOG_INF("Opened file and skiping header");
    	f_lseek(&wav, 44); // skip WAV header
	}

	LOG_INF("Playaudio function begin");

	// i2s warm up
	for (int j = 0; j < 1; j++) {
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("Failed to alloc mem_slab block");
			continue;
		}

   		memset(audio_block, 0, BLOCK_SIZE);

	
		ret = i2s_write(dev_i2s, audio_block, BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("Could not write TX buffer: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &audio_block);
		}
	}

	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("Could not start I2S TX error: %d",ret);
		return ret;
	}

	while ((fr = f_read(&wav, audio_buf, AUDIO_SD_CHUNK, &bytes_read)) == FR_OK && bytes_read > 0) {

		// Alokuj nový blok
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("TX block allocation failed %d", ret);
			continue;
		}
		memcpy(audio_block, audio_buf, bytes_read);

		// Odeslat blok
		ret = i2s_write(dev_i2s, (uint8_t *)audio_block, BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("I2S write error: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &audio_block);
		}		
	}
	
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);

	fr = f_close(&wav);
	if (fr != FR_OK) {
		LOG_ERR("f_close failed: %d", fr);
		return -EIO;
	}

	audioStreamPlay = false;
	audioEventStart = false;
	return 0;
}

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
	i2s_cfg.frame_clk_freq = 44100;
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

void event_handler(struct esb_evt const *event) {
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX SUCCESS EVENT");
		break;
	}
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

int SD_initialize(void) {
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
    if (ret != 0) {
        LOG_ERR("Storage init ERROR!");
        return ret;  // návrat při chybě
    }

    // Získání počtu sektorů
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
    if (ret != 0) {
        LOG_ERR("Unable to get sector count");
        return ret;
    }
    LOG_INF("Block count %u", block_count);

    // Získání velikosti sektoru
    ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
    if (ret != 0) {
        LOG_ERR("Unable to get sector size");
        return ret;
    }
    LOG_INF("Sector size %u", block_size);

    uint64_t memory_size_mb = (uint64_t)block_count * block_size;
    LOG_INF("Memory Size(MB) %u", (uint32_t)(memory_size_mb >> 20));

    // Mount filesystem
    mp.mnt_point = disk_mount_pt;

    ret = fs_mount(&mp);
    if (ret != FR_OK) {
        LOG_INF("Error mounting disk.");
        return ret;
    }

    LOG_INF("Disk mounted.");

    // Projít obsah adresáře
    if (lsdir(disk_mount_pt) == 0) {
        LOG_INF("SD is empty.");
    } else {
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
static int lsdir(const char *path) {

	int res;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	int count = 0;

	fs_dir_t_init(&dirp);

	/* Verify fs_opendir() */
	res = fs_opendir(&dirp, path);
	if (res) {
		LOG_ERR("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	printk("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}
		
		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n",
				entry.name, entry.size);
		}
		
		count++;
	}

	/* Verify fs_closedir() */
	fs_closedir(&dirp);
	if (res == 0) {
		res = count;
	}
	
	return res;
}

#ifdef DEBUG_RESULT_CB
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
    //       ei_wrapper_get_classifier_label(max_i), max_i, votes);

    if (max_i == 0) {
        ++idlefollowup;
    } else {
        idlefollowup = 0;
    }

    /* 7. Detekce gesta */
    if (votes >= MIN_DETECTION_COUNT &&
        max_i != STRAIGHT_IDX && max_i != 3 &&
        ((max_i != 0) || (idlefollowup == 20))) {

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

static void battery_manager_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("Battery mesure thread started");

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

		printk("The battery voltage is:%d mV and precatntage around %u %%\n", batt_mV, batt_pptt);

		/* Burn battery so you can see that this works over time */
		k_sleep(K_SECONDS(60));
	}
	printk("Disable: %d\n", battery_measure_enable(false));
	return 0;
}