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
/*modules for the FEM*/
#include <mpsl_fem_protocol_api.h>
/*moduls for SD card support*/
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
/*end of moduls for SD card support*/

/*Packet ID */
#define PKT_GEST 0xB1
#define PKT_PING 0xB2
#define PKT_STAUDIO 0xAB
#define PKT_AUDIO 0xAD
#define PKT_ENDAUDIO 0xAE

const char *audio_lib[] = {
    "SD:/Msheep.wav",
    "SD:/Dog.wav",
	"SD:/Cat.wav",
	"SD:/Horse.wav",
    "SD:/Elephant.wav",
	"SD:/Monkey.wav",
	"SD:/Owl.wav"
};

enum GestureID {
	G_IDLE = 0,
    G_CIRCLE1 = 1,
    G_DROP = 2,
    G_NOISE = 3,
    G_SQUARE = 4,
    G_STRAIGHT = 5,
};

#define MSGQ_LENGTH 64

struct gest_msg {
	uint8_t cmd;
	uint8_t gesture_id;
	uint8_t pipeprefix;
    uint8_t rsv;
};

static volatile bool esb_tx_done = true;   // bool used for controling TX logic of gest

uint8_t addr_prefix[2] = {0xA1, 0xA2};

K_MSGQ_DEFINE(msg_queue, sizeof(struct gest_msg), MSGQ_LENGTH, 4);

#define SEND_AUDIO_PRIORITY 1
#define SEND_AUDIO_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(sendaudio_stack_area, SEND_AUDIO_THREAD_STACK_SIZE);
static struct k_thread sendaudio_thread_data;

/**/
LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

/*Comands for gests taht can be recieved*/
#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)
	 
struct esb_payload rx;

void event_handler(struct esb_evt const *event) {

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		esb_tx_done = true;
		break;

	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		esb_tx_done = true;
		break;

	case ESB_EVENT_RX_RECEIVED:
		LOG_DBG("RX RECIEVED EVENT");
		if (esb_read_rx_payload(&rx) == 0) {

			uint8_t PC_ID = rx.data[0];
			switch (PC_ID) {
				case PKT_PING:
					LOG_INF("PING PACKET RECIEVED");
					break;

				case PKT_GEST:
						struct gest_msg g = {
								.cmd  = rx.data[0],
								.gesture_id = rx.data[1],
								.pipeprefix = rx.data[2],
								.rsv  = rx.data[3],
							};

						if (k_msgq_put(&msg_queue, &g, K_NO_WAIT) != 0) {
							LOG_ERR("MSG queue full! Dropping packet!");
						}else {
							LOG_INF("Recieved gesture comand 0x%02X with value %d pipefromprefix %02X", g.cmd , g.gesture_id, g.pipeprefix);
						}
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
#define AUDIO_CHUNK_SIZE (CONFIG_ESB_MAX_PAYLOAD_LENGTH-1) //working fine 192
#define AUDIO_SD_CHUNK AUDIO_CHUNK_SIZE*16 //working fine 3072
uint8_t audio_buffer[AUDIO_SD_CHUNK];

#define PING_INTERVAL_MS  100
#define PING_PAUSE_MS     20
#define PING_PKT_LEN      1

/*Ends of moduls used for sandig audio data*/

/*
 *  Note the fatfs library is able to mount only strings inside _VOLUME_STRS
 *  in ffconf.h
 */

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

static int lsdir(const char *path);

static int send_audio(const char *path);

/*End of moduls for SD card initialization*/

int esb_initialize(void);

int mpsl_initialization(void);

int SD_initialize(void);

void print_audio_data(uint8_t *data, size_t size);

void send_audio_thread(void *arg1, void *arg2, void *arg3);

int main(void)
{
	printk("This are the walues of main focus AUDIO_CHUNK_SIZE:%d and AUDIO_SD_CHUNK:%d\n",AUDIO_CHUNK_SIZE, AUDIO_SD_CHUNK);
	int err;
	k_sleep(K_MSEC(200));

	err = SD_initialize();
	if (err){
		LOG_ERR("SD initialization failed, err %d", err);
		return 0;
	}
	
	LOG_INF("Enhanced ShockBurst ptx START");

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
		send_audio_thread,
		NULL, NULL, NULL,
		SEND_AUDIO_PRIORITY, 0, K_NO_WAIT);

	LOG_INF("Initialization complete");

	mpsl_initialization();
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
	if (res) {
		LOG_ERR("Error opening dir %s [%d]\n", path, res);
		return res;
	}

	LOG_INF("\nListing dir %s ...\n", path);
	for (;;) {
		/* Verify fs_readdir() */
		res = fs_readdir(&dirp, &entry);

		/* entry.name[0] == 0 means end-of-dir */
		if (res || entry.name[0] == 0) {
			break;
		}
		if (entry.type == FS_DIR_ENTRY_DIR) {
			LOG_INF("[DIR ] %s\n", entry.name);
		} else {
			LOG_INF("[FILE] %s (size = %zu)\n",
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

	err = esb_set_tx_power(27);
	
	if (err) {
		LOG_ERR("Set TX Power failed");
		return err;
	}
	
	return 0;
}

void print_audio_data(uint8_t *data, size_t size) {
    LOG_DBG("Audio data (in hex):\n");
    for (size_t i = 0; i < size; i++) {
        printk("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printk("\n");  // Vytvoří nový řádek každých 16 bajtů
        }
    }
    printk("\n");
}

static int send_audio(const char *path) {

	LOG_INF("ENTERED send audio, will be sending %s", path);

    static uint8_t audio_buf[AUDIO_SD_CHUNK];

    struct esb_payload tx = { .length = 1, .noack = true, .pipe = 0};
    UINT bytes_read;
    FRESULT fr;
    int err, total = 0;
	esb_tx_done = false;

    FIL wav;
    fr = f_open(&wav, path, FA_READ);
    if (fr) {
        LOG_ERR("open %s failed (%d)", path, fr);
        return -EIO;
    } else {
        LOG_INF("Opened file and skiping header");
    	f_lseek(&wav, 44); // skip WAV header
	}

	for (int i = 0; i < 3; i++) {
		tx.data[0] = PKT_STAUDIO;
		esb_write_payload(&tx);
		k_sleep(K_MSEC(10));
	}

	tx.data[0] = PKT_AUDIO;

	while ((fr = f_read(&wav, audio_buf, AUDIO_SD_CHUNK, &bytes_read)) == FR_OK && bytes_read > 0) {
		
        for (size_t o = 0; o < bytes_read; o += AUDIO_CHUNK_SIZE) {
            size_t len = MIN(AUDIO_CHUNK_SIZE, bytes_read - o);
			memcpy(&tx.data[1], &audio_buf[o], len);
            tx.length = len+1;

            err = esb_write_payload(&tx);
            if (err) {
                LOG_ERR("ESB TX(send) err %d", err);
                //goto out_close;
            }
            //k_sleep(K_USEC(2000));   //1500alse good was 1540 Dealay taht adjust the bitrate for correct audio sound working walue is 1590us
        }

        total += bytes_read;

	}

	fr = f_close(&wav);
	if (fr != FR_OK) {
		LOG_ERR("f_close failed: %d", fr);
		return -EIO;
	}

	tx.length  = 1;
	tx.data[0] = PKT_ENDAUDIO;

	for (int i = 0; i < 3; i++) {
		tx.data[0] = PKT_ENDAUDIO;
		esb_write_payload(&tx);
		k_sleep(K_MSEC(10));
	}

    LOG_INF("Audio done, %d B sent", total);

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

void send_audio_thread(void *arg1, void *arg2, void *arg3) {

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
				LOG_INF("pinging addr with prefix %02X and current prefix index i is: %d", addr_prefix[i], i);
                esb_update_prefix(0,addr_prefix[i]);
                ret = esb_write_payload(&ping);
                k_sleep(K_MSEC(20));// wait for ack which should contain gesture
			}
			LOG_INF("");
			k_sleep(K_MSEC(PING_INTERVAL_MS));

			// check if msg queue contains gesture
			if (k_msgq_get(&msg_queue, &g, K_NO_WAIT) == 0) {
				gesture_received = true;
				while (k_msgq_get(&msg_queue, &g, K_NO_WAIT) == 0) {}
			}
		}

		/* === Gesture recievd send audio === */
		if (g.gesture_id < ARRAY_SIZE(audio_lib)) {
			if (g.gesture_id == G_IDLE) {
				g.gesture_id = sys_rand8_get() % ARRAY_SIZE(audio_lib);
				LOG_INF("random gesture %d Device prefix 0x%02X", g.gesture_id, g.pipeprefix);
			}
			LOG_INF("Recieved gesture stariting to send audio");
			esb_flush_tx(); // maby not necceserry
			esb_update_prefix(0,g.pipeprefix);
			ret = send_audio(audio_lib[g.gesture_id]);
			if(ret) {
				LOG_ERR("There was an issue while sending audio: %d", ret);
			} else {
				LOG_INF("Audio was sent correctly");
			}
		} else {
			LOG_ERR("Unknown gesture ID: %d", g.gesture_id);
		}
		k_sleep(K_MSEC(100)); //wait utnill all packets that might be neccessery to arrive
	}
}

int mpsl_initialization(void){
	int8_t const p_gain;
	mpsl_fem_lna_is_configured(&p_gain);
	printk("This is check if the LNA of FEM is configured outup value is: %d\n", p_gain);
	return 0;
}