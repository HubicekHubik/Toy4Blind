#include "audio_modul.h"
#include "toy_utils.h"
#include "app_state.h"
#include "haptic_modul.h"

#ifdef CONFIG_DISK_ACCESS
	#include "sd_modul.h"
	/*moduls for SD card support*/
	#include <zephyr/storage/disk_access.h>
	#include <zephyr/fs/fs.h>
	#include <ff.h>
#endif

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(audio_modul, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/drivers/i2s.h>

#include <zephyr/sys/iterable_sections.h>
#include <zephyr/kernel.h>

#define AUD_EVENT_MSGQ_LENGTH 16

K_MSGQ_DEFINE(aud_event_msgq, sizeof(struct toy_events), AUD_EVENT_MSGQ_LENGTH, 4);

#define AUDIO_CHUNK_SIZE 1024
#define AUDIO_SD_CHUNK AUDIO_CHUNK_SIZE

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

#ifdef CONFIG_I2S
#define AUDIO_THREAD_STACK_SIZE 2048
#define AUDIO_THREAD_PRIORITY 3
K_THREAD_STACK_DEFINE(audio_haptic_stack_area, AUDIO_THREAD_STACK_SIZE);
static struct k_thread audio_haptic_thread_data;

typedef enum {
    SYS_SOUND_LOW_BATTERY,
    SYS_SOUND_BT_CONNECTED,
    SYS_SOUND_BT_DISCONNECTED,
    SYS_SOUND_MODE_DEFAULT,
    SYS_SOUND_MODE_CHAINS,
    SYS_SOUND_ZAPNUTO,
    SYS_SOUND_VYPNUTO,
	SYS_SOUND_MODE_MIRROR,
	SYS_SOUND_MODE_SWAPPED,
	SYS_SOUND_CATEG_CHANGED,

	SYS_SOUND_COUNT
} sys_sound_t;

static const char *const sys_sound_paths[] = {
    [SYS_SOUND_LOW_BATTERY]    = "SD:/system/lowB.wav",
    [SYS_SOUND_BT_CONNECTED]   = "SD:/system/btOn.wav",
    [SYS_SOUND_BT_DISCONNECTED]= "SD:/system/btOff.wav",
    [SYS_SOUND_MODE_DEFAULT]     = "SD:/system/Bezny.wav",
    [SYS_SOUND_MODE_CHAINS]  = "SD:/system/Retezovy.wav",
	[SYS_SOUND_MODE_MIRROR]	   = "SD:/system/Zrcadlovy.wav",
	[SYS_SOUND_MODE_SWAPPED]   = "SD:/system/Obraceny.wav",
	[SYS_SOUND_VYPNUTO]        = "SD:/system/Vypnout.wav",
	[SYS_SOUND_ZAPNUTO]		   = "SD:/system/Zapnout.wav",
	[SYS_SOUND_CATEG_CHANGED]  = "SD:/system/ctg_chng/"
};

void audio_feedback_thread(void *arg1, void *arg2, void *arg3);
#endif

int audio_modul_init() {

	#ifdef CONFIG_I2S
    k_thread_create(&audio_haptic_thread_data, audio_haptic_stack_area,
                K_THREAD_STACK_SIZEOF(audio_haptic_stack_area),
                audio_feedback_thread,
                NULL, NULL, NULL,
                AUDIO_THREAD_PRIORITY, 0, K_NO_WAIT);
	#endif
    return 0;
}

#ifdef CONFIG_I2S
int find_piano();

int playaudio(const struct device *dev_i2s, const char **paths, uint8_t files_num);

int i2s_initialize(const struct device *dev_i2s); // init of i2s for playing gest

int i2s_initialize(const struct device *dev_i2s){

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
	i2s_cfg.timeout = 3000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;

	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);

	if (ret < 0) {
		LOG_ERR("Failed to configure I2S stream");
		return ret;
	}

	return 0;
}

void audio_feedback_thread(void *arg1, void *arg2, void *arg3) {
	int ret;
	struct toy_events rx_ah;
	LOG_INF("Audio/haptic thread starting...");

	const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2stx));

	ret = i2s_initialize(dev_i2s);
	if (ret) {
		LOG_ERR("Init failed");
	} else {
		LOG_INF("Initilize of i2s was succesful");
	}

	while (true) {
		if (k_msgq_get(&aud_event_msgq, &rx_ah, K_FOREVER) == 0) {
			char temp_path[MAX_PATH];
			char temp_path2[MAX_PATH];
			char temp_path3[MAX_PATH];
			const char *paths[CHAIN_TRESHOLD] = {NULL, NULL, NULL};
			int num_to_play = 1;

			const char *result_path = NULL;
			LOG_INF("Recieved packet type 0x%2x gest type %d", rx_ah.type, rx_ah.payload.gest.type);
			switch (rx_ah.type) {
				case MT_GEST:
					result_path = draw_audio_path(rx_ah.payload.gest.type, 255);
					if ( result_path != NULL) {
						strncpy(temp_path, result_path, MAX_PATH - 1);
					}
					paths[0] = temp_path;
					break;
				case MT_CHAIN:
					LOG_INF("Recieved chain event");
					/*
					num_to_play = 3;
					if (ctg_piano_idx != -1) {
						LOG_INF("Piano found at category index: %d", ctg_piano_idx);
					} else {
						LOG_WRN("Piano directory (P_...) not found on SD card!");
						break;
					}
					
					for (size_t i = 0; i < CHAIN_TRESHOLD; i++) {
						result_path = draw_audio_path(rx_ah.payload.audio.chain[i], ctg_piano_idx);
						if ( result_path != NULL && i == 0) {
							strncpy(temp_path, result_path, MAX_PATH - 1);
							paths[0] = temp_path;
						}
						if ( result_path != NULL && i == 1) {
							strncpy(temp_path2, result_path, MAX_PATH - 1);
							paths[1] = temp_path2;
						}
						if ( result_path != NULL && i == 2) {
							strncpy(temp_path3, result_path, MAX_PATH - 1);
							paths[2] = temp_path3;
						}
					}
					*/

					result_path = permute_and_find(rx_ah.payload.audio.chain);
					if ( result_path != NULL) {
						strncpy(temp_path, result_path, MAX_PATH - 1);
					}
					paths[0] = temp_path;
					break;
				case MT_LOW_BATERY:
					paths[0] = sys_sound_paths[SYS_SOUND_LOW_BATTERY];
					break;
				case MT_BT_CONNECTED:
					paths[0] = sys_sound_paths[SYS_SOUND_BT_CONNECTED];
					break;
				case MT_BT_DISCONNECT:
					paths[0] = sys_sound_paths[SYS_SOUND_BT_DISCONNECTED];
					break;
				case MT_LSM6DSL_OFF:
					paths[0] = sys_sound_paths[SYS_SOUND_VYPNUTO];
					break;
				case MT_LSM6DSL_ON:
					paths[0] = sys_sound_paths[SYS_SOUND_ZAPNUTO];
					break;
				case MT_G_MODE_CHANGE:
					switch (rx_ah.payload.gest.type) {
						case GAME_MODE_DEFAULT:
							paths[0] = sys_sound_paths[SYS_SOUND_MODE_DEFAULT];
							break;
						case GAME_MODE_CHAINS:
							paths[0] = sys_sound_paths[SYS_SOUND_MODE_CHAINS];
							break;
						case GAME_MODE_MIRROR:
							paths[0] = sys_sound_paths[SYS_SOUND_MODE_MIRROR];
							break;
						case GAME_MODE_SWAPPED:
							paths[0] = sys_sound_paths[SYS_SOUND_MODE_SWAPPED];
							break;
						default:
							LOG_INF("Unknown gamemode in gest haptic thread: 0x%2x", rx_ah.type);
							break;
					}
					break;
				case MT_CHANGE_CATEGORY:
						sprintf(temp_path, "%s%c.wav",sys_sound_paths[SYS_SOUND_CATEG_CHANGED], rx_ah.payload.gest.type);
						LOG_INF("Categ change sys path is %s",temp_path);
						paths[0] = temp_path;
					break;
				default:
					LOG_INF("Unknown comand in gest haptic thread: %d", rx_ah.type);
					break;
			}
			if(paths[0][0] != '\0') {
				playaudio(dev_i2s, paths, num_to_play);
			}else{
				LOG_INF("Someting went wrong while looking for path skipping gest paly");
			}
		}
	}
}
#endif

#if defined(CONFIG_I2S) && defined(CONFIG_FILE_SYSTEM)
int playaudio(const struct device *dev_i2s, const char **paths, uint8_t files_num) {
	k_sem_take(&sd_data_sem, K_FOREVER);
	audioPlaying = true;
	void *audio_block;
	int ret = 0;

	static uint8_t audio_buf[CHAIN_TRESHOLD][AUDIO_SD_CHUNK];

	UINT bytes_read[CHAIN_TRESHOLD];
	FRESULT fr;

	FIL wav[CHAIN_TRESHOLD];

	for (size_t i = 0; i < files_num; i++) {
		if (f_open(&wav[i], paths[i], FA_READ)) {
			LOG_ERR("open %s failed (%d)", paths[i], fr);
			ret = -EIO;
			goto cleanup;
		} else {
		LOG_INF("Playing from path: %s and skipping header",paths[i]);
		f_lseek(&wav[i], 44); // skip WAV header
		}
	}
	// i2s warm up
	for (int j = 0; j < 3; j++) {
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0)
		{
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
		LOG_ERR("Could not start I2S TX error: %d", ret);
		return ret;
	}
	while (true) {
		bool any_data = false;
		
		for (size_t i = 0; i < files_num; i++) {
			if ((fr = f_read(&wav[i], audio_buf[i], AUDIO_SD_CHUNK, &bytes_read[i])) == FR_OK) {
				if(bytes_read[i] > 0) {
					any_data = true;
					//LOG_INF("FILE %d and bytes read is %d", i, bytes_read[i]);
				}
			}
		}

		if (!any_data) break;

		// I2S block allocation
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &audio_block, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("TX block allocation failed %d", ret);
			continue;
		}
		
		int16_t *out_samples = (int16_t *)audio_block;
		size_t sample_count = AUDIO_SD_CHUNK / 2;

		for (size_t i = 0; i < sample_count; i++) {

			int32_t mixed_32 = 0;

			for (size_t s = 0; s < files_num; s++) {
				if (bytes_read[s] >= (i + 1) * 2) {
					int16_t *sample_ptr = (int16_t *)&audio_buf[s][i * 2];
					mixed_32 += *sample_ptr;
				}
			}
			float scaled = (mixed_32 * (int32_t)volume) / 10 * files_num;

			// in case of overflow – clamp
			if (scaled > 32767) scaled = 32767;
			if (scaled < -32768) scaled = -32768;

			out_samples[i] = (int16_t)scaled;
		}
		
		// Send block to driver
		ret = i2s_write(dev_i2s, (uint8_t *)audio_block, BLOCK_SIZE);
		if (ret != 0) {
			LOG_ERR("I2S write error: %d", ret);
			k_mem_slab_free(&tx_0_mem_slab, &audio_block);
		}
	}

	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_STOP);
	i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);

	cleanup:
		for (int i = 0; i < files_num; i++) {
			fr = f_close(&wav[i]);
			if (fr != FR_OK) {
				LOG_ERR("f_close failed: %d", fr);
				ret = -EIO;
			}
		}

		audioPlaying = false;
		k_sem_give(&sd_data_sem);
		return ret;
}
#endif