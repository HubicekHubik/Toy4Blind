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

LOG_MODULE_REGISTER(audio, CONFIG_LOG_DEFAULT_LEVEL);

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
#define AUDIO_HAPTIC_THREAD_STACK_SIZE 2048
#define AUDIO_HAPTIC_THREAD_PRIORITY 3
K_THREAD_STACK_DEFINE(audio_haptic_stack_area, AUDIO_HAPTIC_THREAD_STACK_SIZE);
static struct k_thread audio_haptic_thread_data;

void audio_haptic_feedback_thread(void *arg1, void *arg2, void *arg3);
#endif



void LMR_control_thread(void *arg1, void *arg2, void *arg3);

int audio_modul_init() {

	#ifdef CONFIG_I2S
    k_thread_create(&audio_haptic_thread_data, audio_haptic_stack_area,
                K_THREAD_STACK_SIZEOF(audio_haptic_stack_area),
                audio_haptic_feedback_thread,
                NULL, NULL, NULL,
                AUDIO_HAPTIC_THREAD_PRIORITY, 0, K_NO_WAIT);
	#endif
    return 0;
}

#ifdef CONFIG_I2S
int playaudio(const struct device *dev_i2s, const char *path);

int i2s_initialize(const struct device *dev_i2s); // init of i2s for playing audio

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

void audio_haptic_feedback_thread(void *arg1, void *arg2, void *arg3) {
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

			const char *result_path = NULL;
			
			switch (rx_ah.type) {
				case MT_GEST:
					result_path = draw_audio_path(rx_ah.payload.gest.type);
					if ( result_path != NULL) {
						strncpy(temp_path, result_path, sizeof(temp_path) - 1);
					}
					break;
				case MT_LOW_BATERY:
					strncpy(temp_path, "SD:/system/lowB.wav", sizeof(temp_path) - 1);
					break;
				case MT_BT_CONNECTED:
					strncpy(temp_path, "SD:/system/btOn.wav", sizeof(temp_path) - 1);
					break;
				case MT_BT_DISCONNECT:
					strncpy(temp_path, "SD:/system/btOff.wav", sizeof(temp_path) - 1);
					break;
				default:
					LOG_INF("Unknown comand in audio haptic thread: %d", rx_ah.type);
					break;
			}
			if(temp_path != NULL) {
				playaudio(dev_i2s, temp_path);
			}else{
				LOG_INF("Someting went wrong while looking for path skipping audio paly");
			}
		}
	}
}
#endif

#if defined(CONFIG_I2S) && defined(CONFIG_FILE_SYSTEM)
int playaudio(const struct device *dev_i2s, const char *path) {
	k_sem_take(&sd_data_sem, K_FOREVER);
	audioPlaying = true;
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
		LOG_INF("Playing audio from path: %s and skipping header",path);
		f_lseek(&wav, 44); // skip WAV header
	}

	// i2s warm up
	for (int j = 0; j < 1; j++) {
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

	while ((fr = f_read(&wav, audio_buf, AUDIO_SD_CHUNK, &bytes_read)) == FR_OK && bytes_read > 0)
	{
		int16_t *samples = (int16_t *)audio_buf;
		size_t count = bytes_read / 2; // 16bit PCM

		for (size_t i = 0; i < count; i++) {
			float scaled = samples[i] * (volume / 10.0f);

			// přetečení – clamp
			if (scaled > 32767.0f)
				scaled = 32767.0f;
			if (scaled < -32768.0f)
				scaled = -32768.0f;

			samples[i] = (int16_t)scaled;
		}

		// Alokace bloku
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

	audioPlaying = false;
	k_sem_give(&sd_data_sem);
	return 0;
}
#endif