#include "nvs_modul.h"

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/sys/errno_private.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(nvs_modul, CONFIG_LOG_DEFAULT_LEVEL);

#define NVS_SCHEMA_VERSION   1
#define NVS_ID_SCHEMA_VERSION 0xFFFE
#define NVS_ID_POWSTATE 	  0x0001
#define NVS_ID_VOLUME   	  0x0002
#define NVS_ID_CUR_CATEG  	  0x0003
#define NVS_ID_CUR_GAME  	  0x0004
//#define NVS_ID_RBTCNT		  0x0005

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

static struct nvs_fs fs;

int nvs_init_datarec(uint8_t *volume, uint8_t *current_category, bool *deviceRunning, toy_game_mode_t *current_gameM) {
	int rc = 0;
	struct flash_pages_info info;
	uint32_t ver;

	fs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_WRN("Flash device %s is not ready", fs.flash_device->name);
		return -ENODEV;
	}
	fs.offset = NVS_PARTITION_OFFSET;
	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (rc) {
		LOG_WRN("Unable to get page info, rc=%d", rc);
		return rc;
	}
	fs.sector_size = info.size;
	fs.sector_count = 2U;

	rc = nvs_mount(&fs);
	if (rc) {
		LOG_WRN("Flash Init failed, rc=%d", rc);
		return rc;
	}

	rc = nvs_read(&fs, NVS_ID_SCHEMA_VERSION, &ver, sizeof(ver));

	if (rc <= 0 || ver != NVS_SCHEMA_VERSION) {
		LOG_INF("NVS schema mismatch, clearing NVS");
		nvs_clear(&fs);

		rc = nvs_mount(&fs);  // <--- znovu mount po vymazání
		if (rc) {
			LOG_ERR("NVS mount failed after clear, rc=%d", rc);
			return rc;
		}

		LOG_INF("is NVS ready: %d", fs.ready);

		ver = NVS_SCHEMA_VERSION;
		rc = nvs_write(&fs, NVS_ID_SCHEMA_VERSION, &ver, sizeof(ver));
	    if (rc < 0) {
			LOG_ERR("Failed to write schema version, rc=%d", rc);
			return rc;
    	}
	}else {
		LOG_INF("NVS schema match: %u == %u, using same NVS", ver , NVS_SCHEMA_VERSION);
	}

	uint8_t  def_power   = 1;
	uint32_t def_sound   = 0;
	uint8_t  def_volume  = 5;
	uint8_t  def_gameM   = 0;

	nvs_restore(NVS_ID_POWSTATE, deviceRunning, sizeof(*deviceRunning), &def_power);
	LOG_INF("Device power state is %d", *deviceRunning);
	nvs_restore(NVS_ID_CUR_CATEG, current_category, sizeof(*current_category), &def_sound);
	LOG_INF("Soundset index is %d", *current_category);
	nvs_restore(NVS_ID_VOLUME, volume, sizeof(*volume), &def_volume);
	LOG_INF("Volume is %d", *volume);
	nvs_restore(NVS_ID_CUR_GAME, current_gameM, sizeof(*current_gameM), &def_gameM);
	LOG_INF("Game Mode is %d", *current_gameM);
	return 0;
}

int nvs_restore(uint16_t nvs_id, void *data, size_t len ,const void * def){
	int rc = nvs_read(&fs, nvs_id, data, len);
	if (rc <= 0) {
		memcpy(data, def, len);

		rc = nvs_write(&fs, nvs_id, data, len);
		if (rc < 0) {
			LOG_ERR("NVS write failed id=%d rc=%d", nvs_id, rc);
			return rc;
		} else if (rc == 0) {
			LOG_INF("Tried to write same data to NVS wriet was terminated");
		}

		LOG_INF("NVS id %d defaulted", nvs_id);
	} else {
		LOG_INF("NVS id %d restored", nvs_id);
	}
	return 0;
}

int nvs_save_volume(uint8_t volume) {
	int rc = nvs_write(&fs, NVS_ID_VOLUME, &volume, sizeof(volume));
	if (rc < 0) {
		LOG_ERR("Failed to write schema version, rc=%d", rc);
		return rc;
	}
	return 0;
}

int nvs_save_power_state(bool deviceRunning) {
	int rc = nvs_write(&fs, NVS_ID_POWSTATE, &deviceRunning, sizeof(deviceRunning));
	if (rc < 0) {
		LOG_ERR("Failed to write schema version, rc=%d", rc);
		return rc;
	}
	return 0;
}

int nvs_save_categ(uint8_t currentCategory) {
	int rc = nvs_write(&fs, NVS_ID_CUR_CATEG, &currentCategory, sizeof(currentCategory));
	if (rc < 0) {
		LOG_ERR("Failed to write schema version, rc=%d", rc);
		return rc;
	}
	return 0;
}

int nvs_save_gameM(toy_game_mode_t currentGameMode) {
	int rc = nvs_write(&fs, NVS_ID_CUR_GAME, &currentGameMode, sizeof(currentGameMode));
	if (rc < 0) {
		LOG_ERR("Failed to write schema version, rc=%d", rc);
		return rc;
	}
	return 0;
}