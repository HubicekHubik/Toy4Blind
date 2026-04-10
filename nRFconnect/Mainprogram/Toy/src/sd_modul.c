#include "sd_modul.h"
#include "app_state.h"
#include "toy_utils.h"

#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <ff.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

LOG_MODULE_REGISTER(SD_modul, CONFIG_LOG_DEFAULT_LEVEL);

K_SEM_DEFINE(sd_data_sem, 1, 1);

#include <zephyr/sys/byteorder.h>
#include <zephyr/random/random.h>

#define DISK_DRIVE_NAME "SD"
#define DISK_MOUNT_PT "/" DISK_DRIVE_NAME ":"

static uint8_t data_buffer[AUDIO_DATA_CHUNK_SIZE]; 
static struct file_data rec_fd;
static uint32_t buffer_index = 0;

static const char *disk_mount_pt = DISK_MOUNT_PT;

static FATFS fat_fs;

static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

K_MSGQ_DEFINE(aud_dataq, sizeof(struct file_data), 20, 4);

#define AD_WRITE_THREAD_STACK_SIZE 4096
#define AD_WRITE_THREAD_PRIORITY 9
K_THREAD_STACK_DEFINE(ad_write_stack_area, AD_WRITE_THREAD_STACK_SIZE);
static struct k_thread ad_write_thread_data;

static void ad_write_thread(void *arg1, void *arg2, void *arg3);

int SD_init(void);

int sd_modul_init() {
	int err;
	err = SD_init();
	if (err) {
		LOG_ERR("SD initialization failed, err %d", err);
		return err;
	}

	k_thread_create(&ad_write_thread_data, ad_write_stack_area,
					K_THREAD_STACK_SIZEOF(ad_write_stack_area),
					ad_write_thread,
					NULL,NULL,NULL,
					AD_WRITE_THREAD_PRIORITY, 0, K_NO_WAIT);

	return 0;
}

int SD_init(void)
{
	static const char *disk_pdrv = DISK_DRIVE_NAME;
	int ret = 0;

	uint32_t block_count = 0;
	uint32_t block_size = 0;

	ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_INIT, NULL);
	if (ret != 0)
	{
		LOG_ERR("Storage init ERROR!");
		return ret;
	}

	ret = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
	if (ret != 0)
	{
		LOG_ERR("Unable to get sector count");
		return ret;
	}
	LOG_INF("Block count %u", block_count);

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

int lsdir(const char *path)
{
    int res;
    struct fs_dir_t dirp;
    static struct fs_dirent entry;
    int actual_dir_count = 0;

	actual_categories_count = 0;
    memset(categories, 0, sizeof(categories));
    memset(soundset_dirs, 0, sizeof(soundset_dirs));

    fs_dir_t_init(&dirp);
    res = fs_opendir(&dirp, path);
    if (res) {
        LOG_ERR("Error opening dir %s [%d]", path, res);
        return res;
    }

    while (actual_dir_count < MAX_DIRS) {
        res = fs_readdir(&dirp, &entry);
        if (res || entry.name[0] == 0) break;

        if (entry.type == FS_DIR_ENTRY_DIR && entry.name[0] != '.') {
			if (strcmp(entry.name, "SYSTEM~1") == 0) continue; // skip system directory created by FATFS
            if(entry.name[1] == '_') {
				soundset_dirs[actual_dir_count].category = entry.name[0];
				strncpy(soundset_dirs[actual_dir_count].dir_name, &entry.name[2], MAX_DIR_NAME_LEN - 1);
			} else {
				soundset_dirs[actual_dir_count].category = 'Q';
				strncpy(soundset_dirs[actual_dir_count].dir_name, entry.name, MAX_DIR_NAME_LEN - 1);
			}
			soundset_dirs[actual_dir_count].dir_name[MAX_DIR_NAME_LEN - 1] = '\0';
			soundset_dirs[actual_dir_count].files = count_files_in_dir(path, entry.name);
			/*
			LOG_INF("Saved [DIR %d]: %s (files: %d) category: %c", 
					actual_dir_count, 
					soundset_dirs[actual_dir_count].dir_name, 
					soundset_dirs[actual_dir_count].files,
					soundset_dirs[actual_dir_count].category);
			*/
			char current_categ = soundset_dirs[actual_dir_count].category;

			//'Q' used for system sounds
			if (current_categ != 'Q') {
				int found_idx = -1;

				for (int i = 0; i < actual_categories_count; i++) {
					if (categories[i].categ_id == current_categ) {
						found_idx = i;
						break;
					}
				}

				if (found_idx == -1 && actual_categories_count < MAX_CATEGORIES) {
					found_idx = actual_categories_count;
					categories[found_idx].categ_id = current_categ;
					categories[found_idx].dir_count = 0;
					actual_categories_count++;
				}

				if (found_idx != -1 && categories[found_idx].dir_count < MAX_DIRS_COUNT) {
					categories[found_idx].dir_indices[categories[found_idx].dir_count] = actual_dir_count;
					categories[found_idx].dir_count++;
				}
			}

			actual_dir_count++;
        }
    }

    fs_closedir(&dirp);
    return actual_dir_count;
}

const char *draw_audio_path(int gesture_id) {
	struct fs_dir_t dirp;
    struct fs_dirent entry;
    static char path_buffer[MAX_PATH];
    if (current_category >= actual_categories_count) return NULL;

	struct category_group my_category = categories[current_category];

	if (my_category.dir_count < gesture_id) return NULL;
	
	struct audio_dir my_dir = soundset_dirs[my_category.dir_indices[gesture_id]];
    uint8_t total_files = my_dir.files;

    if (total_files == 0) return NULL;

    int picked_sound = sys_rand32_get() % total_files;

    char full_dir_path[MAX_PATH/2];
    snprintf(full_dir_path, sizeof(full_dir_path), "%s/%c_%s",DISK_MOUNT_PT, my_dir.category, my_dir.dir_name);
    int current_index = 0;
	bool found = false;
    fs_dir_t_init(&dirp);
    if (fs_opendir(&dirp, full_dir_path) == 0) {
        while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != 0) {
            if (entry.type == FS_DIR_ENTRY_FILE) {
                if (current_index == picked_sound) {
                    snprintf(path_buffer, sizeof(path_buffer), "%s/%s", full_dir_path, entry.name);
                    found = true;
					break;
                }
                current_index++;
            }
        }
        fs_closedir(&dirp);
    }
	return found ? (path_buffer + strlen(DISK_MOUNT_PT)) : NULL;
}

void send_ad_data_to_phone() {
	struct fs_dir_t dirp;
    struct fs_dirent entry;
    static char full_dir_path[MAX_PATH];
	char send_buffer[MAX_PATH + 1];
	send_buffer[0] = MT_RECV_SD_DATA;
	struct category_group *tx_categ;

	lsdir(disk_mount_pt);

	for (size_t i = 0; i < actual_categories_count; i++) {
		tx_categ = &categories[i];
		struct audio_dir *tx_dir;
		for (size_t idx = 0; idx < tx_categ->dir_count; idx++) {
			uint8_t actual_dir_idx = tx_categ->dir_indices[idx];
			tx_dir = &soundset_dirs[actual_dir_idx];

    		snprintf(full_dir_path, sizeof(full_dir_path), "%s/%c_%s",DISK_MOUNT_PT, tx_dir->category, tx_dir->dir_name);

			fs_dir_t_init(&dirp);
			if (fs_opendir(&dirp, full_dir_path) == 0) {
				int dir_len = snprintf(&send_buffer[1], sizeof(send_buffer) - 1, "%c/%s/", tx_dir->category, tx_dir->dir_name);
    			bt_nus_send(master_conn, (const uint8_t *)send_buffer, dir_len + 1);

				while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != 0) {
					int len = snprintf(&send_buffer[1], sizeof(send_buffer) - 1, "%c/%s/%s", tx_dir->category, tx_dir->dir_name, entry.name);
					//LOG_INF("Sending first data 0x%2x and string %s",send_buffer[0], &send_buffer[1]);
					bt_nus_send(master_conn, (const uint8_t *)send_buffer, len + 1);
					
				}
				fs_closedir(&dirp);
			}
		}
		k_sleep(K_MSEC(10));
	}
}

int delete_sd_data(const char *path_from_phone) {
    struct fs_dir_t dirp;
    struct fs_dirent entry;
    int res;
    char full_folder_path[MAX_PATH];

    snprintf(full_folder_path, sizeof(full_folder_path), "%s/%s", DISK_MOUNT_PT, path_from_phone);
    LOG_INF("DEBUG: Pokus o smazání: %s", full_folder_path);

    fs_dir_t_init(&dirp);
    res = fs_opendir(&dirp, full_folder_path);
    
    if (res != 0) {
        LOG_INF("Cesta není adresář, mažu jako soubor.");
        return fs_unlink(full_folder_path);
    }

    while (fs_readdir(&dirp, &entry) == 0 && entry.name[0] != 0) {
        if (strcmp(entry.name, ".") == 0 || strcmp(entry.name, "..") == 0) {
            continue;
        }

        char file_to_delete[MAX_PATH];
        snprintf(file_to_delete, sizeof(file_to_delete), "%s/%s", full_folder_path, entry.name);

        res = fs_unlink(file_to_delete);
        if (res == 0) {
            LOG_INF("Smazán soubor uvnitř: %s", file_to_delete);
        } else {
            LOG_ERR("Chyba při mazání souboru %s (%d)", file_to_delete, res);
        }
    }

    fs_closedir(&dirp);

    res = fs_unlink(full_folder_path);
    if (res == 0) {
        LOG_INF("Složka úspěšně odstraněna: %s", full_folder_path);
    } else {
        LOG_ERR("Nepodařilo se smazat prázdnou složku! Kód: %d", res);
    }

    return res;
}

void rename_sd_file(const uint8_t* raw_data, uint16_t total_len) {
    uint8_t old_len = raw_data[0]; // Délka staré cesty (poslaná z Androidu)
    char dir_name[MAX_DIR_NAME_LEN];
	size_t i;
	for (i = 0; i < old_len; i++) {
		if (raw_data[1+i] == '/')break;
		dir_name[i] = raw_data[1+i];		
	}
	dir_name[i] = '\0';
	
	char old_name_part[128];
    char new_name_part[128];
    
    char full_old_path[160];
    char full_new_path[160];
    FRESULT res;

    memcpy(old_name_part, &raw_data[1], old_len);
    old_name_part[old_len] = '\0';
    
    uint8_t new_len = total_len - 1 - old_len; 
    memcpy(new_name_part, &raw_data[1 + old_len], new_len);
    new_name_part[new_len] = '\0';

    snprintf(full_old_path, sizeof(full_old_path), "%s/%s", DISK_MOUNT_PT, old_name_part);
    snprintf(full_new_path, sizeof(full_new_path), "%s/%s/%s", DISK_MOUNT_PT, dir_name,new_name_part);

    LOG_INF("Přejmenovávám: %s", full_old_path);
    LOG_INF("Na: %s", full_new_path);

    res = fs_rename(full_old_path, full_new_path);
    
    if (res == FR_OK) {
        LOG_INF("Přejmenování úspěšné");
    } else {
        LOG_ERR("Chyba f_rename: %d", res);
    }
}

void rename_sd_folder(const uint8_t* raw_data, uint16_t total_len) {
    uint8_t old_len = raw_data[0]; // Délka staré cesty (poslaná z Androidu)
	LOG_INF("tohle je format dat pro rename folder %s", raw_data);
	char old_name_part[MAX_DIR_NAME_LEN];
    char new_name_part[MAX_DIR_NAME_LEN];
    
    char full_old_path[MAX_PATH];
    char full_new_path[MAX_PATH];
    FRESULT res;

    memcpy(old_name_part, &raw_data[1], old_len);
    old_name_part[old_len] = '\0';
    
    uint8_t new_len = total_len - 1 - old_len; 
    memcpy(new_name_part, &raw_data[1 + old_len], new_len);
    new_name_part[new_len] = '\0';

    snprintf(full_old_path, sizeof(full_old_path), "%s/%s", DISK_MOUNT_PT, old_name_part);
    snprintf(full_new_path, sizeof(full_new_path), "%s/%s", DISK_MOUNT_PT, new_name_part);

    LOG_INF("Přejmenovávám: %s", full_old_path);
    LOG_INF("Na: %s", full_new_path);

    res = fs_rename(full_old_path, full_new_path);
    
    if (res == FR_OK) {
        LOG_INF("Přejmenování úspěšné");
    } else {
        LOG_ERR("Chyba f_rename: %d", res);
    }
}

void add_dir(const uint8_t* raw_data, uint16_t total_len) {
	char add_dir[MAX_DIR_NAME_LEN];
	memcpy(&add_dir, raw_data, total_len);
	add_dir[total_len] = '\0';

	char full_path[MAX_PATH];
	snprintf(full_path, sizeof(full_path), "%s/%s", DISK_MOUNT_PT, add_dir);
	LOG_INF("Vytvářím adresář %s", full_path);

	int res = fs_mkdir(full_path);

	if (res == FR_OK) {
        LOG_INF("Adresář vytvořen");
    } else {
        LOG_ERR("Chyba mkdir: %d", res);
    }
}

void delete_category(const uint8_t* raw_data, uint16_t total_len) {
	char category = raw_data[0];
	
	for (size_t i = 0; i < MAX_DIRS; i++) {
		if (soundset_dirs[i].category == category) {
			char dir_path[MAX_PATH];
			snprintf(dir_path, sizeof(dir_path), "%c_%s", category, soundset_dirs[i].dir_name);
			delete_sd_data(dir_path);
		}
	}
}

void rename_category(const uint8_t* raw_data, uint16_t total_len) {
	char old_c = raw_data[1];
	char new_c = raw_data[2];

	LOG_INF("Entered rename categ function");

	for (size_t i = 0; i < MAX_DIRS; i++) {
		if (soundset_dirs[i].category == old_c) {
			char old_full_path[MAX_PATH];
            char new_full_path[MAX_PATH];
			
			snprintf(old_full_path, sizeof(old_full_path), "%s/%c_%s", 
                     DISK_MOUNT_PT, old_c, soundset_dirs[i].dir_name);
            
            snprintf(new_full_path, sizeof(new_full_path), "%s/%c_%s", 
                     DISK_MOUNT_PT, new_c, soundset_dirs[i].dir_name);

			LOG_INF("Přejmenovávám složku kategorie: %s -> %s", old_full_path, new_full_path);
            
            int res = fs_rename(old_full_path, new_full_path);
            if (res == 0) {
                soundset_dirs[i].category = new_c;
            } else {
                LOG_ERR("Chyba přejmenování složky v kategorii: %d", res);
            }
		}
	}
}

int count_files_in_dir(const char *parent_path, const char *dir_name)
{
    struct fs_dir_t sub_dirp;
    struct fs_dirent entry;
    char full_path[64];
    int file_count = 0;

    snprintf(full_path, sizeof(full_path), "%s/%s", parent_path, dir_name);
    
    fs_dir_t_init(&sub_dirp);
    if (fs_opendir(&sub_dirp, full_path) == 0) {
        while (fs_readdir(&sub_dirp, &entry) == 0 && entry.name[0] != 0) {
            if (entry.type == FS_DIR_ENTRY_FILE) {
                file_count++;
            }
        }
        fs_closedir(&sub_dirp);
    }
    return file_count;
}

static void ad_write_thread(void *arg1, void *arg2, void *arg3) {
	LOG_INF("Thread ad_write_thread started");
    char file_name[128];
    char full_path[160];
    uint32_t file_size = 0;
    uint32_t file_name_len = 0;
    uint32_t received_so_far = 0;
    FIL new_wav;
    FRESULT fr;
    bool file_open = false;

    while (true) {
        if (k_msgq_get(&aud_dataq, &rec_fd, K_FOREVER) == 0) {
			k_sem_take(&sd_data_sem, K_FOREVER);
			switch (rec_fd.file_msg_type) {
			case MT_FILE:
				file_size = sys_get_le32(rec_fd.data);
				file_name_len = sys_get_le16(&rec_fd.data[4]);
				buffer_index = 0; // Resetujeme index pro nový soubor

				if (file_name_len >= sizeof(file_name)) {
					file_name_len = sizeof(file_name) - 1;
				}

				memcpy(file_name, &rec_fd.data[6], file_name_len);
				file_name[file_name_len] = '\0';
				snprintf(full_path, sizeof(full_path), "SD:/%s",file_name);
				
				LOG_INF("Vytvarim soubor: %s, velikost: %u velikost_nazvu: %d", full_path, file_size, file_name_len);
				
				fr = f_open(&new_wav, full_path, FA_WRITE | FA_CREATE_ALWAYS);
				if (fr == FR_OK) {
					file_open = true;
					received_so_far = 0;
				} else {
					LOG_ERR("Chyba f_open: %d", fr);
				}
				break;
			case MT_FILE_TRANSFER:
				if (file_open) {
					memcpy(&data_buffer[buffer_index], rec_fd.data, rec_fd.data_len);
					buffer_index += rec_fd.data_len;
					received_so_far += rec_fd.data_len;

					if (buffer_index >= AUDIO_DATA_CHUNK_SIZE || received_so_far >= file_size) {
						UINT written;
						fr = f_write(&new_wav, data_buffer, buffer_index, &written);
						
						if (fr != FR_OK) {
							LOG_WRN("Chyba zapisu na SD: %d", fr);
						}
						
						buffer_index = 0;
						//LOG_INF("Zapsano %u bytes, celkem zapsano: %u/%u", written, received_so_far, file_size);
						if (received_so_far >= file_size) {
							f_close(&new_wav);
							file_open = false;
							LOG_INF("Soubor uspesne ulozen. Celkem: %u", received_so_far);
							
							/*const struct device *dev_i2s = DEVICE_DT_GET(DT_ALIAS(i2stx));
							playaudio(dev_i2s, full_path);*/
							send_ad_data_to_phone();
						}
					}
				}
				break;
			case MT_REQUEST_SD_DATA:
				LOG_INF("Recieved sd data req form phone");
				send_ad_data_to_phone();
				break;
			case MT_DELETE_SD_FILE:
				delete_sd_data(rec_fd.data);
				break;
			case MT_RENAME_SD_FILE:
				rename_sd_file(rec_fd.data, rec_fd.data_len);
				break;
			case MT_RENAME_SD_FOLDER:
				rename_sd_folder(rec_fd.data, rec_fd.data_len);
				break;
			case MT_ADD_SD_DIR:
				add_dir(rec_fd.data, rec_fd.data_len);
				break;
			case MT_DELETE_SD_CATEGORY:
				delete_category(rec_fd.data, rec_fd.data_len);
				break;
			case MT_RENAME_SD_CATEGORY:
				rename_category(rec_fd.data, rec_fd.data_len);
				break;
			default:
				printk("Unknown comand in ad_write_thread throwing away 0x%2x.\n", rec_fd.file_msg_type);
				break;
			}
			if(rec_fd.file_msg_type != MT_FILE && 
				rec_fd.file_msg_type != MT_FILE_TRANSFER && 
				rec_fd.file_msg_type != MT_REQUEST_SD_DATA) {
				uint8_t cmd = MT_CLEAR_DATA;
				bt_nus_send(master_conn, &cmd, sizeof(cmd));
				k_sleep(K_MSEC(10));
				send_ad_data_to_phone();
			}
			k_sem_give(&sd_data_sem);
		}
    }
}