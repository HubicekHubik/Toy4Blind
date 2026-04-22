#ifndef SD_MODUL_H_
#define SD_MODUL_H_
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

extern struct k_msgq aud_dataq;

extern struct k_sem sd_data_sem;

int sd_modul_init(void);

int count_files_in_dir(const char *parent_path, const char *dir_name);

int lsdir(const char *path);

const char *draw_audio_path(int gesture_id, uint8_t ctg_override);

void send_ad_data_to_phone();

int delete_sd_data(const char* path);

void delete_category(const uint8_t* raw_data, uint16_t total_len);

void rename_sd_file(const uint8_t* raw_data, uint16_t total_len);

void rename_sd_folder(const uint8_t* raw_data, uint16_t total_len);

void rename_category(const uint8_t* raw_data, uint16_t total_len);

void add_dir(const uint8_t* raw_data, uint16_t total_len);

const char* permute_and_find(const uint8_t* chain);
#endif