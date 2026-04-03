#ifndef APP_STATE_H_
#define APP_STATE_H_
#include <stdint.h>
#include <stdbool.h>

#define VOLUME_STEP 1
#define MAX_VOLUME 20
#define MIN_VOLUME 0

#define MAX_PATH 128
#define MAX_DIRS 40
#define MAX_CATEGORIES 8
#define MAX_DIR_NAME_LEN 32
#define MAX_FILE_NAME_LEN 3

#define AUDIO_DATA_LEN CONFIG_BT_L2CAP_TX_MTU - 3 
#define AUDIO_DATA_RAW_LEN AUDIO_DATA_LEN - 1
#define AUDIO_DATA_CHUNK_SIZE (AUDIO_DATA_RAW_LEN) * 10

extern struct k_msgq sysfb_msgq;

struct audio_dir {
	char category;
	uint8_t files;
	char dir_name[MAX_DIR_NAME_LEN];
};

struct category_group {
    char categ_id;
    uint8_t dir_indices[8];
    uint8_t dir_count;
};

struct file_data {
	uint8_t file_msg_type;
	uint16_t data_len;
	uint8_t data[243];
};

extern bool device_running;
extern uint8_t volume;
extern uint8_t current_category;
extern struct audio_dir soundset_dirs[MAX_DIRS];
extern struct category_group categories[MAX_CATEGORIES];
extern uint8_t actual_categories_count;
extern struct bt_conn *master_conn;
extern bool audioPlaying;
#endif