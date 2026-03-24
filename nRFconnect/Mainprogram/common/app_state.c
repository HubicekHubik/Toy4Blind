#include "app_state.h"
#include "toy_utils.h"

#include <zephyr/kernel.h>
#include <stdlib.h>

bool device_running = true;
uint8_t volume = 5;

uint8_t current_category = 0;
uint8_t actual_categories_count = 0;

struct audio_dir soundset_dirs[MAX_DIRS];
struct category_group categories[MAX_CATEGORIES];
struct bt_conn *master_conn;

bool audioPlaying = false;

#define SYSFBQ_LENGTH 16

K_MSGQ_DEFINE(sysfb_msgq, sizeof(struct toy_events), SYSFBQ_LENGTH, 4);
