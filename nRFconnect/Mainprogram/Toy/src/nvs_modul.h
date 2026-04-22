#ifndef NVS_MODUL_H_
#define NVS_MODUL_H_
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "app_state.h"

int nvs_init_datarec(uint8_t *volume, uint8_t *current_category, bool *deviceRunning, toy_game_mode_t *current_gameM);

int nvs_restore(uint16_t nvs_id, void *data, size_t len ,const void * def);

int nvs_save_volume(uint8_t volume);

int nvs_save_power_state(bool deviceRunning);

int nvs_save_categ(uint8_t currentCategory);

int nvs_save_gameM(toy_game_mode_t currentGameMode);
#endif