#ifndef BLE_MODULE_H_
#define BLE_MODULE_H_
#include <stdint.h>

extern struct k_msgq but2ble_q;

extern uint8_t connected_count;
extern uint8_t act_toy_idx;

int ble_modul_init(void);

#endif