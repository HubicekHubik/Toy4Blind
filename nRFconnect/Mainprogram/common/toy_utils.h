#ifndef TOY_UTILS_H
#define TOY_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*MSG types */
#define MT_GEST 			  	0xB1 //MESSAGE TYPE GESTURE
#define MT_LMR 				 	0xB2 //MESSAGE TYPE LINEAR MOTOR EVENT
#define MT_SYSINF 				0x1F //MESSAGE TYPE SYSTEM INFO
#define MSG_TYPE_SYSCHNG 		0x8C //MESSAGE TYPE SYSTEM CHANGE
#define MT_LSM6DSL_ON			0x6A //MESSAGE TYPE LSM6DSL ON
#define MT_LSM6DSL_OFF			0x6F //MESSAGE TYPE LSM6DSL OFF
/*Remotes msg to toy*/
#define MSG_TYPE_TOY_SWITCH 	0x67 //
#define MT_BT_DISCONNECT 		0xBE //MESSAGE TYPE BLUETOOTH ON/OFF
#define MT_BT_CONNECTED 		0xBC
#define MSG_TYPE_TURNOFF 		0x0F
#define MT_VOL_UP 				0x08
#define MT_VOL_DOWN 			0x09
#define MT_VOL_MUTE 			0x0A
#define MSG_TYPE_SOUNDSET 		0x6E
/*App & toy msg*/
#define MT_REQUEST_SD_DATA 		0x6D
#define MT_RECV_SD_DATA 		0x7D
#define MT_DELETE_SD_FILE 		0xDF
#define MT_DELETE_SD_CATEGORY 	0xDC
#define MT_RENAME_SD_FILE 		0xEF
#define MT_RENAME_SD_FOLDER 	0xED
#define MT_RENAME_SD_CATEGORY 	0xEC
#define MT_ADD_SD_FILE 			0xAF
#define MT_ADD_SD_DIR 			0xAD
#define MT_ADD_SD_CATEG 		0xAC
#define MT_CLEAR_DATA 			0xCD
#define MT_CHANGE_CATEGORY 		0xCC

#define MT_DEBUG_DATA			0xDB
/*Phones msg to toy*/
#define MT_FILE 0xFE
#define MT_FILE_TRANSFER 0xF6

#define MT_LOW_BATERY 0xB0
#define CLICK_EFECT 			102
#define CONNECTED_EFFECT 		103
#define CHARGING_EFFECT 		104
struct but_ev{
	uint8_t cmd;
};

struct ei_ev{
	float values[5];
	float anomaly;
};

struct audio_ev {
	uint8_t type;
	uint8_t prob;
	uint16_t anomaly;
	uint8_t sign;
};

struct lmr_ev {
	uint8_t type;
	uint8_t effect;	//number to identify effect that should be used
	uint8_t ms_duration; //not implemented yet
};

struct bat_ev {
	uint16_t V_bat;
	uint16_t V_ched;
	uint16_t V_ching;
};

struct sys_ev {
	uint8_t low_bat;
	uint8_t device_on_chg;
	uint8_t device_charged;
};


struct toy_events {
    uint8_t type;   // MSG_TYPE
    uint8_t len;    // Data size
    union {         
        struct audio_ev gest;
        struct lmr_ev   lmr;
        struct bat_ev powInf;
        struct sys_ev   sys;
    } payload;
} __attribute__((packed));
#endif