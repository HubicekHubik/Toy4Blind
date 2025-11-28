#include "device_info.h"
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>

// Static opslag van het device ID
static uint32_t device_id;

void init_device_info(void)
{
    uint8_t hwid[8];
    if (hwinfo_get_device_id(hwid, sizeof(hwid)) >= 8) {
        device_id = sys_get_le32(&hwid[0]) ^ sys_get_le32(&hwid[4]);
    } else {
        device_id = 0xFFFFFFFF;
    }
}

uint32_t get_device_id(void)
{
    return device_id;
}