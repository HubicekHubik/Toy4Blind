#ifndef DEVICE_INFO_H
#define DEVICE_INFO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void init_device_info(void);
uint32_t get_device_id(void);

/* Defineer board ID hier op basis van config */
#if defined(CONFIG_BOARD_NRF21540DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x11
#elif defined(CONFIG_BOARD_NRF52840DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x12
#elif defined(CONFIG_BOARD_NRF5340DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x13
#elif defined(CONFIG_BOARD_NRF5340_AUDIO_DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x14
#elif defined(CONFIG_BOARD_NRF7002DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x15
#elif defined(CONFIG_BOARD_NRF54L15DK) && defined(CONFIG_MPSL_FEM)
    #define BOARD_ID 0x16

#elif defined(CONFIG_BOARD_NRF21540DK)
    #define BOARD_ID 0x01
#elif defined(CONFIG_BOARD_NRF52840DK)
    #define BOARD_ID 0x02
#elif defined(CONFIG_BOARD_NRF5340DK)
    #define BOARD_ID 0x03
#elif defined(CONFIG_BOARD_NRF5340_AUDIO_DK)
    #define BOARD_ID 0x04
#elif defined(CONFIG_BOARD_NRF7002DK)
    #define BOARD_ID 0x05
#elif defined(CONFIG_BOARD_NRF54L15DK)
    #define BOARD_ID 0x06

#elif defined(CONFIG_BOARD_RAYTAC_MDBT50Q_DB_40_NRF52840)
    #define BOARD_ID 0x21
#elif defined(CONFIG_BOARD_RAYTAC_AN54L15Q_DB)
    #define BOARD_ID 0x22
#else
    #define BOARD_ID 0xFF  // Unknown
#endif

#ifdef __cplusplus
}
#endif

#endif // DEVICE_INFO_H
