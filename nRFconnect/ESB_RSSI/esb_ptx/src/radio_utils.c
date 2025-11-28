#include "radio_utils.h"
#include <hal/nrf_radio.h>

/**
 * @brief Gets the currently active TX power setting from the RADIO peripheral.
 *
 * This function returns the TX power that is currently configured in the RADIO peripheral's TXPOWER register.
 * Note that when using the ESB (Enhanced ShockBurst) protocol, calling esb_set_tx_power() does not
 * immediately update the TXPOWER hardware register. Instead, the desired power level is stored internally
 * in the ESB driver and only applied when a transmission is started (e.g., during esb_write_payload()).
 *
 * Therefore, if this function is called immediately after esb_set_tx_power() and before any payload has been
 * transmitted, it may return the *previous* TX power value. To get the actual active TX power, call this
 * function after at least one successful transmission.
 *
 * @return The current TX power in dBm, as set in the RADIO peripheral.
 */
int8_t get_current_tx_power_dbm(void) {
	nrf_radio_txpower_t raw_power = nrf_radio_txpower_get(NRF_RADIO);

	switch (raw_power) {
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
	case RADIO_TXPOWER_TXPOWER_Pos8dBm:
		return 8;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
	case RADIO_TXPOWER_TXPOWER_Pos7dBm:
		return 7;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
	case RADIO_TXPOWER_TXPOWER_Pos6dBm:
		return 6;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
	case RADIO_TXPOWER_TXPOWER_Pos5dBm:
		return 5;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
	case RADIO_TXPOWER_TXPOWER_Pos4dBm:
		return 4;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
	case RADIO_TXPOWER_TXPOWER_Pos3dBm:
		return 3;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
	case RADIO_TXPOWER_TXPOWER_Pos2dBm:
		return 2;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos1dBm)
	case RADIO_TXPOWER_TXPOWER_Pos1dBm:
		return 1;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_0dBm)
	case RADIO_TXPOWER_TXPOWER_0dBm:
		return 0;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg1dBm)
	case RADIO_TXPOWER_TXPOWER_Neg1dBm:
		return -1;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg2dBm)
	case RADIO_TXPOWER_TXPOWER_Neg2dBm:
		return -2;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg3dBm)
	case RADIO_TXPOWER_TXPOWER_Neg3dBm:
		return -3;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg4dBm)
	case RADIO_TXPOWER_TXPOWER_Neg4dBm:
		return -4;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg5dBm)
	case RADIO_TXPOWER_TXPOWER_Neg5dBm:
		return -5;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg6dBm)
	case RADIO_TXPOWER_TXPOWER_Neg6dBm:
		return -6;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg7dBm)
	case RADIO_TXPOWER_TXPOWER_Neg7dBm:
		return -7;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg8dBm)
	case RADIO_TXPOWER_TXPOWER_Neg8dBm:
		return -8;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg9dBm)
	case RADIO_TXPOWER_TXPOWER_Neg9dBm:
		return -9;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg10dBm)
	case RADIO_TXPOWER_TXPOWER_Neg10dBm:
		return -10;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg12dBm)
	case RADIO_TXPOWER_TXPOWER_Neg12dBm:
		return -12;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg14dBm)
	case RADIO_TXPOWER_TXPOWER_Neg14dBm:
		return -14;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg16dBm)
	case RADIO_TXPOWER_TXPOWER_Neg16dBm:
		return -16;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg18dBm)
	case RADIO_TXPOWER_TXPOWER_Neg18dBm:
		return -18;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg20dBm)
	case RADIO_TXPOWER_TXPOWER_Neg20dBm:
		return -20;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg28dBm)
	case RADIO_TXPOWER_TXPOWER_Neg28dBm:
		return -28;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg40dBm)
	case RADIO_TXPOWER_TXPOWER_Neg40dBm:
		return -40;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg46dBm)
	case RADIO_TXPOWER_TXPOWER_Neg46dBm:
		return -46;
#endif
	default:
		return -128; // ongeldige waarde
	}
}

/**
 * @brief Checks whether a given TX power level in dBm is supported by the hardware.
 *
 * This function evaluates whether a given dBm value corresponds to a valid value
 * defined by the RADIO_TXPOWER register for the current SoC. Not all power levels
 * are available on all Nordic chips.
 *
 * @param dbm Desired TX power level in dBm (e.g., 8, 0, -20).
 * @return true If the value is supported by the RADIO peripheral.
 * @return false If the value is unsupported or invalid for the current hardware.
 */
bool tx_power_supported(int8_t dbm)
{
	switch (dbm) {
#if defined(RADIO_TXPOWER_TXPOWER_Neg100dBm)
	case -100:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg70dBm)
	case -70:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg46dBm)
	case -46:
		return true;
#endif
	case -40:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg30dBm)
	case -30:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg28dBm)
	case -28:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg22dBm)
	case -22:
		return true;
#endif
	case -20:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg18dBm)
	case -18:
		return true;
#endif
	case -16:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg14dBm)
	case -14:
		return true;
#endif
	case -12:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg10dBm)
	case -10:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg9dBm)
	case -9:
		return true;
#endif
	case -8:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg7dBm)
	case -7:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg6dBm)
	case -6:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg5dBm)
	case -5:
		return true;
#endif
	case -4:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Neg3dBm)
	case -3:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg2dBm)
	case -2:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Neg1dBm)
	case -1:
		return true;
#endif
	case 0:
		return true;
#if defined(RADIO_TXPOWER_TXPOWER_Pos1dBm)
	case 1:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos2dBm)
	case 2:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos3dBm)
	case 3:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos4dBm)
	case 4:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos5dBm)
	case 5:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos6dBm)
	case 6:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos7dBm)
	case 7:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos8dBm)
	case 8:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos9dBm)
	case 9:
		return true;
#endif
#if defined(RADIO_TXPOWER_TXPOWER_Pos10dBm)
	case 10:
		return true;
#endif
	default:
		return false;
	}
}

/**
 * @brief Returns the raw value from the RADIO TXPOWER register as-is.
 *
 * This function returns the enum value currently stored in the TXPOWER register,
 * cast to an int8_t. For most Nordic chips, this corresponds directly to the
 * dBm value. However, this function does not perform any validation or mapping.
 *
 * It is useful for debugging or checking the raw register contents.
 *
 * @return int8_t The current TX power register value, typically matching a dBm level.
 */
int8_t get_current_tx_power(void)
{
    nrf_radio_txpower_t raw_power = nrf_radio_txpower_get(NRF_RADIO);
    return (int8_t)raw_power;
}