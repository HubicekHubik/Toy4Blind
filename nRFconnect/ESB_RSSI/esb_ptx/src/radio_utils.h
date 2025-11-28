#ifndef RADIO_UTILS_H
#define RADIO_UTILS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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
int8_t get_current_tx_power_dbm(void);

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
bool tx_power_supported(int8_t dbm);


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
int8_t get_current_tx_power(void);


#ifdef __cplusplus
}
#endif

#endif // RADIO_UTILS_H

