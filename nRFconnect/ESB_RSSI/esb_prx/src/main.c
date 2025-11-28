/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif

#define FLAG_END_OF_TEST 0x01

int8_t tx_fem_power;
int8_t tx_power;
int8_t base_tx_power; // Base TX Power without FEM
int8_t calculated_tx_power; // FEM TX Power
int8_t fem_tx_power_overwrite; // FEM TX Power
int8_t bitrate;

// Version Information
static int buildversion=3;
static const char *board_id = "PRX";

static uint32_t device_id = 0;

void log_and_validate_bitrate(int8_t* bitrate) {
    switch (*bitrate) {
        case ESB_BITRATE_2MBPS:
            printk("Using 2 Mbps bitrate\n");
            break;
        case ESB_BITRATE_1MBPS:
            printk("Using 1 Mbps bitrate\n");
            break;
        default:
            *bitrate = ESB_BITRATE_1MBPS;
            printk("Invalid bitrate, defaulting to 1 Mbps\n");
            break;
    }
}

void init_device_info() {
    uint8_t hwid[8];
    if (hwinfo_get_device_id(hwid, sizeof(hwid)) >= 8) {
        device_id = sys_get_le32(&hwid[0]) ^ sys_get_le32(&hwid[4]);
    } else {
        device_id = 0xFFFFFFFF; // Fallback in case ID is not available
    }
}

uint32_t get_device_id() {
    return device_id;
}

LOG_MODULE_REGISTER(esb_prx, CONFIG_ESB_PRX_APP_LOG_LEVEL);

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);

static void leds_update(uint8_t value)
{
	uint32_t leds_mask =
		(!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
		(!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
		(!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
		(!(value % 8 > 3) ? DK_LED4_MSK : 0);

	dk_set_leds(leds_mask);
}

const char* rssi_quality_description(int8_t rssi_dbm)
{
	if (rssi_dbm >= -30) return "Excellent";
	if (rssi_dbm >= -40) return "Very Good";
	if (rssi_dbm >= -50) return "Good";
	if (rssi_dbm >= -60) return "Fair";
	if (rssi_dbm >= -65) return "Average";
	if (rssi_dbm >= -70) return "Weak";
	if (rssi_dbm >= -75) return "Poor";
	if (rssi_dbm >= -80) return "Very Poor";
	if (rssi_dbm >= -90) return "Critical";
	return "No Link";
}


void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		//printk("Sending: TX SUCCESS EVENT\n");
		break;
	case ESB_EVENT_TX_FAILED:
		printk("Sending: TX FAILED EVENT\n");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			uint8_t led_val = rx_payload.data[1];
			int8_t tx_power = (int8_t)rx_payload.data[2];
			uint32_t device_id = sys_get_le32(&rx_payload.data[3]);
			uint8_t board_id = rx_payload.data[7];
			int8_t rssi_dbm = -((int8_t)NRF_RADIO->RSSISAMPLE);

			const char *board_name;
			switch (board_id) {
				case 0x01: board_name = "nRF21540DK                 "; break;
				case 0x02: board_name = "nRF52840DK           	    "; break;
				case 0x03: board_name = "nRF5340DK                  "; break;
				case 0x04: board_name = "nRF5340_Audio_DK	        "; break;
				case 0x05: board_name = "nRF7002DK                  "; break;
				case 0x06: board_name = "nRF54L15DK                 "; break;

				case 0x11: board_name = "nRF21540DK FEM Active      "; break;				
				case 0x12: board_name = "nRF52840DK FEM Active      "; break;
				case 0x13: board_name = "nRF5340DK  FEM Active      "; break;
				case 0x14: board_name = "nRF5340_Audio_DK FEM Active"; break;
				case 0x15: board_name = "nRF7002DK  FEM Active      "; break;
				case 0x16: board_name = "nRF54L15DK FEM Active      "; break;

				case 0x21: board_name = "Raytac52DK                 "; break;
				case 0x22: board_name = "Raytac54DK                 "; break;
				default:   board_name = "Unknown                    "; break;
			}
			
			bool is_end_of_test = (rx_payload.length >= 9) && (rx_payload.data[8] & FLAG_END_OF_TEST);

			if (is_end_of_test)
			{
				// End of test
				printk("\n");
			}
			else
			{
				printk("ID:%u | Board:%s | Counter:%3d | TX:%3d dBm | RSSI:%3d dBm (%s)\n",
				device_id, board_name, led_val, tx_power, rssi_dbm, rssi_quality_description(rssi_dbm));

				leds_update(led_val);
			}

		} else {
			printk("Error while reading RX packet\n");
		}
		break;
	}
}

#if defined(CONFIG_CLOCK_CONTROL_NRF)
int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		printk("Unable to get the Clock manager\n");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		printk("Clock request failed: %d\n", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			printk("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	printk("HF clock started\n");
	return 0;
}

#elif defined(CONFIG_CLOCK_CONTROL_NRF2)

int clocks_start(void)
{
	int err;
	int res;
	const struct device *radio_clk_dev =
		DEVICE_DT_GET_OR_NULL(DT_CLOCKS_CTLR(DT_NODELABEL(radio)));
	struct onoff_client radio_cli;

	/** Keep radio domain powered all the time to reduce latency. */
	nrf_lrcconf_poweron_force_set(NRF_LRCCONF010, NRF_LRCCONF_POWER_DOMAIN_1, true);

	sys_notify_init_spinwait(&radio_cli.notify);

	err = nrf_clock_control_request(radio_clk_dev, NULL, &radio_cli);

	do {
		err = sys_notify_fetch_result(&radio_cli.notify, &res);
		if (!err && res) {
			printk("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err == -EAGAIN);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	printk("HF clock started\n");

	return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver\n");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	uint8_t base_addr_0[4] = {0xE6, 0xE6, 0xE6, 0xE6};
	uint8_t base_addr_1[4] = {0xC3, 0xC3, 0xC3, 0xC3};
	uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = bitrate;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;
	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}
	
	// Set channel to 80 ( that means 2480 MHz)
	err = esb_set_rf_channel(79);
	if (err) {
		LOG_ERR("Set RF channel failed");
		return err;
	}

	return 0;
}

int main(void)
{
	
	// Set default TX Power ( 8 dBm only supported by nRF52 and nRF54 SOC, for nRF53 max = 0 dBm)
	tx_power = 8;
	fem_tx_power_overwrite = 0; // FEM TX Power overwrite value of ONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB from Kconfig

	bitrate = ESB_BITRATE_1MBPS;  // Default bitrate is 1 Mbps although it can be changed to 2 Mbps
	
	// Application name
	printk("\nEnhanced ShockBurst prx sample\n");
	printk("TX Performance test and RSSI value\n");
	
	// Build information
	printk("\n[%s] Version: %d | Build: %s %s\n", board_id, buildversion, __DATE__, __TIME__);
	printk("Board: %s\n", CONFIG_BOARD);

    init_device_info();
    printk("Device ID: %u\n", get_device_id());
	
	int err;
	int8_t rx_fem_power;
	rx_fem_power = 0;

	// Check FEM Support
	printk("Check FEM TX support\n\n");
	// Check FEM Support
	#if defined(CONFIG_MPSL_FEM)
	if (CONFIG_MPSL_FEM)
	{
		printk("FEM TX support\n\n");
		if (CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB>0) tx_fem_power = CONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB;
		if (fem_tx_power_overwrite>0) 
		{
			tx_fem_power = fem_tx_power_overwrite;
			printk("FEM TX support: enabled (TX gain: %d dBm) - manual overwrite!\n", tx_fem_power);
		}
		else
		{
			printk("FEM TX support: enabled (TX gain: %d dBm)\n", tx_fem_power);
		}

		// Overwrite TX Power with FEM TX Power
		tx_power = tx_fem_power;
	}
	#endif /* defined(CONFIG_MPSL_FEM) */
	if (tx_fem_power==0)
	{
		printk("FEM TX support: disabled\n");
	}

	// Log and validate bitrate
	log_and_validate_bitrate(&bitrate);

	err = clocks_start();
	if (err) {
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		printk("LEDs initialization failed, err %d\n", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		printk("ESB initialization failed, err %d\n", err);
		return 0;
	}

	// Set ESB TX Power
	#if defined(CONFIG_BOARD_NRF5340DK) || (CONFIG_BOARD_NRF7002DK)
	{
		if (tx_fem_power==0)
		{
			tx_power = 0; // nRF5340 and nRF7002 do not support TX Power > 0 dBm
			printk("\nRF5340 and nRF7002 do not support TX Power > 0 dBm, set TX Power to 0 dBm\n");
		}
	}
	#endif

	esb_set_tx_power(tx_power);
	printk("\nSet TX power to: %d dBm (0x%02X)\n", tx_power, (uint8_t)tx_power);

	printk("Initialization complete\n");

	err = esb_write_payload(&tx_payload);
	if (err) {
		printk("Write payload, err %d\n", err);
		return 0;
	}

	printk("Setting up for packet receiption\n");

	err = esb_start_rx();
	if (err) {
		printk("RX setup failed, err %d\n", err);
		return 0;
	}

	printk("\n----------------------\n\n");	

	/* return to idle thread */
	return 0;
}
