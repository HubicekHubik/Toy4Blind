/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
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
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/sys/byteorder.h>
#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif
#include "radio_utils.h"
#include "device_info.h"
#define FLAG_END_OF_TEST 0x01

int err;
int8_t tx_power;
int8_t tx_fem_power;
int8_t min_tx_power;
int8_t max_tx_power;
int8_t base_tx_power; // Base TX Power without FEM
int8_t calculated_tx_power; // FEM TX Power
int8_t fem_tx_power_overwrite; // FEM TX Power
int8_t bitrate;

bool repeatModes;
bool firstRun;
int8_t repeat_tx_power;

// Version Information
static int buildversion=3;
static const char *board_id = "PTX";

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

void set_tx_power_in_payload(struct esb_payload *payload, int8_t dbm) {
	if (payload == NULL || payload->length < 3) {
		// Zorg dat er minstens 3 bytes zijn
		return;
	}
	payload->data[2] = (uint8_t)dbm;
}

LOG_MODULE_REGISTER(esb_ptx, CONFIG_ESB_PTX_APP_LOG_LEVEL);

static bool ready = true;
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00);

#define _RADIO_SHORTS_COMMON                                                   \
	(RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |         \
	 RADIO_SHORTS_ADDRESS_RSSISTART_Msk |                                  \
	 RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		printk("Sending: TX SUCCESS\n");
		break;
	case ESB_EVENT_TX_FAILED:
		if (firstRun){
			printk("\nTX failed. Is the PRX running? Check frequency and bitrate.\n\n");
		}
		else{
			printk("Sending: TX FAILED\n");
		}
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&rx_payload) == 0) {
			/*
			LOG_DBG("xxxxPacket received, len %d : "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x, "
				"0x%02x, 0x%02x, 0x%02x, 0x%02x",
				rx_payload.length, rx_payload.data[0],
				rx_payload.data[1], rx_payload.data[2],
				rx_payload.data[3], rx_payload.data[4],
				rx_payload.data[5], rx_payload.data[6],
				rx_payload.data[7]);

			*/
			LOG_DBG("ACK payload received on PTX, len: %d", rx_payload.length);
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
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
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
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err == -EAGAIN);

	nrf_lrcconf_clock_always_run_force_set(NRF_LRCCONF000, 0, true);
	nrf_lrcconf_task_trigger(NRF_LRCCONF000, NRF_LRCCONF_TASK_CLKSTART_0);

	LOG_DBG("HF clock started");
	return 0;
}

#else
BUILD_ASSERT(false, "No Clock Control driver");
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
	config.retransmit_delay = 600;
	config.bitrate = bitrate;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = true;
	config.tx_output_power = 8;
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

static void leds_update(uint8_t value)
{
	uint32_t leds_mask =
		(!(value % 8 > 0 && value % 8 <= 4) ? DK_LED1_MSK : 0) |
		(!(value % 8 > 1 && value % 8 <= 5) ? DK_LED2_MSK : 0) |
		(!(value % 8 > 2 && value % 8 <= 6) ? DK_LED3_MSK : 0) |
		(!(value % 8 > 3) ? DK_LED4_MSK : 0);

	dk_set_leds(leds_mask);
}

void radio_power_enhance(void)
{
#if defined(NRF_VREQCTRL)
	LOG_INF("\nIncrease Radio TX Power: Enabling VREGRADIO High Voltage Mode\n");
	NRF_VREQCTRL->VREGRADIO.VREQH = 1;
	while(!NRF_VREQCTRL->VREGRADIO.VREQHREADY);	
	LOG_INF("VREQHREADY: %s\n\n", NRF_VREQCTRL->VREGRADIO.VREQHREADY ? "true" : "false");
	LOG_INF("RADIO regulator is now operating at high voltage (1.9V).\n");
#else
	LOG_INF("High voltage radio mode not supported on this device.\n");
#endif
}

int main(void)
{
	// Default TX Power settings
	min_tx_power = 0;   // -40dBm -> 20dBm
	max_tx_power = 8;	// -40dBm -> 20dBm
	fem_tx_power_overwrite = 30; // FEM TX Power overwrite value of ONFIG_MPSL_FEM_NRF21540_TX_GAIN_DB from Kconfig

	bitrate = ESB_BITRATE_1MBPS;  // Default bitrate is 1 Mbps although it can be changed to 2 Mbps

	
	// Set repeat Modes
	repeatModes = true;
	repeat_tx_power = 8;
	
	k_sleep(K_MSEC(1000));
	
	// Application name
	printk("\nEnhanced ShockBurst ptx sample\n");
	printk("TX Performance test and RSSI value\n");
	
	
	// Build information
	printk("\n[%s] Version: %d | Build: %s %s\n", board_id, buildversion, __DATE__, __TIME__);
	printk("Board: %s\n", CONFIG_BOARD);
	
    init_device_info();
    printk("Device ID: %u\n", get_device_id());

	// Set device_id (4 bytes) to data[3..6]
	sys_put_le32(get_device_id(), &tx_payload.data[3]);

	// Set board ID to data[7]
	tx_payload.data[7] = BOARD_ID;

	// Change lengte to: total 9 bytes
	tx_payload.length = 9;

	// Increase Radio TX Power for nRF5340
	#if defined(NRF_VREQCTRL)
		//radio_power_enhance();
	#endif

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
		repeat_tx_power = tx_fem_power;
	}
	#endif /* defined(CONFIG_MPSL_FEM) */
	if (tx_fem_power==0)
	{
		printk("FEM TX support: disabled\n");
	}

	// Log and validate bitrate
	log_and_validate_bitrate(&bitrate);
		
	if (repeatModes)
	{
		printk("Repeat Modes activate on (TX gain: %d dBm)\n\n", repeat_tx_power);
		tx_power = repeat_tx_power;
		firstRun = true;
	}
	else
	{
		printk("Repeat Modes: not active\n\n");
		tx_power = min_tx_power;
	}

	err = clocks_start();
	if (err) {
		return 0;
	}

	err = dk_leds_init();
	if (err) {
		LOG_ERR("LEDs initialization failed, err %d\n", err);
		return 0;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d\n", err);
		return 0;
	}

	printk("Initialization complete\n");
	printk("Sending test packet\n");

	printk("\n----------------------\n\n");	

	k_sleep(K_MSEC(1000));

	tx_payload.noack = false;
	while (1) 
	{
		if (ready) 
		{
			ready = false;
			esb_flush_tx();
			leds_update(tx_payload.data[1]);

			// Power Repeat Test Mode
			if (repeatModes)
			{
				if (firstRun)
				{
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
					set_tx_power_in_payload(&tx_payload, tx_power);
				}

				err = esb_write_payload(&tx_payload);
				if (err) {
					LOG_ERR("Payload write failed, err %d\n", err);
				}

				if (firstRun)
				{
					k_sleep(K_MSEC(10));
					
					base_tx_power = get_current_tx_power_dbm();
					if (tx_fem_power > 0) calculated_tx_power = tx_fem_power - base_tx_power; // Calculate FEM TX Power
					else calculated_tx_power = 0; // No FEM TX Power 	
					printk("Base TX-Power = %d dBm, FEM TX-Power = %d dBm, Totaal TX-Power = %d dBm \n", base_tx_power,calculated_tx_power,base_tx_power+calculated_tx_power);
					
					firstRun= false;
					printk("Repeat modes active\n\n");
				}

				tx_payload.data[1]++;
				tx_payload.data[8] = 0x00; // Geen vlag
				//k_sleep(K_USEC(40));
			}
			
			// Power Range Test Mode
			else
			{
				if (tx_power_supported(tx_power)) 
			{
					// Calculation of TX Power is not always correct!!!!!!!!!!!
					
					esb_set_tx_power(tx_power+tx_fem_power);
					printk("\nSet TX power to: %d dBm (0x%02X)\n", tx_power, (uint8_t)tx_power);
					set_tx_power_in_payload(&tx_payload, tx_power+tx_fem_power);

					err = esb_write_payload(&tx_payload);
					if (err) {
						LOG_ERR("Payload write failed, err %d\n", err);
					}

					tx_payload.data[1]++;
					k_sleep(K_MSEC(10));

					base_tx_power = get_current_tx_power_dbm();
					if (tx_fem_power > 0) calculated_tx_power = tx_fem_power - base_tx_power; // Calculate FEM TX Power
					else calculated_tx_power = 0; // No FEM TX Power 						
					printk("Base TX-Power = %d dBm, FEM TX-Power = %d dBm, Totaal TX-Power = %d dBm \n", base_tx_power,calculated_tx_power,base_tx_power+calculated_tx_power);
				} 
				else 
				{
					printk("\nBase TX power %d dBm is NOT supported.\n", tx_power);
					ready = true;
					k_sleep(K_MSEC(100));
				}

				// set power to new value
				tx_power++;
				
				if (tx_power > max_tx_power)
				{
					tx_payload.data[8] = FLAG_END_OF_TEST;

					err = esb_write_payload(&tx_payload);
					if (err) {
						LOG_ERR("Payload write failed, err %d\n", err);
						}
					k_sleep(K_MSEC(10));				
					printk("\nEnd of test\n\n");
					break;	
				}
				else 
				{
					tx_payload.data[8] = 0x00; // Geen vlag
				}
			}			
			k_sleep(K_MSEC(100));
		}
	}
}
