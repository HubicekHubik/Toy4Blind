#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

//ESB part
#include <esb.h>
uint8_t addr_prefix[1] = {0xA2};

static const int64_t sampling_freq = 52; // in Hz.
static int64_t time_between_samples_us = (1000000 / (sampling_freq - 1));
#define LOG_DEBUG_DEF 4

LOG_MODULE_REGISTER(esb_prx, LOG_DEBUG_DEF);

void event_handler(struct esb_evt const *event);

int esb_initialize(); //init of ESB

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

	LOG_DBG("HF clock started");
	return 0;
}
#else
BUILD_ASSERT(false, "No Clock Control driver");
#endif /* defined(CONFIG_CLOCK_CONTROL_NRF2) */

int main() {
	int err;
    // output immediately without buffering
    setvbuf(stdout, NULL, _IONBF, 0);

	err = clocks_start();
	if (err) {
		printk("Clock initialization failed, err %d", err);
		return EXIT_FAILURE;
	}

	err = esb_initialize();
	if (err) {
		printk("ESB initialization failed, err %d", err);
		return EXIT_FAILURE;
	}

    // get driver for the accelerometer
	struct sensor_value odr_attr;
	const struct device *lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    if (lsm6dsl_dev == NULL) {
        printf("Could not get lsm6dsl_dev device\n");
        return EXIT_FAILURE;
    }

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 52;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printf("Cannot set sampling frequency for accelerometer.\n");
		return EXIT_FAILURE;
	}

	//int const full=2;
	//int samplebufc=0;
	float sample_buf[3];
	
    struct sensor_value accel[3];
	struct esb_payload tx = {
        .noack  = true,
        .length = sizeof(float) * 3
    };

    while (1) {
        // start a timer that expires when we need to grab the next value
        struct k_timer next_val_timer;
        k_timer_init(&next_val_timer, NULL, NULL);
        k_timer_start(&next_val_timer, K_USEC(time_between_samples_us), K_NO_WAIT);

        // read data from the sensor
        if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
            printf("lsm6dsl_dev Sensor sample update error\n");
            return EXIT_FAILURE;
        }

        sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, accel);

		sample_buf[0] = sensor_value_to_float(&accel[0]);
		sample_buf[1] = sensor_value_to_float(&accel[1]);
		sample_buf[2] = sensor_value_to_float(&accel[2]);
		
		/*printf("TXdata print: %.3f\t%.3f\t%.3f\r\n",
			sample_buf[0], sample_buf[1], sample_buf[2]);
		*/

		//if(samplebufc == full + 1){
			// copy to ESB tx packet
		memcpy(tx.data, sample_buf, sizeof(sample_buf));
		
		err = esb_write_payload(&tx);
		if (err) {
			printk("esb_write_payload failed, err: %d\n", err);
			err = esb_flush_tx();
			if (err) {
				printk("Flush tx failed, err: %d", err);
			}
		}
			//samplebufc = 0;
		//}
        // busy loop until next value should be grabbed
        while (k_timer_status_get(&next_val_timer) <= 0);
    }
	return err;
}

void event_handler(struct esb_evt const *event) {
	switch (event->evt_id) {
		case ESB_EVENT_RX_RECEIVED:
			break;

		case ESB_EVENT_TX_SUCCESS:
			break;

		case ESB_EVENT_TX_FAILED:
			break;
	}
}

int esb_initialize() {
	int err;
	/* These are nonarbitrary default addresses.*/
	uint8_t base_addr_0[4] = {0xF1, 0xF1, 0xF1, 0xF1};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.tx_mode  = ESB_TXMODE_AUTO;
	config.tx_output_power = 8;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PTX;
	config.selective_auto_ack = false;


	if (IS_ENABLED(CONFIG_ESB_FAST_SWITCHING)) {
		config.use_fast_ramp_up = true;
	}

	err = esb_init(&config);
	if (err) {
		return err;
	}

	// Set channel to 80 ( that means 2480 MHz)
	err = esb_set_rf_channel(79);
	if (err) {
		printk("Set RF channel failed");
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return err;
}