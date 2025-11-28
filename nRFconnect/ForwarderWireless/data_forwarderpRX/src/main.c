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
const int8_t DEVICE_PREFIX = 0xA2;

#define Q_BLOCK_SIZE (sizeof(float) * 3)
#define Q_LENGTH 800

K_MSGQ_DEFINE(imuQueue, Q_BLOCK_SIZE, Q_LENGTH, 4);

// Parse thread part
#define AXIS_COUNT 3
#define SAMPLES_PER_PKT 1

/* nastav podle tvého formátu: pokud posíláš int16_t per axis, použ int16_t */
typedef struct {
    float ax;
    float ay;
    float az;
} sample_t;

/* maximální počet samplů, které chceme pojmout v bufferu (např. 200 ~ 4s při 50Hz) */
//#define SAMPLE_Q_LEN  800
//K_MSGQ_DEFINE(sampleQueue, sizeof(sample_t), SAMPLE_Q_LEN, 4);
/* imuQueue už máš - obsahuje celé payloady (např. 4*6B) */
void parser_thread(void *arg1, void *arg2, void *arg3);

//K_THREAD_DEFINE(parser_thread_id, 1024, parser_thread, NULL, NULL, NULL,
//                3, 0, 0);

//static const int64_t sampling_freq = 52; // in Hz.
//static int64_t time_between_samples_us = (1000000 / (sampling_freq - 1));
#define SAMPLE_INTERVAL_MS 20 // 50 Hz

#define LOG_DEBUG_DEF 4

LOG_MODULE_REGISTER(esb_prx, LOG_DEBUG_DEF);

void event_handler(struct esb_evt const *event);

static struct esb_payload rx_payload;

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

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return 0;
	}

	LOG_INF("Start of te read and write part of program");

    int64_t next = k_uptime_get();
    sample_t s;

    while (1) {
        // start a timer that expires when we need to grab the next value
        next += SAMPLE_INTERVAL_MS;
		/*
		struct k_timer next_val_timer;
        k_timer_init(&next_val_timer, NULL, NULL);
        k_timer_start(&next_val_timer, K_USEC(time_between_samples_us), K_NO_WAIT);
		*/
		
        // read data from the Queue
		if (k_msgq_get(&imuQueue, &s, K_FOREVER) == 0) {
			printk("%f\t%f\t%f\r\n",
				s.ax, s.ay, s.az);
		}
        /*printf("%.3f\t%.3f\t%.3f\r\n",
            sensor_value_to_double(&sample_buf[0]),
            sensor_value_to_double(&sample_buf[1]),
            sensor_value_to_double(&sample_buf[2]));
		*/

        // busy loop until next value should be grabbed
        /*int64_t now = k_uptime_get();
        int64_t sleep_ms = next - now;
        if (sleep_ms > 0) {
            k_msleep((uint32_t)sleep_ms);
        } else {
            // nestíháme - nastav next = now aby se vyhnul driftu
            next = now;
        }*/
    }
	return err;
}

void event_handler(struct esb_evt const *event) {
	switch (event->evt_id) {
		case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			//printk("This is the actual size of thy rx_payload:%d B\n", sizeof(rx_payload.data));
			if (k_msgq_put(&imuQueue, rx_payload.data, K_NO_WAIT) != 0) {
				LOG_ERR("IMU queue full! Dropping packet!");
			}
		}
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
	uint8_t addr_prefix[1] = {DEVICE_PREFIX};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.tx_mode  = ESB_TXMODE_AUTO;
	config.tx_output_power = 8;
	config.bitrate = ESB_BITRATE_1MBPS;
	config.event_handler = event_handler;
	config.mode = ESB_MODE_PRX;
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

	err = esb_set_tx_power(27);
	if (err) {
		LOG_ERR("Set TX Power failed");
		return err;
	}
	return err;
}

/*
void parser_thread(void *arg1, void *arg2, void *arg3) {
    float tmp_samples[SAMPLES_PER_PKT][AXIS_COUNT];
	LOG_INF("Start of the parser thread");
    while (1) {
        blokuje dokud nepřijde celý paket z imuQueue
        if (k_msgq_get(&imuQueue, tmp_samples, K_FOREVER) != 0) {
            continue;
        }
        vlož každý sample do sampleQueue
        for (int i = 0; i < SAMPLES_PER_PKT; i++) {
            sample_t s;
            konverze na float (pokud je potřeba)
            s.ax = (float)tmp_samples[i][0]; SCALE např. 1.0 nebo /16384.0
            s.ay = (float)tmp_samples[i][1];
            s.az = (float)tmp_samples[i][2];
			//printf("Imue value parsed: %.3f %.3f %.3f");
            if (k_msgq_put(&sampleQueue, &s, K_NO_WAIT) != 0) {
                queue plná -> rozhodni (drop nejstarší? drop nového?). Zde dropujeme nejnovější.
                LOG_WRN("sampleQueue full, dropping sample");
            }
        }
    }
}*/