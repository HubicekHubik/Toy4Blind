#include "bat_measures_modul.h"
#include "app_state.h"
#include "toy_utils.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bat_measure, CONFIG_LOG_DEFAULT_LEVEL);

/*Power/Battery modules*/
#include <math.h>
#include <stdlib.h>
#include "battery.h"
#include <zephyr/drivers/adc.h>
#define LOW_BATTERY_MV 3300

struct bat_ev {
	uint16_t V_bat;
	uint16_t V_ched;
	uint16_t V_ching;
};

bool charging = false;
bool charged = false;

#define BAT_MEASURE_MS 60000
/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(adc0)

/* Auxiliary macro to obtain channel vref, if available. */
#define CHANNEL_VREF(node_id) DT_PROP_OR(node_id, zephyr_vref_mv, 0)
static const struct device *adc = DEVICE_DT_GET(ADC_NODE);

static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,))};

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)
#define BATTERY_THREAD_STACK_SIZE 1024
#define BATTERY_THREAD_PRIORITY 10

K_THREAD_STACK_DEFINE(battery_stack_area, BATTERY_THREAD_STACK_SIZE);
static struct k_thread battery_thread_data;
static void battery_manager_thread(void *arg1, void *arg2, void *arg3);

void resolve_batery_event(struct bat_ev *pow_inf);

int bat_measure_modul_init() {
    int ret;

	for (size_t i = 0; i < CHANNEL_COUNT; i++)
	{
		if (channel_cfgs[i].channel_id != 7) {
			LOG_INF("Setting up ADC with channel %u!", channel_cfgs[i].channel_id);
			ret = adc_channel_setup(adc, &channel_cfgs[i]);
				if (ret < 0) {
					LOG_INF("ADC channel setup failed: %d\n", ret);
					return ret;
			}
		}
	}

    ret = battery_measure_enable(true);
	if (ret != 0) {
		LOG_INF("Failed initialize battery measurement: %d", ret);
		return ret;
	}

    k_thread_create(&battery_thread_data, battery_stack_area,
                K_THREAD_STACK_SIZEOF(battery_stack_area),
                battery_manager_thread,
                NULL, NULL, NULL,
                BATTERY_THREAD_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

static void battery_manager_thread(void *arg1, void *arg2, void *arg3) {	
	LOG_INF("Battery mesure thread started");
    int ret;
	uint16_t sample_buffer;

	struct adc_sequence seq = {
		.buffer      = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution  = 12,
		.oversampling = 8
	};

	struct bat_ev powInf;

	int32_t last_ched = -1;
	int32_t last_ching = -1;

	while (true) {
		bool changed = false;
		for (size_t i = 0; i < CHANNEL_COUNT - 1; i++) {
			seq.channels = BIT(channel_cfgs[i].channel_id);
			ret = adc_read(adc, &seq);
			if (ret == 0) {
				int32_t mv0 = sample_buffer;
				adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_3, 12, &mv0);
				if(i == 0){
					powInf.V_ched = mv0;
				} else {
					powInf.V_ching = mv0;
				} 
			} else {
				LOG_INF("ADC read failed: %d\n", ret);
			}
		
			uint16_t ching_diff = abs(last_ching - powInf.V_ching);
			uint16_t ched_diff = abs(last_ched - powInf.V_ched);
			if ((ching_diff > 400) || (ched_diff > 400)) {
				LOG_INF("Voltaage on pins changed sanding msg %d %d", ching_diff, ched_diff);				
				last_ching = powInf.V_ching;
				last_ched = powInf.V_ched;
				changed = true;
			}
		}
		
		/*battery state reading*/
		static int64_t last_time = -60000;
		int64_t now = k_uptime_get();

		if (now - last_time >= BAT_MEASURE_MS) {
			uint16_t batt_mV = battery_sample();
			powInf.V_bat = batt_mV;
			if (batt_mV < 0) {
				LOG_INF("Failed to read battery voltage: %d", batt_mV);
			}
			last_time = now;
			changed = true;
		}
		
		if (changed) {
            resolve_batery_event(&powInf);
        }

		k_sleep(K_MSEC(1000));
	}
}

void resolve_batery_event(struct bat_ev *pow_inf) {
    int ret;
    struct toy_events tx_bat_state = {
		.type = MT_BAT_INF
	};

	//LOG_INF("Value of V_ching : %d of V_ched : %d of V_bat : %d", pow_inf->V_ching, pow_inf->V_ched, pow_inf->V_bat);
    if ((pow_inf->V_ching == 0) && (pow_inf->V_ched == 0)) {
        LOG_INF("Device not on the charger");
        charging = false;
        charged = false;
    }

    if ((pow_inf->V_ching > 400) && !charging) {
        LOG_INF("Device was placed on charger");
        charging = true;
		tx_bat_state.type = MT_CHARGING;
    }

    if ((pow_inf->V_ched > 600) && (pow_inf->V_ching == 0)) {
        LOG_INF("Device is charged");
        if (!charged) {
			tx_bat_state.type = MT_CHARGED;
            charged = true;
        }
    }

    if ((pow_inf->V_bat < LOW_BATTERY_MV) && !charging) {
        LOG_INF("Battery low warning is device on %d", device_running);
        if(device_running) {
            LOG_INF("Battery low sending low battery warning");
			tx_bat_state.type = MT_LOW_BATERY;
        }
    }
	tx_bat_state.len = sizeof(struct pow_ev);
	tx_bat_state.payload.power.V_bat = pow_inf->V_bat;
	tx_bat_state.payload.power.charging = charging;
	tx_bat_state.payload.power.charged = charged;
	tx_bat_state.payload.power.deviceOn = device_running;
    ret = k_msgq_put(&sysfb_msgq, &tx_bat_state, K_NO_WAIT);
}