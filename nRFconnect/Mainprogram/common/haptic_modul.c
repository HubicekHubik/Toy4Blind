#include "haptic_modul.h"
#include "toy_utils.h"
#include "app_state.h"
#include "pwm_effects.h"
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(haptic_modul, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/drivers/pwm.h>
#define PWM0_NODE DT_NODELABEL(pwm0)

#define LMR_EVENT_MSGQ_LENGTH 32
K_MSGQ_DEFINE(lmr_msgq, sizeof(struct toy_events), LMR_EVENT_MSGQ_LENGTH, 4);

#define PWM_THREAD_PRIORITY 11
#define LMRPWM_THREAD_STACK_SIZE 1024

K_THREAD_STACK_DEFINE(lmrPwm_stack_area, LMRPWM_THREAD_STACK_SIZE);
static struct k_thread lmrPwm_thread_data;

void LMR_control_thread(void *arg1, void *arg2, void *arg3);

int haptic_modul_init() {

	k_thread_create(&lmrPwm_thread_data, lmrPwm_stack_area,
					K_THREAD_STACK_SIZEOF(lmrPwm_stack_area),
					LMR_control_thread,
					NULL, NULL, NULL,
					PWM_THREAD_PRIORITY, 0, K_NO_WAIT);

    return 0;
}

void LMR_control_thread(void *arg1, void *arg2, void *arg3) {
	const struct device *pwm0_dev = DEVICE_DT_GET(PWM0_NODE);
	
	struct toy_events rx_lmr_e;

	if (!device_is_ready(pwm0_dev))
	{
		LOG_ERR("PWM0 device not ready\n");
	}
	else
	{
		LOG_INF("LMR si ready\n");
	}
	while (true) {
		//k_sem_take(&run_sem, K_FOREVER); // pokud je OFF, vlákno se zastaví
    	//k_sem_give(&run_sem);
		if (k_msgq_get(&lmr_msgq, &rx_lmr_e, K_FOREVER) == 0) {
			LOG_INF("LMR effect recieved: %d and time is %d", rx_lmr_e.payload.lmr.effect, rx_lmr_e.payload.lmr.ms_duration);
			if (rx_lmr_e.type != MT_LMR) continue;
			switch (rx_lmr_e.payload.lmr.effect) {
				case 1:
					effect_alternate(pwm0_dev);
					break;
				case 2:
					effect_pulse_both(pwm0_dev);
					break;
				case 3:
					effect_fade(pwm0_dev);
					break;
				case 4:
					effect_fade_right(pwm0_dev);
					break;
				case 5:
					effect_fade_left(pwm0_dev);
					break;
				case 6:
					effect_grad(pwm0_dev);
					break;
				case CHARGING_EFFECT:
					effect_docked(pwm0_dev);
					break;
				case CLICK_EFECT:
					effect_click(pwm0_dev);
					break;
				case CONNECTED_EFFECT:
					LOG_INF("Playing conncted effect");
					effect_connected(pwm0_dev);
					break;
				default:
					break;
			}
		}
	}
}
