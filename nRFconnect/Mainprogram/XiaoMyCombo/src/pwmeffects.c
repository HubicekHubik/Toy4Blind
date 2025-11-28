/*Modules used for I2S and PWM*/
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>

#include "pwmeffects.h"

#define PWM_PERIOD_USEC 20000  // 20 ms perioda (50 Hz)

void effect_alternate(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start < 3000)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 0 ON
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);                    // Motor 1 OFF
		k_sleep(K_MSEC(300));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);                    // Motor 0 OFF
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 1 ON
		k_sleep(K_MSEC(300));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_pulse_both(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start < 3000)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		k_sleep(K_MSEC(150));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
		k_sleep(K_MSEC(100));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	uint32_t duty;
	while ((k_uptime_get() - start < 3000)) {
		// Fade in
		for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
		// Fade out
		for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade_left(const struct device *pwm_dev)
{
    int64_t start = k_uptime_get();
    uint32_t duty;

    while ((k_uptime_get() - start < 3000)) {

        /* Fade-in */
        for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
            pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }

        /* Fade-out */
        for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
            pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }
    }

    /* stop – obě strany ticho */
    pwm_set(pwm_dev, 0,  PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade_right(const struct device *pwm_dev)
{
    int64_t start = k_uptime_get();
    uint32_t duty;

    while ((k_uptime_get() - start < 3000)) {

        /* Fade-in */
        for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 200) {
            pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }

        /* Fade-out */
        for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 200) {
            pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0,    0);
            k_sleep(K_MSEC(50));
        }
    }

    /* stop – obě strany ticho */
    pwm_set(pwm_dev, 1,  PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
}
