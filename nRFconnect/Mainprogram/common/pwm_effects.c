/*Modules used for I2S and PWM*/
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>

#include "pwm_effects.h"

#define PWM_PERIOD_USEC 20000  // 20 ms perioda (50 Hz)
#define EFFECT_DURATION_MS 600 
void effect_alternate(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 0 ON
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);                    // Motor 1 OFF
		k_sleep(K_MSEC(250));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);                    // Motor 0 OFF
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0); // Motor 1 ON
		k_sleep(K_MSEC(250));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_pulse_both(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {
		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
		k_sleep(K_MSEC(200));

		pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
		pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
		k_sleep(K_MSEC(150));
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_fade(const struct device *pwm_dev) {
	int64_t start = k_uptime_get();
	uint32_t duty;
	while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {
		for (duty = 0; duty <= PWM_PERIOD_USEC / 2; duty += 1000) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
		for (duty = PWM_PERIOD_USEC / 2; duty >= 200; duty -= 1000) {
			pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
			pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
			k_sleep(K_MSEC(50));
		}
	}
	pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
	pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_ping_pong(const struct device *pwm_dev) {
    int64_t start = k_uptime_get();
    uint32_t duty = PWM_PERIOD_USEC * 0.7; 

    while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {
        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
        k_sleep(K_MSEC(150));

        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
        k_sleep(K_MSEC(150));
    }

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_rumble_wave(const struct device *pwm_dev) {
    int64_t start = k_uptime_get();
    uint32_t duty;
    uint32_t step = PWM_PERIOD_USEC / 5;

    while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {

        for (duty = PWM_PERIOD_USEC * 0.2; duty <= PWM_PERIOD_USEC * 0.9; duty += step) {
            pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty, 0);
            pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty, 0);
            k_sleep(K_MSEC(50));
        }
        

        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
        k_sleep(K_MSEC(300));
    }

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_heartbeat(const struct device *pwm_dev) {
    int64_t start = k_uptime_get();
    
    uint32_t strong_duty = PWM_PERIOD_USEC * 0.85;
    uint32_t soft_duty = PWM_PERIOD_USEC * 0.50;

    while ((k_uptime_get() - start < EFFECT_DURATION_MS)) {
        
        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, strong_duty, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, strong_duty, 0);
        k_sleep(K_MSEC(80));
        
        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
        
        k_sleep(K_MSEC(120));

        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, soft_duty, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, soft_duty, 0);
        k_sleep(K_MSEC(60));
        
        pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
        pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
        
        k_sleep(K_MSEC(200)); 
    }
}

void effect_docked(const struct device *pwm_dev) {
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
    k_sleep(K_MSEC(150));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);

    k_sleep(K_MSEC(80));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC / 2, 0);
    k_sleep(K_MSEC(120));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_click(const struct device *pwm_dev) {
    printk("Click effect\n");
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC * 4 / 5, 0);
    k_sleep(K_MSEC(60));
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
}

void effect_connected(const struct device *pwm_dev) {
    uint32_t duty_low = PWM_PERIOD_USEC * 0.6;
    uint32_t duty_high = PWM_PERIOD_USEC * 0.8;

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty_high, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty_high, 0);
    k_sleep(K_MSEC(60));
    
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
    k_sleep(K_MSEC(80));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, duty_low, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, duty_low, 0);
    k_sleep(K_MSEC(140));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}

void effect_grad(const struct device *pwm_dev){

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC/4, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC/4, 0);
    k_sleep(K_MSEC(200));
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
    k_sleep(K_MSEC(200));
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC/2, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC/2, 0);
    k_sleep(K_MSEC(200));
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
    k_sleep(K_MSEC(200));
    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, PWM_PERIOD_USEC * 3 / 4, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, PWM_PERIOD_USEC * 3 / 4, 0);
    k_sleep(K_MSEC(200));

    pwm_set(pwm_dev, 0, PWM_PERIOD_USEC, 0, 0);
    pwm_set(pwm_dev, 1, PWM_PERIOD_USEC, 0, 0);
}