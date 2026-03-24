#include "but_modul.h"
#include "ble_modul.h"
#include "toy_utils.h"
#include "haptic_modul.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(but_modul, CONFIG_LOG_DEFAULT_LEVEL);

#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <math.h>

/*modules for the Buttons*/
#define BUTTONS_NODE DT_PATH(buttons)
#define BUTTON_SPEC(node_id) GPIO_DT_SPEC_GET(node_id, gpios)

static const struct gpio_dt_spec buttons[] = {
    	DT_FOREACH_CHILD_SEP(BUTTONS_NODE, BUTTON_SPEC, (,))
};

#define BUTTON_COUNT ARRAY_SIZE(buttons)

static struct gpio_callback cb_data_p0; //Callback-data for channel 0 refers to gpio0
static struct gpio_callback cb_data_p1; //Callback-data for channel 1 refers ti goio1

#define DEBOUNCE_MS 1

void button_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

#define BUTTONS_PRIORITY 12
#define BUTTONS_THREAD_STACK_SIZE 4096

K_THREAD_STACK_DEFINE(buttons_stack_area, BUTTONS_THREAD_STACK_SIZE);
static struct k_thread buttons_thread_data;

volatile bool btn_pressed[5] = {false, false, false, false, false};

void read_buttons(void *arg1, void *arg2, void *arg3);

int but_modul_init(void) {
    k_thread_create(&buttons_thread_data, buttons_stack_area,
    K_THREAD_STACK_SIZEOF(buttons_stack_area),
    read_buttons,
    NULL, NULL, NULL,
    BUTTONS_PRIORITY, 0, K_NO_WAIT);
    return 0;
}

void read_buttons(void *arg1, void *arg2, void *arg3) {
    int ret;
    gpio_port_pins_t mask_gpio0 = 0;
    gpio_port_pins_t mask_gpio1 = 0;

    // Configure buttons as input with pull-up
	for (size_t i = 0; i < BUTTON_COUNT; i++) {
		if (!device_is_ready(buttons[i].port)) {
			LOG_ERR("Button device %s is not ready", buttons[i].port->name);
			return;
		}

		ret = gpio_pin_configure_dt(&buttons[i], GPIO_INPUT);
		if (ret){
			LOG_ERR("Something went wrong initilising gpio %d",ret);
			return;
		}

		ret = gpio_pin_interrupt_configure_dt(&buttons[i], GPIO_INT_EDGE_BOTH);
		if (ret){
			LOG_ERR("Something went wrong initilising gpio %d",ret);
			return;
		}

        if (buttons[i].port ==
            DEVICE_DT_GET(DT_NODELABEL(gpio0))) {

            mask_gpio0 |= BIT(buttons[i].pin);

        } else if (buttons[i].port ==
            DEVICE_DT_GET(DT_NODELABEL(gpio1))) {

            mask_gpio1 |= BIT(buttons[i].pin);
        }
	}

    // Set callback handlers
    gpio_init_callback(&cb_data_p0, button_changed, mask_gpio0);
    gpio_add_callback(DEVICE_DT_GET(DT_NODELABEL(gpio0)), &cb_data_p0);

    gpio_init_callback(&cb_data_p1, button_changed, mask_gpio1);
    gpio_add_callback(DEVICE_DT_GET(DT_NODELABEL(gpio1)), &cb_data_p1);

    LOG_INF("Buttons initialized");

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

void button_changed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    int index = 0;
	bool pressed = false;
    // Zjistí, které tlačítko vyvolalo interrupt
	for (size_t i = 0; i < BUTTON_COUNT; i++) {
		if (pins & BIT(buttons[i].pin)) {
			index = i;
			pressed = gpio_pin_get(dev, buttons[i].pin);
			btn_pressed[i] = pressed;
			break;
		}
	}

    static int64_t last_time[BUTTON_COUNT] = {0};
    static int64_t press_start[BUTTON_COUNT] = {0};

    int64_t now = k_uptime_get();
    //if (now - last_time[index] < DEBOUNCE_MS) return;
    last_time[index] = now;

	struct but_ev button_msg; 

	struct toy_events tx_lmr_e = {0};

	if (index == 2) {

        if (pressed && press_start[index] == 0) {
            // just pressed
            press_start[index] = now;
        }

        if (!pressed) {
			int time = now - press_start[index];
            if (time >= 3000) {
                LOG_INF("LONG PRESS detected on button 3 (3s) lets search for toy\n");
				button_msg.cmd = MSG_TYPE_TOY_SWITCH;
				k_msgq_put(&but2ble_q, &button_msg, K_NO_WAIT);
				LOG_INF("Button %d released\n", index + 1);
				btn_pressed[index] = false;
				press_start[index] = 0;
				return;
            }
		    press_start[index] = 0;
        }
    }

    if (!pressed) {
        LOG_INF("Button %d released\n", index + 1);
		switch (index)
		{
		case 0:
			button_msg.cmd=MSG_TYPE_SOUNDSET;
			break;
		case 1:
			button_msg.cmd=MT_BT_DISCONNECT;
			break;
		case 2:
			act_toy_idx++;
			act_toy_idx%=CONFIG_BT_MAX_CONN;
			LOG_INF("Current act_toy_idx = %d", act_toy_idx);
			return;
		case 3:
			button_msg.cmd=MT_VOL_UP;
			break;
		case 4:
			button_msg.cmd=MT_VOL_DOWN;
			break;
		default:
			break;
		}
		k_msgq_put(&but2ble_q, &button_msg, K_NO_WAIT);
		btn_pressed[index] = false;
    } else {
        LOG_INF("Button %d pressed\n", index + 1);
		tx_lmr_e.payload.lmr.effect = CLICK_EFECT;
		k_msgq_put(&lmr_msgq, &tx_lmr_e, K_NO_WAIT);
    }
}
