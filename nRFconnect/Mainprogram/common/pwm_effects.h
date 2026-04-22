#ifndef PWM_EFFECTS_H_
#define PWM_EFFECTS_H_
#include <zephyr/device.h>
/*
Definition of the effects used by pwm 
*/

/**Fading effect both motors are active
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_fade(const struct device *pwm_dev);

/**Alternate effect uses boath motors and rings as phone
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_alternate(const struct device *pwm_dev);

/**Pulse effect fast switching motor 1 and 2
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_pulse_both(const struct device *pwm_dev);

/**Fading effect only motor 2 is active
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_rumble_wave(const struct device *pwm_dev);

/**Rumbling effect
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_ping_pong(const struct device *pwm_dev);

/**Ping pong effect
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_grad(const struct device *pwm_dev);
/**Effect both motors are active 
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_docked(const struct device *pwm_dev);
/**Effect only one motor is active resonse to button press
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_click(const struct device *pwm_dev);

void effect_connected(const struct device *pwm_dev);

void effect_heartbeat(const struct device *pwm_dev);
#endif