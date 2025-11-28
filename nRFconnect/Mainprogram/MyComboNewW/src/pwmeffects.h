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
void effect_fade_left(const struct device *pwm_dev);

/**Fading effect only motor 1 is active 
 * 
 * @param pwm_dev device reference from a devicetree node identifier.
*/
void effect_fade_right(const struct device *pwm_dev);