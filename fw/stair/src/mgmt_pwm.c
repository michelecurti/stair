#include "mgmt_hw.h"

static uint8_t max_pwm, max_hw_pwm;
static uint8_t step_incr = 3;
static uint8_t step_decr = 1;

static uint8_t adapt_step(uint8_t step)
{
	uint8_t res;

	res = (255 - step) * max_hw_pwm / 255 / 4;
	return res == 0 ? 1 : res;
}

void mgmt_pwm_init(void)
{
	max_pwm = max_hw_pwm = mgmt_hw_get_max_pwm();
}

/* adjust maximum brightness from 50% to 100% */
void mgmt_pwm_set_max(uint8_t max)
{
	max_pwm = max_hw_pwm - max_hw_pwm * (255 - max) / 2 / 255;
}

/* set ON ramp speed */
void mgmt_pwm_set_incr_speed(uint8_t speed)
{
	step_incr = adapt_step(speed);
}

/* set OFF ramp speed */
void mgmt_pwm_set_decr_speed(uint8_t speed)
{
	step_decr = adapt_step(speed);
}

/* increase PWM of a led */
void mgmt_pwm_increase(uint8_t led)
{
	if (pwms[led] + step_incr > max_pwm)
	{
		pwms[led] = max_pwm;
	}
	else
	{
		pwms[led] += step_incr;
	}
}

/* decrease PWM of a led */
void mgmt_pwm_decrease(uint8_t led)
{
	if (pwms[led] >= step_decr)
	{
		pwms[led] -= step_decr;
	}
	else
	{
		pwms[led] = 0;
	}
}

/* set PWM value */
void mgmt_pwm_set_led(uint8_t led, uint8_t pwm)
{
	pwms[led] = max_hw_pwm * pwm / 255;
}
