#include "mgmt_hw.h"
#include "mgmt_pwm.h"
#include "app.h"

#define RAMP_FACTOR		(4)
#define STAIR_TIME_MIN		(10 * APP_TICK_FREQ)
#define STAIR_TIME_MAX		(120 * APP_TICK_FREQ)
#define STEP_TIME_MIN		(APP_TICK_FREQ / 2)
#define STEP_TIME_MAX		(2 * APP_TICK_FREQ)

#define RELAY_TIME		(5 * 60 * APP_TICK_FREQ)

struct button
{
	uint8_t		state;
	uint8_t		old_state;
	uint8_t		curr_led;
	uint16_t	counter;
};

static struct button btns[BTNS_CNT];
static uint32_t counters[LEDS_CNT];

static uint32_t stair_time;
static uint16_t step_time;
static uint32_t relay_time;

static void update_parameters(void)
{
	uint8_t v;

	/* stair time */
	v = mgmt_hw_get_adc(ADC_STAIR_TIME);
	stair_time = STAIR_TIME_MIN + (STAIR_TIME_MAX - STAIR_TIME_MIN) * v / 255;

	/* step time */
	v = mgmt_hw_get_adc(ADC_STEP_TIME);
	step_time = STEP_TIME_MIN + (STEP_TIME_MAX - STEP_TIME_MIN) * v / 255;

	/* ramp ON speed */
	v = mgmt_hw_get_adc(ADC_RAMP_ON);
	mgmt_pwm_set_incr_speed(v);

	/* ramp OFF speed */
	v = mgmt_hw_get_adc(ADC_RAMP_OFF);
	mgmt_pwm_set_decr_speed(v);

	/* maximum PWM */
	v = mgmt_hw_get_adc(ADC_PWM_MAX);
	mgmt_pwm_set_max(v);
}

static void led_action(uint8_t btn, uint8_t led)
{
	switch (btn)
	{
	case 0:
		counters[led] = stair_time;
		break;

	case 1:
		counters[LEDS_CNT - led - 1] = stair_time;
		break;

	default:
		break;
	}
}

void app_init(void)
{

}

void app_fsm(void)
{
	uint8_t i;
	uint32_t tick;
	static uint32_t old_tick, led_cnt;

	tick = mgmt_hw_get_app_tick();

	if (tick == old_tick)
	{
		return;
	}

	old_tick = tick;

	/* signal that application is running */
	mgmt_hw_led_status((tick >> 6) & 0x01);

	/* update application parameters from ADC */
	update_parameters();

	for (i = 0; i < BTNS_CNT; i++)
	{
		/* read new button state */
		btns[i].state = mgmt_hw_get_button(i);

		if (btns[i].state && !btns[i].old_state)
		{
			/* the button is pressed, start the sequence */
			btns[i].curr_led = LEDS_CNT;

			/*
			 * set delay to 1, so the first action is executed
			 * immediately
			 */
			btns[i].counter = 1;

			/* switch ON the relay */
			relay_time = RELAY_TIME;
			mgmt_hw_set_relay(1);
		}

		if (btns[i].curr_led > 0 && btns[i].counter > 0)
		{
			if (--btns[i].counter == 0)
			{
				btns[i].curr_led--;

				/* execute current led action */
				led_action(i, btns[i].curr_led);

				/*
				 * if there are other leds to manage,
				 * schedule the next action
				 */
				if (btns[i].curr_led > 0)
				{
					btns[i].counter = step_time;
				}
			}
		}

		btns[i].old_state = btns[i].state;
	}

	/* switch OFF the rele when in stnadby */
	if (relay_time > 0)
	{
		if (--relay_time == 0)
		{
			mgmt_hw_set_relay(0);
		}
	}

	led_cnt++;

	/*
	 * When counter is greater than 0, the led is gradually switched ON.
	 * When counter is 0, the led is gradually switched OFF.
	 */
	for (i = 0; i < LEDS_CNT; i++)
	{
		if (counters[i] > 0)
		{
			relay_time = RELAY_TIME;
			counters[i]--;

			if (led_cnt % RAMP_FACTOR == 0)
			{
				mgmt_pwm_increase(i);
			}
		}
		else
		{
			if (led_cnt % RAMP_FACTOR == 0)
			{
				mgmt_pwm_decrease(i);
			}
		}
	}
}
