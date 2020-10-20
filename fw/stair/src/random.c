#include <stdlib.h>
#include "mgmt_hw.h"
#include "mgmt_pwm.h"
#include "random.h"

#define STEPS	100
#define TICKS	20

enum fsm_e
{
	FSM_IDLE,
	FSM_RUN,
	FSM_RESET,
};

static uint32_t curr_step = STEPS;
static enum fsm_e fsm_state;

void random_start(void)
{
	fsm_state = FSM_RUN;
}

void random_fsm(void)
{
	int i;
	static uint32_t old_tick;
	uint32_t tick;
	static uint32_t cnt;

	tick = mgmt_hw_get_app_tick();

	if (tick == old_tick)
	{
		return;
	}

	old_tick = tick;

	if (++cnt < TICKS)
	{
		return;
	}

	cnt = 0;

	switch (fsm_state)
	{
	case FSM_IDLE:
		break;
	case FSM_RUN:
		mgmt_pwm_set_led(rand() % LEDS_CNT, 255);

		if (--curr_step == 0)
		{
			fsm_state = FSM_RESET;
		}

		break;

	case FSM_RESET:
		for (i = 0; i < LEDS_CNT; i++)
		{
			//mgmt_pwm_set_led(i, 0);
		}

		curr_step = STEPS;
		break;
	}
}
