#ifndef __MGMT_HW_H
#define __MGMT_HW_H

#include <stdint.h>

#define LEDS_CNT		(17)
#define BTNS_CNT		(2)

void mgmt_hw_init(void);
void mgmt_hw_fsm(void);

uint8_t mgmt_hw_get_max_pwm(void);

uint8_t mgmt_hw_get_button(uint8_t id);
uint32_t mgmt_hw_get_app_tick(void);

void mgmt_hw_led_status(uint8_t state);

enum mgmt_hw_adc_e
{
	ADC_STAIR_TIME,
	ADC_STEP_TIME,
	ADC_RAMP_ON,
	ADC_RAMP_OFF,
	ADC_PWM_MAX,
};

uint8_t mgmt_hw_get_adc(enum mgmt_hw_adc_e idx);

void mgmt_hw_set_relay(uint8_t state);

extern uint32_t pwms[LEDS_CNT];

#endif
