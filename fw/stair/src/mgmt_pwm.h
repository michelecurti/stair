#ifndef __MGMT_PWM_H
#define __MGMT_PWM_H

#include <stdint.h>

void mgmt_pwm_init(void);
void mgmt_pwm_timer_irq(void);
void mgmt_pwm_set_max(uint8_t max);
void mgmt_pwm_set_incr_speed(uint8_t speed);
void mgmt_pwm_set_decr_speed(uint8_t speed);
void mgmt_pwm_increase(uint8_t led);
void mgmt_pwm_decrease(uint8_t led);
void mgmt_pwm_set_led(uint8_t led, uint8_t pwm);
#endif
