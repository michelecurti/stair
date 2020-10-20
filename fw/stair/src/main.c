#include "mgmt_hw.h"
#include "mgmt_pwm.h"
#include "app.h"
#include "random.h"

int main(void)
{
	/* intialize hw */
	mgmt_hw_init();

	/* intialize PWM management */
	mgmt_pwm_init();

	/* intialize application */
	app_init();

	for(;;)
	{
		mgmt_hw_fsm();

		/* manage application */
		app_fsm();


		random_fsm();
	}
}
