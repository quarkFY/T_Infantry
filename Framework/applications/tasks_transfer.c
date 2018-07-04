#include "tasks_transfer.h"
#include "stdint.h"
#include "tasks_motor.h"
#include "cmsis_os.h"
#include "pid_regulator.h"
#include "stdlib.h"
#include "utilities_debug.h"
#include "drivers_uartupper_user.h"
#include "drivers_canmotor_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_arm.h"
#include "peripheral_sov.h"
#include "tasks_motor.h"
#include "drivers_i2c.h"

#include <stdlib.h>
#include <math.h>

int tmpFlag=0;

void TransferTask(void const * argument)
{
	while(1)
		
	{
	 if(tmpFlag)
	 {
		 i2c_start();
		 tmpFlag =0;
	 }

	}
}

