#include "FreeRTOS.h"
#include "task.h"
#include "hacs_app.h"
#include "hacs_platform.h"

int main(void)
{
	/* Platform init */
	hacs_platform_init();

	/* Application init */
	hacs_app_init();

	/* Start the scheduler; Never return */
	vTaskStartScheduler();

	/* Should never get here */
	return -1;
}
