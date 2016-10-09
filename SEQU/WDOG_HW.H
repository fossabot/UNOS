/*
***************************************************************************

		Watch Dog Hardware Definitions for
			wdog.c
			by
		L. J. Sciacca

    Date: 4-Oct-1991

    Latest: LJS 4-Oct-1991 Creation

***************************************************************************
*/

#define IPC_CARD_WDT

#ifdef IPC_CARD_WDT

#define ENABLE_WDT 0x443
#define DISABLE_WDT 0x043

#else
#define WDOG_BASE	0x330
#define WDOG_COMM_BASE 	0x2E8

#define WATCHDOG_CLOCK_FREQ	 225.0		/* Hz */
#define WATCHDOG_PERIOD		1		/* second */
#endif


