/************************************************************************/
/*                                                                                                                                              */
/*                                                              ANALOGUE.C                                                              */
/*                                                                                                                                              */
/*                              Analogue input, for reading the beacon signal                   */
/*                                                                                                                                              */
/*                                                                                                                                              */
/*              Programmer:             P. Moylan                                                                               */
/*              Last modified:          28 July 1992                                                            */
/*              Status:                         Working.                                                                        */
/*                                                                                                                                              */
/*                                                                                                                                              */
/************************************************************************/

/*
Latest: LJS Adapted for JAH-1A project. Mods needed to change error
	handling and beacon simulator tasks.

*/
#include <dos.h>                /* FROM DOS IMPORT disable, enable,                             */
						/*              inportb, outportb                                               */
#include <math.h>               // Import fabs()

#include <stdlib.h>             /* FROM StdLib IMPORT atexit                                    */

#include "hwpc.h"               /* FROM hwpc IMPORT fpvdSetVector                               */
#include "general.h"
#include "analogue.h"
#include "main_ext.h"   // Import get_simulation()

#include <beacon.h>             /* FROM Beacon IMPORT BeaconSamplingInterval    */
#include "simbeaco.h"
						/* FROM SimBeacon IMPORT FetchSimulatorSample,  */
						/*              InitBeaconSimulator, GoSimulator                */
						/*              SetSimulatorChannel                                             */
#include <unosasm.h>    /* FROM unosasm IMPORT return_interrupt_status  */

#include <unos.h>               /* FROM UNOS IMPORT create_semaphore,                   */
						/*              init_semaphore, wait, _signal                   */

#include <nvramext.h>           /* FROM NVRAM IMPORT                                                    */
						/*              get_current_beacon_number                               */
#include "simintel.h"   // Import get_intelsat_bcn ()

/************************************************************************/
/*                                                                                                                                              */
/*                              DEFINITION OF HARDWARE PORTS, ETC.                                              */
/*                                                                                                                                              */
/* Note that IOBase is not the same as the factory default setting.             */
/*                                                                                                                                              */
/************************************************************************/

#define IntMaskRegister         0x21

#define IOBase                  0x310
#define ADinLow                 IOBase
#define ADinHigh                IOBase+1
#define ADgain                  IOBase+1
#define ADscan                  IOBase+2
#define DigitalPort             IOBase+3
#define DAC0Low                 IOBase+4
#define DAC0Hi                  IOBase+5
#define ADstatus                IOBase+8
#define ADcontrol               IOBase+9
#define Base8253                IOBase+12
#define Control8253             Base8253+3

#define SoftwareStart IOBase

#define INTERRUPT_LEVEL         5               /* Normally assigned to printer #2      */

#define INTERRUPT_MASK          (1 << INTERRUPT_LEVEL)

#define InterruptNo             INTERRUPT_LEVEL+8  /* hardware interrupt number */

/* Definitions for various constants used in programming the control    */
/* register for the A/D converter.                                                                                                                                                              */
#define START_CONVERSION_ON_TIMER 0x03
#define INTERRUPT_ENABLE_ON_CONVERSION_COMPLETE 0x80
#define TRIGGER_ENABLE 0x08


/* The timer is driven from a 1MHz clock.               */

#define OSCILLATOR_FREQUENCY            1.0E6

/************************************************************************/
/*                                                                                                                                              */
/*                              CONSTANTS AND VARIABLES GLOBAL TO THIS MODULE                   */
/*                                                                                                                                              */
/************************************************************************/


/* A flag, set during module initialization, to say whether we should   */
/* use the beacon simulator software rather than the A/D hardware.              */

private BOOLEAN Simulating;

/* The number of samples in a collection period.                */

private unsigned int SampleSize = 1;

/* The sum of all data values collected during the current collection   */
/* period.  (A collection period is, in the current version, half a             */
/* second.  At the end of each collection period we supply the average  */
/* value to the caller of FetchSample.)                                                                 */

private long DataSum;

/* Semaphore for synchronism with the data-gathering mechanism.  A              */
/* signal on DataReady means that the current collection period is over */
/* and that the value of DataSum is available to be used.                               */

private unsigned int DataReady;

/* Flag to permit the detection of overrun errors.  DataTaken = TRUE            */
/* means that we have taken a copy of DataSum and that DataSum is ready         */
/* to receive a new value.  DataTaken = FALSE means that a new value            */
/* has been loaded into DataSum but has not been read by the consumer.          */

private BOOLEAN DataTaken = TRUE, DataOverrun = FALSE;

/* Procedure oldhandler is the interrupt routine which was in use at            */
/* system startup.  We restore it on shutdown.                                                          */

private void interrupt (*oldhandler)(void);

/* The current beacon channel number.  This is used by the interrupt            */
/* routine, so we protect critical sections by disabling interrupts.            */

private BYTE BeaconChannel;

/* Remark: the A/D multiplexer is irrelevant to this application.  The          */
/* beacon signal always comes in on A/D channel 0, and the beacon                       */
/* channel is set by writing to DigitalPort: bits 3-0 specify the                       */
/* channel number, and bit 4 is the strobe bit, which must be pulsed            */
/* low to force the beacon receiver to accept the channel number.                       */
/* The receiver must be told the channel number periodically, even if           */
/* it is not changing, or it will forget it (?!).  To guard against                     */
/* things like a channel change in the middle of a conversion, we set           */
/* the channel number inside the interrupt routine.                                                     */


/************************************************************************/
/*                                              EXTERNALLY CALLABLE FUNCTIONS                                   */
/************************************************************************/

private void HardwareSetup (unsigned long count);               /* FORWARD */

/************************************************************************/

void StartBeaconSampling (double frequency)

	/* Starts a timer-controlled analogue sampling operation.  This                 */
    /* function should be called when the beacon task is ready to start         */
	/* collecting data - i.e. after the task has started up.  Parameter             */
	/* frequency is in units of samples per second.                                                 */

	{
	BeaconChannel = get_current_beacon_select ( GetCurrentSatelliteNumber() );

	if ( !Simulating )
		{
		/* A distinction must be drawn here between the sampling                        */
		/* frequency which we use and that used by the Beacon Task.                     */
		/* Each "sample" given to the BeaconTask is in fact the average         */
		/* of SampleSize samples collected by this module.                                      */

		SampleSize = BeaconSamplingInterval*frequency + 0.5;
		HardwareSetup (OSCILLATOR_FREQUENCY / frequency + 0.5);
		}

} // end of StartBeaconSampling

/************************************************************************/

double FetchSample (void)

    /* Waits until the next sample is available, then returns it.  (It          */
    /* is assumed that StartBeaconSampling has been called - otherwise          */
	/* this function will never return.)  Data overrun errors are                   */
	/* reported to the Alarm module, but not to the caller.                                 */

		{
		long sum;

	if ( Simulating ) return FetchSimulatorSample ();

	/* Wait for the end of the current collection period; then copy                 */
	/* the value of DataSum.                                                                                                */

	wait (DataReady);

	if ( DataOverrun)
		{
		DataOverrun = FALSE;
		// fvdErrorSet (BEACON_SAMPLE_OVERRUN);
		}

	sum = DataSum;  DataTaken = TRUE;

	/* Convert the integer value to a real voltage.  The A/D output is              */
    /* offset coded, where 0 means -10V, 2048 means 0V, and 4095 means          */
    /* +9.995V.  In addition we have to account for the fact that we            */
    /* have collected the sum of SampleSize readings.                                           */

    return sum/(204.8*SampleSize) - 10.0;
    }

/************************************************************************/

void SetBeaconChannel (BYTE newchannel)

    /* Change the beacon channel.               */

    {
	if (Simulating)
		{
		SetSimulatorChannel (newchannel);
		}
	else
		{
		BOOLEAN int_status = return_interrupt_status();

	/* All we do here is record the new channel number.  The actual         */
	/* hardware setting is done by the interrupt routine.                           */

	disable();
	BeaconChannel = newchannel;
	if (int_status) enable();

		}
    }

/************************************************************************/
/*                                              THE INTERRUPT ROUTINE                                                   */
/************************************************************************/

private void interrupt InterruptHandler (void)

    /* This procedure is entered each time an A/D conversion is                         */
    /* complete.  The start pulse to the A/D converter comes (in this           */
    /* version of the software) from a free-running 8253 oscillator,            */
	/* therefore the interrupt routine is entered at a frequency                    */
    /* specified when procedure HardwareSetup programs the timer chip.          */

		{
		static long CumulativeTotal = 0;
		static int SamplesToGo = 1;

    /* Read a 16-bit value from the two A/D ports.  The low order four          */
	/* bits hold the channel number, and may be discarded.                                  */

    /* inportb returns an unsigned char, the type conversion from that          */
    /* to a long gives a -ve if the msb is 1. Use type casting instead.         */
	/* Correction added by AB 22/6/92.                                      */

    CumulativeTotal+=(((long)inportb(ADinLow)+256*(long)inportb(ADinHigh))>>4);


    /* Acknowledge the interrupt. */

    outportb (ADstatus, 0);

	/* Re-arm the interrupt controller.  (This produces no risk of          */
	/* this routine being re-entered, since processor interrupts are        */
	/* still disabled.)                                                                                                     */

		outportb (0x20, 0x20);          /* EOI to controller 1 */

	/* Update the sample count, and check whether we are at the end of      */
	/* a collection period.                                                                                         */

    if (!--SamplesToGo)
		{
	/* End of collection period.  Pass the result to the task                       */
		/* which is waiting for it.                                                                             */

		/* Data Overrun has occurred if the consumer hasn't yet                 */
		/* taken the data.                                                                                              */

		DataOverrun = ! DataTaken;

		/* Pulse the "beacon channel strobe" bit low to pass the                */
	/* beacon channel number to the beacon receiver.  A hardware            */
	/* limitation in the beacon receiver means that we have to keep         */
	/* telling it the channel number, even when it is not changing.         */

		outportb (DigitalPort, BeaconChannel);
		outportb (DigitalPort, BeaconChannel+16);

	/* Put the new value in DataSum, and signal its availability.           */
		/* Also reset our private static variables.                                             */

		DataSum = CumulativeTotal;  DataTaken = FALSE;
		CumulativeTotal = 0;
		SamplesToGo = SampleSize;
		_signal (DataReady);
		}


	}

/************************************************************************/
/*                              OTHER INTERNAL DATA COLLECTION FUNCTIONS                                */
/************************************************************************/

private void LoadCounter (unsigned int counter, unsigned int value)

    /* Loads value into an 8253 counter, and puts the counter into its          */
	/* rate generator mode using a binary (not BCD) count.                                  */
    /* The 8253 timer/counter control byte has the following fields:            */
    /*                          bits 7-6                select counter (0, 1, or 2)                             */
	/*                              bits 5-4                read/load method (normally 3, to access */
	/*                                                              first least significant byte and then   */
    /*                                                          most significant byte of the count)             */
	/*                              bits 3-1                mode (0 to 5; mode 2 = rate generator   */
    /*                                                          is what we need)                                                */
	/*                              bit 0                           0 = binary counter, 1 = BCD                     */

    {
    outportb (Control8253, (counter<<6) + (3<<4) + (2<<1));
    outportb (Base8253+counter, value & 0xFF);
    outportb (Base8253+counter, value>>8);
    }

/************************************************************************/

private void HardwareSetup (unsigned long count)

    /* Starts sampling at the desired rate, by loading the 8253                         */
    /* timer/counter registers and setting up the A/D control options           */
    /* to give timer-triggered conversions on channel 0 with an                         */
    /* interrupt at the end of each conversion.                                                         */

	{
	unsigned int auxcount;

    /* Set A/D channel to 0 and gain to 1.              */

		outportb (ADscan, 0x0);  outportb (ADgain, 0);

	/* Set counter 0 to mode 0, without loading a count into the            */
    /* counter.  I'm not sure why we need to do this, but the ITK               */
    /* software does it (there is no documentation, other than my               */
    /* assembly language dump of the ITK driver code, on how the                */
    /* hardware uses the counters).                                                                             */

    outportb (Control8253, 0x30);

    /* The A/D sampling rate is set by having timers 1 and 2 in tandem,         */
    /* so we have to factor our count into two numbers.  A breakdown            */
	/* into prime factors would be the most accurate method, but we                 */
    /* don't need that level of precision so we use a cruder method:            */
    /* make auxcount as small as possible while ensuring that the two           */
    /* counts will fit into 16-bit registers.                                                           */

		auxcount = (unsigned int ) ((2*count)/131071UL + 1);
    if (auxcount < 2) auxcount = 2;
    count = (count + auxcount/2) / auxcount;

		LoadCounter (1, (unsigned int)count);
    LoadCounter (2, auxcount);

    /* Set up the A/D control register for internal timer triggering,           */
	/* no DMA, interrupts enabled, and trigger enabled.                                             */

		outportb (ADcontrol,
						START_CONVERSION_ON_TIMER |
						INTERRUPT_ENABLE_ON_CONVERSION_COMPLETE |
						(INTERRUPT_LEVEL<<4));

	/* Clear a pending spurious interrupt.          */

    outportb (ADstatus, 0);

    /* Enable interrupts in the interrupt mask register.                */

    outportb (IntMaskRegister,
						inportb(IntMaskRegister) & (0xFF - INTERRUPT_MASK));

    }

/************************************************************************/
/*                                              PROGRAM TERMINATION HANDLER                                             */
/************************************************************************/

private void Cleanup (void)

	/* Stops A/D operations and removes the interrupt handler.                              */

    {
	/* Turn off all control options, but keep interrupt level intact.               */
	/* This allows software start only, interrupts and DMA disabled.                */

    outportb (ADcontrol, INTERRUPT_LEVEL<<4);

    /* Disable interrupts in the interrupt mask register.               */

    outportb (IntMaskRegister, inportb(IntMaskRegister) | INTERRUPT_MASK);

    /* De-install interrupt handler.            */

	fpvdSetVector (InterruptNo, oldhandler);
    }

/************************************************************************/
/*                                              MODULE INITIALIZATION                                                   */
/************************************************************************/

BOOLEAN InitBeaconInput (BOOLEAN SimulateBeacon)

	/* Must be called at the start of program execution.  If the result             */
    /* FALSE is returned, initialization has failed.                                            */

    {
    /* Create some semaphores.          */

    DataReady = create_semaphore ();
    if (DataReady EQ 0xFFFF) return FALSE;
    init_semaphore (DataReady, 0, 1);

    Simulating = SimulateBeacon;

    if (Simulating)
		{
		/* Initialise the beacon simulator task module.         */

	if (!InitBeaconSimulator()) return FALSE;
		}

    else
		{
		/* Disable interrupts in the interrupt mask register.  We don't         */
		/* want interrupts enabled until StartBeaconSampling is called.         */

		outportb (IntMaskRegister, inportb(IntMaskRegister) | INTERRUPT_MASK);

		/* Install the interrupt handler.               */

		atexit (Cleanup);
		oldhandler = fpvdSetVector (InterruptNo, InterruptHandler);

	}

    return TRUE;
    }
