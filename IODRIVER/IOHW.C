/*
********************************************************************************
							iohw.c

Routines to interface to hardware.

********************************************************************************
*/
#include <stdlib.h>
#include <conio.h>
#include <dos.h>

/* PC-74 drive control output and signal input card info		*/
#define base_74 0x218       		/* PC-14 base address in hex       */
#define signal_channel 	0			/* Signal A/D channel number 0-->7 */
#define Tacho_signal_channel 7 /* Signal A/D channel number 0-->7 */
#define signal_gain		1			/* Amp gain for this channel 	*/
#define sig_gain_code   0x0		/* 0= 1X, 1= 2X, 2= 4X, 3= 8X	*/
#define full_scale 		5.0/signal_gain
#define Step_down_ratio 4 	/* Ratio of voltage divider circuit */
#define Degree_const 1		/* Motor speed per volt in degrees per second */

#include <reg_74.h>			/* Definitions for registers and flags for pc_74 */

// Encoder stuff -----------------------------------------
#define BaseAddr    0x310       /* PC-14 base address in hex        */
#define DIO0A       0           /* Offset of port A of the first 8255   */
#define DIO0B       1           /* Offset of port B of the first 8255   */
#define DIO0C       2           /* Offset of port C of the first 8255   */
#define DIO0CTRL    3           /* Offset of the control register       */
#define DIO1A		4
#define DIO1B		5
#define DIO1C		6
#define DIO1CTRL	7

#define CONV_AZ 40.0/842.0      /* degrees/counts		*/
#define CONV_EL -45.0/176.0

/* Encoder register variables etc.	*/
static unsigned int port0a, port0b, port0c, port0ctrl;
static unsigned int port1a, port1b, port1c, port1ctrl;

/* ADC channel-gain array of gain-codes*/
static	int	gain_74[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

/*******************************************************************************
							PC14 card routines

*******************************************************************************/

/*
***************************************************************************
init_pc14

***************************************************************************
*/
void init_pc14 ( ) {

		/* Calculate register addresses */
		port0a = BaseAddr + DIO0A;
		port0b = BaseAddr + DIO0B;
		port0c = BaseAddr + DIO0C;
		port0ctrl = BaseAddr + DIO0CTRL;

		port1a = BaseAddr + DIO1A;
		port1b = BaseAddr + DIO1B;
		port1c = BaseAddr + DIO1C;
		port1ctrl = BaseAddr + DIO1CTRL;


		/* 0x4A = 10010010, port A to input, port B to input and c for output    */
		outportb( port0ctrl, 0x92 );
		outportb ( port1ctrl, 0x92 );

		/* Send inhibit signal to decoder to indicate that computer is ready to */
		/* read in data.                                             */
		outportb ( port0c, 0x5 );	// clear

} // end of init_pc14

/*
***************************************************************************
read_az_encoder

***************************************************************************
*/
double read_az_encoder ( void ) {

				double temp_pos;
				unsigned int data0t, data1t;
				int temp_mask;
				unsigned int temp_int;
				int temp_i;

				outportb ( port0c, 0x00 );	// inhihib
				delay ( 1 );
				data0t = inportb ( port0a );
				data1t = inportb ( port0b );
				outportb ( port0c, 0x04 );

				temp_int = (unsigned int)( ((0x3f&data1t)<<8)|(0xff&data0t));
				temp_mask = 0x2000 & temp_int;

				if ( temp_mask )
					temp_i = (temp_int - 0x3fff);
				else
					temp_i = temp_int;

				temp_pos = (CONV_AZ)*(double)(temp_i);

				return ( temp_pos );

} // end of read_az_encoder

/*
***************************************************************************
read_el_encoder

***************************************************************************
*/
double read_el_encoder ( void ) {

				double temp_pos;
				unsigned int data0t, data1t;
				int temp_mask;
				unsigned int temp_int;
				int temp_i;

				outportb ( port1c, 0x00 );	// inhihib
				delay ( 1 );
				data0t = inportb ( port1a );
				data1t = inportb ( port1b );
				outportb ( port1c, 0x04 );

				temp_int = (unsigned int)( ((0x3f&data1t)<<8)|(0xff&data0t));
				temp_mask = 0x2000 & temp_int;

				if ( temp_mask )
					temp_i = (temp_int - 0x3fff);
				else
					temp_i = temp_int;

				temp_pos = (CONV_EL)*(double)(temp_i);

				return ( temp_pos );

} // end of read_el_encoder


/*******************************************************************************
							PC74 card routines

*******************************************************************************/

/*
***************************************************************************
clean

***************************************************************************
*/
void clean(void)

{
adcsr_bits		adcsr_d;

adgcr_bits		adgcr_d;

register int 	i;

/* First we disable everything. */

	adcsr_d.bi.mode = 0;
	adcsr_d.bi.i_en = 0;
	adcsr_d.bi.d_en = 0;
	adcsr_d.bi.clr_err = 1;

	outp(adcsr, adcsr_d.by);

	inp(addatl);						 /* Then we clear the A/D done bit. */
	inp(addath);

	w_busy;			 					 /* Wait for any current conversion to end. */

	 /* Now wait either for 100 uS or for done.  */
	for (i = 0; ((i < 20) && (!(inp(adcsr) & 0x80))); i++);

	inp(addatl);						 /* Then we clear the A/D done bit. */
	inp(addath);

	outp(adcsr, adcsr_d.by);  		 /* and clear any error bits. */

}  // end of clean


/*
***************************************************************************
init_pc74

***************************************************************************
*/
void init_pc74(void)

{
adcsr_bits		adcsr_d;

adgcr_bits		adgcr_d;

	clean();

	adgcr_d.bi.chan = 0;
	adgcr_d.bi.gain = 0;

	outp(adgcr, adgcr_d.by);


	outp(tmrctr, 0x35);				 /* Sets a frequency of 1 KHz */

}  /* end of init_pc74 */

void ad_in(int chan, int *in_val)
{

adcsr_bits		adcsr_d;

adgcr_bits		adgcr_d;

	if ((chan < 16) && (chan > -1)) {

		clean();
		adgcr_d.bi.chan = chan;
		adgcr_d.bi.gain = gain_74[chan];

		outp(adgcr, adgcr_d.by);

		w_done;							 /* Wait for A/D completion. */

		*in_val = (inp(addath) << 8) | inp(addatl);
		outp(adgcr, adgcr_d.by);

		w_done;							 /* Wait for A/D completion. */

		*in_val = (inp(addath) << 8) | inp(addatl);

		adcsr_d.by = inp(adcsr);
	}

}   /* end of ad_in */

static void da_out ( int chan, int out_val ) {

	if ((chan < 2) && (chan > -1)) {
		outp ((dadatl0 + chan*2), out_val);
		outp ((dadath0 + chan*2), (out_val >> 8));
		}

} // end of da_out

void dac_out ( int ch, double voltage, double min, double max ) {
	int volt;  // convert float point voltage

	voltage=(voltage - min) / (max - min) * 4096;

	if (voltage<0.0)
		voltage=0.0;    // 0 <= volt <= 4095
	else
		if (voltage>4095.0)
			voltage=4095.0;

	volt=(int) voltage;

	da_out(ch,volt);                 // Calls DAC function to output analog

} // end of dac_out

/*
********************************************************************************
OutputAzVolts

********************************************************************************
*/
void OutputAzVolts_temp ( double volts ) {

	dac_out ( 0, volts, -5, 5 );

} // end of OutputAzVolts

/*
********************************************************************************
OutputElVolts

********************************************************************************
*/
void OutputElVolts_temp ( double volts ) {

	dac_out ( 1, volts, -5, 5 );

} // end of OutputElVolts

/*
******************************************************************************
InputSigVolts

******************************************************************************
*/
float InputSigVolts ( void )
{
	int	adc_value;
	float	temp;

	gain_74[signal_channel] = sig_gain_code;
	ad_in ( signal_channel, &adc_value );
	temp = (adc_value - 2048) * full_scale / 2048 ;

	return ( temp );

} /* end of InputSigVolts */
/*
******************************************************************************
	TachoSig

******************************************************************************
*/
double	TachoSig(void)
{
	int	adc_value;
	float	temp, velocity;

	/* sample from pc-74 card and process */
	gain_74[Tacho_signal_channel] = sig_gain_code;
	ad_in ( Tacho_signal_channel, &adc_value );
	temp = (adc_value - 2048) * full_scale / 2048 ;

	/* convert to velocity */
	velocity = temp * Step_down_ratio * Degree_const;

	return ( velocity );
} /* end of TachoSig */
/*
******************************************************************************
*/

