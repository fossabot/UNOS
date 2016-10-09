/*
****************************************************************************

				Australia Telescope Parser Module

 ----------------------------------------------------------------------------
 The code is written in C for the Intel 80960 microprocessor and was
 developed to be run as a task in the UNOS operating System ( Betz 1989 ).

 Author:	L. Sciacca

 Date:		6-Jan-1990

 Project:	Australia Telescope Digital Control System

 Latest:	6-Jan-1990

****************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <general.h>
#include <unos.h>

#include <posext.h>
#include <tparser.h>
#include <nvramext.h>
#include "comdec.h"

#include <seq.h>
#include <seqext.h>

#include "taskname.h"

#define ESC '\033'
#define RET 0x0D

#define AZ_AXIS 0
#define EL_AXIS 1


/*---- PASSWORD Command */
static void k_handler ( dec_input_struc *input_ptr );
static void ke_handler ( dec_input_struc *input_ptr );
static void key_handler ( dec_input_struc *input_ptr );

static void nokey_handler ( dec_input_struc *input_ptr );
static void no_input_handler ( dec_input_struc *input_ptr );

/*---- Top level handlers */
static void p_handler ( dec_input_struc *input_ptr );
static void i_handler ( dec_input_struc *input_ptr );
static void a_handler ( dec_input_struc *input_ptr );
static void b_handler ( dec_input_struc *input_ptr );
static void d_handler ( dec_input_struc *input_ptr );
static void f_handler ( dec_input_struc *input_ptr );
static void g_handler ( dec_input_struc *input_ptr );
static void h_handler ( dec_input_struc *input_ptr );
static void m_handler ( dec_input_struc *input_ptr );
static void o_handler ( dec_input_struc *input_ptr );
static void r_handler ( dec_input_struc *input_ptr );
static void s_handler ( dec_input_struc *input_ptr );

/*---- ALPH Command */
static void al_handler ( dec_input_struc *input_ptr );
static void alp_handler ( dec_input_struc *input_ptr );
static void alph_handler ( dec_input_struc *input_ptr );
static void alph__handler ( dec_input_struc *input_ptr );

/*---- BETA Command */
static void be_handler ( dec_input_struc *input_ptr );
static void bet_handler ( dec_input_struc *input_ptr );
static void beta_handler ( dec_input_struc *input_ptr );
static void beta__handler ( dec_input_struc *input_ptr );

/*---- DEF Command */
static void de_handler ( dec_input_struc *input_ptr );
static void def_handler ( dec_input_struc *input_ptr );

/*---- FFHI FFIN FFLO Commands */
static void ff_handler ( dec_input_struc *input_ptr );
static void ffh_handler ( dec_input_struc *input_ptr );
static void ffhi_handler ( dec_input_struc *input_ptr );
static void ffhi__handler ( dec_input_struc *input_ptr );

static void ffi_handler ( dec_input_struc *input_ptr );
static void ffin_handler ( dec_input_struc *input_ptr );
static void ffin__handler ( dec_input_struc *input_ptr );

static void ffl_handler ( dec_input_struc *input_ptr );
static void fflo_handler ( dec_input_struc *input_ptr );
static void fflo__handler ( dec_input_struc *input_ptr );

/*---- GET ----*/
static void ge_handler ( dec_input_struc *input_ptr );
static void get_handler ( dec_input_struc *input_ptr );

/*---- HELP ----*/
static void he_handler ( dec_input_struc *input_ptr );
static void hel_handler ( dec_input_struc *input_ptr );
static void help_handler ( dec_input_struc *input_ptr );
static void leave_help_handler ( dec_input_struc *input_ptr );

/*---- MAXA MAXV MINV MITI MITR Commands */
static void ma_handler ( dec_input_struc *input_ptr );
static void max_handler ( dec_input_struc *input_ptr );
static void maxa_handler ( dec_input_struc *input_ptr );
static void maxa__handler ( dec_input_struc *input_ptr );
static void maxv_handler ( dec_input_struc *input_ptr );
static void maxv__handler ( dec_input_struc *input_ptr );

static void mi_handler ( dec_input_struc *input_ptr );
static void min_handler ( dec_input_struc *input_ptr );
static void minv_handler ( dec_input_struc *input_ptr );
static void minv__handler ( dec_input_struc *input_ptr );

static void mit_handler ( dec_input_struc *input_ptr );
static void miti_handler ( dec_input_struc *input_ptr );
static void miti__handler ( dec_input_struc *input_ptr );

static void mitr_handler ( dec_input_struc *input_ptr );
static void mitr__handler ( dec_input_struc *input_ptr );

/*---- OUT AZ/EL Command */
static void ou_handler ( dec_input_struc *input_ptr );
static void out_handler ( dec_input_struc *input_ptr );
static void out__handler ( dec_input_struc *input_ptr );
static void out_a_handler ( dec_input_struc *input_ptr );
static void out_az_handler ( dec_input_struc *input_ptr );
static void out_e_handler ( dec_input_struc *input_ptr );
static void out_el_handler ( dec_input_struc *input_ptr );

static void out_n_handler ( dec_input_struc *input_ptr );
static void out_no_handler ( dec_input_struc *input_ptr );
static void out_nor_handler ( dec_input_struc *input_ptr );

static void out_t_handler ( dec_input_struc *input_ptr );
static void out_to_handler ( dec_input_struc *input_ptr );
static void out_tog_handler ( dec_input_struc *input_ptr );

static void out_t_handler ( dec_input_struc *input_ptr );
static void out_tu_handler ( dec_input_struc *input_ptr );
static void out_tun_handler ( dec_input_struc *input_ptr );




/*---- RATG RES RREG Commands */
static void ra_handler ( dec_input_struc *input_ptr );
static void rat_handler ( dec_input_struc *input_ptr );
static void ratg_handler ( dec_input_struc *input_ptr );
static void ratg__handler ( dec_input_struc *input_ptr );

static void re_handler ( dec_input_struc *input_ptr );
static void res_handler ( dec_input_struc *input_ptr );
static void res__handler ( dec_input_struc *input_ptr );

static void rr_handler ( dec_input_struc *input_ptr );
static void rre_handler ( dec_input_struc *input_ptr );
static void rreg_handler ( dec_input_struc *input_ptr );
static void rreg__handler ( dec_input_struc *input_ptr );

/*---- PROP Command */
static void pr_handler ( dec_input_struc *input_ptr );
static void pro_handler ( dec_input_struc *input_ptr );
static void prop_handler ( dec_input_struc *input_ptr );
static void prop__handler ( dec_input_struc *input_ptr );

/*---- INTE INTN INTP Commands */
static void in_handler ( dec_input_struc *input_ptr );
static void int_handler ( dec_input_struc *input_ptr );
static void inte_handler ( dec_input_struc *input_ptr );
static void inte__handler ( dec_input_struc *input_ptr );
static void intn_handler ( dec_input_struc *input_ptr );
static void intn__handler ( dec_input_struc *input_ptr );
static void intp_handler ( dec_input_struc *input_ptr );
static void intp__handler ( dec_input_struc *input_ptr );

/*---- SEL_AZ SEL_EL SIGN SET Commands */
static void s_handler ( dec_input_struc *input_ptr );
static void se_handler ( dec_input_struc *input_ptr );
static void sel_handler ( dec_input_struc *input_ptr );
static void sel__handler ( dec_input_struc *input_ptr );
static void sel_a_handler ( dec_input_struc *input_ptr );
static void sel_az_handler ( dec_input_struc *input_ptr );
static void sel_i_handler ( dec_input_struc *input_ptr );
static void sel_id_handler ( dec_input_struc *input_ptr );
static void sel_e_handler ( dec_input_struc *input_ptr );
static void sel_el_handler ( dec_input_struc *input_ptr );
static void si_handler ( dec_input_struc *input_ptr );
static void sig_handler ( dec_input_struc *input_ptr );
static void sign_handler ( dec_input_struc *input_ptr );
static void sign__handler ( dec_input_struc *input_ptr );

static void set_handler ( dec_input_struc *input_ptr );

static void st_handler ( dec_input_struc *input_ptr );
static void ste_handler ( dec_input_struc *input_ptr );
static void step_handler ( dec_input_struc *input_ptr );
static void step__handler ( dec_input_struc *input_ptr );
static void step_a_handler ( dec_input_struc *input_ptr );
static void step_az_handler ( dec_input_struc *input_ptr );
static void step_az__handler ( dec_input_struc *input_ptr );
static void step_e_handler ( dec_input_struc *input_ptr );
static void step_el_handler ( dec_input_struc *input_ptr );
static void step_el__handler ( dec_input_struc *input_ptr );

static void step_p_handler ( dec_input_struc *input_ptr );
static void step_pe_handler ( dec_input_struc *input_ptr );
static void step_per_handler ( dec_input_struc *input_ptr );
static void step_per__handler ( dec_input_struc *input_ptr );

/*---- Error Handlers */
static void esc_handler ( dec_input_struc *input_ptr );
static void err_handler_kbd ( dec_input_struc *input_ptr );
static void illegal_addr_handler ( dec_input_struc *input_ptr );


static void get_prop_val ( dec_input_struc *input_ptr );
static void get_inte_val ( dec_input_struc *input_ptr );
static void get_intn_val ( dec_input_struc *input_ptr );
static void get_intp_val ( dec_input_struc *input_ptr );

static void get_alph_par ( dec_input_struc *input_ptr );
static void get_beta_par ( dec_input_struc *input_ptr );
static void get_ffhi_par ( dec_input_struc *input_ptr );
static void get_ffin_par ( dec_input_struc *input_ptr );

static void get_fflo_par ( dec_input_struc *input_ptr );
static void get_maxa_par ( dec_input_struc *input_ptr );
static void get_maxv_par ( dec_input_struc *input_ptr );
static void get_minv_par ( dec_input_struc *input_ptr );

static void get_miti_par ( dec_input_struc *input_ptr );
static void get_mitr_par ( dec_input_struc *input_ptr );
static void get_ratg_par ( dec_input_struc *input_ptr );
static void get_res_par ( dec_input_struc *input_ptr );

static void get_rreg_par ( dec_input_struc *input_ptr );
static void get_sign_par ( dec_input_struc *input_ptr );
static void get_step_az_par ( dec_input_struc *input_ptr );
static void get_step_el_par ( dec_input_struc *input_ptr );
static void get_step_per_par ( dec_input_struc *input_ptr );


static void get_parameters_from_nvram ( void );
static void set_parameters_into_nvram ( void );



static double get_double_from_kbd ( dec_input_struc *input_ptr,
										double old_double );

static int get_int_from_kbd ( dec_input_struc *input_ptr,
							int old_int );


/* Position Loop parameters */
static controller_parameter_struct pospar [ 2 ];


/* declare the address map structure */
// static added
static addr_struc addr_map;

/* declare the decoder structures for input from the keyboard task, and
the address decode terminator
*/

static dec_input_struc dec_kbd;	/* decoder for keyboard */
static dec_input_struc decterm;	/* decoder for illegal sending task */

/* set up the tables used to decode the sending task address */

static char * valid_addr [ 2 ] = { kbd_parser_name, NULL };
static dec_input_struc *addr_vec [ 2 ] = { &dec_kbd, &decterm };

static unsigned int fp_string_count;
static unsigned int int_string_count;

static unsigned int axis_type = AZ_DRIVE;
static char parser_mess_buf [ 100 ];
static char display_string [ 80 ];
static char error_string [ 80 ];
static char lock_string [ 10 ];
static char command_lock = 1;
static char tune_screen;
static int axis_tune_data_out = AZ_AXIS;
static unsigned char dataport_switch = 0; /* Default, normal dataport */
							/* 1 is Tune, 2 is STOP */
static unsigned char dataport_switch_prev;

static char fp_string [ 50 ];
static char int_string [ 50 ];

static systemid_struct systemid;

/************************* VECTOR DECODER TABLE ***************************/
/* We must get past this first table before we permit changing variables */
/* this could be made more elaborate by adding a password instead */
static unsigned char vdt_K_par [ ] = { 'k',
									ESC,
									0xff };

static unsigned char vdt_KE_par [ ] = { 	'e',
									ESC,
									0xff };

static unsigned char vdt_KEY_par [ ] = { 'y',
									ESC,
									0xff };


/* Top level command structure */
static unsigned char vdt_top_level_par [ ] = { 'a',
									'b',
									'd',
									'f',
									'g',
									'h',
									'i',
									'k',
									'm',
									'o',
									'p',
									'r',
									's',
									ESC,
									0xff };

/* decode tables */


/*====> ALPH */
static unsigned char vdt_A_par [ ] = {
									'l',
									ESC,
									0xff };

static unsigned char vdt_AL_par [ ] = {
									'p',
									ESC,
									0xff };

static unsigned char vdt_ALP_par [ ] = {
									'h',
									ESC,
									0xff };

static unsigned char vdt_ALPH_par [ ] = {
									' ',
									ESC,
									0xff };

/*====> BETA */
static unsigned char vdt_B_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_BE_par [ ] = {
									't',
									ESC,
									0xff };

static unsigned char vdt_BET_par [ ] = {
									'a',
									ESC,
									0xff };

static unsigned char vdt_BETA_par [ ] = {
									' ',
									ESC,
									0xff };




/*====> DEF */
static unsigned char vdt_D_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_DE_par [ ] = {
									'f',
									ESC,
									0xff };

/*====> FFHI FFLO FFIN */
static unsigned char vdt_F_par [ ] = {
									'f',
									ESC,
									0xff };

static unsigned char vdt_FF_par [ ] = {
									'h',
									'i',
									'l',
									ESC,
									0xff };

static unsigned char vdt_FFH_par [ ] = {
									'i',
									ESC,
									0xff };

static unsigned char vdt_FFHI_par [ ] = {
									' ',
									ESC,
									0xff };


static unsigned char vdt_FFL_par [ ] = {
									'o',
									ESC,
									0xff };

static unsigned char vdt_FFLO_par [ ] = {
									' ',
									ESC,
									0xff };


static unsigned char vdt_FFI_par [ ] = {
									'n',
									ESC,
									0xff };

static unsigned char vdt_FFIN_par [ ] = {
									' ',
									ESC,
									0xff };

/*====> GET */
static unsigned char vdt_G_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_GE_par [ ] = {
									't',
									ESC,
									0xff };
/*====> HELP */
static unsigned char vdt_H_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_HE_par [ ] = {
									'l',
									ESC,
									0xff };

static unsigned char vdt_HEL_par [ ] = {
									'p',
									ESC,
									0xff };


static unsigned char vdt_leave_help_par [ ] = {
									ESC,
									0xff };


/*====> INTE INTN INTP */
static unsigned char vdt_I_par [ ] = {
									'n',
									ESC,
									0xff };

static unsigned char vdt_IN_par [ ] = {
									't',
									ESC,
									0xff };

static unsigned char vdt_INT_par [ ] = {
									'e',
									'n',
									'p',
									ESC,
									0xff };

static unsigned char vdt_INTE_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_INTN_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_INTP_par [ ] = {
									' ',
									ESC,
									0xff };



static unsigned char vdt_M_par [ ] = {
									'a',
									'i',
									ESC,
									0xff };

static unsigned char vdt_MA_par [ ] = {
									'x',
									ESC,
									0xff };

static unsigned char vdt_MAX_par [ ] = {
									'a',
									'v',
									ESC,
									0xff };

static unsigned char vdt_MAXA_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_MAXV_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_MI_par [ ] = {
									'n',
									't',
									ESC,
									0xff };

static unsigned char vdt_MIN_par [ ] = {
									'v',
									ESC,
									0xff };

static unsigned char vdt_MINV_par [ ] = {
									' ',
									ESC,
									0xff };


static unsigned char vdt_MIT_par [ ] = {
									'i',
									'r',
									ESC,
									0xff };

static unsigned char vdt_MITI_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_MITR_par [ ] = {
									' ',
									ESC,
									0xff };


/*====> OUT AZ/EL */
static unsigned char vdt_O_par [ ] = { 'u',
									ESC,
									0xff };

static unsigned char vdt_OU_par [ ] = { 't',
									ESC,
									0xff };

static unsigned char vdt_OUT_par [ ] = { ' ',
									ESC,
									0xff };

static unsigned char vdt_OUT__par [ ] = { 'a',
									'e',
									'n',
									't',
									ESC,
									0xff };

static unsigned char vdt_OUT_A_par [ ] = { 'z',
									ESC,
									0xff };

static unsigned char vdt_OUT_E_par [ ] = { 'l',
									ESC,
									0xff };

static unsigned char vdt_OUT_T_par [ ] = { 'o',
									'u',
									ESC,
									0xff };

static unsigned char vdt_OUT_TU_par [ ] = { 'n',
									ESC,
									0xff };

static unsigned char vdt_OUT_TO_par [ ] = { 'g',
									ESC,
									0xff };

static unsigned char vdt_OUT_N_par [ ] = { 'o',
									ESC,
									0xff };

static unsigned char vdt_OUT_NO_par [ ] = { 'r',
									ESC,
									0xff };


/*====> PROP */
static unsigned char vdt_P_par [ ] = { 'r',
									ESC,
									0xff };

static unsigned char vdt_PR_par [ ] = { 'o',
									ESC,
									0xff };

static unsigned char vdt_PRO_par [ ] = { 'p',
									ESC,
									0xff };

static unsigned char vdt_PROP_par [ ] = {
									' ',
									ESC,
									0xff };



static unsigned char vdt_R_par [ ] = {
									'a',
									'e',
									'r',
									ESC,
									0xff };

static unsigned char vdt_RA_par [ ] = {
									't',
									ESC,
									0xff };

static unsigned char vdt_RAT_par [ ] = {
									'g',
									ESC,
									0xff };

static unsigned char vdt_RATG_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_RE_par [ ] = {
									's',
									ESC,
									0xff };

static unsigned char vdt_RES_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_RR_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_RRE_par [ ] = {
									'g',
									ESC,
									0xff };

static unsigned char vdt_RREG_par [ ] = {
									' ',
									ESC,
									0xff };



static unsigned char vdt_S_par [ ] = {
									'e',
									'i',
									't',
									ESC,
									0xff };

static unsigned char vdt_SI_par [ ] = {
									'g',
									ESC,
									0xff };

static unsigned char vdt_SIG_par [ ] = {
									'n',
									ESC,
									0xff };

static unsigned char vdt_SIGN_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_SE_par [ ] = {
									'l',
									't',
									ESC,
									0xff };

static unsigned char vdt_SEL_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_SEL__par [ ] = {
									'a',
									'e',
									'i',
									ESC,
									0xff };

static unsigned char vdt_SEL_A_par [ ] = {
									'z',
									ESC,
									0xff };

static unsigned char vdt_SEL_E_par [ ] = {
									'l',
									ESC,
									0xff };

static unsigned char vdt_SEL_I_par [ ] = {
									'd',
									ESC,
									0xff };



static unsigned char vdt_ST_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_STE_par [ ] = {
									'p',
									ESC,
									0xff };

static unsigned char vdt_STEP_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_STEP__par [ ] = {
									'a',
									'e',
									'p',
									ESC,
									0xff };

static unsigned char vdt_STEP_A_par [ ] = {
									'z',
									ESC,
									0xff };

static unsigned char vdt_STEP_E_par [ ] = {
									'l',
									ESC,
									0xff };

 
static unsigned char vdt_STEP_AZ_par [ ] = {
									' ',
									ESC,
									0xff };

static unsigned char vdt_STEP_EL_par [ ] = {
									' ',
									ESC,
									0xff };


static unsigned char vdt_STEP_P_par [ ] = {
									'e',
									ESC,
									0xff };

static unsigned char vdt_STEP_PE_par [ ] = {
									'r',
									ESC,
									0xff };

 
static unsigned char vdt_STEP_PER_par [ ] = {
									' ',
									ESC,
									0xff };


/* Floating Point Data Tables */
static unsigned char vdt_get_fp_par [ ] = {
									'0'|0x80,
									'9',
									'.',
									'-',
									RET,
									ESC,
									0xff };

/* Integer Data Tables */
static unsigned char vdt_get_int_par [ ] = {
									'0'|0x80,
									'9',
									'-',
									RET,
									ESC,
									0xff };


/************************* VECTOR TABLE *********************************/
/* We must get past this first table before we permit changing variables */
/* this could be made more elaborate by adding a password instead */

void ( *vt_K_par [ ] ) ( dec_input_struc * ) = { k_handler,
												esc_handler,
												nokey_handler
											   };


void ( *vt_KE_par [ ] ) ( dec_input_struc * ) = { ke_handler,
												esc_handler,
												nokey_handler
											   };

void ( *vt_KEY_par [ ] ) ( dec_input_struc * ) = { key_handler,
												esc_handler,
												nokey_handler
											   };

/* Top level commands - the first character */
void ( *vt_top_level_par [ ] ) ( dec_input_struc * ) = { a_handler,
												b_handler,
												d_handler,
												f_handler,
												g_handler,
												h_handler,
												i_handler,
												k_handler,
												m_handler,
												o_handler,
												p_handler,
												r_handler,
												s_handler,
												esc_handler,
												err_handler_kbd
											   };

/*---- A ----*/
void ( *vt_A_par [ ] ) ( dec_input_struc * ) = { al_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_AL_par [ ] ) ( dec_input_struc * ) = { alp_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_ALP_par [ ] ) ( dec_input_struc * ) = { alph_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_ALPH_par [ ] ) ( dec_input_struc * ) = { alph__handler,
												esc_handler,
												err_handler_kbd
											   };

/*---- B ----*/
void ( *vt_B_par [ ] ) ( dec_input_struc * ) = { be_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_BE_par [ ] ) ( dec_input_struc * ) = { bet_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_BET_par [ ] ) ( dec_input_struc * ) = { beta_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_BETA_par [ ] ) ( dec_input_struc * ) = { beta__handler,
												esc_handler,
												err_handler_kbd
											   };
 
/*---- D ----*/
void ( *vt_D_par [ ] ) ( dec_input_struc * ) = { de_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_DE_par [ ] ) ( dec_input_struc * ) = { def_handler,
												esc_handler,
												err_handler_kbd
											   };


/*---- F ----*/
void ( *vt_F_par [ ] ) ( dec_input_struc * ) = { ff_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FF_par [ ] ) ( dec_input_struc * ) = { ffh_handler,
												ffi_handler,
                                                ffl_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_FFH_par [ ] ) ( dec_input_struc * ) = { ffhi_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FFHI_par [ ] ) ( dec_input_struc * ) = { ffhi__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FFI_par [ ] ) ( dec_input_struc * ) = { ffin_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FFIN_par [ ] ) ( dec_input_struc * ) = { ffin__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FFL_par [ ] ) ( dec_input_struc * ) = { fflo_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_FFLO_par [ ] ) ( dec_input_struc * ) = { fflo__handler,
												esc_handler,
												err_handler_kbd
											   };

/*---- G ----*/
void ( *vt_G_par [ ] ) ( dec_input_struc * ) = { ge_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_GE_par [ ] ) ( dec_input_struc * ) = { get_handler,
												esc_handler,
												err_handler_kbd
											   };

/*---- H ----*/
void ( *vt_H_par [ ] ) ( dec_input_struc * ) = { he_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_HE_par [ ] ) ( dec_input_struc * ) = { hel_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_HEL_par [ ] ) ( dec_input_struc * ) = { help_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_leave_help_par [ ] ) ( dec_input_struc * ) = { leave_help_handler,
												no_input_handler
											   };

/*---- M ----*/
void ( *vt_M_par [ ] ) ( dec_input_struc * ) = { ma_handler,
												mi_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MA_par [ ] ) ( dec_input_struc * ) = { max_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_MAX_par [ ] ) ( dec_input_struc * ) = { maxa_handler,
												maxv_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MAXA_par [ ] ) ( dec_input_struc * ) = { maxa__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MAXV_par [ ] ) ( dec_input_struc * ) = { maxv__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MI_par [ ] ) ( dec_input_struc * ) = { min_handler,
												mit_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_MIT_par [ ] ) ( dec_input_struc * ) = { miti_handler,
												mitr_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MITI_par [ ] ) ( dec_input_struc * ) = { miti__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MITR_par [ ] ) ( dec_input_struc * ) = { mitr__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_MIN_par [ ] ) ( dec_input_struc * ) = { minv_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_MINV_par [ ] ) ( dec_input_struc * ) = { minv__handler,
												esc_handler,
												err_handler_kbd
											   };
/*---- O -----*/
void ( *vt_O_par [ ] ) ( dec_input_struc * ) = { ou_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_OU_par [ ] ) ( dec_input_struc * ) = { out_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_OUT_par [ ] ) ( dec_input_struc * ) = { out__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_OUT__par [ ] ) ( dec_input_struc * ) = { out_a_handler,
												out_e_handler,
												out_n_handler,
												out_t_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_OUT_A_par [ ] ) ( dec_input_struc * ) = { out_az_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_OUT_E_par [ ] ) ( dec_input_struc * ) = { out_el_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_OUT_N_par [ ] ) ( dec_input_struc * ) = { out_no_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_OUT_NO_par [ ] ) ( dec_input_struc * ) = { out_nor_handler,
												esc_handler,
												err_handler_kbd
											   };



void ( *vt_OUT_T_par [ ] ) ( dec_input_struc * ) = { out_to_handler,
												out_tu_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_OUT_TO_par [ ] ) ( dec_input_struc * ) = { out_tog_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_OUT_TU_par [ ] ) ( dec_input_struc * ) = { out_tun_handler,
												esc_handler,
												err_handler_kbd
											   };


/*---- R ----*/
void ( *vt_R_par [ ] ) ( dec_input_struc * ) = { ra_handler,
												re_handler,
												rr_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_RA_par [ ] ) ( dec_input_struc * ) = { rat_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_RAT_par [ ] ) ( dec_input_struc * ) = { ratg_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_RATG_par [ ] ) ( dec_input_struc * ) = { ratg__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_RE_par [ ] ) ( dec_input_struc * ) = { res_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_RES_par [ ] ) ( dec_input_struc * ) = { res__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_RR_par [ ] ) ( dec_input_struc * ) = { rre_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_RRE_par [ ] ) ( dec_input_struc * ) = { rreg_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_RREG_par [ ] ) ( dec_input_struc * ) = { rreg__handler,
												esc_handler,
												err_handler_kbd
											   };





/*---- S ----*/
void ( *vt_S_par [ ] ) ( dec_input_struc * ) = { se_handler,
												si_handler,
												st_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_SI_par [ ] ) ( dec_input_struc * ) = { sig_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_SIG_par [ ] ) ( dec_input_struc * ) = { sign_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_SIGN_par [ ] ) ( dec_input_struc * ) = { sign__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_SE_par [ ] ) ( dec_input_struc * ) = { sel_handler,
												set_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_SEL_par [ ] ) ( dec_input_struc * ) = { sel__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_SEL__par [ ] ) ( dec_input_struc * ) = { sel_a_handler,
												sel_e_handler,
												sel_i_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_SEL_A_par [ ] ) ( dec_input_struc * ) = { sel_az_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_SEL_E_par [ ] ) ( dec_input_struc * ) = { sel_el_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_SEL_I_par [ ] ) ( dec_input_struc * ) = { sel_id_handler,
												esc_handler,
												err_handler_kbd
											   };



void ( *vt_ST_par [ ] ) ( dec_input_struc * ) = { ste_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_STE_par [ ] ) ( dec_input_struc * ) = { step_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_STEP_par [ ] ) ( dec_input_struc * ) = { step__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_STEP__par [ ] ) ( dec_input_struc * ) = { step_a_handler,
												step_e_handler,
												step_p_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_STEP_A_par [ ] ) ( dec_input_struc * ) = { step_az_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_STEP_E_par [ ] ) ( dec_input_struc * ) = { step_el_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_STEP_AZ_par [ ] ) ( dec_input_struc * ) = { step_az__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_STEP_EL_par [ ] ) ( dec_input_struc * ) = { step_el__handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_STEP_P_par [ ] ) ( dec_input_struc * ) = { step_pe_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_STEP_PE_par [ ] ) ( dec_input_struc * ) = { step_per_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_STEP_PER_par [ ] ) ( dec_input_struc * ) = { step_per__handler,
												esc_handler,
												err_handler_kbd
											   };


/*---- P ----- */


void ( *vt_P_par [ ] ) ( dec_input_struc * ) = { pr_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_PR_par [ ] ) ( dec_input_struc * ) = { pro_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_PRO_par [ ] ) ( dec_input_struc * ) = { prop_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_PROP_par [ ] ) ( dec_input_struc * ) = { prop__handler,
												esc_handler,
												err_handler_kbd
											   };

/*---- I ----- */
void ( *vt_I_par [ ] ) ( dec_input_struc * ) = { in_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_IN_par [ ] ) ( dec_input_struc * ) = { int_handler,
												esc_handler,
												err_handler_kbd
											   };


void ( *vt_INT_par [ ] ) ( dec_input_struc * ) = { inte_handler,
												intn_handler,
												intp_handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_INTE_par [ ] ) ( dec_input_struc * ) = { inte__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_INTN_par [ ] ) ( dec_input_struc * ) = { intn__handler,
												esc_handler,
												err_handler_kbd
											   };

void ( *vt_INTP_par [ ] ) ( dec_input_struc * ) = { intp__handler,
												esc_handler,
												err_handler_kbd
											   };

/*
========================================================================
	Routines to load parameters. These routines are called after a space
	is entered after a valid command sequence. Valid keys for these
	routines are 0-9, -, +,.
========================================================================
*/

/*---- Load double value into inte parameter */
void ( *vt_get_inte_par [ ] ) ( dec_input_struc * ) = { get_inte_val,
												get_inte_val,
												get_inte_val,
												get_inte_val,
												esc_handler,
												err_handler_kbd
											   };

/*---- Load double value into inte parameter */
void ( *vt_get_intn_par [ ] ) ( dec_input_struc * ) = { get_intn_val,
												get_intn_val,
												get_intn_val,
												get_intn_val,
												esc_handler,
												err_handler_kbd
											   };

/*---- Load double value into inte parameter */
void ( *vt_get_intp_par [ ] ) ( dec_input_struc * ) = { get_intp_val,
												get_intp_val,
												get_intp_val,
												get_intp_val,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into prop parameter */
void ( *vt_get_prop_par [ ] ) ( dec_input_struc * ) = { get_prop_val,
												get_prop_val,
												get_prop_val,
												get_prop_val,
												get_prop_val,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into alpha parameter */
void ( *vt_get_alph_par [ ] ) ( dec_input_struc * ) = { get_alph_par,
												get_alph_par,
												get_alph_par,
												get_alph_par,
												get_alph_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into betaa parameter */
void ( *vt_get_beta_par [ ] ) ( dec_input_struc * ) = { get_beta_par,
												get_beta_par,
												get_beta_par,
												get_beta_par,
												get_beta_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into ffhi parameter */
void ( *vt_get_ffhi_par [ ] ) ( dec_input_struc * ) = { get_ffhi_par,
												get_ffhi_par,
												get_ffhi_par,
												get_ffhi_par,
												get_ffhi_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into ffina parameter */
void ( *vt_get_ffin_par [ ] ) ( dec_input_struc * ) = { get_ffin_par,
												get_ffin_par,
												get_ffin_par,
												get_ffin_par,
												get_ffin_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into ffloa parameter */
void ( *vt_get_fflo_par [ ] ) ( dec_input_struc * ) = { get_fflo_par,
												get_fflo_par,
												get_fflo_par,
												get_fflo_par,
												get_fflo_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into maxaa parameter */
void ( *vt_get_maxa_par [ ] ) ( dec_input_struc * ) = { get_maxa_par,
												get_maxa_par,
												get_maxa_par,
												get_maxa_par,
												get_maxa_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into maxv parameter */
void ( *vt_get_maxv_par [ ] ) ( dec_input_struc * ) = { get_maxv_par,
												get_maxv_par,
												get_maxv_par,
												get_maxv_par,
												get_maxv_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into minv parameter */
void ( *vt_get_minv_par [ ] ) ( dec_input_struc * ) = { get_minv_par,
												get_minv_par,
												get_minv_par,
												get_minv_par,
												get_minv_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into mitia parameter */
void ( *vt_get_miti_par [ ] ) ( dec_input_struc * ) = { get_miti_par,
												get_miti_par,
												get_miti_par,
												get_miti_par,
												get_miti_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into mitra parameter */
void ( *vt_get_mitr_par [ ] ) ( dec_input_struc * ) = { get_mitr_par,
												get_mitr_par,
												get_mitr_par,
												get_mitr_par,
												get_mitr_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into ratga parameter */
void ( *vt_get_ratg_par [ ] ) ( dec_input_struc * ) = { get_ratg_par,
												get_ratg_par,
												get_ratg_par,
												get_ratg_par,
												get_ratg_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into resa parameter */
void ( *vt_get_res_par [ ] ) ( dec_input_struc * ) = { get_res_par,
												get_res_par,
												get_res_par,
												get_res_par,
												get_res_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into rrega parameter */
void ( *vt_get_rreg_par [ ] ) ( dec_input_struc * ) = { get_rreg_par,
												get_rreg_par,
												get_rreg_par,
												get_rreg_par,
												get_rreg_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into signa parameter */
void ( *vt_get_sign_par [ ] ) ( dec_input_struc * ) = { get_sign_par,
												get_sign_par,
												get_sign_par,
												get_sign_par,
												esc_handler,
												err_handler_kbd
											   };

/*---- Load double value into step_az parameter */
void ( *vt_get_step_az_par [ ] ) ( dec_input_struc * ) = { get_step_az_par,
												get_step_az_par,
												get_step_az_par,
												get_step_az_par,
												esc_handler,
												err_handler_kbd
											   };

/*---- Load double value into step_az parameter */
void ( *vt_get_step_el_par [ ] ) ( dec_input_struc * ) = { get_step_el_par,
												get_step_el_par,
												get_step_el_par,
												get_step_el_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- Load double value into step_az parameter */
void ( *vt_get_step_per_par [ ] ) ( dec_input_struc * ) = { get_step_per_par,
												get_step_per_par,
												get_step_per_par,
												get_step_per_par,
												esc_handler,
												err_handler_kbd
											   };


/*---- table to decode an illegal sending task */

unsigned char vdt_term [ 1 ] = { 0xff };
void ( *vt_term [ 1 ] ) ( dec_input_struc * ) = { illegal_addr_handler };






/**************************** KEY **************************************/
/*	Decoder handlers to make up key command */
/*--- K ----*/
void k_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_KE_par;
	input_ptr->vector_table_ptr = vt_KE_par;

	strcpy ( error_string,   "                                             ");
	/* do NOT echo keys to screen. Like a true password. */

} /* end of k_handler */

/*--- E ----*/
void ke_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_KEY_par;
	input_ptr->vector_table_ptr = vt_KEY_par;

} /* end of ke_handler */

/*---- Y ----*/
void key_handler ( dec_input_struc *input_ptr ) {

	/* point to the top level command decoder */
	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;

	if ( command_lock ) {
		strcpy ( lock_string, "UNLOCKED" );
		strcpy ( error_string, "                                              ");	
		command_lock = 0;
	}
	else {
		strcpy ( lock_string, "LOCKED  " );
		command_lock = 1;
	}

} /* end of key_handler */





/**************************** ALPH **************************************/
/*	Decoder handlers to make up alph command */
/*--- A ----*/
void a_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_A_par;
	input_ptr->vector_table_ptr = vt_A_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of a_handler */

/*--- L ----*/
void al_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_AL_par;
	input_ptr->vector_table_ptr = vt_AL_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of al_handler */

/*---- P ----*/
void alp_handler ( dec_input_struc *input_ptr ) {

	/* point to the p decoder */
	input_ptr->valid_data_table_ptr = vdt_ALP_par;
	input_ptr->vector_table_ptr = vt_ALP_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of alp_handler */

/*---- H ----*/
void alph_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_ALPH_par;
	input_ptr->vector_table_ptr = vt_ALPH_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of alph_handler */

/*---- SPACE ----*/
void alph__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_alph_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of alph__handler */





/**************************** BETA **************************************/
/*	Decoder handlers to make up beta command */
/*--- B ----*/
void b_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_B_par;
	input_ptr->vector_table_ptr = vt_B_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of b_handler */

/*--- BE ----*/
void be_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_BE_par;
	input_ptr->vector_table_ptr = vt_BE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of be_handler */

/*---- BET ----*/
void bet_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_BET_par;
	input_ptr->vector_table_ptr = vt_BET_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of bet_handler */

/*---- BETA ----*/
void beta_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_BETA_par;
	input_ptr->vector_table_ptr = vt_BETA_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of beta_handler */

/*---- SPACE ----*/
void beta__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_beta_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of beta__handler */







/**************************** DEF ***************************/
/*	Decoder handlers to make up DEF command */
/*--- D ----*/
void d_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_D_par;
	input_ptr->vector_table_ptr = vt_D_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of d_handler */

/*--- DE ----*/
void de_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_DE_par;
	input_ptr->vector_table_ptr = vt_DE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of de_handler */

/*---- DEF ----*/
void def_handler ( dec_input_struc *input_ptr ) {

    /* Load default parameters from ROM (Factory Settings) */

	init_default_controller_parameters ( axis_type, &pospar [ axis_type ] );

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of def_handler */








/**************************** FFHI FFIN FFLO ***************************/
/*	Decoder handlers to make up beta command */
/*--- F ----*/
void f_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_F_par;
	input_ptr->vector_table_ptr = vt_F_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of f_handler */

/*--- FF ----*/
void ff_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_FF_par;
	input_ptr->vector_table_ptr = vt_FF_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ff_handler */

/*---- FFH ----*/
void ffh_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_FFH_par;
	input_ptr->vector_table_ptr = vt_FFH_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ffh_handler */

/*---- FFHI ----*/
void ffhi_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_FFHI_par;
	input_ptr->vector_table_ptr = vt_FFHI_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ffhi_handler */

/*---- SPACE ----*/
void ffhi__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_ffhi_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of ffhi__handler */

/*---- FFI ----*/
void ffi_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_FFI_par;
	input_ptr->vector_table_ptr = vt_FFI_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ffi_handler */

/*---- FFIN ----*/
void ffin_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_FFIN_par;
	input_ptr->vector_table_ptr = vt_FFIN_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ffin_handler */

/*---- SPACE ----*/
void ffin__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_ffin_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of ffin__handler */

/*---- FFL ----*/
void ffl_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_FFL_par;
	input_ptr->vector_table_ptr = vt_FFL_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ffl_handler */

/*---- FFLO ----*/
void fflo_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_FFLO_par;
	input_ptr->vector_table_ptr = vt_FFLO_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of fflo_handler */

/*---- SPACE ----*/
void fflo__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_fflo_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of fflo__handler */





/**************************** GET ***************************/
/*	Decoder handlers to make up GET command */
/*--- G ----*/
void g_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_G_par;
	input_ptr->vector_table_ptr = vt_G_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of g_handler */

/*--- GE ----*/
void ge_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_GE_par;
	input_ptr->vector_table_ptr = vt_GE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ge_handler */

/*---- GET ----*/
void get_handler ( dec_input_struc *input_ptr ) {

	get_parameters_from_nvram ( );

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of get_handler */





/**************************** HELP ***************************/
/*	Decoder handlers to make up HELP command */
/*--- h ----*/
void h_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_H_par;
	input_ptr->vector_table_ptr = vt_H_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of h_handler */

/*--- HE ----*/
void he_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_HE_par;
	input_ptr->vector_table_ptr = vt_HE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of he_handler */

/*--- HEL ----*/
void hel_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_HEL_par;
	input_ptr->vector_table_ptr = vt_HEL_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of hel_handler */

/*---- HELP ----*/
void help_handler ( dec_input_struc *input_ptr ) {

	tune_screen = TUNE_HELP_SCREEN;

	input_ptr->valid_data_table_ptr = vdt_leave_help_par;
	input_ptr->vector_table_ptr = vt_leave_help_par;
	strcpy ( display_string, "                                          ");

} /* end of help_handler */

/*---- ESC - LEAVE_HELP ----*/
void leave_help_handler ( dec_input_struc *input_ptr ) {

	tune_screen = TUNE_AZEL_SCREEN;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of leave_help_handler */





/**************************** MAXA MAXV MINV MITI MITR *******************/
/*																		 */
/*	Decoder handlers to make up beta command */
/*--- M ----*/
void m_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_M_par;
	input_ptr->vector_table_ptr = vt_M_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of m_handler */

/*--- MA ----*/
void ma_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_MA_par;
	input_ptr->vector_table_ptr = vt_MA_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ma_handler */

/*---- MAX ----*/
void max_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_MAX_par;
	input_ptr->vector_table_ptr = vt_MAX_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of max_handler */

/*---- MAXA ----*/
void maxa_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MAXA_par;
	input_ptr->vector_table_ptr = vt_MAXA_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of maxa_handler */

/*---- MAXA SPACE ----*/
void maxa__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_maxa_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of maxa__handler */

/*---- MAXV ----*/
void maxv_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MAXV_par;
	input_ptr->vector_table_ptr = vt_MAXV_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of maxv_handler */

/*---- SPACE ----*/
void maxv__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_maxv_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of maxv__handler */

/*---- MI ----*/
void mi_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_MI_par;
	input_ptr->vector_table_ptr = vt_MI_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of mi_handler */

/*---- MIN ----*/
void min_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MIN_par;
	input_ptr->vector_table_ptr = vt_MIN_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of min_handler */


/*---- MINV ----*/
void minv_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MINV_par;
	input_ptr->vector_table_ptr = vt_MINV_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of minv_handler */


/*---- MINV SPACE ----*/
void minv__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_minv_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of minv__handler */



/*---- MIT ----*/
void mit_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MIT_par;
	input_ptr->vector_table_ptr = vt_MIT_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of mit_handler */


/*---- MITI ----*/
void miti_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MITI_par;
	input_ptr->vector_table_ptr = vt_MITI_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of miti_handler */


/*---- MITI SPACE ----*/
void miti__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_miti_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of miti__handler */


/*---- MITR ----*/
void mitr_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_MITR_par;
	input_ptr->vector_table_ptr = vt_MITR_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of mitr_handler */


/*---- MITR SPACE ----*/
void mitr__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_mitr_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of mitr__handler */




/**************************** OUT AZ/EL NOR TUN *************************/
/*	Decoder handlers to make up OUT command */
/*--- o ----*/
void o_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_O_par;
	input_ptr->vector_table_ptr = vt_O_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of o_handler */

/*--- OU ----*/
void ou_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OU_par;
	input_ptr->vector_table_ptr = vt_OU_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ou_handler */

/*--- OUT ----*/
void out_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_par;
	input_ptr->vector_table_ptr = vt_OUT_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_handler */

/*---- OUT_ ----*/
void out__handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT__par;
	input_ptr->vector_table_ptr = vt_OUT__par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out__handler */


/*---- OUT_A ----*/
void out_a_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_A_par;
	input_ptr->vector_table_ptr = vt_OUT_A_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_a_handler */


/*---- OUT_E ----*/
void out_e_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_E_par;
	input_ptr->vector_table_ptr = vt_OUT_E_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_e_handler */


/*---- OUT_AZ ----*/
void out_az_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

	axis_tune_data_out = AZ_AXIS;

} /* end of out_az_handler */


/*---- OUT_EL ----*/
void out_el_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

	axis_tune_data_out = EL_AXIS;


} /* end of out_el_handler */






/*---- OUT_T ----*/
void out_t_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_T_par;
	input_ptr->vector_table_ptr = vt_OUT_T_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_t_handler */


/*---- OUT_N ----*/
void out_n_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_N_par;
	input_ptr->vector_table_ptr = vt_OUT_N_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_n_handler */


/*---- OUT_NO ----*/
void out_no_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_NO_par;
	input_ptr->vector_table_ptr = vt_OUT_NO_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_out_no_handler */

/*---- OUT_NOR ----*/
void out_nor_handler ( dec_input_struc *input_ptr ) {

	/* If the dataport is presently off, only update the old setting */
	if ( dataport_switch == 2 )
		dataport_switch_prev = 0;
	else
		dataport_switch = 0;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of out_nor_handler */



/*---- OUT_TO ----*/
void out_to_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_OUT_TO_par;
	input_ptr->vector_table_ptr = vt_OUT_TO_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_to_handler */

 
/*---- OUT_TOG ----*/
void out_tog_handler ( dec_input_struc *input_ptr ) {

	if ( dataport_switch < 2 ) {
		dataport_switch_prev = dataport_switch;
		dataport_switch = 2;
		}
	else {
		dataport_switch = dataport_switch_prev;
        }

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of out_tog_handler */


/*---- OUT_TU ----*/
void out_tu_handler ( dec_input_struc *input_ptr ) {


	input_ptr->valid_data_table_ptr = vdt_OUT_TU_par;
	input_ptr->vector_table_ptr = vt_OUT_TU_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of out_tu_handler */

 
/*---- OUT_TUN ----*/
void out_tun_handler ( dec_input_struc *input_ptr ) {

	/* If the dataport is presently off, only update the old setting */
	if ( dataport_switch == 2 )
		dataport_switch_prev = 1;
	else
		dataport_switch = 1;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of out_tun_handler */




/**************************** PROP **************************************/
/*	Decoder handlers to make up prop command */
/*--- P ----*/
void p_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_P_par;
	input_ptr->vector_table_ptr = vt_P_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of p_handler */

/*--- R ----*/
void pr_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_PR_par;
	input_ptr->vector_table_ptr = vt_PR_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of pr_handler */

/*---- O ----*/
void pro_handler ( dec_input_struc *input_ptr ) {

	/* point to the p decoder */
	input_ptr->valid_data_table_ptr = vdt_PRO_par;
	input_ptr->vector_table_ptr = vt_PRO_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of pro_handler */

/*---- P ----*/
void prop_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_PROP_par;
	input_ptr->vector_table_ptr = vt_PROP_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of prop_handler */

/*---- SPACE ----*/
void prop__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_prop_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of prop_ */





/**************************** RATG RES RREG *******************/
/*																		 */
/*	Decoder handlers to make up beta command */
/*--- R ----*/
void r_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_R_par;
	input_ptr->vector_table_ptr = vt_R_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of r_handler */

/*--- RA ----*/
void ra_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RA_par;
	input_ptr->vector_table_ptr = vt_RA_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ra_handler */

/*---- RAT ----*/
void rat_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RAT_par;
	input_ptr->vector_table_ptr = vt_RAT_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of rat_handler */

/*---- RATG ----*/
void ratg_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_RATG_par;
	input_ptr->vector_table_ptr = vt_RATG_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ratg_handler */

/*---- RATG SPACE ----*/
void ratg__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_ratg_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of ratg__handler */


/*--- RE ----*/
void re_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RE_par;
	input_ptr->vector_table_ptr = vt_RE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ra_handler */

/*---- RES ----*/
void res_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RES_par;
	input_ptr->vector_table_ptr = vt_RES_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of res_handler */


/*---- RES SPACE ----*/
void res__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_res_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of res__handler */



/*--- RR ----*/
void rr_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RR_par;
	input_ptr->vector_table_ptr = vt_RR_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of rr_handler */

/*---- RRE ----*/
void rre_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_RRE_par;
	input_ptr->vector_table_ptr = vt_RRE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of rre_handler */

/*---- RREG ----*/
void rreg_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_RREG_par;
	input_ptr->vector_table_ptr = vt_RREG_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of rreg_handler */


/*---- RREG SPACE ----*/
void rreg__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_rreg_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of rreg__handler */






/**************************** SIGN *******************/
/*																		 */
/*	Decoder handlers to make up beta command */
/*--- S ----*/
void s_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_S_par;
	input_ptr->vector_table_ptr = vt_S_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of r_handler */

/*--- SI ----*/
void si_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SI_par;
	input_ptr->vector_table_ptr = vt_SI_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of si_handler */

/*---- SIG ----*/
void sig_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SIG_par;
	input_ptr->vector_table_ptr = vt_SIG_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sig_handler */

/*---- SIGN ----*/
void sign_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_SIGN_par;
	input_ptr->vector_table_ptr = vt_SIGN_par;

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sign_handler */

/*---- SIGN SPACE ----*/
void sign__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_int_par;
	input_ptr->vector_table_ptr = vt_get_sign_par;

	/* Empty float buffer */
	strncpy ( int_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	int_string_count = 0;

} /* end of sign__handler */


/*--- SE ----*/
void se_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SE_par;
	input_ptr->vector_table_ptr = vt_SE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of se_handler */

/*--- SEL ----*/
void sel_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SEL_par;
	input_ptr->vector_table_ptr = vt_SEL_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sel_handler */

/*--- SEL_ ----*/
void sel__handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SEL__par;
	input_ptr->vector_table_ptr = vt_SEL__par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sel__handler */

/*--- SEL_A ----*/
void sel_a_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SEL_A_par;
	input_ptr->vector_table_ptr = vt_SEL_A_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sel_a_handler */

/*--- SEL_AZ ----*/
void sel_az_handler ( dec_input_struc *input_ptr ) {

	axis_type = AZ_DRIVE;
	tune_screen = TUNE_AZEL_SCREEN;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of sel_az_handler */

/*--- SEL_E ----*/
void sel_e_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SEL_E_par;
	input_ptr->vector_table_ptr = vt_SEL_E_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sel_e_handler */

/*--- SEL_EL ----*/
void sel_el_handler ( dec_input_struc *input_ptr ) {

	axis_type = EL_DRIVE;
	tune_screen = TUNE_AZEL_SCREEN;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of sel_el_handler */

/*--- SEL_I ----*/
void sel_i_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_SEL_I_par;
	input_ptr->vector_table_ptr = vt_SEL_I_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of sel_a_handler */

/*--- SEL_ID ----*/
void sel_id_handler ( dec_input_struc *input_ptr ) {

	tune_screen = TUNE_ID_SCREEN;

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of sel_az_handler */


/*--- SET ----*/
void set_handler ( dec_input_struc *input_ptr ) {

	set_parameters_into_nvram ( );

	init_position_controllers ( );

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of sel_handler */








/*--- ST ----*/
void st_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_ST_par;
	input_ptr->vector_table_ptr = vt_ST_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of st_handler */

/*--- STE ----*/
void ste_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STE_par;
	input_ptr->vector_table_ptr = vt_STE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of ste_handler */

/*--- STEP ----*/
void step_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_par;
	input_ptr->vector_table_ptr = vt_STEP_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_handler */

/*--- STEP_ ----*/
void step__handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP__par;
	input_ptr->vector_table_ptr = vt_STEP__par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step__handler */

/*--- STEP_A ----*/
void step_a_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_A_par;
	input_ptr->vector_table_ptr = vt_STEP_A_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_a_handler */

/*--- STEP_E ----*/
void step_e_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_E_par;
	input_ptr->vector_table_ptr = vt_STEP_E_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_a_handler */


/*--- STEP_AZ ----*/
void step_az_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_AZ_par;
	input_ptr->vector_table_ptr = vt_STEP_AZ_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_az_handler */

 
/*--- STEP_EL ----*/
void step_el_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_EL_par;
	input_ptr->vector_table_ptr = vt_STEP_EL_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_el_handler */



/*--- STEP_AZ_ ----*/
void step_az__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_step_az_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of step_az_handler */

 
/*--- STEP_EL ----*/
void step_el__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_step_el_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of step_el_handler */


/*--- STEP_P ----*/
void step_p_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_P_par;
	input_ptr->vector_table_ptr = vt_STEP_P_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_p_handler */

/*--- STEP_PE ----*/
void step_pe_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_PE_par;
	input_ptr->vector_table_ptr = vt_STEP_PE_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_pe_handler */


/*--- STEP_PER ----*/
void step_per_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_STEP_PER_par;
	input_ptr->vector_table_ptr = vt_STEP_PER_par;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of step_per_handler */

 
/*--- STEP_PER_ ----*/
void step_per__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_step_per_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;


} /* end of step_per__handler */









/**************************** INTE INTP INTN ***************************/
/*	Decoder handlers to make up int command */
/*--- I ----*/
void i_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_I_par;
	input_ptr->vector_table_ptr = vt_I_par;

	/* This must be done at the start of each command */
    display_string [ 0 ] = NULL;
	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

	strcpy ( error_string,   "                                             ");

} /* end of i_handler */

/*--- N ----*/
void in_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_IN_par;
	input_ptr->vector_table_ptr = vt_IN_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of in_handler */

/*---- T ----*/
void int_handler ( dec_input_struc *input_ptr ) {

	input_ptr->valid_data_table_ptr = vdt_INT_par;
	input_ptr->vector_table_ptr = vt_INT_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of int_handler */

/*---- E ----*/
void inte_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_INTE_par;
	input_ptr->vector_table_ptr = vt_INTE_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of inte_handler */

/*---- SPACE ----*/
void inte__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_inte_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of inte__handler */

/*---- N ----*/
void intn_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_INTN_par;
	input_ptr->vector_table_ptr = vt_INTN_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of intn_handler */

/*---- SPACE ----*/
void intn__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_intn_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of intn__handler */

/*---- P ----*/
void intp_handler ( dec_input_struc *input_ptr ) {

	/* Set tables to accept space bar */
	input_ptr->valid_data_table_ptr = vdt_INTP_par;
	input_ptr->vector_table_ptr = vt_INTP_par;

	strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of intp_handler */

/*---- SPACE ----*/
void intp__handler ( dec_input_struc *input_ptr ) {

	/* Set tables to grab floating point value */
	input_ptr->valid_data_table_ptr = vdt_get_fp_par;
	input_ptr->vector_table_ptr = vt_get_intp_par;

	/* Empty float buffer */
	strncpy ( fp_string, "", 1);

	/* Add the input character to the display string for the screen task */
	strncat ( display_string, (char * )&input_ptr->data_byte, 1 );
	fp_string_count = 0;

} /* end of intp__handler */







/*
===========================================================================

	GET VALUE HANDLERS

===========================================================================
*/

/*---- PROP Get double prop gain---- */
void get_prop_val ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].prop_gain = get_double_from_kbd ( input_ptr, pospar [ axis_type ].prop_gain );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_prop_val */

/*---- INTE Get double integrator gain---- */
void get_inte_val ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].integral_gain = get_double_from_kbd ( input_ptr, pospar [ axis_type ].integral_gain );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_inte_val */


/*---- INTN Get double integrator negative limit ---- */
void get_intn_val ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].int_negative_limit = get_double_from_kbd ( input_ptr, pospar [ axis_type ].int_negative_limit );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_intn_val */

/*---- Get double intp---- */
void get_intp_val ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].int_positive_limit = get_double_from_kbd ( input_ptr, pospar [ axis_type ].int_positive_limit );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_intp_val */



/*---- Get double alpha ---- */
void get_alph_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_alpha = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_alpha );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_alph_par */



/*---- Get double beta ---- */
void get_beta_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_beta = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_beta );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_beta_par */

	
/*---- Get double ffhi ---- */
void get_ffhi_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_ff_high = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_ff_high );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_ffhi_par */

  

/*---- Get double ffin ---- */
void get_ffin_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].initial_mit_ff_gain = get_double_from_kbd ( input_ptr, pospar [ axis_type ].initial_mit_ff_gain );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_ffin_par */

  

/*---- Get double fflo ---- */
void get_fflo_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_ff_low = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_ff_low );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_fflo_par */

  

/*---- Get double maxa ---- */
void get_maxa_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].max_acceleration = get_double_from_kbd ( input_ptr, pospar [ axis_type ].max_acceleration );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_maxa_par */


/*---- Get double maxv ---- */
void get_maxv_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].max_volts = get_double_from_kbd ( input_ptr, pospar [ axis_type ].max_volts );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_maxv_par */

 
/*---- Get double minv ---- */
void get_minv_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].min_volts = get_double_from_kbd ( input_ptr, pospar [ axis_type ].min_volts );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_minv_par */

 
/*---- Get double miti ---- */
void get_miti_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_maxincr = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_maxincr );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_miti_par */

 
/*---- Get double mitr ---- */
void get_mitr_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].mit_maxref = get_double_from_kbd ( input_ptr, pospar [ axis_type ].mit_maxref );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_mitr_par */

 
/*---- Get double ratg ---- */
void get_ratg_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].rate_gain = get_double_from_kbd ( input_ptr, pospar [ axis_type ].rate_gain );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_ratg_par */

 
/*---- Get double res ---- */
void get_res_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].res_bw = get_double_from_kbd ( input_ptr, pospar [ axis_type ].res_bw );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_res_par */

	  
/*---- Get double rreg ---- */
void get_rreg_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].rate_limit_region = get_double_from_kbd ( input_ptr, pospar [ axis_type ].rate_limit_region );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_rreg_par */

 

/*---- Get int sign ---- */
void get_sign_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	pospar [ axis_type ].sign = get_int_from_kbd ( input_ptr, pospar [ axis_type ].sign );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );

} /* end of get_sign_par */


/*---- Get step az volts ---- */
void get_step_az_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	systemid.azv = get_double_from_kbd ( input_ptr, systemid.azv );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );
	else
		{
		set_systemid ( systemid );
		}

} /* end of get_step_par */

/*---- Get step el volts ---- */
void get_step_el_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	systemid.elv = get_double_from_kbd ( input_ptr, systemid.elv );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );
	else
		{
		set_systemid ( systemid );
		}

} /* end of get_step_el_par */


/*---- Get step period (sec) ---- */
void get_step_per_par ( dec_input_struc *input_ptr ) {

	/* Use the return value from this to set prop gain */
	systemid.step_per = get_double_from_kbd ( input_ptr, systemid.step_per );

	if ( input_ptr->data_byte != RET )
		strncat ( display_string, (char *)&input_ptr->data_byte, 1 );
	else
		{
		set_systemid ( systemid );
		}

} /* end of get_step_per_par */











  
/******************************** ESC (PC only) ***************************/
void esc_handler ( dec_input_struc *input_ptr ) {
	/* On hitting the escape key, go to the top of the tables */

	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	strcpy ( display_string, "                                          ");

} /* end of esc_handler */


/**************************** BAD Character *******************************/
/*---- ILLEGAL CHARACTER */
void err_handler_kbd ( dec_input_struc *input_ptr ) {

	/* Force the user to reenter the command */
	strcpy ( error_string,   "Invalid Key. Try again.                      ");
	input_ptr->valid_data_table_ptr = vdt_top_level_par;
	input_ptr->vector_table_ptr = vt_top_level_par;
	fp_string [ 0 ] = NULL;
	strcpy ( display_string, "                                          ");

} /* end of err_handler */



void nokey_handler ( dec_input_struc *input_ptr ) {

	/* Set the global pointer to the possible command list */
	input_ptr->valid_data_table_ptr = vdt_K_par;
	input_ptr->vector_table_ptr = vt_K_par;

} /* end of err_handler */


void no_input_handler ( dec_input_struc *input_ptr ) {

	/* Do nothing ! */
	 input_ptr=input_ptr;

} /* end of no_input_handler */









/**************************** ILLEGAL SENDER ******************************/
/*---- ILLEGAL task signal */
void illegal_addr_handler ( dec_input_struc *input_ptr ) {

	input_ptr = input_ptr;

} /* end of illegal_addr_handler */

/*------------------------ End of Decoder Handlers ----------------------*/


double get_double_from_kbd ( dec_input_struc *input_ptr,
							double old_double ) {

	char *endptr;
	double value;

	/* If more than 50 chars then signal error and clear */
	fp_string_count++;
	if ( fp_string_count > 50 ) {
		input_ptr->valid_data_table_ptr = vdt_top_level_par;
		input_ptr->vector_table_ptr = vt_top_level_par;
		fp_string [ 0 ] = NULL;
		}

	/* If a RET has been received then this is the entire word */
	if ( input_ptr->data_byte != RET ) {
		/* Grab each character and place in a string */
		/* char is in input_ptr->data_byte */
		strncat ( fp_string, (char*)&input_ptr->data_byte, 1 );
		}
	else {
		/* Convert string to a double */
		value = strtod ( fp_string, &endptr );
		input_ptr->valid_data_table_ptr = vdt_top_level_par;
		input_ptr->vector_table_ptr = vt_top_level_par;
		fp_string [ 0 ] = NULL;
		strcpy ( display_string, "                                          ");
		return ( value );
		}

	return ( old_double );

} /* end of get_double_from_kbd */



int get_int_from_kbd ( dec_input_struc *input_ptr,
							int old_int ) {

	int value;

	/* If more than 50 chars then signal error and clear */
	int_string_count++;
	if ( int_string_count > 50 ) {
		input_ptr->valid_data_table_ptr = vdt_top_level_par;
		input_ptr->vector_table_ptr = vt_top_level_par;
		int_string [ 0 ] = NULL;
		}

	/* If a RET has been received then this is the entire word */
	if ( input_ptr->data_byte != RET ) {
		/* Grab each character and place in a string */
		/* char is in input_ptr->data_byte */
		strncat ( int_string, (char*)&input_ptr->data_byte, 1 );
		}
	else {
		/* Convert string to a double */
		value = atoi ( int_string );
		input_ptr->valid_data_table_ptr = vdt_top_level_par;
		input_ptr->vector_table_ptr = vt_top_level_par;
		int_string [ 0 ] = NULL;
		strcpy ( display_string, "                                          ");
		return ( value );
		}

	return ( old_int );

} /* end of get_int_from_kbd */



/*
**************************************************************
initialise_tune_decoder

	Routine to be called at start up. Set up tune decoder
	structure and tables.
**************************************************************
*/
void initialise_tune_decoder ( ) {

	/********************************************************************/
	/* now initialise the various decoding structures					*/
	/* firstly the address structure									*/

	addr_map.mess_ptr = (unsigned char *)&parser_mess_buf;
	addr_map.valid_addr_tab_ptr = valid_addr;
	addr_map.addr_vector_ptr = addr_vec;

	/*---- initialise the character decoder structures ----*/

	dec_kbd.char_dec_mode = 0;
	if ( command_lock == 1 ) {
		dec_kbd.valid_data_table_ptr = vdt_K_par;
		dec_kbd.vector_table_ptr = vt_K_par;
		}
	else {
		dec_kbd.valid_data_table_ptr = vdt_top_level_par;
		dec_kbd.vector_table_ptr = vt_top_level_par;
		}

	decterm.char_dec_mode = 0;
	decterm.valid_data_table_ptr = vdt_term;
	decterm.vector_table_ptr = vt_term;

	strcpy ( display_string, "                                        ");
	strcpy ( error_string,   "                                        ");

	if ( command_lock == 1 )
		strcpy ( lock_string, "LOCKED  " );
	else
		strcpy ( lock_string, "UNLOCKED" );

	tune_screen = TUNE_AZEL_SCREEN;

	return_systemid ( &systemid );

} /* end of initialise_tune_decoder */


/*
Put in screen task
*/
void tune_parser ( ) {

	/* start timer for monitor task */
	decode_mbx_mess ( &addr_map, 1);

	if ( command_lock )
		strcpy ( error_string, "You must enter a password.                  " );

} /* end of tune_parser */



/*
**************************************************************
return_display_string

	Routine to return the display string for data entry purposes.

	Normally called by screen task.

Parameters:
	address of string to copy array into.

**************************************************************
*/
void return_display_string ( unsigned char * dest ) {


	strcpy ( (char *)dest, (char *)display_string );

} /* end of return_display_string */


/*
**************************************************************
return_parser_par_copy

	Routine to return the parsers copy of the control parameters.

	Normally called by screen task.

    There is no need here for critical section protection.

Parameters:
	none

Returns:
	pospar structure of current active axis

**************************************************************
*/
controller_parameter_struct return_parser_par_copy ( int * drive_type ) {

	*drive_type = axis_type;

	return ( pospar [ axis_type ] );


} /* end of return_parser_par_copy */


/*
**************************************************************
return_error_string

	Routine to return the display string for data entry purposes.

	Normally called by screen task.

Parameters:
	address of string to copy array into.

**************************************************************
*/
void return_error_string ( unsigned char * dest ) {


	strcpy ( (char *)dest, (char *)error_string );

} /* end of return_error_string */

/*
**************************************************************
return_lock_string

	Routine to return the display string for data entry purposes.

	Normally called by screen task.

Parameters:
	address of string to copy array into.

**************************************************************
*/
void return_lock_string ( unsigned char * dest ) {

	strcpy ( (char *)dest, (char *)lock_string );

} /* end of return_lock_string */

/*
**************************************************************
init_con_parameters

	This routine is called when the screen task is initialised

**************************************************************
*/
void init_con_parameters ( ) {

	get_controller_parameters ( EL_DRIVE, &pospar [ EL_DRIVE ] );
	get_controller_parameters ( AZ_DRIVE, &pospar [ AZ_DRIVE ] );

} /* end of init_con_parameters */

/*
**************************************************************
get_parameters_from_nvram

	This routine is called when the user enters a 'get' command.

**************************************************************
*/
void get_parameters_from_nvram ( ) {

	get_controller_parameters ( axis_type, &pospar [ axis_type ] );

} /* end of get_parameters_from_nvram */



/*
**************************************************************
set_parameters_into_nvram

	This routine is called when the user enters a 'set' command.

**************************************************************
*/
void set_parameters_into_nvram ( ) {

	put_controller_parameters ( axis_type, &pospar [ axis_type ] );

} /* end of set_parameters_into_nvram */


/*
**************************************************************
return_tune_screen

	Routine to return the help code for the tune screen.

	Normally called by screen task.

    There is no need here for critical section protection.

Parameters:
	none

Returns:
	screen code ( see tparser.h )

**************************************************************
*/
char return_tune_screen ( void ) {

	return ( tune_screen );

} /* end of return_tune_screen */

 
/*
**************************************************************
return_axis_tune_data_out 

	Routine to return the axis that we will be spitting data out
for.

	Normally called by tuneout.c module.

Parameters:
	none

Returns:
	int axis_tune_data_out

**************************************************************
*/
int return_axis_tune_data_out ( ) {

	return ( axis_tune_data_out );

} /* end of return_axis_tune_data_out */


 
/*
**************************************************************
return_dataport_switch

	Routine to return a code to determine how the dataport will
	operate.

	Normally called by sequencer module.

Parameters:
	none

Returns:
	char code = 0 Normal dataport (spew_data once a second)
				1 Tune data (once every control period)
				2 Stop dataport

**************************************************************
*/
unsigned char return_dataport_switch ( ) {

	return ( dataport_switch );

} /* end of return_dataport_switch */






/*--------------------------- End of Tuning Parser --------------------*/