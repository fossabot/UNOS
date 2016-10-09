/***************************************************************************
								TIED SOFTWARE
 PROJECT	:- TS3000

 MODULE 	:- GLOBALH

 FILE 		:- ERR_NUMS_H

 PROGRAMMER :- R Cockerell

 Copyright 1994. The University of Newcastle Research Associates Ltd.

****************************************************************************
* POLYTRON VERSION CONTROL SOFTWARE _ PVCS (tm) Information
*
*      $Revision:   1.1.1.0  $
*      $Date:   29 Apr 1994 19:41:14  $
*
****************************************************************************/

#ifndef __ERR_NUMS_H
#define __ERR_NUMS_H

/* Errors reported by the Command Parser module. */
#define CONTROL_POINT_TIMEOUT_ERR		1
#define INVALID_LCMU_COMMAND_ERR		2
#define INVALID_RCMU_COMMAND_ERR		3
#define LCMU_RX_TIMEOUT_ERR				4
#define RCMU_RX_TIMEOUT_ERR				5
#define	UNOS_ERR						6
#define	PARSER_NVRAM_GET_ERROR			7
#define	ORBIT_INIT_NVRAM_FAIL			7
#define SEQ_NVRAM_ERR					7
#define	BEACON_DATA_NVRAM_ERROR			7

/* Errors reported by the Orbit Track module. */
#define COLD_START_FAILURE				8
#define WARM_START_FAIL					9
#define SCAN_TIMEOUT					10
#define SCAN_OUT_OF_RANGE				11
#define SCAN_DATA_NVRAM_FAIL			12
#define	SCAN_TERMINATED_OK				13
#define ORBIT_TRACK_WARNING				14
#define ORBIT_TRACK_ERROR				15
#define ORBIT_TRACK_TIMEOUT				16
#define ORBIT_CHANGE_WARNING			17
#define POSITION_SETTLING_TIMEOUT		18

/* The following errors are reported by the Sequencer module. */

#define UNABLE_TO_CONTROL_ERR			19
#define DRIVE_INTERFACE_ERR				20
#define DRIVE_START_FAILURE_ERR			21
#define DRIVE_SYSTEM_FAILURE			22

/* Addition by RHM 7/5/92, for new errors from the encoders. */
#define AZ_LOW_ACCURACY					23
#define EL_LOW_ACCURACY					24

#define INVALID_SEQ_COMMAND_ERR			25
#define AZ_CW_SW_LIMIT_ERR				26
#define AZ_CCW_SW_LIMIT_ERR				27
#define EL_UP_SW_LIMIT_ERR				28
#define EL_DOWN_SW_LIMIT_ERR			29

#define AZ_FINAL_LIMIT_ERR			    30

#define	DRIVE_OFF_ERR					31
#define PLC_FORCED_OFF_ERR				32
#define AZCMD_CW_LIMIT_ERR				33
#define AZCMD_CCW_LIMIT_ERR				34
#define ELCMD_UP_LIMIT_ERR				35
#define ELCMD_DOWN_LIMIT_ERR			36

#define AZ_PRELIMIT_CW_ERR				37
#define AZ_PRELIMIT_CCW_ERR				38

#define INTELSAT_DATA_OLD               39

/* ACS fuses F1 and F2 states are reported to the CMU
	by the ATCU, as they control inputs to the ATCU. */
#define ACS_FUSE_F1_BLOWN 				40
#define ACS_FUSE_F2_BLOWN               41

#define INTELSAT_DATA_DECAYING          42

#define COLD_START_WITH_GOOD_MODEL	43 /* CONFIRM-STARTS mod. */
#define WARM_START_WITH_BAD_MODEL	44 /* CONFIRM-STARTS mod. */

#ifdef SETAS
#define CONTROL_PORT_CHECKSUM			45
#define CONTROL_PORT_SYNTAX				46
#define CONTROL_PORT_PROTOCOL			47
#else
#define EL_PRELIMIT_UP_ERR				45
#define EL_PRELIMIT_DOWN_ERR			46
#define EL_FINAL_LIMIT_ERR				47
#endif

/* Errors reported by the Beacon module */
/* New error numbers 48-51 added 5/2/92 by PJM */
#define BEACON_POLL_ERROR               48
#define BEACON_LEVEL_UNAVAILABLE        49
#define BEACON_LEVEL_UNDERFLOW          50
#define BEACON_LEVEL_OVERFLOW			51
#define	BEACON_IS_HIGH					52
#define	BEACON_IS_LOW					53
#define	BEACON_RATE_HIGH				54
#define	BEACON_COMMS_ERROR				55
#define	BEACON_PROTOCOL_ERROR			56
#define BEACON_OSCILLATOR_OUT_OF_LOCK   57
#define BEACON_RESPONSE_TIMEOUT	        59    /* Added by DJLB */

/* Errors reported by the I/O Module	*/
#define	AZ_ENCODER_NO_RESPONSE			60
#define	EL_ENCODER_NO_RESPONSE			61
#define	AZ_ENCODER_FAULT				62
#define	EL_ENCODER_FAULT				63

/* Errors reported by the Error Handler module. */
#define	INVALID_ERROR_CODE_ERR			64

#endif /* __ERR_NUMS_H */
