/*
 * simplemotion_defs.h
 * This file contains addresses for simple motion parameters (SMP's) and if parameter is a bit field
 * or enum (low number of choices), then also bits or choices in the register are defined here.
 *
 *  Created on: 25.2.2012
 *      Author: Tero
 */


/* SMV2 address space areas:
 * 0 NULL, can read or write but has no effect
 * 1-63 SM bus specific parameters
 * 128-167 digital I/O registers for GPIO
 * 168-199 analog I/O reigisters for GPIO
 * 201-8190 application specific parameter addresses. in this case motor drive registers
 * 8191 NOP command. if SM command SMPCMD_SET_PARAM_ADDR with SMP_ADDR_NOP as parameter is sent, no actions are taken in nodes. it's NOP (no operation) command.
 */


#ifndef SIMPLEMOTION_DEFS_H_
#define SIMPLEMOTION_DEFS_H_

/*
 * bit shift macros
 */

#ifndef BV
//bitvise shifts
#define BV(bit) (1<<(bit))
#define BVL(bit) (1L<<(bit))
#endif


//SMP command types
#define SMPCMD_SETPARAMADDR 2
#define SMPCMD_24B 1
#define SMPCMD_32B 0
#define SMPRET_OTHER 3
#define SMPRET_16B 2
#define SMPRET_24B 1
#define SMPRET_32B 0


/*
 * SimpleMotion V2 constants
 */

/* broadcast address, if packet is sent to SM_BROADCAST_ADDR then all nodes in the network
 * will read it but no-one will send a reply because it would cause packet collision*/
 #define SM_BROADCAST_ADDR 0

//these bits define attributes over parameter address to retreive different types of values as return data
#define SMP_VALUE_MASK  0x0000
#define SMP_MIN_VALUE_MASK  0x4000 //so requesting i.e. param (SMP_TIMEOUT|SMP_MIN_VALUE_MASK) will give the minimum settable value for param SMP_TIMEOUT
#define SMP_MAX_VALUE_MASK  0x8000
//mask to filter attributes
#define SMP_ATTRIBUTE_BITS_MASK  0xC000//C=1100
//mask for addresses
#define SMP_ADDRESS_BITS_MASK  0x1FFF //E=1110


//SMP packet header bits (2 bits). These determine content length and type
#define SMPCMD_SET_PARAM_ADDR 2
#define SMPCMD_24B 1
#define SMPCMD_32B 0
#define SMPRET_CMD_STATUS 3
#define SMPRET_16B 2
#define SMPRET_24B 1
#define SMPRET_32B 0

//if command is to set param addr to this -> NOP
#define SMP_ADDR_NOP 0x3fff

//SMPCommand values that are returned with every SMPCommand when SMPRET_OTHER set as SMP_RETURN_PARAM_LEN:
#define SMP_CMD_STATUS_ACK 0
#define SMP_CMD_STATUS_NACK 1
#define SMP_CMD_STATUS_INVALID_ADDR 2
#define SMP_CMD_STATUS_INVALID_VALUE 4
#define SMP_CMD_STATUS_VALUE_TOO_HIGH 8
#define SMP_CMD_STATUS_VALUE_TOO_LOW 16
#define SMP_CMD_STATUS_OTHER_MASK32 (3L<<30) //1byte packet. payload codes:

//params 0-63 are reserved for SM specific params, application specifics are >64-8090
#define SMP_SM_RESERVED_ADDRESS_SPACE 63

//SMP=Simple motion parameter
#define SMP_NULL 0
#define SMP_NODE_ADDRSS 1
#define SMP_BUS_MODE 2
#define SMP_SM_VERSION 3
#define SMP_SM_VERSION_COMPAT 4
#define SMP_BUS_SPEED 5
#define SMP_BUFFER_FREE_BYTES 6
#define SMP_BUFFERED_CMD_STATUS 7
#define SMP_BUFFERED_CMD_PERIOD 8
#define SMP_RETURN_PARAM_ADDR 9
#define SMP_RETURN_PARAM_LEN 10
#define SMP_TIMEOUT 12
#define SMP_CUMULATIVE_STATUS 13 //error bits are set here if any, (SMP_CMD_STATUS_... bits). clear by writing 0

//bit mask
#define SM_BUFCMD_STAT_IDLE 1
#define SM_BUFCMD_STAT_RUN 2
#define SM_BUFCMD_STAT_UNDERRUN 4
#define SM_BUFCMD_STAT_OVERRUN 8

//each DIGITAL variable is 16 bits thus holds 16 i/o's
#define SMP_DIGITAL_IN_VALUES_1 128
#define SMP_DIGITAL_IN_VALUES_2 129
#define SMP_DIGITAL_IN_VALUES_3 130
#define SMP_DIGITAL_IN_VALUES_4 131

#define SMP_DIGITAL_IN_CHANGED_1 132
#define SMP_DIGITAL_IN_CHANGED_2 133
#define SMP_DIGITAL_IN_CHANGED_3 134
#define SMP_DIGITAL_IN_CHANGED_4 135

#define SMP_DIGITAL_OUT_VALUE_1 136
#define SMP_DIGITAL_OUT_VALUE_2 137
#define SMP_DIGITAL_OUT_VALUE_3 138
#define SMP_DIGITAL_OUT_VALUE_4 139

//set IO pin direction: 0=input, 1=output. May not be supported by HW.
#define SMP_DIGITAL_IO_DIRECTION_1 140
#define SMP_DIGITAL_IO_DIRECTION_2 141
#define SMP_DIGITAL_IO_DIRECTION_3 142
#define SMP_DIGITAL_IO_DIRECTION_4 143

#define SMP_ANALOG_IN_VALUE_1 168
#define SMP_ANALOG_IN_VALUE_2 169
#define SMP_ANALOG_IN_VALUE_3 170
#define SMP_ANALOG_IN_VALUE_4 171
#define SMP_ANALOG_IN_VALUE_5 172
// continue to .. 183

#define SMP_ANALOG_OUT_VALUE_1 184
#define SMP_ANALOG_OUT_VALUE_2 185
#define SMP_ANALOG_OUT_VALUE_3 186
#define SMP_ANALOG_OUT_VALUE_4 187
#define SMP_ANALOG_OUT_VALUE_5 188
// continue to .. 200


/*
 * List of motor control parameters.
 *
 * WARNING: Don't assume anything about these variables! Refer to SimpleMotion V2 documentation
 * for details. If unknown, ask Granite Devices.
 */

 //control setpoint values (pos, vel or torq depending on control mode):
#define SMP_INCREMENTAL_SETPOINT 550
#define SMP_ABSOLUTE_SETPOINT 551
//old names for backwards compatibility:
#define SMP_INCREMENTAL_POS_TARGET SMP_INCREMENTAL_SETPOINT
#define SMP_ABSOLUTE_POS_TARGET SMP_ABSOLUTE_SETPOINT

#define SMP_FAULTS 552
	//bitfield bits:
	#define FLT_FOLLOWERROR	BV(1)
	#define FLT_OVERCURRENT BV(2)
	#define FLT_COMMUNICATION BV(3)
	#define FLT_ENCODER	BV(4)
	#define FLT_OVERTEMP BV(5)
	#define FLT_UNDERVOLTAGE BV(6)
	#define FLT_OVERVOLTAGE BV(7)
	#define FLT_PROGRAM_OR_MEM BV(8)
	#define FLT_HARDWARE BV(9)
	#define FLT_OVERVELOCITY BV(10)
	#define FLT_INIT BV(11)
	#define FLT_MOTION BV(12)
	#define FLT_RANGE BV(13)
	#define FLT_PSTAGE_FORCED_OFF BV(14)
	#define FLT_HOST_COMM_ERROR BV(15)
	//IO side macros
	#define FLT_GC_COMM BV(15)
	#define FLT_QUEUE_FULL FLT_PROGRAM_OR_MEM
	#define FLT_SM485_ERROR FLT_COMMUNICATION
	#define FLT_FIRMWARE FLT_PROGRAM_OR_MEM //non-recoverable program error
	#define FLT_ALLOC FLT_PROGRAM_OR_MEM //memory etc allocation failed

#define SMP_FIRST_FAULT 8115
#define SMP_STATUS 553
	//bitfield bits:
	#define STAT_POWER_ON BV(0)
	#define STAT_TARGET_REACHED BV(1)//this is 1 when trajectory planner target reached
	#define STAT_FERROR_RECOVERY BV(2)
	#define STAT_RUN BV(3)//run is true only if motor is being actually driven. run=0 clears integrators etc
	#define STAT_ENABLED BV(4)
	#define STAT_FAULTSTOP BV(5)
	#define STAT_FERROR_WARNING BV(6)//follow error warning, recovering or disabled
	#define STAT_STO_ACTIVE BV(7)
	#define STAT_SERVO_READY BV(8)//ready for user command: initialized, running (no fault), not recovering, not homing & no homing aborted, not running sequence
	#define STAT_BRAKING BV(10)
	#define STAT_HOMING BV(11)
	#define STAT_INITIALIZED BV(12)
	#define STAT_VOLTAGES_OK BV(13)
	#define STAT_PERMANENT_STOP BV(15)//outputs disabled until reset

#define SMP_SYSTEM_CONTROL 554 //writing 1 initiates settings save to flash, writing 2=device restart, 4=abort buffered motion
	//possible values listed
	//save active settings  to flash:
	#define SMP_SYSTEM_CONTROL_SAVECFG 1
	//restart drive:
	#define SMP_SYSTEM_CONTROL_RESTART 2
	//abort buffered commands (discard rest of buffer)
	#define SMP_SYSTEM_CONTROL_ABORTBUFFERED 4
	//next 3 for factory use only:
    #define SMP_SYSTEM_SAMPLE_TEST_VARIABLES 8 //production testing function
    #define SMP_SYSTEM_START_PRODUCTION_TEST 16 //production testing function
    #define SMP_SYSTEM_STOP_PRODUCTION_TEST 32 //production testing function

	//follow error tolerance for position control:
#define SMP_POS_FERROR_TRIP 555
//velocity follow error trip point. units: velocity command (counts per PIDcycle * divider)
#define SMP_VEL_FERROR_TRIP 556
//recovery velocity from disabled/fault state. same scale as other velocity limits
#define SMP_POS_ERROR_RECOVERY_SPEED 557

//motor type:
#define SMP_MOTOR_MODE 558
	//factory state, no choice:
	#define MOTOR_NONE 0
    //DC servo:
    #define MOTOR_DC 1
	//2 phase AC motor
    #define MOTOR_AC_VECTOR_2PHA 2  /*2 phase ac or bldc */
	//3 phase AC motor
    #define MOTOR_AC_VECTOR 3 /*3 phase ac or bldc */
	//for drive internal use only:
	#define _MOTOR_LAST 7

//#define CFG_INPUT_FILTER_LEN 30
#define SMP_CONTROL_MODE 559
	//control mode choices:
	#define CM_TORQUE 3
	#define CM_VELOCITY 2
	#define CM_POSITION 1
#define SMP_INPUT_MULTIPLIER 560
#define SMP_INPUT_DIVIDER 561
//setpoint source
#define SMP_INPUT_REFERENCE_MODE 562
	//choices:
	#define SMP_INPUT_REFERENCE_MODE_SERIALONLY 0
	#define SMP_INPUT_REFERENCE_MODE_PULSETRAIN 1
	#define SMP_INPUT_REFERENCE_MODE_QUADRATURE 2
	#define SMP_INPUT_REFERENCE_MODE_PWM 3
	#define SMP_INPUT_REFERENCE_MODE_ANALOG 4
	#define SMP_INPUT_REFERENCE_MODE_RESERVED1 5
	#define SMP_INPUT_REFERENCE_MODE_RESERVED2 6
	#define SMP_INPUT_REFERENCE_MODE_LAST_ 6

//setpoint source offset when absolute setpoint is used (pwm/analog)
#define SMP_ABS_IN_OFFSET 563
//scaler, not used after VSD-E
#define SMP_ABS_IN_SCALER 564

//FB1 device resolution, pulses per rev
#define SMP_ENCODER_PPR 565
//motor polepairs, not used with DC motor
#define SMP_MOTOR_POLEPAIRS 566


//flag bits & general
#define SMP_DRIVE_FLAGS 567
	//bitfield bits:
	#define FLAG_DISABLED_AT_STARTUP BV(0)
	#define FLAG_NO_DCBUS_FAULT BV(1)
	#define FLAG_INVERT_ENCODER BV(3)
	#define FLAG_INVERT_MOTOR_DIRECTION BV(4) /*invert positive direction*/
	#define FLAG_DISABLE_DEADTIMECORR_LO_SPEED BV(5)
	#define FLAG_USE_PULSE_IN_ACCEL_LIMIT BV(7)
	#define FLAG_2PHASE_AC_MOTOR BV(9)
	#define FLAG_ALLOW_VOLTAGE_CLIPPING BV(10)
	#define FLAG_USE_INPUT_LP_FILTER BV(11)
	#define FLAG_USE_PID_CONTROLLER BV(12)//PIV is the default if bit is 0
	#define FLAG_INVERTED_HALLS BV(13)
	#define FLAG_USE_HALLS BV(14)
    #define FLAG_MECH_BRAKE_DURING_PHASING BV(15)
#define SMP_MOTION_FAULT_THRESHOLD 568
#define SMP_HV_VOLTAGE_HI_LIMIT 569
#define SMP_HV_VOLTAGE_LOW_LIMIT 570
#define SMP_MOTOR_MAX_SPEED 572


//primary feedback loop 200-299
#define SMP_VEL_I 200
#define SMP_POS_P 201
#define SMP_VEL_P 202
#define SMP_VEL_FF 220
#define SMP_ACC_FF 221

//anti dither limits
#define SMP_ANTIDITHER_MODE 230

//secondary feedback loop 300-399
//NOT IMPLEMENTED YET

//current loop related 400-499
//direct torque gains, not used after VSD-E
#define SMP_TORQ_P 401
#define SMP_TORQ_I 402
//motor inductance and resistance
#define SMP_MOTOR_RES 405
#define SMP_MOTOR_IND 406

//continuous current limit mA
#define SMP_TORQUELIMIT_CONT 410
//peak current limit mA
#define SMP_TORQUELIMIT_PEAK 411
//homing current limit mA
#define SMP_TORQUELIMIT_HOMING 415
//next 2 set fault sensitivity
#define SMP_TORQUEFAULT_MARGIN 420
#define SMP_TORQUEFAULT_OC_TOLERANCE 421

//motor thermal time constant in seconds:
#define SMP_MOTOR_THERMAL_TIMECONSTANT 430
//how fast to ramp up voltage during phase search:
#define SMP_PHASESEARCH_VOLTAGE_SLOPE 480
//by default this is calculated from other motor params:
#define SMP_PHASESEARCH_CURRENT 481
//selector value 0-9 = 100 - 3300Hz (see Granity):
#define SMP_TORQUE_LPF_BANDWIDTH 490

//motor rev to rotary/linear unit scale. like 5mm/rev or 0.1rev/rev. 30bit parameter & scale 100000=1.0
#define SMP_AXIS_SCALE 491
//output unit 0=mm 1=um 2=inch 3=revolutions 4=degrees
#define SMP_AXIS_UNITS 492
//0=none 1=qei1 2=qei2 3=resolver 4=ssi 5=biss
#define SMP_FB1_DEVICE_SELECTION 493
#define SMP_FB2_DEVICE_SELECTION 494
//in 1/2500 seconds.
#define SMP_GOAL_FAULT_FILTER_TIME 495
//in speed scale
#define SMP_OVERSPEED_FAULT_LIMIT 496
//0=min 6=max
#define SMP_OVERCURRENT_FAULT_SENSITIVITY 497
//when hit limt switch, do: 0=nothing, 1=disable torque, 2=fault stop, 3=servoed stop
#define SMP_LIMIT_SW_FUNCTION 498
		//choices:
        #define SMP_LIMIT_SW_DISABLED 0
        #define SMP_LIMIT_SW_NOTORQUE 1
        #define SMP_LIMIT_SW_FAULTSTOP 2
        #define SMP_LIMIT_SW_SERVOSTOP 3
        #define _SMP_LIMIT_SW_LAST 3

#define SMP_CONTROL_BITS1 2533
	//bitfiled values:
	//CB1 & CB2 enables must be 1 to have drive enabled
	//Control bits 1 are controlled by host software
	#define SMP_CB1_ENABLE BV(0)
	#define SMP_CB1_CLEARFAULTS BV(1)
	#define SMP_CB1_QUICKSTOP BV(2)
	#define SMP_CB1_USE_TRAJPLANNER BV(3)
	#define SMP_STATIC_CBS1 (SMP_CB1_ENABLE|SMP_CB1_USE_TRAJPLANNER)

#define SMP_CONTROL_BITS2 2534
	//bitfiled values:
	//both enables must be 1 to have drive enabled
	//Control bits 2 are controlled by physical inputs
	#define SMP_CB2_ENABLE BV(0)
	#define SMP_CB2_ENA_POS_FEED BV(1)
	#define SMP_CB2_ENA_NEG_FEED BV(2)
	#define SMP_CB2_HOMESW_ON BV(3)
    #define SMP_CB2_CLEARFAULTS BV(4)

//trajectory planner
#define SMP_TRAJ_PLANNER_ACCEL 800
#define SMP_TRAJ_PLANNER_STOP_DECEL 801
#define SMP_TRAJ_PLANNER_VEL 802
//accel limit for homing & ferror recovery
//if set to 0, use CFG_TRAJ_PLANNER_ACCEL instead (for backwards compatibility)
#define SMP_TRAJ_PLANNER_HOMING_ACCEL 803
#define SMP_TRAJ_PLANNER_HOMING_VEL 804
#define SMP_TRAJ_PLANNER_HOMING_BITS 806
	//homing config bit field. illegal to set Not to use INDEX or HOME
	#define HOMING_POS_INDEX_POLARITY BV(0)
	#define HOMING_POS_INDEX_SEARCH_DIRECTION BV(1)
	#define HOMING_POS_HOME_SWITCH_POLARITY BV(2)
	#define HOMING_USE_HOME_SWITCH BV(3)
	#define HOMING_USE_INDEX_PULSE BV(4)
	#define HOMING_HOME_AT_POWER_ON BV(5)
	#define HOMING_FULL_SPEED_OFFSET_MOVE BV(6)
	#define HOMING_ENABLED BV(7) /*if 0, homing cant be started */
	#define _HOMING_CFG_MAX_VALUE 0x00ff

//starting and stopping homing:
#define SMP_HOMING_CONTROL 7532


//hard stop triggered by follow error in encoder counts
#define SMP_TRAJ_PLANNER_HOMING_HARD_STOP_THRESHOLD 808
//32bit position offset in encoder counts, value must not overflow on divider*offset
#define SMP_TRAJ_PLANNER_HOMING_OFFSET 823
//signed 32bit absolute position limits, active after homing. disabled when both=0
#define SMP_ABSPOSITION_HI_LIMIT 835
#define SMP_ABSPOSITION_LO_LIMIT 836

//readouts
#define SMP_ACTUAL_BUS_VOLTAGE 900
#define SMP_ACTUAL_TORQUE 901
#define SMP_ACTUAL_VELOCITY_FB 902
#define SMP_ACTUAL_POSITION_FB 903
#define SMP_SCOPE_CHANNEL_VALUE 904
#define SMP_SCOPE_CHANNEL_SELECT 905

#define SMP_MECH_BRAKE_RELEASE_DELAY 910
#define SMP_BRAKE_STOP_ENGAGE_DELAY 911
#define SMP_DYNAMIC_BRAKING_SPEED 912


//////////////////////////////////////////////////////////////////////////////////RUNTIME PARAMS 5000-9999

#define SMP_CAPTURE_SOURCE 5020
	//bitfield values (shift these with BV())
	#define CAPTURE_TORQUE_TARGET 1
	#define CAPTURE_TORQUE_ACTUAL 2
	#define CAPTURE_VELOCITY_TARGET 3
	#define CAPTURE_VELOCITY_ACTUAL 4
	#define CAPTURE_POSITION_TARGET 5
	#define CAPTURE_POSITION_ACTUAL 6
	#define CAPTURE_FOLLOW_ERROR 7
	#define CAPTURE_OUTPUT_VOLTAGE 8
	#define CAPTURE_BUS_VOLTAGE 9
	#define CAPTURE_STATUSBITS 10
	#define CAPTURE_FAULTBITS 11
	#define CAPTURE_P_OUT 12
	#define CAPTURE_I_OUT 13
	#define CAPTURE_D_OUT 14
	#define CAPTURE_FF_OUT 15
	#define CAPTURE_RAW_POS 25
	//8 bit signed values combined, only for return data, not for scope
	#define CAPTURE_TORQ_AND_FERROR 26

	//rest are availalbe in debug/development firmware only:
	#define CAPTURE_PWM1 16
	#define CAPTURE_PWM2 17
	#define CAPTURE_PWM3 18
	#define CAPTURE_DEBUG1 19
	#define CAPTURE_DEBUG2 20
	#define CAPTURE_CURRENT1 21
	#define CAPTURE_CURRENT2 22
	#define CAPTURE_ACTUAL_FLUX 23
	#define CAPTURE_OUTPUT_FLUX 24

#define SMP_CAPTURE_TRIGGER 5011
	//choices:
	#define TRIG_NONE 0
	#define TRIG_INSTANT 1
	#define TRIG_FAULT 2
	#define TRIG_TARGETCHANGE 3
	#define TRIG_TARGETCHANGE_POS 4
	#define TRIG_SERIALCMD 5

#define SMP_CAPTURE_SAMPLERATE 5012
//rdonly
#define SMP_CAPTURE_BUF_LENGHT 5013
//this is looped 0-n to make samples 0-n readable from SMP_CAPTURE_BUFFER_GET_VALUE
#define SMP_CAPTURE_BUFFER_GET_ADDR 5333
#define SMP_CAPTURE_BUFFER_GET_VALUE 5334

//#define RUNTIME_FEATURES1 6000
#define SMP_SERIAL_NR 6002
#define SMP_DRIVE_CAPABILITIES 6006
#define SMP_FIRMWARE_VERSION 6010
#define SMP_FIRMWARE_BACKWARDS_COMP_VERSION 6011
#define SMP_GC_FIRMWARE_VERSION 6014
#define SMP_GC_FIRMWARE_BACKWARDS_COMP_VERSION 6015
//added in ioni:
#define SMP_FIRMWARE_BUILD_REVISION 6016
#define SMP_DEVICE_TYPE 6020
#define SMP_PID_FREQUENCY 6055

//affects only in MC_VECTOR & MC_BLDC mode
//used in drive development only, do not use if unsure
#define SMP_OVERRIDE_COMMUTATION_ANGLE 8000
#define SMP_TORQUE_CMD_OVERRIDE 8001
#define SMP_OVERRIDE_COMMUTATION_FREQ 8003
#define SMP_OVERRIDE_REGENRES_DUTY 8004

/*IO side CPU sends encoder counter at index every time index is encountered. homing uses this info */
#define SMP_INDEX_PULSE_LOCATION 8005


//GC side debug params
#define SMP_DEBUGPARAM1 8100
#define SMP_DEBUGPARAM2 8101
#define SMP_DEBUGPARAM3 8102
//for STM32 side override
#define SMP_DEBUGPARAM4 8103
#define SMP_DEBUGPARAM5 8104
#define SMP_DEBUGPARAM6 8105
#define SMP_FAULT_LOCATION1  8110 //this is for GraniteCore side
#define SMP_FAULT_LOCATION2  8111 //this is for Argon IO side
#define SMP_OVERRIDE_PARAM_ADDR_1 8120
#define SMP_OVERRIDE_PARAM_ADDR_2 8121
#define SMP_OVERRIDE_PARAM_ADDR_3 8122
#define SMP_OVERRIDE_PARAM_VAL_1 8125
#define SMP_OVERRIDE_PARAM_VAL_2 8126
#define SMP_OVERRIDE_PARAM_VAL_3 8127


/*DFU Bootloader parameters begin:*/
//SMP_FAULTS bitfield bits:
#define FLT_INVALID_FW BV(5)
#define FLT_FLASHING_FAIL BV(6)
#define FLT_FLASHING_COMMSIDE_FAIL BV(7)
//SMP_STATUS bitfield bits:
#define STAT_GC_FLASHED_ONCE BV(1)//to prevent flashing it twice in same GCBL session which causes error

//write a value here to execute a bootloader function
#define SMP_BOOTLOADER_FUNCTION 191
//upload 16bits of data to buffer. max buffer length 4096 bytes before it must be written with "write block" function
//total uploaded amount must be multiple of 4 bytes before issuing write function
#define SMP_BOOTLOADER_UPLOAD 192
//bootloaded status
#define SMP_BOOTLOADER_STAT 193
        #define BOOTLOADER_STAT_FLASH_VERIFIED_OK BV(0)

/*DFU Bootloader parameters end*/



#endif /* SIMPLEMOTION_DEFS_H_ */
