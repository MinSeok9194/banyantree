/*=====================(C) COPYRIGHT 2009 Insem Inc.=========================
program 		: KROAD ALARM CONTROLLER  
processor	: STM32F103xx
compiler		: IAR 4.41A Compiler 
program BY 	: YN.Kim
date 			: 2009.	  .
copy right 	: Insem Inc.
===========================================================================*/
 
/* Function Prototypes */
void internal_exec_proc(void);
void adpcm_stop(void);

void Thermo_sensor_execution(unsigned char index);
void stay_temp_control(unsigned char index);
void going_out_temp_control(unsigned char index);
void empty_temp_control(unsigned char index);
void fan_speed_control(unsigned char speed_value, unsigned char index);
void Thermo_sensor_ondol_execution(unsigned char index);
void stay_ondol_control(unsigned char index);
void going_out_ondol_control(unsigned char index);
void empty_ondol_control(unsigned char index);
void ondol_on(unsigned char index);
void ondol_off(unsigned char index);
void fcu_4pipe_check(void);




s8 Moneyexit_cal(u16 money);

//=======================================================================
//=======================================================================

#pragma pack(1)
typedef struct {
	union {
		struct {
			u8 m5000;
			u8 m1000;
			u16 mcoin_total;
		}m;
	}morder;
	
	u8 Dispenser_order;
	u16 Dispense_Total_Momey;
	u16 Dispense_Coin;
	u8 Dispense_Note1k;
	u8 Dispense_Note5k;

	u32 Complete_Dispense_Money;
	s8 Dispense_op;
	u8 Dispenser_Error;
}__Morder;
				
#pragma pack(1)
typedef struct {
	struct {
		unsigned Coin_Complete	:1;
		unsigned Note_Complete	:1;
	}s;
	u32 Complete_Dispense_Money;
}_Order_Err;

typedef union {
	_Order_Err stat;
	u8 buf[sizeof(_Order_Err)];
}__Order_Err;

//=======================================================================
#ifdef __IEXEC_H__
const u8 b_decimal_to_hex_table_ie[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee
};

#define ieEX

#else
#define ieEX extern
#endif

ieEX __Morder Morder, _Morder;
ieEX __Order_Err Order_Err;

//=======================================================================
ieEX __bits exec;
#define mop_f	exec.s.b0
#define i_reset_f	exec.s.b1



ieEX u8  sq_count;
ieEX u8  fcu_check_count;
ieEX u8  fan_mode_back[8];
ieEX u8  air_floor_mode_back[8];
//=======================================================================
