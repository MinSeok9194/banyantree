/*=====================(C) COPYRIGHT 2014 PLUS-H.=========================
program		    : CONTROL BOX(RCS-300-RCU)
processor 	  : STM32F103xx
compiler		  : IAR 6.2A Compiler 
program BY	  : H.H.Hwang
date			    : 2014.	 .
copy right	  : P L U S  -  H. 
============================================================================*/
#ifndef __MAIN__
#define __MAIN__

#ifdef  __MAIN_H__
#define maEx   
#else
#define maEx extern
#endif


/******************************** structure ***********************************************/

/******************************** Define **************************************************/
#define debug_mode
#define version_no		0x1010

/******************************** Include **************************************************/
#include "stm32f10x_lib.h"
#include "bit_define.h"
#include "stm32f10x_init.h"

#include "uart1.h"
#include "uart2.h"
#include "uart3.h"


#include "24lcxx.h"
#include "iexec.h"

#include "switch_io.h"
/******************************** Bit Define **********************************************/


/******************************** global parameter ****************************************/

itEx u8 room_data_buf[200];
itEx u8 ts_air_set_temp_from_PC[4];
itEx u8 ts_floor_set_temp_from_PC[4];


itEx u8 ts_set_temp_send_count[4];
itEx u8 ts_switch_toggle_bit[4];
itEx u8 ts_first_power_on[4];
itEx u8 f_first_nt_power_on[4];
itEx u8 f_first_ls_power_on[4];
itEx u8 f_first_ks_power_on[4];

itEx u8  flag_fcu_action_time[4];
itEx u16 fcu_action_time[4];
itEx u8  flag_floor_action_time[4];
itEx u16 floor_action_time[4];




/******************************** Function ************************************************/

#endif
/******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/
