/*******************************(C) COPYRIGHT 2007 INSEM Inc.****************************************/
/* processor 	  : CORETEX-M3(STM32F10X)         		    				    */
/* compiler       : EWARM Compiler								    */
/* program by	  : YN.Kim									    */
/* History:											    */
/* 04/13/2007     : Version 1.0									    */
/* copy right	  : Insem Inc.									    */
/****************************************************************************************************/

/******************************** Function ************************************************/
void Internal_timer_Proc(void);
void Timer_setup(u8 tmno, u16 timer); 
void CORTEX_initial(void);
void Gp_initial(void);
void PLL_initial(void);
void NVIC_initial(void);
void SYSTICK_initial(void);
void GPIO_port_initial(void);
void Iwdog_initial(u16 wtime);

void EXTI_initial(void);
u16 hex2dec(u16);
u16 dec2hex(u16 decimal);
void dip_switch_read(void);
void delay(unsigned int);
void chime_execution(void);
void eeprom_save_check(void);
void eeprom_save_comp(unsigned char eep_addr);
void digital_volume_execution(unsigned char index, unsigned char vol_data);
void spi_clk_out(void);
void eeprom_save_comp_level(unsigned char eep_addr);
void light_ex_input(void);
void delay_us(u8 time_us);
void delay_ms(u8 time_ms);
/******************************** structure ***********************************************/
#define led1		  GpioC->Bit.b13
#define led2		  GpioC->Bit.b3

#define led3   	  GpioA->Bit.b0
#define led4  		GpioB->Bit.b9
#define led5  		GpioB->Bit.b8
#define P_UART3_DIR		GpioB->Bit.b8

#define tcrst   	GpioD->Bit.b7

//========= Relay port1 define =========
#define P_rly_1     GpioE->Bit.b15    //cn2.1
#define P_rly_2     GpioE->Bit.b14    //cn2.2
#define P_rly_3     GpioE->Bit.b13    //cn2.3
#define P_rly_4     GpioE->Bit.b12    //cn2.4
#define P_rly_5     GpioE->Bit.b11    //cn2.5
#define P_rly_6     GpioE->Bit.b10    //cn2.6
#define P_rly_7     GpioE->Bit.b9     //cn2.7
#define P_rly_8     GpioE->Bit.b8     //cn2.8

#define P_rly_9     GpioD->Bit.b15    //cn3.1
#define P_rly_10    GpioD->Bit.b14    //cn3.2
#define P_rly_11    GpioD->Bit.b13    //cn3.3
#define P_rly_12    GpioD->Bit.b12    //cn3.4
#define P_rly_13    GpioD->Bit.b11    //cn3.5
#define P_rly_14    GpioD->Bit.b10    //cn3.6
#define P_rly_15    GpioD->Bit.b9     //cn3.7
#define P_rly_16    GpioD->Bit.b8     //cn3.8


#define P_ex_led1   GpioE->Bit.b0    //cn4.1
#define P_ex_led2   GpioE->Bit.b1    //cn4.2
#define P_ex_led3   GpioE->Bit.b2    //cn4.3
#define P_ex_led4   GpioE->Bit.b3    //cn4.4
#define P_ind_led   GpioE->Bit.b4    //cn4.5
#define P_ks_led    GpioE->Bit.b5    //cn4.6
#define P_dnd_led   GpioE->Bit.b6    //cn4.7
#define P_mur_led   GpioE->Bit.b7    //cn4.8

#define P_ks_A              GpioC->Bit.b9
#define P_ks_B              GpioC->Bit.b8
#define emergency_switch    GpioC->Bit.b7
#define DRS                 GpioC->Bit.b6
#define base_light_1        GpioB->Bit.b15
#define P_dnd_switch        GpioB->Bit.b14
#define P_chime_switch      GpioB->Bit.b12

#define P_chime_mute        GpioB->Bit.b2
#define P_chime_out         GpioB->Bit.b5
#define P_alarm_out         GpioD->Bit.b6

#define P_tuner_enable      GpioD->Bit.b5
#define P_24lcxx_enable     GpioD->Bit.b4


#define P_dip_0             GpioC->Bit.b0
#define P_dip_1             GpioC->Bit.b1
#define P_dip_2             GpioC->Bit.b2
#define P_dip_3             GpioA->Bit.b11
#define P_dip_4             GpioA->Bit.b12
#define P_dip_5             GpioA->Bit.b15
#define P_dip_6             GpioC->Bit.b10
#define P_dip_7             GpioC->Bit.b11

#define P_dip_rd_0          GpioD->Bit.b0
#define P_dip_rd_1          GpioD->Bit.b1

#define P_volum_sck         GpioB->Bit.b4
#define P_volum_data        GpioB->Bit.b3
#define P_volum_sc_0        GpioD->Bit.b2
#define P_volum_sc_1        GpioC->Bit.b12


#define P_ex_input_0        GpioB->Bit.b1    //cn5.1
#define P_ex_input_1        GpioB->Bit.b0    //cn5.2
#define P_ex_input_2        GpioC->Bit.b5    //cn5.3
#define P_ex_input_3        GpioC->Bit.b4    //cn5.4
#define P_ex_input_4        GpioA->Bit.b7    //cn5.5
#define P_ex_input_5        GpioA->Bit.b6    //cn5.6
#define P_ex_input_6        GpioA->Bit.b5    //cn5.7
#define P_ex_input_7        GpioA->Bit.b4    //cn5.8

/******************************** Define **************************************************/
#define setbit(address,bit) 	(address |= (1<<bit))
#define clrbit(address,bit)  	(address &= ~(1<<bit))
#define chkbit(address,bit)  	(address & (1<<bit))
#define tgbit(address, bit)	((address) ^= 1<<bit)

/******************************** Bit Define **********************************************/
#define tm1ms_f     stm32_init.bit.b0




typedef struct {
	u8 u1c;
	u8 pc;
	u8 u3c;
  	u8 m25wc;
   u8 eeprom;
	u8 keyc;
	u8 erxc;
	u8 rxc;
}_ltm;
typedef union {
	_ltm s;
	u8 buf[sizeof(_ltm)];
}__Extm;

typedef union {
	_ltm s;
	u8 buf[sizeof(_ltm)];
}mcoin_total;

/******************************** global parameter ****************************************/
#ifdef  __STM32F10X_INIT_H__
#define itEx

itEx u16 Wdtcnt =3000;

GPIO_InitTypeDef GPIO_InitStructure;
RCC_ClocksTypeDef RCC_ClockFreq;
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;
SPI_InitTypeDef  SPI_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
EXTI_InitTypeDef EXTI_InitStructure;

#else
#define itEx extern

itEx u16 Wdtcnt;

itEx GPIO_InitTypeDef GPIO_InitStructure;
itEx RCC_ClocksTypeDef RCC_ClockFreq;
itEx NVIC_InitTypeDef NVIC_InitStructure;
itEx USART_InitTypeDef USART_InitStructure;
itEx SPI_InitTypeDef  SPI_InitStructure;
itEx TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
itEx EXTI_InitTypeDef EXTI_InitStructure;

#endif


itEx u16 timer_buf[32];
itEx __bits stm32_init;
itEx __tm tm;
itEx __Extm Extm;

itEx u8 w8;
itEx u16 w16;
itEx u32 w32, timec;
itEx u64 w64;
itEx s8 sw8;
itEx s16 sw16;
itEx s32 sw32;
itEx s64 sw64;

itEx __wbits watchdog;

itEx u16 timer_1sec;
itEx u16 sub_call_timer;

itEx u16 ind_timer;
itEx u8  dip_switch_buf[2];
itEx u8   chime_sq;
itEx u8   f_chime_keep;
itEx u8   f_chime_sq_timer;
itEx u16  chime_sq_timer;
itEx u16  chime_sq_comp;

itEx u8   i2c_temp_buf[8];
itEx u8   eeprom_save_count;
itEx u8   eep_room_data_buf[100];
itEx u8   main_id_buf;

itEx u16  entry_lamp_delay_time;
itEx u16  timer_1sec_entry;

itEx u8   f_volume_control;
itEx u16  f_uart3_timer_cnt;

itEx u8   f_common_send_time;
itEx u16  common_send_time;
itEx u8   f_monitor_ok;
itEx u8   f_monitor_ok_2;
itEx u16  monitor_timer;
itEx u16  monitor_timer_2;

itEx u8   cb_mode;
itEx u8   light_4_acu;
itEx u16  check_temp_send_time;
itEx u16  check_temp_send_time_2;

itEx u16  em_sign_keep_timer;
itEx u16  em_sign_timer;

itEx u8   u3_dir_delay_counter;
itEx u16  handy_sign_timer;
itEx u64  restroom_off_timer;
itEx u8   touch_sensor;
itEx u16  dnd_count; 
itEx u8   f_dnd_ok;
itEx u8   f_light_data_buf;
itEx u16   f_uart3_data_send_time;
itEx u8   f_uart3_data_send_ok;
itEx u16  plc_reset_air_timer[2];
itEx u16  power_reset_timer;
itEx u8   f_test_mode;
itEx u16  clean_timer;
itEx u8   clean_ok_flag;
itEx u8   ts_id[2];
itEx u8   ondol_set_flag[4];
itEx u8   f_common_send_time_2;
itEx u16  common_send_time_2;
itEx u32  wait_timer;
/******************* (C) COPYRIGHT 2007 INSEM Inc ***************************************END OF FILE****/

