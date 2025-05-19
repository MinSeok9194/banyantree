/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		: HDPVR FRONT PANEL 
processor		: STM32F103xx
compiler		: IAR 6.21A Compiler 
program BY 	: H.H.Hwang
date 			  : 2011.	11  .
copy right 	: Micronic Korea.
===========================================================================*/

#ifdef __SWITCH_IO_H__
#define swEX



#else
#define swEX extern

#endif


void relay_out_execution(void);
void key_sensor_check(void);
void guest_in_execution(void);
void guest_out_executiom(void);
void enterance_lamp_switch_check(void);
void dnd_switch_check(void);
void mur_switch_check(void);
void dnd_mur_execution(unsigned char kind, unsigned char comd);
void chime_switch_check(void);
void door_sensor_check(void);
void window_sensor_check(void);
void emergency_switch_check(void);
void wait_sign_check(void);

u1EX u8   key_sensor_back;
u1EX u16  key_sensor_chet;
u1EX u16  relay_all_off_delay_timer;
u1EX u8   flag_relay_all_off_delay_timer;
u1EX u8   f_first_room_inout;
u1EX u8   f_enterance_lamp_delay;
u1EX u16  enterance_lamp_delay_timer;
u1EX u16  enterance_chet;
u1EX u8   f_enterance_push;
u1EX u16  dnd_chet;
u1EX u8   f_dnd_push;
u1EX u16  mur_chet;

u1EX u16  lead_chet;

u1EX u8   f_mur_push;
u1EX u16  chime_chet;
u1EX u8   f_chime_push;

u1EX u8   eep_test;

u1EX u8   volume_chime_value;
u1EX u8   volume_alram_value;
u1EX u16  door_chet;
u1EX u16  window_chet;
u1EX u16  emergency_chet;

u1EX u16  wait_sign_timer;
u1EX u16  wait_sign_chet;
u1EX u8   dnd_stop_count;
u1EX u8   led_count;

u1EX u16  ex_chet1;
u1EX u16  ex_chet2;
u1EX u16  ex_chet3;
u1EX u16  ex_chet4;
u1EX u16  ex_chet5;
u1EX u16  ex_chet6;
u1EX u16  ex_chet7;
u1EX u16  ex_chet8;
u1EX u16  ex_chet9;
u1EX u16  ex_chet10;
u1EX u16   guest_alrm_timer_1;
u1EX u16   guest_alrm_timer_2;

u1EX u8   ex_in_flag1;
u1EX u8   ex_in_flag2;
u1EX u8   ex_in_flag3;
u1EX u8   ex_in_flag4;
u1EX u8   ex_in_flag5;
u1EX u8   ex_in_flag6;
u1EX u8   ex_in_flag7;
u1EX u8   ex_in_flag8;
u1EX u8   ex_in_flag9;
u1EX u8   ex_in_flag10;
u1EX u8   f_clean_wait;
u1EX u8   fcu_back_buf[4];
u1EX u16   fcu_timer;
u1EX u16   fcu_timer_2;
u1EX u16   fcu_timer_3;

u1EX __dwbits switch_flag1;
#define f_em_flag      			switch_flag1.s.b0
#define f_chime_sign   			switch_flag1.s.b1
#define f_dnd_sign                      switch_flag1.s.b2
#define f_dnd_con                       switch_flag1.s.b3
#define f_lead_sw                       switch_flag1.s.b4
#define f_clean_finish                  switch_flag1.s.b5
#define f_fcu_timer                     switch_flag1.s.b6
#define f_fcu_timer_2                   switch_flag1.s.b7
#define f_fcu_timer_3                   switch_flag1.s.b8
//========================================================================
//========================================================================

