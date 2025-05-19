/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		: HDPVR FRONT PANEL 
processor		: STM32F101xx
compiler		: IAR 4.41A Compiler 
program BY 	: YN.Kim
date 			: 2008.	  .
copy right 	: Insem Inc.
===========================================================================*/
#define __SWITCH_IO_H__

#include "stm32f10x_lib.h"
#include "main.h"

//=== Key sensor check routine ===
void key_sensor_check(void)
{
  unsigned char key_temp = 0;
  
    if((dip_switch_buf[0] & 0x40) == 0){    //Direct key sensor check
      if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9)) key_temp |= 0x01;
      if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)) key_temp |= 0x02;
      
      if(key_temp == key_sensor_back)
      {
        if(key_sensor_chet > 500){    //500msec
          key_sensor_chet = 0;
          if(key_temp != 2) f_clean_wait = 0; //û�ҿϷ� �ƴϸ� flag clear          
          switch(key_temp)
          {
            case 0: room_data_buf[40] &= 0xfd;  room_data_buf[49] = 0;  guest_out_executiom(); break;    //����ó��
            case 1: room_data_buf[40] |= 0x02;  room_data_buf[49] = 1;  guest_in_execution(); break;     //û����
            case 2: room_data_buf[40] &= 0xfd;  
                         f_clean_wait = 1; //clean flag on
                         guest_out_executiom(); 
                         if(clean_ok_flag)
                         {
                            room_data_buf[49] = 2;  
                            f_clean_finish = 1;  
                         }
            break;   //û�ҿϷ�
            case 3: room_data_buf[40] |= 0x02;  room_data_buf[49] = 3;  guest_in_execution(); break;     //���
          }
        }
      }
      else
      {
        key_sensor_back = key_temp;
        key_sensor_chet = 0;
      }
    }
    else{                                   //����� key sensor execution
        
      if(cb_mode != 4 && cb_mode != 6) //17�� (��) Type �ƴҶ�..
      {
        if(f_ks_error)
        {
          room_data_buf[49] = 3;  guest_in_execution();
        }
        else
        {
          if(f_ks_rx_ok)
          {
            if(ks_key_status == key_sensor_back)
            {
              switch(ks_key_status & 0x30)
              {
                case 0x00: room_data_buf[40] &= 0xfd;  room_data_buf[49] = 0;  guest_out_executiom(); break;    //����ó��
                case 0x10: room_data_buf[40] |= 0x02;  room_data_buf[49] = 1;  guest_in_execution(); break;     //û����
                case 0x20: room_data_buf[40] &= 0xfd;  room_data_buf[49] = 2;  f_clean_finish = 1;  guest_out_executiom(); break;    //û�ҿϷ�
                case 0x30: room_data_buf[40] |= 0x02;  room_data_buf[49] = 3;  guest_in_execution(); break;     //���
              }
            }
            else
            {
              key_sensor_back = ks_key_status;
            }
          }
        }
      }
      else //17�� (��) Type �����϶�..
      {
            if(ks_key_status == key_sensor_back)
            {
              switch(ks_key_status & 0x30)
              {
                case 0x00: room_data_buf[40] &= 0xfd;  room_data_buf[49] = 0;  guest_out_executiom(); break;    //����ó��
                case 0x10: room_data_buf[40] |= 0x02;  room_data_buf[49] = 1;  guest_in_execution(); break;     //û����
                case 0x20: room_data_buf[40] &= 0xfd;  room_data_buf[49] = 2;  f_clean_finish = 1;  guest_out_executiom(); break;    //û�ҿϷ�
                case 0x30: room_data_buf[40] |= 0x02;  room_data_buf[49] = 3;  guest_in_execution(); break;     //���
              }
            }
            else
            {
              key_sensor_back = ks_key_status;
            }          
      }   
    }
}

//Guest IN execution routine
void guest_in_execution(void)
{
  unsigned char i;
  
  relay_all_off_delay_timer = 0;
  flag_relay_all_off_delay_timer = 0;   
  
  room_data_buf[45] |= 0x01;  //P
  if(!f_first_room_inout)
  {
    f_first_room_inout = 1;
    flag_fcu_action_time[0] = 1;      //FCU ��� ����
    flag_fcu_action_time[1] = 1;
    flag_fcu_action_time[2] = 1;
    flag_fcu_action_time[3] = 1;
    flag_floor_action_time[0] = 1;    //�µ� ��� ����
    flag_floor_action_time[1] = 1;
    flag_floor_action_time[2] = 1;
    flag_floor_action_time[3] = 1;


    room_data_buf[45] = 0xff;         //Key IN�� ����Ǵ� ����
    //room_data_buf[46] = 0xff;        //Key IN�� ����Ǵ� ����
    room_data_buf[47] = 0xff;
    room_data_buf[50] = 0xff;
    room_data_buf[51] = 0xff;
    room_data_buf[52] = 0xff;
    room_data_buf[53] = 0xff;
    f_enterance_lamp_delay = 0;
    enterance_lamp_delay_timer = 0;   //�Ա��� Time Delay clear
    
    for(i=0; i<16; ++i) dimmer_level[i] = 99;    
  }     
}

//Guest OUT execution routine 
void guest_out_executiom(void)
{
  unsigned char i;
  
  if(f_first_room_inout)
  {
    //f_first_room_inout = 0;
    room_data_buf[36] &= 0xf7;        //Radio off
    room_data_buf[37] &= 0xf7;        //Radio off
    room_data_buf[38] &= 0xf7;        //Radio off
    room_data_buf[39] &= 0xf7;        //Radio off
    
    room_data_buf[54] = 0;
    room_data_buf[55] = 0;
    room_data_buf[56] = 0;
    room_data_buf[57] = 0;    
  }
  if(flag_relay_all_off_delay_timer){ //10sec delay
    room_data_buf[45] = 0;            //Relay All OFF
    room_data_buf[46] = 0;
    room_data_buf[47] = 0;
    room_data_buf[50] = 0;
    room_data_buf[51] = 0;
    room_data_buf[52] = 0;
    room_data_buf[53] = 0;
    
    f_first_room_inout = 0;
    for(i=0; i<16; ++i) dimmer_level[i] = 0;
  }
}

// Enterance lamp switch check routine 
void enterance_lamp_switch_check(void)
{
  if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)))
    {
      if(++lead_chet > 1000)
      {
          f_lead_sw = 1;
        
          if(room_data_buf[40] & 0x02)  //�Խ��ΰ�?
          {       
               f_dnd_sign = 1;
                  
               if(f_dnd_ok)
               {
                   f_dnd_ok = 0;
                   
                   switch(led_count)
                   {
                   case 0:
                     P_dnd_led = 0;
                     P_mur_led = 1;
                     ++led_count;
                     break;
                     
                   case 1:
                     P_dnd_led = 1;
                     P_mur_led = 0;
                     led_count = 0;
                     break;
                   }
                    
                   ++dnd_stop_count;
                      
                   if(dnd_stop_count > 3) 
                   {
                     dnd_stop_count = 0;
                     f_dnd_sign = 0;
                     f_dnd_con = 0;
                   }
               }                    
          }
          else
          {
                P_dnd_led = 0;
                P_mur_led = 0;              
          }
      }
    }
    else
    {
        lead_chet = 0;
        f_lead_sw = 0;
    }
  /*
  if(room_data_buf[40] & 0x02){             //�Խ��ΰ�?
    if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)))
    {
      if(++enterance_chet > 1000)
      {
        if(!f_enterance_push)
        {
          f_enterance_push = 1;
          //room_data_buf[45] ^= 0x02;        //�Ա��� ����
          if(room_data_buf[45] & 0x02)
          {
            f_enterance_lamp_delay = 0;
            enterance_lamp_delay_timer = 0;   //�Ա��� Time Delay clear
          }
        }
      }
    }
    else
    {
      enterance_chet = 0;
      f_enterance_push = 0;
    }
  }
  */
}

void dnd_switch_check(void)
{ 
  
  if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14)))
    {
      if(++dnd_chet > 1000)
      {
         if(!f_dnd_push)
         {
            f_dnd_push = 1;
            dnd_chet = 0;
            dnd_mur_execution('D','T');
         }
      }
    }
    else
    {
        f_dnd_push = 0;
        dnd_chet = 0;
    }
}

//MUR Switch check
void mur_switch_check(void)
{
    
    if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)))
    {
      if(++mur_chet > 1000)
      {
         if(!f_mur_push)
         {
            f_mur_push = 1;
            mur_chet = 0;
            dnd_mur_execution('M','T');
         }
      }
    }
    else
    {
        f_mur_push = 0;
        mur_chet = 0;
    }
}

void dnd_mur_execution(unsigned char kind, unsigned char comd)
{
  switch(comd)
  {
    case 'N': 
      room_data_buf[40] &= 0xf3;  
      if(kind == 'D') room_data_buf[40] |= 0x04; 
      else room_data_buf[40] |= 0x08; 
      break;
      
    case 'F': room_data_buf[40] &= 0xf3;  break;
    case 'T':
      if(kind == 'D')
      {
        if(room_data_buf[40] & 0x04) room_data_buf[40] &= 0xf3;
        else
        {
         room_data_buf[40] &= 0xf3;
         room_data_buf[40] |= 0x04;
        }
      }
      else
      {
        if(room_data_buf[40] & 0x08) room_data_buf[40] &= 0xf3;
        else
        {
         room_data_buf[40] &= 0xf3;
         room_data_buf[40] |= 0x08;
        }
      }
      break;
  }
}

//Chime switch check routine
void chime_switch_check(void)
{
  if(!f_lead_sw)
  {     
    if(room_data_buf[40] & 0x08) //MUR LED
    {
        P_mur_led = 0;
        P_dnd_led = 1;
    }
    else P_mur_led = 1;
            
    if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))) //���Ӻ� ���� 
    {
      if(++chime_chet > 500)
      {
        //P_ind_led = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
        if(room_data_buf[40] & 0x04) f_dnd_con = 1;
        
        if(!(room_data_buf[40] & 0x04))  //DND�� �ƴϸ� ����
        {            
          if(!f_chime_push)
          {
            f_chime_push = 1;
            if(!f_chime_keep)
            {
              f_chime_keep = 1;
              chime_sq = 0;
              f_chime_sq_timer = 1;
              digital_volume_execution(0,room_data_buf[79]);
              
              handy_sign_timer = 0;
              
              if(room_data_buf[40] & 0x02) //�Խ��϶�
              {
                room_data_buf[48] |= 0x80;
                f_chime_sign = 1;
              }
            }
          }
        }
      }
    }
    else
    {
      chime_chet = 0;
      f_chime_push = 0;
      //P_ind_led = 1;
    }
    
    if((f_dnd_con == 1) && (room_data_buf[40] & 0x04)) //DND �϶� ���Ӻ� ������ DND ������ 
    {
         f_dnd_sign = 1;
            
         if(f_dnd_ok)
         {
             f_dnd_ok = 0;
             P_dnd_led ^= 1;
             ++dnd_stop_count;
                
             if(dnd_stop_count > 6) 
             {
               dnd_stop_count = 0;
               f_dnd_sign = 0;
               f_dnd_con = 0;
             }
         }    
    }  
    else if(room_data_buf[40] & 0x04) //DND LED
    {
        P_dnd_led = 0;
        P_mur_led = 1;
    }  
    else P_dnd_led = 1;
    
  }
}  

void door_sensor_check(void)
{
  /*
  if(!(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))){
    if(++door_chet > 1000){
      door_chet = 0;
      room_data_buf[40] &= 0xbf;    //door close
    }
  }
  else{
    if(++door_chet > 1000){
      door_chet = 0;
      room_data_buf[40] |= 0x40;    //door open
    }
  }
  */
}

void emergency_switch_check(void)
{
  if(!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)))
  {
      if(++emergency_chet > 1000)
      {
        emergency_chet = 0;
       if((room_data_buf[42] & 0x01) == 0)
       {
          if(!f_em_flag)
          {
            f_em_flag = 1;    //Emergency call
            
            em_sign_keep_timer = 0;
            em_sign_timer = 0;
          }
          
      }
    }
  }
    else
    {
      if(++emergency_chet > 1000)
      {
        emergency_chet = 0;
        P_ex_led1 = 1;              //ttt
        room_data_buf[42] &= 0xfe; // Emergency clear
        f_em_flag = 0;
      }
    }
}

void wait_sign_check(void)      //�ӽ� ���� ��Ʈ
{
  /*
  if(!(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)))
  {
    if(++wait_sign_chet > 1000)
    {
      wait_sign_chet = 0;
      room_data_buf[40] |= 0x80;    //window close
      wait_sign_timer = 0;              
    }
  }
  else
  {
    wait_sign_chet = 0;
  }
  */
}

//======= Relay out execution routine ==========
void relay_out_execution(void)
{
  switch(cb_mode)
  {
  case 0: //stadard type
    
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_1 = 1; //SPARE 1
          
          P_rly_9 = 1; //SPARE 3
          P_rly_10 = 1; //SPARE 4
          P_rly_11 = 1; //SPARE 5
          P_rly_12 = 1; //SPARE 6
          P_rly_13 = 1; //SPARE 7
          P_rly_15 = 1; 
          P_rly_16 = 1;
      }
      else
      {
          P_rly_8 = 0;
          P_rly_7 = 0;
          P_rly_1 = 0;
          
          P_rly_9 = 0;
          P_rly_10 = 0;
          P_rly_11 = 0;
          P_rly_12 = 0;
          P_rly_13 = 0;
          P_rly_15 = 0;
          P_rly_16 = 0;
      }      
      
      if(room_data_buf[45] & 0x10) P_rly_2 = 1; //LIGHT 5B (����)
      else P_rly_2 = 0;      
      
      if(room_data_buf[46] & 0x01) P_rly_3 = 1; //LIGHT 4B (ȭ���)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3B (ħ��3)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2B (ħ��2)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1B (ħ��1)
      else P_rly_6 = 0;
    
    break;
    
  case 1: //Spa suite ( 4~9 floor type )

      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          
          P_rly_9 = 1; //SPARE 3
          P_rly_10 = 1; //SPARE 4
          P_rly_11 = 1; //SPARE 5
          P_rly_12 = 1; //SPARE 6
          P_rly_13 = 1; //SPARE 7
          P_rly_14 = 1; //SPARE
      }
      else
      {
          P_rly_8 = 0;
          P_rly_7 = 0;
          
          P_rly_9 = 0;
          P_rly_10 = 0;
          P_rly_11 = 0;
          P_rly_12 = 0;
          P_rly_13 = 0;
          P_rly_14 = 0;
      }      
      
      if(room_data_buf[46] & 0x02) P_rly_13 = 1; //LIGHT 1A (�Ž� ȭ���)
      else P_rly_13 = 0;        
      
      if(room_data_buf[45] & 0x80) P_rly_15 = 1; //LIGHT 8B (�Ž�4)
      else P_rly_15 = 0;        
      
      if(room_data_buf[45] & 0x40) P_rly_16 = 1; //LIGHT 7B (�Ž�3)
      else P_rly_16 = 0;  
      
      if(room_data_buf[45] & 0x20) P_rly_1 = 1; //LIGHT 6B (�Ž�2)
      else P_rly_1 = 0;     
      
      if(room_data_buf[45] & 0x10) P_rly_2 = 1; //LIGHT 5B (�Ž�1)
      else P_rly_2 = 0;      
      
      if(room_data_buf[46] & 0x01) P_rly_3 = 1; //LIGHT 4B (ħ�� ȭ���)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3B (ħ��3)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2B (ħ��2)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1B (ħ��1)
      else P_rly_6 = 0;
    
    break;
    
  case 2: //spa suite ( 10 floor type ) , ( 11 floor type )
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_14 = 1; //Dimming power ( spare )
          P_rly_9 = 1; //spare
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_14 = 0;
          P_rly_9 = 0;
      }
      
      if(room_data_buf[46] & 0x04) P_rly_10 = 1; //LIGHT 4A (�Ž� ȭ��� 2)
      else P_rly_10 = 0;             
      
      if(room_data_buf[46] & 0x02) P_rly_11 = 1; //LIGHT 3A (�Ž� ȭ��� 1)
      else P_rly_11 = 0;               
      
      if(room_data_buf[47] & 0x02) P_rly_12 = 1; //LIGHT 2A (�Ž�4)
      else P_rly_12 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_13 = 1; //LIGHT 1A (�Ž�3)
      else P_rly_13 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_15 = 1; //LIGHT 8B (�Ž�2)
      else P_rly_15 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_16 = 1; //LIGHT 7B (�Ž�1)
      else P_rly_16 = 0;
      
      if(room_data_buf[46] & 0x01) P_rly_1 = 1; //LIGHT 6B (ħ�� ȭ���)
      else P_rly_1 = 0;     
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5B (ħ��5)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4B (ħ��4)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3B (ħ��3)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2B (ħ��2)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1B (ħ��1)
      else P_rly_6 = 0;      
      
      
    break;
    
  case 3: //P-suite ( 18 floor �� type )
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A
          P_rly_7 = 1; //���� B
          P_rly_6 = 1; //�Ϲ�����
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_6 = 0;
      }
      
      if(room_data_buf[47] & 0x10) P_rly_9 = 1; //LIGHT 5A (���̴׷� 4)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x08) P_rly_10 = 1; //LIGHT 4A (���̴׷� 3)
      else P_rly_10 = 0;          
      
      if(room_data_buf[47] & 0x04) P_rly_11 = 1; //LIGHT 3A (���̴׷� 2)
      else P_rly_11 = 0;             
      
      if(room_data_buf[47] & 0x02) P_rly_12 = 1; //LIGHT 2A (���̴׷� 1)
      else P_rly_12 = 0;               
      
      if(room_data_buf[47] & 0x01) P_rly_13 = 1; //LIGHT 1A (�Ա�-�Ž�,���� 4)
      else P_rly_13 = 0;            
      
      if(room_data_buf[45] & 0x80) P_rly_14 = 1; //LIGHT 8B (�Ա�-�Ž�,���� 3)
      else P_rly_14 = 0;          
      
      if(room_data_buf[45] & 0x40) P_rly_15 = 1; //LIGHT 7B (�Ա�-�Ž�,���� 2)
      else P_rly_15 = 0;      
      
      if(room_data_buf[45] & 0x20) P_rly_16 = 1; //LIGHT 6B (�Ա�-�Ž�,���� 1)
      else P_rly_16 = 0;
      
      if(room_data_buf[46] & 0x01) P_rly_1 = 1; //LIGHT 5B (�Ž�ȭ���)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_2 = 1; //LIGHT 4B (�Ž�4)
      else P_rly_2 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_3 = 1; //LIGHT 3B (�Ž�3)
      else P_rly_3 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_4 = 1; //LIGHT 2B (�Ž�2)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_5 = 1; //LIGHT 1B (�Ž�1)
      else P_rly_5 = 0;      
      
      
    break;    
    
  case 4: //P-suite ( 19 floor �� type )
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A
          P_rly_7 = 1; //���� B
          P_rly_6 = 1; //�Ϲ�����
          P_rly_10 = 1; //SPARE1
          P_rly_9 = 1; //SPARE2
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_6 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }
      
      if(room_data_buf[46] & 0x02) P_rly_11 = 1; //LIGHT 3A (ħ��2 ȭ���)
      else P_rly_11 = 0;             
      
      if(room_data_buf[47] & 0x02) P_rly_12 = 1; //LIGHT 2A (ħ��-2, 3)
      else P_rly_12 = 0;               
      
      if(room_data_buf[47] & 0x01) P_rly_13 = 1; //LIGHT 1A (ħ��-2 , 2)
      else P_rly_13 = 0;            
      
      if(room_data_buf[45] & 0x80) P_rly_14 = 1; //LIGHT 8B (ħ��-2 , 1)
      else P_rly_14 = 0;          
      
      if(room_data_buf[45] & 0x40) P_rly_15 = 1; //LIGHT 7B �Ա�(����)
      else P_rly_15 = 0;      
      
      if(room_data_buf[45] & 0x20) P_rly_16 = 1; //LIGHT 6B �Ա�(����)
      else P_rly_16 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_1 = 1; //LIGHT 5B �Ա�(����)
      else P_rly_1 = 0;
      
      if(room_data_buf[46] & 0x01) P_rly_2 = 1; //LIGHT 4B (ħ��1 ȭ���)
      else P_rly_2 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_3 = 1; //LIGHT 3B (ħ��3)
      else P_rly_3 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_4 = 1; //LIGHT 2B (ħ��2)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_5 = 1; //LIGHT 1B (ħ��1)
      else P_rly_5 = 0;      
      
    break;       
    
  case 5: //P-suite ( 18 floor �� type )
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A
          P_rly_7 = 1; //���� B
          P_rly_6 = 1; //�Ϲ�����
          P_rly_9 = 1; //SPARE2
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_6 = 0;
          P_rly_9 = 0;
      }
      
      if(room_data_buf[47] & 0x10) P_rly_10 = 1; //LIGHT 4A (���̴׷�, �Ž� 7)
      else P_rly_10 = 0;             
      
      if(room_data_buf[47] & 0x08) P_rly_11 = 1; //LIGHT 3A (���̴׷�, �Ž� 6)
      else P_rly_11 = 0;             
      
      if(room_data_buf[47] & 0x04) P_rly_12 = 1; //LIGHT 2A (���̴׷�, �Ž� 5)
      else P_rly_12 = 0;               
      
      if(room_data_buf[47] & 0x02) P_rly_13 = 1; //LIGHT 1A (���̴׷�, �Ž� 4)
      else P_rly_13 = 0;            
      
      if(room_data_buf[45] & 0x80) P_rly_14 = 1; //LIGHT 8B (���̴׷�, �Ž� 3)
      else P_rly_14 = 0;          
      
      if(room_data_buf[45] & 0x40) P_rly_15 = 1; //LIGHT 7B (���̴׷�, �Ž� 2) 
      else P_rly_15 = 0;      
      
      if(room_data_buf[45] & 0x20) P_rly_16 = 1; //LIGHT 6B (���̴׷�, �Ž� 1)
      else P_rly_16 = 0;
      
      if(room_data_buf[46] & 0x01) P_rly_1 = 1; //LIGHT 5B (�Ž� ȭ���)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_2 = 1; //LIGHT 4B (�Ա��Ž� 4)
      else P_rly_2 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_3 = 1; //LIGHT 3B (�Ա��Ž� 3)
      else P_rly_3 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_4 = 1; //LIGHT 2B (�Ա��Ž� 2)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_5 = 1; //LIGHT 1B (�Ա��Ž� 1)
      else P_rly_5 = 0;      
      
    break;           
    
  case 6: //P-suite ( 19 floor �� type )
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A
          P_rly_7 = 1; //���� B
          P_rly_6 = 1; //�Ϲ�����
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_6 = 0;
      }
      if(room_data_buf[46] & 0x02) P_rly_9 = 1; //LIGHT 5A (ħ�� 2 ȭ��ǵ�)
      else P_rly_9 = 0;
      
      if(room_data_buf[47] & 0x10) P_rly_10 = 1; //LIGHT 4A (���̴׷�, �Ž� 7)
      else P_rly_10 = 0;             
      
      if(room_data_buf[47] & 0x08) P_rly_11 = 1; //LIGHT 3A (���̴׷�, �Ž� 6)
      else P_rly_11 = 0;             
      
      if(room_data_buf[47] & 0x04) P_rly_12 = 1; //LIGHT 2A (���̴׷�, �Ž� 5)
      else P_rly_12 = 0;               
      
      if(room_data_buf[47] & 0x02) P_rly_13 = 1; //LIGHT 1A (���̴׷�, �Ž� 4)
      else P_rly_13 = 0;            
      
      if(room_data_buf[45] & 0x80) P_rly_14 = 1; //LIGHT 8B (���̴׷�, �Ž� 3)
      else P_rly_14 = 0;          
      
      if(room_data_buf[45] & 0x40) P_rly_15 = 1; //LIGHT 7B (���̴׷�, �Ž� 2) 
      else P_rly_15 = 0;      
      
      if(room_data_buf[45] & 0x20) P_rly_16 = 1; //LIGHT 6B (���̴׷�, �Ž� 1)
      else P_rly_16 = 0;
      
   //   if(room_data_buf[46] & 0x01) P_rly_1 = 1; //LIGHT 5B (�Ž� ȭ���)
   //   else P_rly_1 = 0;
      
      if(room_data_buf[46] & 0x01) P_rly_2 = 1; //LIGHT 4B (ħ�� ȭ��� 1)
      else P_rly_2 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_3 = 1; //LIGHT 3B (�Ա��Ž� 3)
      else P_rly_3 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_4 = 1; //LIGHT 2B (�Ա��Ž� 2)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_5 = 1; //LIGHT 1B (�Ա��Ž� 1)
      else P_rly_5 = 0;      
      
    break;       
    
  case 7: //�Ű�(Ŭ����) A Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
      }
      
      if(room_data_buf[47] & 0x40) P_rly_9 = 1; //LIGHT 14 (I)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x20) P_rly_10 = 1; //LIGHT 13 (B)
      else P_rly_10 = 0;                
      
      if(room_data_buf[47] & 0x10) P_rly_11 = 1; //LIGHT 12 (B)
      else P_rly_11 = 0;                
      
      if(room_data_buf[47] & 0x08) P_rly_12 = 1; //LIGHT 11 (C)
      else P_rly_12 = 0;                             
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (C)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (C)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (C)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (G) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (G)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (G)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (F)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (F)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (F)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (F)
      else P_rly_6 = 0;      
      
    break;           
    
  case 8: //�Ű�(Ŭ����) B Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_12 = 1; //SPARE#1
          P_rly_11 = 1; //SPARE#2
          P_rly_10 = 1; //SPARE#3
          P_rly_9 = 1; //SPARE#4
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_12 = 0;    
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (C)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (C)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (C)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (C) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (C)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (H)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (H)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (G)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (G)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (G)
      else P_rly_6 = 0;      
      
    break;             
    
  case 9: //�Ű�(Ŭ����) C Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_12 = 1; //SPARE#1
          P_rly_11 = 1; //SPARE#2
          P_rly_10 = 1; //SPARE#3
          P_rly_9 = 1; //SPARE#4          
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_12 = 0;    
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;          
      }
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (H)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (H)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (H)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (H) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (I)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (I)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (I)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (F)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (F)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (F)
      else P_rly_6 = 0;      
      
    break;        
    
  case 10: //�Ű�(Ŭ����) D Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_13 = 1; //SPARE#1
          P_rly_12 = 1; //SPARE#2
          P_rly_11 = 1; //SPARE#3
          P_rly_10 = 1; //SPARE#4
          P_rly_9 = 1;  //SPARE#5
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_13 = 0;
          P_rly_12 = 0;
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }                     
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (D)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (D)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (B) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (B)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (B)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (B)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (C)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (C)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (C)
      else P_rly_6 = 0;      
      
    break;     

  case 11: //�Ű�(Ŭ����) E Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_15 = 1; //SPARE#1
          P_rly_14 = 1; //SPARE#2
          P_rly_13 = 1; //SPARE#3
          P_rly_12 = 1; //SPARE#4
          P_rly_11 = 1; //SPARE#5
          P_rly_10 = 1; //SPARE#6
          P_rly_9 = 1; //SPARE#7
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_15 = 0;
          P_rly_14 = 0;
          P_rly_13 = 0;
          P_rly_12 = 0;
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }                     
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (I) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (H)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (H)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (H)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (C)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (C)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (C)
      else P_rly_6 = 0;      
      
    break;      
    
  case 12: //�Ű�(Ŭ����) F,F-1 Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
      }
      
      if(room_data_buf[47] & 0x40) P_rly_9 = 1; //LIGHT 14 (B)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x20) P_rly_10 = 1; //LIGHT 13 (M)
      else P_rly_10 = 0;                
      
      if(room_data_buf[47] & 0x10) P_rly_11 = 1; //LIGHT 12 (M)
      else P_rly_11 = 0;                
      
      if(room_data_buf[47] & 0x08) P_rly_12 = 1; //LIGHT 11 (M)
      else P_rly_12 = 0;                             
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (E)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (E)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (E)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (E) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (E)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (E)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (E)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (C)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (C)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (C)
      else P_rly_6 = 0;      
      
    break;           
    
  case 13: //�Ű�(Ŭ����) G,H Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_12 = 1; //SPARE#1
          P_rly_11 = 1; //SPARE#2
          P_rly_10 = 1; //SPARE#3
          P_rly_9 = 1; //SPARE#4                  
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_12 = 0;
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }
      
      /*if(room_data_buf[47] & 0x40) P_rly_9 = 1; //LIGHT 14 (B)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x20) P_rly_10 = 1; //LIGHT 13 (M)
      else P_rly_10 = 0;                
      
      if(room_data_buf[47] & 0x10) P_rly_11 = 1; //LIGHT 12 (M)
      else P_rly_11 = 0;                
      
      if(room_data_buf[47] & 0x08) P_rly_12 = 1; //LIGHT 11 (M)
      else P_rly_12 = 0;  */                      
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (B)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (B)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (B)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (B) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (J)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (J)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (J)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (L)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (L)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (L)
      else P_rly_6 = 0;      
      
    break;   
    
  case 14: //�Ű�(Ŭ����) I Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_10 = 1; //SPARE#1
          P_rly_9 = 1;  //SPARE#2
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }
      
      /*if(room_data_buf[47] & 0x40) P_rly_9 = 1; //LIGHT 14 (B)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x20) P_rly_10 = 1; //LIGHT 13 (M)
      else P_rly_10 = 0;     
      */
      
      if(room_data_buf[47] & 0x10) P_rly_11 = 1; //LIGHT 12 (J)
      else P_rly_11 = 0;                
      
      if(room_data_buf[47] & 0x08) P_rly_12 = 1; //LIGHT 11 (J)
      else P_rly_12 = 0;                   
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (J)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (J)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (J)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (G) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (G)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (G)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (B)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (B)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (B)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (B)
      else P_rly_6 = 0;      
      
    break;        
    
  case 15: //�Ű�(Ŭ����) J Type
      
      if(room_data_buf[45] & 0x01)
      {
          P_rly_8 = 1; //���� A,B
          P_rly_7 = 1; //�Ϲ�����
          P_rly_12 = 1; //SPARE#1
          P_rly_11 = 1; //SPARE#2
          P_rly_10 = 1; //SPARE#3
          P_rly_9 = 1; //SPARE#4                 
      }
      else
      {
          P_rly_8 = 0; 
          P_rly_7 = 0;
          P_rly_12 = 0;
          P_rly_11 = 0;
          P_rly_10 = 0;
          P_rly_9 = 0;
      }
      
      /*if(room_data_buf[47] & 0x40) P_rly_9 = 1; //LIGHT 14 (B)
      else P_rly_9 = 0;              
      
      if(room_data_buf[47] & 0x20) P_rly_10 = 1; //LIGHT 13 (M)
      else P_rly_10 = 0;     
      
      if(room_data_buf[47] & 0x10) P_rly_11 = 1; //LIGHT 12 (J)
      else P_rly_11 = 0;                
      
      if(room_data_buf[47] & 0x08) P_rly_12 = 1; //LIGHT 11 (J)
      else P_rly_12 = 0;    
      
      */
      
      if(room_data_buf[47] & 0x04) P_rly_13 = 1; //LIGHT 10 (C)
      else P_rly_13 = 0;                         
      
      if(room_data_buf[47] & 0x02) P_rly_14 = 1; //LIGHT 9 (C)
      else P_rly_14 = 0;            
      
      if(room_data_buf[47] & 0x01) P_rly_15 = 1; //LIGHT 8 (C)
      else P_rly_15 = 0;          
      
      if(room_data_buf[45] & 0x80) P_rly_16 = 1; //LIGHT 7 (I) 
      else P_rly_16 = 0;      
      
      if(room_data_buf[45] & 0x40) P_rly_1 = 1; //LIGHT 6 (I)
      else P_rly_1 = 0;
      
      if(room_data_buf[45] & 0x20) P_rly_2 = 1; //LIGHT 5 (I)
      else P_rly_2 = 0;
      
      if(room_data_buf[45] & 0x10) P_rly_3 = 1; //LIGHT 4 (G)
      else P_rly_3 = 0;      
      
      if(room_data_buf[45] & 0x08) P_rly_4 = 1; //LIGHT 3 (G)
      else P_rly_4 = 0;
      
      if(room_data_buf[45] & 0x04) P_rly_5 = 1; //LIGHT 2 (G)
      else P_rly_5 = 0;
      
      if(room_data_buf[45] & 0x02) P_rly_6 = 1; //LIGHT 1 (G)
      else P_rly_6 = 0;      
      
    break;           
  }  
}