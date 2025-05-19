/*=====================(C) COPYRIGHT 2009 Insem Inc.=========================
program 		: KROAD ALARM CONTROLLER  
processor	: STM32F103xx
compiler		: IAR 4.41A Compiler 
program BY 	: H.H.Hwang
date 			: 2009.	  .
copy right 	: PLUS - H
===========================================================================*/
#define __IEXEC_H__

#include "stm32f10x_lib.h"
#include "main.h"

//========================================================================

//========================================================================
u8 iexecctrl=7;
void internal_exec_proc(void) 
{
  key_sensor_check();
  enterance_lamp_switch_check();
  dnd_switch_check();
  mur_switch_check();
  chime_switch_check();
  door_sensor_check();
  //window_sensor_check();
  emergency_switch_check();
  wait_sign_check();
 // light_ex_input();
  
  switch(sq_count & 0x07){
    case 0: Thermo_sensor_execution(0); ++sq_count; break;  //++sq_count; break;
    case 1: Thermo_sensor_execution(1); sq_count = 0; break;
    //case 2: Thermo_sensor_execution(2); ++sq_count; break;
  //  case 1: Thermo_sensor_ondol_execution(0); sq_count = 0; break;
//    case 2: Thermo_sensor_execution(2); ++sq_count; break;
//    case 3: Thermo_sensor_execution(3); ++sq_count; break;
    //case 4: Thermo_sensor_ondol_execution(0); ++sq_count; break;
    //case 5: Thermo_sensor_ondol_execution(1); sq_count=0; break;
//   case 6: Thermo_sensor_ondol_execution(2); ++sq_count; break;
//    case 7: Thermo_sensor_ondol_execution(3); sq_count = 0; break;
  }
  fcu_4pipe_check();
  relay_out_execution();
}







void fcu_4pipe_check(void)
{
  unsigned int set_temp_temp;
  unsigned int set_temp_temp_c;
  unsigned int set_temp_temp_w;
  unsigned int check_temp_temp;
  
  if(room_data_buf[74] == 2){   //환절기 모드 인가?
    if(room_data_buf[83+fcu_check_count] == 0){
      if((room_data_buf[40] & 0x03) == 0){  //공실이고 퇴실인경우는 공실기준온도로 냉,난 전환한다.
        set_temp_temp_c = (b_decimal_to_hex_table_ie[room_data_buf[72]]) * 10;       //냉방 공실 기준온도
        set_temp_temp_w = (b_decimal_to_hex_table_ie[room_data_buf[71]]) * 10;       //난방 공실기준온도
        check_temp_temp = (room_data_buf[4+fcu_check_count] & 0x7f) * 10;
        if(room_data_buf[4+fcu_check_count] & 0x80) check_temp_temp += 5;
        if(check_temp_temp > set_temp_temp_c){    //현재온도가 냉방공실 기준온도보다 큰가?
          room_data_buf[20+fcu_check_count] |= 0x04;  //냉방적용
        }
        else{
          if(check_temp_temp < set_temp_temp_w){  //현재온도가 난방 공실 기준온도보다 작은가?
            room_data_buf[20+fcu_check_count] &= 0xfb;  //난방적용
          }
        }
      }
      else{
        set_temp_temp = (room_data_buf[0+fcu_check_count] & 0x7f) * 10;
        if(room_data_buf[0+fcu_check_count] & 0x80) set_temp_temp += 5;
        check_temp_temp = (room_data_buf[4+fcu_check_count] & 0x7f) * 10;
        if(room_data_buf[4+fcu_check_count] & 0x80) check_temp_temp += 5;

        if(check_temp_temp < set_temp_temp){
          if((set_temp_temp - check_temp_temp) > (room_data_buf[75]*10)){
            if(room_data_buf[20+fcu_check_count] & 0x04){
              room_data_buf[20+fcu_check_count] &= 0xfb;  //난방적용
              room_data_buf[20+fcu_check_count] |= 0x80;
            }
          }
        }
        else{
          if(check_temp_temp > set_temp_temp){
            if((check_temp_temp - set_temp_temp) > (room_data_buf[75]*10)){
              if(!(room_data_buf[20+fcu_check_count] & 0x04)){
                room_data_buf[20+fcu_check_count] |= 0x04;  //냉방적용
                room_data_buf[20+fcu_check_count] |= 0x80;
              }
            }
          }
        }
      }
    }
    if(++fcu_check_count >= 4) fcu_check_count = 0;
  }
}

// ========== FCU control routine ===========
void Thermo_sensor_execution(unsigned char index)
{
  if((room_data_buf[16+index] & 0x80) || flag_fcu_action_time[index]){ //즉시제어? 또는 실행시간?
    if((room_data_buf[60+index] & 0x02) || (room_data_buf[20+index] & 0x04)){   //동시제어? 또는 냉방인가?
      flag_fcu_action_time[index] = 0;
      room_data_buf[16+index] &= 0x7f;
      
      if(room_data_buf[40] & 0x02){       //입실?
        if(room_data_buf[41] & 0x02)  going_out_temp_control(index);          //심야제어?  //외출차 적용
        else{
          if(room_data_buf[16+index] & 0x04) 
          {
            stay_temp_control(index);        //재실제어   //온도센서 스위치 ON?
          }
          else{
            room_data_buf[20+index] &= 0xfc;                                  //공실모드 적용
            fan_speed_control(0,index);
          }
        }
      }
      else{
        if(room_data_buf[40] & 0x01){     //Check IN?
          if(room_data_buf[16+index] & 0x04) going_out_temp_control(index);   //온도센서 ON상태이면 외출차 제어  
          else{
            room_data_buf[20+index] &= 0xfc;                                  //공실모드 적용
            fan_speed_control(0,index);
          }
        }
        else{
          if(room_data_buf[41] & 0x01) going_out_temp_control(index);         //예냉.예열?  외출차 적용
          else empty_temp_control(index);                                      //공실제어
        }
      }
      if((room_data_buf[20+index] & 0x03) != (fan_mode_back[index] & 0x03)){
        fan_mode_back[index] &= 0xfc;
        fan_mode_back[index] |= (room_data_buf[20+index] & 0x03);
        room_data_buf[99] |= 0x02;
      }
      
    }
  }
}




void stay_temp_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  set_temp_temp = (room_data_buf[0+index] & 0x7f) * 10;
  if(room_data_buf[0+index] & 0x80) set_temp_temp += 5;
  check_temp_temp = (room_data_buf[4+index] & 0x7f) * 10;
  if(room_data_buf[4+index] & 0x80) check_temp_temp += 5;
  
  room_data_buf[20+index] &= 0xfc;
  room_data_buf[20+index] |= 0x02;                //재실 모드 적용
  
  if(room_data_buf[20+index] & 0x04){             //냉방 모드?
    if(check_temp_temp > set_temp_temp){
      if((check_temp_temp - set_temp_temp) > 9){    //9는 0.9도를 의미함
        if((check_temp_temp - set_temp_temp) > 30)  fan_speed_control(3,index);     //Speed Hi
        else{
          if(((check_temp_temp - set_temp_temp) > 1) && ((check_temp_temp - set_temp_temp) <= 19))  fan_speed_control(1,index);            //Speed Low
          else if(((check_temp_temp - set_temp_temp) >= 20) && ((check_temp_temp - set_temp_temp) <= 29))  fan_speed_control(2,index);     //Speed Mid
          else fan_speed_control(3,index);     //Speed High
        }
      }
      else{
        if((room_data_buf[16+index] & 0x03) != 0) fan_speed_control(1,index);
      }
    }
    else{
      if(check_temp_temp < set_temp_temp) fan_speed_control(0,index);
    }
  }
  else{                                 //난방 모드
    if(set_temp_temp > check_temp_temp){
      if((set_temp_temp - check_temp_temp) > 9){    //9는 0.9도를 의미함
        if((set_temp_temp - check_temp_temp) > 30)  fan_speed_control(3,index);     //Speed Hi
        else{
          if(((set_temp_temp - check_temp_temp) > 1) && ((set_temp_temp - check_temp_temp) <= 19))  fan_speed_control(1,index);            //Speed Low
          else if(((set_temp_temp - check_temp_temp) >= 20) && ((set_temp_temp - check_temp_temp) <= 29))  fan_speed_control(2,index);     //Speed Mid
          else fan_speed_control(3,index);     //Speed High
        }
      }
      else{
        if((room_data_buf[16+index] & 0x03) != 0) fan_speed_control(1,index);
      }
    }
    else{
      if(set_temp_temp < check_temp_temp) fan_speed_control(0,index);
    }    
  }
}


void going_out_temp_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  set_temp_temp = (room_data_buf[0+index] & 0x7f) * 10;
  if(room_data_buf[0+index] & 0x80) set_temp_temp += 5;
  check_temp_temp = (room_data_buf[4+index] & 0x7f) * 10;
  if(room_data_buf[4+index] & 0x80) check_temp_temp += 5;
  
  room_data_buf[20+index] &= 0xfc;
  room_data_buf[20+index] |= 0x01;                //외출차 모드 적용
  
  if(room_data_buf[20+index] & 0x04){             //냉방 모드?
    set_temp_temp += room_data_buf[70];           //외출온도차 더함
    if(check_temp_temp > set_temp_temp){
      if((check_temp_temp - set_temp_temp) > 9){    //9는 0.9도를 의미함
        if((check_temp_temp - set_temp_temp) > 30)  
        {
          room_data_buf[95+index] |= 0x01; // aircon_on flag
          
          fan_speed_control(3,index);     //Speed Hi
        }
        else{
          
          room_data_buf[95+index] |= 0x01; // aircon_on flag
          
          if(((check_temp_temp - set_temp_temp) > 1) && ((check_temp_temp - set_temp_temp) <= 19))  fan_speed_control(1,index);            //Speed Low
          else if(((check_temp_temp - set_temp_temp) >= 20) && ((check_temp_temp - set_temp_temp) <= 29))  fan_speed_control(2,index);     //Speed Mid
          else fan_speed_control(3,index);     //Speed High
        }
      }
      else{
        
        room_data_buf[95+index] = 0x02; // aircon_off flag
        
        if((room_data_buf[16+index] & 0x03) != 0) fan_speed_control(1,index);
      }
    }
    else{
      if(check_temp_temp < set_temp_temp) fan_speed_control(0,index);
    }
  }
  else{                                 //난방 모드
    check_temp_temp += room_data_buf[70];           //외출온도차 더함
    if(set_temp_temp > check_temp_temp){
      if((set_temp_temp - check_temp_temp) > 9){    //9는 0.9도를 의미함
        if((set_temp_temp - check_temp_temp) > 30)  
        {
          room_data_buf[95+index] |= 0x01; // aircon_on flag
          
          fan_speed_control(3,index);     //Speed Hi
        }
        else{
          
          room_data_buf[95+index] |= 0x01; // aircon_on flag
          
          if(((set_temp_temp - check_temp_temp) > 1) && ((set_temp_temp - check_temp_temp) <= 19))  fan_speed_control(1,index);            //Speed Low
          else if(((set_temp_temp - check_temp_temp) >= 20) && ((set_temp_temp - check_temp_temp) <= 29))  fan_speed_control(2,index);     //Speed Mid
          else fan_speed_control(3,index);     //Speed High 
        }
      }
      else{
        
        room_data_buf[95+index] = 0x02; // aircon_off flag
        
        if((room_data_buf[16+index] & 0x03) != 0) fan_speed_control(1,index);
      }
    }
    else{
      if(set_temp_temp < check_temp_temp) fan_speed_control(0,index);
    }    
  }
}


void empty_temp_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  check_temp_temp = (room_data_buf[4+index] & 0x7f) * 10;
  if(room_data_buf[4+index] & 0x80) check_temp_temp += 5;
  
  room_data_buf[20+index] &= 0xfc; //공실 모드 적용
  
  if(room_data_buf[20+index] & 0x04){             //냉방 모드?
    set_temp_temp = (b_decimal_to_hex_table_ie[room_data_buf[72]]) * 10;       //냉방 공실 기준온도
    if(check_temp_temp > set_temp_temp){
      if((check_temp_temp - set_temp_temp) > 9)
      {
          room_data_buf[95+index] = 0x01; // aircon_on flag
          
          fan_speed_control(3,index);     //Speed High
      }
    }
    else
    {
      if(check_temp_temp < set_temp_temp) fan_speed_control(0,index);
      
      room_data_buf[95+index] = 0x02; // aircon off flag
    }
  }
  else{                                 //난방 모드
    set_temp_temp = (b_decimal_to_hex_table_ie[room_data_buf[71]]) * 10;           //난방 공실 기준온도
    if(check_temp_temp < set_temp_temp){
      if((set_temp_temp - check_temp_temp) > 9)
      {
          room_data_buf[95+index] = 0x01; // aircon on flag 
          
          fan_speed_control(3,index);     //Speed High
      }
    }
    else
    {
      if(check_temp_temp > set_temp_temp) fan_speed_control(0,index);
      
      room_data_buf[95+index] = 0x02; // aircon off flag
    }
  }
}


void fan_speed_control(unsigned char speed_value, unsigned char index)
{
  if(room_data_buf[54+index] == 0)
  {
    switch(speed_value)
    {
      case 0: room_data_buf[16+index] &= 0xfc;  break;
      case 1: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x01; break;
      case 2: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x02; break;
      case 3: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x03; break;
    }
  }
  else
  {
    if(speed_value == 0) room_data_buf[16+index] &= 0xfc;
    else{
      switch(room_data_buf[54+index] & 0x03)
      {
        case 1: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x01; break;
        case 2: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x02; break;
        case 3: room_data_buf[16+index] &= 0xfc;  room_data_buf[16+index] |= 0x03; break;
      }
    }
  }
}
//============= 바닥 온돌 제어 routine ================
//
void Thermo_sensor_ondol_execution(unsigned char index)
{
    if(room_data_buf[60+index] & 0x01){     //한실? (냉,난방,환절기 상관없이 동작)
      if(flag_floor_action_time[index] || (room_data_buf[20+index] & 0x80)){
        room_data_buf[20+index] &= 0x7f;
        flag_floor_action_time[index] = 0;

            if(room_data_buf[40] & 0x02){     //입실인가?
            if(room_data_buf[41] & 0x02) going_out_ondol_control(index);      //심야제어인가?
            else{
              if(room_data_buf[16+index] & 0x20)  stay_ondol_control(index);  //재실제어 //온돌스위치가 ON인가?
              else{
                ondol_off(index);
                room_data_buf[20+index] &= 0xcf;    //공실모드 적용
              }
            }
          }
          else{
            if(room_data_buf[40] & 0x01){   //Check IN인가?
              if(room_data_buf[16+index] & 0x20) going_out_ondol_control(index);    //외출차 제어 //온돌스위치가 ON인가?
              else{
                ondol_off(index);
                room_data_buf[20+index] &= 0xcf;    //공실모드 적용
              }
            }
            else{
              if(room_data_buf[41] & 0x01)  going_out_ondol_control(index);    //외출차 제어 //예냉.예열
              else empty_ondol_control(index); //공실제어
            }
          }
      }
    }
        
    /*if((room_data_buf[20+index] & 0x04) != (air_floor_mode_back[index] & 0x04))
    {
      air_floor_mode_back[index] &= 0xfb;
      air_floor_mode_back[index] = (room_data_buf[20+index] & 0x04);
      if(air_floor_mode_back[index] & 0x04)     //냉방일때
      {
        ondol_off(index);                       //바닥난방 off
        room_data_buf[20+index] &= 0xcf;        //공실모드 적용
      }
    }*/
    
    if((room_data_buf[20+index] & 0x30) != (fan_mode_back[index] & 0x30)){
      fan_mode_back[index] &= 0xcf;
      fan_mode_back[index] |= (room_data_buf[20+index] & 0x30);
      room_data_buf[99] |= 0x04;
    }
}



void stay_ondol_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  room_data_buf[20+index] &= 0xcf;
  room_data_buf[20+index] |= 0x20;
  
  set_temp_temp = (room_data_buf[8+index] & 0x7f) * 10;
  if(room_data_buf[8+index] & 0x80) set_temp_temp += 5;
  check_temp_temp = (room_data_buf[12+index] & 0x7f) * 10;
  if(room_data_buf[12+index] & 0x80) check_temp_temp += 5;
 
  if(set_temp_temp < check_temp_temp) ondol_off(index);
  else{
    if((set_temp_temp - check_temp_temp) >= 10) ondol_on(index);
  }
}

void going_out_ondol_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  room_data_buf[20+index] &= 0xcf;
  room_data_buf[20+index] |= 0x10;
  
  set_temp_temp = (room_data_buf[8+index] & 0x7f) * 10;
  if(room_data_buf[8+index] & 0x80) set_temp_temp += 5;
  check_temp_temp = (room_data_buf[12+index] & 0x7f) * 10;
  if(room_data_buf[12+index] & 0x80) check_temp_temp += 5;
  check_temp_temp += room_data_buf[70];   //외출온도차를 더함.
  
  if(set_temp_temp < check_temp_temp) ondol_off(index);
  else{
    if((set_temp_temp - check_temp_temp) >= 10) ondol_on(index);
  }
}

void empty_ondol_control(unsigned char index)
{
  unsigned int set_temp_temp;
  unsigned int check_temp_temp;
  
  room_data_buf[20+index] &= 0xcf;
  
  set_temp_temp = (b_decimal_to_hex_table_ie[room_data_buf[73]]) * 10;          //바닥공실기준온도로 적용
  check_temp_temp = (room_data_buf[12+index] & 0x7f) * 10;
  if(room_data_buf[12+index] & 0x80) check_temp_temp += 5;
  
  if(set_temp_temp < check_temp_temp) ondol_off(index);
  else{
    if((set_temp_temp - check_temp_temp) >= 10) ondol_on(index);
  }
}


void ondol_on(unsigned char index)
{
  room_data_buf[16+index] |= 0x10;
  room_data_buf[99] |= 0x04;
}
void ondol_off(unsigned char index)
{
  room_data_buf[16+index] &= 0xef;
  room_data_buf[99] |= 0x04;
}

//========================================================================
//========================================================================

