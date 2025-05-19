/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		:   
processor	  : STM32F103xx
compiler		: IAR 6.41A Compiler 
program BY 	: H.H.Hwang
date 			  : 2013.	  .
copy right 	: Plus - H.
===========================================================================*/

#define __UART2_H__

#include "stm32f10x_lib.h"
#include "main.h"

//=======================================================================
/* UART1_initial	:								              */												
//=======================================================================
void UART2_initial(void){
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

  /* RS485 UART2 Direction port define */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
        
	/* Enable GPIOA and USART2 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	 /* Enable USART2 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  
  /* Configure USART2 Tx (PA9) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART2 Rx (PA10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
/* USART2 configuration ------------------------------------------------------*/
  /* USART2 configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
        - USART Clock disabled
        - USART CPOL: Clock is active low
        - USART CPHA: Data is captured on the second edge 
        - USART LastBit: The clock pulse of the last data bit is not output to 
                         the SCLK pin
  */
  USART_InitStructure.USART_BaudRate = 19200;
  USART_InitStructure.USART_WordLength = USART_WordLength_9b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_InitStructure.USART_Clock = USART_Clock_Disable;
//  USART_InitStructure.USART_CPOL = USART_CPOL_Low;
//  USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
//  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;

  /* Configure the USART1 */
  USART_Init(USART2, &USART_InitStructure);

/* Enable the USART Transmoit interrupt: this interrupt is generated when the 
   USART1 transmit data register is empty */  
//  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

/* Enable the USART Receive interrupt: this interrupt is generated when the 
   USART1 receive data register is not empty */
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  /* Enable USART1 */
  USART_Cmd(USART2, ENABLE);

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



//=======================================================================
void uart2_frame_proc() 
{

  UART2_tx_check();
  UART2_rx_check();
  
}

/* UART1 data arrange routine */
void volume_jig_data_set(void) 
{
  unsigned char crc = 0;
  unsigned char i = 0,j;
  
  P_UART2_DIR = 1;
  uart2_tx_data_buf[i++] = ' ';                               //Preamble
  uart2_tx_data_buf[i++] = '(';                               //STX
  ++i;
  uart2_tx_data_buf[i++] = 0x01;                           //Device bit
  uart2_tx_data_buf[i++] = 0xca;                           //Device ID
  uart2_tx_data_buf[i++] = room_data_buf[79];             //Data#1
  uart2_tx_data_buf[i++] = room_data_buf[80];             //Data#n
  uart2_tx_data_buf[2] = i+1;                               //Length
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                               //CRC
  uart2_tx_data_buf[i++] = ')';                               //ETX

  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;

  
 
}

//
//============ Thermo sensor data arrange routine =======================
//
void Thermo_sensor_data_set(unsigned char index, unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = index+0x10;                        //Device ID
  
  if(index_bit & 0x01){
    if(f_ts_power_on[0]){
      uart2_tx_data_buf[i++] = room_data_buf[0];
    }
    else uart2_tx_data_buf[i++] = ts_air_set_temp_from_PC[0];
    uart2_tx_data_buf[i++] = ts_floor_set_temp_from_PC[0];
    uart2_tx_data_buf[i++] = ts_switch_toggle_bit[0];
    uart2_tx_data_buf[i++] = ts_status_read(0);
    uart2_tx_data_buf[i++] = ts_control_data_read(0);
  }
  if(index_bit & 0x02){
    if(f_ts_power_on[1])  uart2_tx_data_buf[i++] = room_data_buf[1];
    else uart2_tx_data_buf[i++] = ts_air_set_temp_from_PC[1];
    uart2_tx_data_buf[i++] = ts_floor_set_temp_from_PC[1];
    uart2_tx_data_buf[i++] = ts_switch_toggle_bit[1];
    uart2_tx_data_buf[i++] = ts_status_read(1);
    uart2_tx_data_buf[i++] = ts_control_data_read(1);
  }
  if(index_bit & 0x04){
    if(f_ts_power_on[2])  uart2_tx_data_buf[i++] = room_data_buf[2];
    else uart2_tx_data_buf[i++] = ts_air_set_temp_from_PC[2];
    uart2_tx_data_buf[i++] = ts_floor_set_temp_from_PC[2];
    uart2_tx_data_buf[i++] = ts_switch_toggle_bit[2];
    uart2_tx_data_buf[i++] = ts_status_read(2);
    uart2_tx_data_buf[i++] = ts_control_data_read(2);
  }
  if(index_bit & 0x08){
    if(f_ts_power_on[3])  uart2_tx_data_buf[i++] = room_data_buf[4];
    else uart2_tx_data_buf[i++] = ts_air_set_temp_from_PC[3];
    uart2_tx_data_buf[i++] = ts_floor_set_temp_from_PC[3];
    uart2_tx_data_buf[i++] = ts_switch_toggle_bit[3];
    uart2_tx_data_buf[i++] = ts_status_read(3);
    uart2_tx_data_buf[i++] = ts_control_data_read(3);
  }
  uart2_tx_data_buf[i++] = room_data_buf[28];               //대기 상.하한 온도
  uart2_tx_data_buf[i++] = room_data_buf[29];               //바닥 상.하한 온도
  uart2_tx_data_buf[i++] = room_data_buf[30];               //외기 온도
  
  uart2_tx_data_buf[i++] = bojung_temp[index];              //보정온도 h@20140704
  uart2_tx_data_buf[i++] = 0;
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length

  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
  
    if(++ts_set_temp_send_count[index] > 3){
      ts_set_temp_send_count[index] = 5;
      ts_air_set_temp_from_PC[index] = 0xff;
      ts_floor_set_temp_from_PC[index] = 0xff;
      //if(temp_control_set) temp_control_set = 0;       
    }

  
  timer_cnt = 30;
}

unsigned char ts_status_read(unsigned char index)
{
  unsigned char return_data = 0;
  
  if(room_data_buf[40] & 0x01) return_data |= 0x01;         //Check IN 
  if(room_data_buf[40] & 0x02) return_data |= 0x02;         //Guest IN
  if(room_data_buf[20+index] & 0x04) return_data |= 0x04;   //Cool mode
  switch(room_data_buf[24+index] & 0x30)
  {
    case 0x00: return_data |= 0x00;    break;               //대기전용
    case 0x10: return_data |= 0x08;    break;               //바닥전용
    case 0x20: return_data |= 0x10;    break;               //대기/바닥 동시제어
  }
  if(room_data_buf[40] & 0x10) return_data |= 0x20;         //Windows OPEN
  if(room_data_buf[83+index] == 0) return_data |= 0x40;           //환절기 mode
  //if(room_data_buf[74] == 2) return_data |= 0x40;           //환절기 mode
  if(room_data_buf[41] & 0x08) return_data |= 0x80;         //온도표시방법 1=설정온도표시, 0=현재온도표시
  return(return_data);
}
unsigned char ts_control_data_read(unsigned char index)
{
  unsigned char return_data = 0;
  
  return_data = room_data_buf[16+index] & 0x03;             //Speed data
  if(room_data_buf[16+index] & 0x10) return_data |= 0x04;   //바닥밸브 open or close?
  if(room_data_buf[16+index] & 0x04) return_data |= 0x08;   //대기센서 스위치 ON/OFF?
  if(room_data_buf[16+index] & 0x20) return_data |= 0x10;   //바닥센서 스위치 ON/OFF?
  //  if(room_data_buf[24+index] & 0x08) return_data |= 0x20;   //화씨 or 도씨?       //h@20150910
  //if(room_data_buf[20+index] & 0x40) return_data |= 0x20;       //송풍모드          //h@20150910
  if(room_data_buf[20+index] & 0x08) return_data |= 0x40;   //Auto or Manual mode
  if(f_ts_power_on[index] == 1) return_data |= 0x80;   //Auto or Manual mode
  return(return_data);
}

//
//================ Night Table data arrange routine ===================
//
void night_table_data_set(unsigned char index, unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  unsigned char led_temp = 0;
 
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = index + 0x20;                      //Device ID
  
  if(index_bit & 0x01){
//    uart2_tx_data_buf[i++] = ((room_data_buf[45] & 0x7f) >> 2);  //room_data_buf[50+index];
   
   switch(cb_mode)
   {
    case 0:  //standard
      
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[46] & 0x01) led_temp |= 0x08;
      if(room_data_buf[45] & 0x10) led_temp |= 0x30;
      if(((room_data_buf[45] & 0x1e)== 0) && ((room_data_buf[46] & 0x01)==0)) led_temp |= 0x40;
      
      break;
      
    case 1:  //spa suite (4~9 floor type)
      
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x02) led_temp |= 0x02;
      if(room_data_buf[45] & 0x04) led_temp |= 0x04;
      if(room_data_buf[45] & 0x04) led_temp |= 0x08;
      if(room_data_buf[46] & 0x01) led_temp |= 0x10;
      if(room_data_buf[46] & 0x01) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x06)== 0) && ((room_data_buf[46] & 0x01)==0)) led_temp |= 0x40;      
      
      break;
      
    case 4: //p-suite ( 17 -좌 )
      
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x08) led_temp |= 0x08;
      if(room_data_buf[46] & 0x01) led_temp |= 0x10;
      if(room_data_buf[46] & 0x01) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x0e)== 0) && ((room_data_buf[46] & 0x01)==0)) led_temp |= 0x40;
      
      break; 
      
   case 2: //spa suite (10,11 floor type)
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x10) led_temp |= 0x08;
      if(room_data_buf[45] & 0x20) led_temp |= 0x10;
      if(room_data_buf[46] & 0x01) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x3e)== 0) && ((room_data_buf[46] & 0x01)==0)) led_temp |= 0x40;        
     
      break;
      
   case 6: //19층 우
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x08) led_temp |= 0x08;
      if(room_data_buf[46] & 0x01) led_temp |= 0x10;
      if(room_data_buf[46] & 0x01) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x0e)== 0) && ((room_data_buf[46] & 0x01)==0)) led_temp |= 0x40;        
     
      break;
      
   case 7: //신관(클럽동) A Type
      
      if(room_data_buf[47] & 0x01) led_temp |= 0x01;
      if(room_data_buf[47] & 0x02) led_temp |= 0x02;
      if(room_data_buf[47] & 0x04) led_temp |= 0x04;
      if(room_data_buf[47] & 0x08) led_temp |= 0x08;
      if(room_data_buf[47] & 0x04) led_temp |= 0x10;
      if(room_data_buf[47] & 0x08) led_temp |= 0x20;
      if(((room_data_buf[47] & 0x0f)== 0)) led_temp |= 0x40;       
     
      break;
      
   case 8: //신관(클럽동) B Type
      
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x10) led_temp |= 0x08;
      if(room_data_buf[45] & 0x20) led_temp |= 0x10;
      if(room_data_buf[45] & 0x20) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x3e)== 0)) led_temp |= 0x40;       
     
      break;      
      
   case 9: //신관(클럽동) C Type
     
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x02) led_temp |= 0x02;
      if(room_data_buf[45] & 0x04) led_temp |= 0x04;
      if(room_data_buf[45] & 0x04) led_temp |= 0x08;
      if(room_data_buf[45] & 0x04) led_temp |= 0x10;
      if(room_data_buf[45] & 0x04) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x06)== 0)) led_temp |= 0x40;          
      
      break;
      
   case 10: //신관(클럽동) D Type
      
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[47] & 0x01) led_temp |= 0x08;
      if(room_data_buf[47] & 0x02) led_temp |= 0x10;
      if(room_data_buf[47] & 0x02) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x0e)== 0) && ((room_data_buf[47] & 0x03) == 0)) led_temp |= 0x40;          
      
      break;      
  
   case 11: //신관(클럽동) E Type
   case 13: //신관(클럽동 G Type
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x10) led_temp |= 0x08;
      if(room_data_buf[45] & 0x20) led_temp |= 0x10;
      if(room_data_buf[45] & 0x40) led_temp |= 0x20;
      if((room_data_buf[45] & 0x7e)== 0) led_temp |= 0x40;         
  
      break;
      
   case 12: //신관(클럽동) F,F-1 Type
  
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x04) led_temp |= 0x04;
      if(room_data_buf[47] & 0x01) led_temp |= 0x08;
      if(room_data_buf[47] & 0x02) led_temp |= 0x10;
      if(room_data_buf[47] & 0x04) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x06)== 0) && ((room_data_buf[47] & 0x07)==0)) led_temp |= 0x40;       
  
      break;
      
   case 15: //신관(클럽동) J Type
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x04) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x08) led_temp |= 0x08;
      if(room_data_buf[45] & 0x10) led_temp |= 0x10;
      if(room_data_buf[45] & 0x10) led_temp |= 0x20;
      if((room_data_buf[45] & 0x1e)== 0) led_temp |= 0x40;          
     
     break;
     
   case 14: //신관(클럽동) I Type
     
      if(room_data_buf[45] & 0x02) led_temp |= 0x01;
      if(room_data_buf[45] & 0x02) led_temp |= 0x02;
      if(room_data_buf[45] & 0x08) led_temp |= 0x04;
      if(room_data_buf[45] & 0x08) led_temp |= 0x08;
      if(room_data_buf[45] & 0x10) led_temp |= 0x10;
      if(room_data_buf[45] & 0x10) led_temp |= 0x20;
      if((room_data_buf[45] & 0x1a)== 0) led_temp |= 0x40;            
     
     break;
   }
   
    uart2_tx_data_buf[i++] = led_temp;
    
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][1];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][2];
    uart2_tx_data_buf[i++] = nt_control_data_read(index);
  }
  if(index_bit & 0x02){
    
   switch(cb_mode)
   {
    case 4: //p-suite ( 17 -좌 )
      
      if(room_data_buf[45] & 0x80) led_temp |= 0x01;
      if(room_data_buf[45] & 0x80) led_temp |= 0x02;
      if(room_data_buf[47] & 0x01) led_temp |= 0x04;
      if(room_data_buf[47] & 0x01) led_temp |= 0x08;
      if(room_data_buf[46] & 0x02) led_temp |= 0x10;
      if(room_data_buf[46] & 0x02) led_temp |= 0x20;
      if(((room_data_buf[45] & 0x80)== 0) && ((room_data_buf[46] & 0x02)==0) && ((room_data_buf[47] & 0x01)==0)) led_temp |= 0x40;
      
      break;
      
    case 6: //19층 우
     
      if(room_data_buf[47] & 0x02) led_temp |= 0x01;
      if(room_data_buf[47] & 0x04) led_temp |= 0x02;
      if(room_data_buf[47] & 0x08) led_temp |= 0x04;
      if(room_data_buf[47] & 0x08) led_temp |= 0x08;
      if(room_data_buf[46] & 0x02) led_temp |= 0x10;
      if(room_data_buf[46] & 0x02) led_temp |= 0x20;
      if(((room_data_buf[47] & 0x0e)== 0) && ((room_data_buf[46] & 0x02)==0)) led_temp |= 0x40;        
     
      break;
   }
   
    uart2_tx_data_buf[i++] = led_temp;    
    
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][1];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][2];
    uart2_tx_data_buf[i++] = nt_control_data_read(index);
  }
  if(index_bit & 0x04){
    uart2_tx_data_buf[i++] = room_data_buf[index+50];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][1];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][2];
    uart2_tx_data_buf[i++] = nt_control_data_read(index);
  }
  if(index_bit & 0x08){
    uart2_tx_data_buf[i++] = room_data_buf[index+50];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][1];
    uart2_tx_data_buf[i++] = nt_switch_toggle_bit[index][2];
    uart2_tx_data_buf[i++] = nt_control_data_read(index);
  }
  uart2_tx_data_buf[i++] = nt_room_status();
  
  uart2_tx_data_buf[i++] = room_data_buf[90];   //year
  
  if(f_ts_power_on[index]) uart2_tx_data_buf[i++] = room_data_buf[index];  //TS1 set temp 
  else uart2_tx_data_buf[i++] = ts_air_set_temp_from_PC[index];
  
  uart2_tx_data_buf[i++] = room_data_buf[4+index];    //TS1 check temp
  
  uart2_tx_data_buf[i++] = room_data_buf[93];   //hour
  uart2_tx_data_buf[i++] = room_data_buf[94];   //min
  uart2_tx_data_buf[i++] = cb_mode;                 //acu mode;
  uart2_tx_data_buf[i++] = room_data_buf[28];   //대기 상한,하한
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
  
    if(++ts_set_temp_send_count[index] > 3){
      ts_set_temp_send_count[index] = 5;
      ts_air_set_temp_from_PC[index] = 0xff;
      ts_floor_set_temp_from_PC[index] = 0xff;
    }
  
  timer_cnt = 30;  
}

unsigned char nt_control_data_read(unsigned char index)
{
  unsigned char return_data = 0;
    
    return_data = room_data_buf[16+index] & 0x03;             //Speed data
    if(room_data_buf[16+index] & 0x04) return_data |= 0x04;   //TS ON/OFF status
    if(room_data_buf[20+index] & 0x04) return_data |= 0x08;   //Cool or warm
    
    return(return_data);  
}

unsigned char nt_room_status(void)
{
  unsigned char return_data = 0;
  
  if(room_data_buf[40] & 0x01) return_data |= 0x01;         //Check IN 
  if(room_data_buf[40] & 0x02) return_data |= 0x02;         //Guest IN
  if(room_data_buf[20] & 0x04) return_data |= 0x04;         //Cool mode
  if(room_data_buf[40] & 0x04) return_data |= 0x08;         //DND
  if(room_data_buf[40] & 0x08) return_data |= 0x10;         //MUR
  if(room_data_buf[40] & 0x10) return_data |= 0x20;         //Window
  if(room_data_buf[40] & 0x20) return_data |= 0x40;         //Message
  if(room_data_buf[40] & 0x40) return_data |= 0x80;         //Door open
  return(return_data);
}
//
//================ Light switch data arrange routine ===================
//
void light_switch_data_set(unsigned char index, unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;                 
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = index + 0x30;                      //Device ID
  
  if(index_bit & 0x01) 
  {
    uart2_tx_data_buf[i] = 0;
          
    switch(cb_mode)
    {
    case 0:
          if(room_data_buf[45] & 0x02) uart2_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x04) uart2_tx_data_buf[i] |= 0x04;
          if((room_data_buf[45] & 0x06) == 0) uart2_tx_data_buf[i] |= 0x01;
          break;
          
    case 1:
      
          if(room_data_buf[45] & 0x02) uart2_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart2_tx_data_buf[i] |= 0x02;       
          if(room_data_buf[45] & 0x08) uart2_tx_data_buf[i] |= 0x04;
          
      break;
    }
          
    ++i;
    uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = ls_control_data_read(index);
  }
  if(index_bit & 0x02) 
  {
    uart2_tx_data_buf[i] = 0;
    
      switch(cb_mode)
      {
        case 0:
          
            if(room_data_buf[45] & 0x08) uart2_tx_data_buf[i] |= 0x02;
            
            break;
            
        case 1:
            
            if(dimmer_level[0] == 0) uart2_tx_data_buf[i] |= 0x02; 
            
          break;
      }
          
    ++i;
    uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = ls_control_data_read(index);
  }
  if(index_bit & 0x04) 
  {
    uart2_tx_data_buf[i] = 0;

      switch(cb_mode)
      {
      case 0:
        
          if(room_data_buf[45] & 0x20) uart2_tx_data_buf[i] |= 0x01;
          
          break;
          
      case 1:
        
          if(room_data_buf[46] & 0x01) uart2_tx_data_buf[i] |= 0x02;
          if(room_data_buf[46] & 0x02) uart2_tx_data_buf[i] |= 0x04;
          if((room_data_buf[46] & 0x03) == 0) uart2_tx_data_buf[i] |= 0x01;
          
        
        break;
      }
    
    ++i;
    uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = ls_control_data_read(index);      
  }
  if(index_bit & 0x08) 
  {
    uart2_tx_data_buf[i] = 0;
          
          if(room_data_buf[45] & 0x02) uart2_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x04) uart2_tx_data_buf[i] |= 0x04;
          if(((room_data_buf[45] & 0x06) == 0) && ((room_data_buf[46] & 0x01) == 0) && (dimmer_level[0] == 0)) uart2_tx_data_buf[i] |= 0x01;
    
    ++i;
    uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
    uart2_tx_data_buf[i++] = ls_control_data_read(index);      
  }  
  if(index_bit & 0x10) 
  {
    uart2_tx_data_buf[i] = 0;

          if(room_data_buf[47] & 0x08) uart2_tx_data_buf[i] |= 0x01;
        //  if(room_data_buf[47] & 0x10) uart2_tx_data_buf[i] |= 0x02;
        //  if(room_data_buf[47] & 0x20) uart2_tx_data_buf[i] |= 0x04;    
      
      ++i;
      uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
      uart2_tx_data_buf[i++] = ls_control_data_read(index);            
  }
  if(index_bit & 0x20) 
  {
    uart2_tx_data_buf[i] = 0;

          if(dimmer_level[0] == 0) uart2_tx_data_buf[i] |= 0x02; 
          
      ++i;
      uart2_tx_data_buf[i++] = light_switch_toggle_bit[index][0];
      uart2_tx_data_buf[i++] = ls_control_data_read(index);            
  }  
  uart2_tx_data_buf[i++] = cb_mode;
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
}

unsigned char ls_control_data_read(unsigned char index)
{
  unsigned char return_data = 0;
  
  if(room_data_buf[40] & 0x01) return_data |= 0x01;         //Check IN 
  if(room_data_buf[40] & 0x02) return_data |= 0x02;         //Guest IN
  if(room_data_buf[40] & 0x80) return_data |= 0x04;         //Please wait sign
  if(room_data_buf[40] & 0x04) return_data |= 0x08;         //DND
  if(room_data_buf[40] & 0x08) return_data |= 0x10;         //MUR
  if(room_data_buf[40] & 0x10) return_data |= 0x20;         //Window
  if(room_data_buf[40] & 0x20) return_data |= 0x40;         //Message
  if(room_data_buf[40] & 0x40) return_data |= 0x80;         //Door open
  return(return_data);
}

//
//========== Key sensor data arrange =============
//
void key_sendor_data_set(unsigned char index,unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
 
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = index + 0x60;                      //Device ID
  
  if(index_bit & 0x01){
    uart2_tx_data_buf[i++] = ks_switch_toggle_bit[index];
    uart2_tx_data_buf[i++] = ks_control_data_read(index);
  }
  if(index_bit & 0x02){
    uart2_tx_data_buf[i++] = ks_switch_toggle_bit[index];
    uart2_tx_data_buf[i++] = ks_control_data_read(index);
  }
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
  
}

unsigned char ks_control_data_read(unsigned char index)
{
  unsigned char return_data = 0;
  
  if(room_data_buf[40] & 0x01) return_data |= 0x01;         //Check IN 
  if(room_data_buf[40] & 0x02) return_data |= 0x02;         //Guest IN
  if(room_data_buf[40] & 0x04) return_data |= 0x10;         //DND
  if(room_data_buf[40] & 0x08) return_data |= 0x20;         //MUR
  //if(room_data_buf[46] & 0x04) return_data |= 0x10;         //전열교환기 상태
  //if(room_data_buf[45] & 0x02) return_data |= 0x20;         //입구등 상태
  return(return_data);
}

void dimmer_data_set(void)
{
  unsigned char crc = 0;
  unsigned char i = 0,j;
  
  P_UART2_DIR = 1;
  
  ++i;
  uart2_tx_data_buf[i++] = 0x40;                                    //Device ID
  if(dimmer_level[0] > 0x60) dimmer_level[0] = 0x60;    //dimming over calculate
  uart2_tx_data_buf[i++] = dimmer_level[0];                   //Data#1
  uart2_tx_data_buf[i++] = dimmer_level[1];                   //Data#2
  uart2_tx_data_buf[i++] = dimmer_level[2];                   //Data#3 
  uart2_tx_data_buf[i++] = dimmer_level[3];                   //Data#$ 
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
  timer_cnt = 0;
}

void audio_data_set(void)
{
  unsigned char crc = 0;
  unsigned char i = 0,j;
  unsigned char temp=0;
  
  P_UART2_DIR = 1;
  uart2_tx_data_buf[i++] = ' ';                               //Preamble
  uart2_tx_data_buf[i++] = '(';                               //STX
  ++i;
  uart2_tx_data_buf[i++] = 0x55;  //0x01;                           //Device bit
  uart2_tx_data_buf[i++] = 0x70;                           //Device ID
  if(room_data_buf[40] & 0x02) temp |= 0x01;              //Guest IN
  if(f_chime_keep) temp |= 0x02;                //Chime ON
  if(f_alram_on)   temp |= 0x04;                //Alram ON
  if(room_data_buf[36] & 0x08) temp |= 0x08;              //Audio ON
  if(room_data_buf[40] & 0x04) temp |= 0x10;              //DND ON
  if(room_data_buf[40] & 0x08) temp |= 0x20;              //MUR ON
  if(room_data_buf[36] & 0x40) temp |= 0x40;              //MUTE ON
  
  uart2_tx_data_buf[i++] = temp;
  uart2_tx_data_buf[i++] = room_data_buf[36] & 0x07;      //Radio channel
  
  uart2_tx_data_buf[i++] = audio_volume[0];             //Audio volume
  uart2_tx_data_buf[i++] = room_data_buf[79];           //Chime volume
  uart2_tx_data_buf[i++] = room_data_buf[80];           //Alram volume
  uart2_tx_data_buf[2] = i+1;                               //Length
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                               //CRC
  uart2_tx_data_buf[i++] = ')';                               //ETX

  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
  
}

void chime_ind_data_set(void)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = 0x50;                              //Device ID
  
  uart2_tx_data_buf[i++] = 0;
  uart2_tx_data_buf[i++] = chime_ind_toggle_bit[0];
  uart2_tx_data_buf[i++] = ls_control_data_read(0);
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
}

void common_data_set(void)
{
  unsigned char crc = 0;
  unsigned char i = 0,j;
  unsigned char temp=0;
    
  P_UART2_DIR = 1;
  ++i;
  uart2_tx_data_buf[i++] = 0xff;                        //Device ID
  uart2_tx_data_buf[i++] = 0xff;                        //Device ID
  
  if(room_data_buf[40] & 0x01) temp |= 0x01;            //Check IN 
  if(room_data_buf[40] & 0x02) temp |= 0x02;            //Guest IN
  if(room_data_buf[40] & 0x10) temp |= 0x04;            //Window
  if(room_data_buf[20] & 0x04) temp |= 0x08;            //Cool mode
  if(room_data_buf[40] & 0x04) temp |= 0x10;            //DND
  if(room_data_buf[40] & 0x08) temp |= 0x20;            //MUR
  if(room_data_buf[82] & 0x08) temp |= 0x80;            //현재조도상태 1=밝게하라(밝다),0=어둡게하라(어둡다)
  uart2_tx_data_buf[i++] = temp;
  uart2_tx_data_buf[i++] = room_data_buf[82];           //조도 조절법, 복귀시간 및 현재 상태
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
}

void ex_relay_data_set(unsigned char index, unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = index + 0x70;                      //Device ID
  
  if(index_bit & 0x01)
  {
    uart2_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {
      case 0: //stadard Type
        
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x80;  ///플랜지 배수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x40;  //플랜지 급수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x20;  //플랜지 경보 SW ??   
        
        break;
        
    case 1: //spa suite ( 4~9 floor type )
    case 2: //spa suite ( 10 floor type )
    case 3: //p-suite ( 16 좌 type )
    case 4: //p-suite ( 17좌 type )
    case 5: //18 우 type
    case 6: //19 우 type
    case 8: //신관(클럽동) B Type
    case 9: //신관(클럽동) C Type
    case 10: //신관(클럽동) D Type      
    case 11: //신관(클럽동) E Type       
    case 13: //신관(클럽동) G,H Type
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type      
      
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x80;  ///SPARE1
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x40;  //SPARE2
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x20;  //SPARE3     
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x10;  //SPARE4
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x08;  //SPARE5
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x04;  //SPARE6       
        
      break;
      
    case 7: //신관(클럽동) A Type
        if(room_data_buf[47] & 0x80) uart2_tx_data_buf[i] |= 0x80; //LIGHT 15 (I)
        if(room_data_buf[48] & 0x01) uart2_tx_data_buf[i] |= 0x40; //LIGHT 16 (I)
        if(room_data_buf[48] & 0x02) uart2_tx_data_buf[i] |= 0x20; //LIGHT 17 (K)
        if(room_data_buf[48] & 0x04) uart2_tx_data_buf[i] |= 0x10; //LIGHT 18 (K)
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x08;  ///플랜지 배수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x04;  //플랜지 급수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x02;  //플랜지 경보 SW ??          
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x01;  //SPARE1          
        
      break;
    
  case 12: //신관(클럽동) F,F-1 Type
     
        if(room_data_buf[47] & 0x80) uart2_tx_data_buf[i] |= 0x10; //LIGHT 15 (B)
        if(room_data_buf[48] & 0x01) uart2_tx_data_buf[i] |= 0x20; //LIGHT 16 (B)
        if(room_data_buf[48] & 0x02) uart2_tx_data_buf[i] |= 0x40; //LIGHT 17 (B)
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x80; //SPARE1
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x08;  ///플랜지 배수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x04;  //플랜지 급수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x02;  //플랜지 경보 SW ??          
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x01;  //SPARE2 
      
      break;
    }
    
    ++i;    
  }
  if(index_bit & 0x02)
  {
     uart2_tx_data_buf[i] = 0;    
     
    switch(cb_mode)
    {
    case 0: //Stadard Type
    case 7: //신관(클럽동) A Type
    case 8: //신관(클럽동) B Type
    case 9: //신관(클럽동) C Type        
    case 10: //신관(클럽동) D Type    
    case 11: //신관(클럽동) E Type           
    case 12: //신관(클럽동) F,F-1 Type
    case 13: //신관(클럽동) G,H Type
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type
      
        if((room_data_buf[16] & 0x03) != fcu_back_buf[0]){ //FCU BACK ROUTINE
        uart2_tx_data_buf[i] |= 0xc0;
        fcu_back_buf[0] = room_data_buf[16] & 0x03;
        f_fcu_timer = 0;
        fcu_timer = 0;
      }      
        if(f_fcu_timer){       
          switch(room_data_buf[16] & 0x03)
          {
              case 0:  uart2_tx_data_buf[i] |= 0xc0; break;//P_rly_9 = 1; P_rly_10 = 1; break; //OFF
              
              case 1:  uart2_tx_data_buf[i] &= 0x3f; break;//P_rly_9 = 0; P_rly_10 = 0; break; //L
              
              case 2:  uart2_tx_data_buf[i] |= 0x80; break;//P_rly_9 = 1; P_rly_10 = 0; break; //M
                            
              case 3:  uart2_tx_data_buf[i] |= 0x40; break;//P_rly_9 = 0; P_rly_10 = 1; break; //H
          }    
        }
    
      // ****** 난방 , 냉방 VALVE ****** //
      if(room_data_buf[20] & 0x04) //냉방 
      {
        if((room_data_buf[16]  & 0x03) == 0){       //냉방 밻브 close
          uart2_tx_data_buf[i] |= 0x20;
          uart2_tx_data_buf[i] |= 0x08;  
       }
        else{
          uart2_tx_data_buf[i] |= 0x20;
          uart2_tx_data_buf[i] |= 0x04;    
        }
      }
      else  //난방
      {
        if((room_data_buf[16]  & 0x03) == 0){       //난방 밻브 close
          uart2_tx_data_buf[i] |= 0x08;
          uart2_tx_data_buf[i] |= 0x20;
        }
        else{
          uart2_tx_data_buf[i] |= 0x08;
          uart2_tx_data_buf[i] |= 0x10;
        }
      }    
      
      break;
      
    case 1: //spa suite ( 4~9 floor type )
    case 2: //spa suite ( 10 floor type )
    case 3: //p-suite ( 16 좌 type ) 
    case 4: //p-suite ( 17 좌 type )
    case 5: //p-suite ( 18 우 type )
    case 6: //p-suite ( 19 우 type )
        
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x80;  ///플랜지 배수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x40;  //플랜지 급수 SW ??
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x20;  //플랜지 경보 SW ??          
        
      break;
    }
    
    ++i;
  }  
  if(index_bit & 0x04) // FCU EX RELAY 1
  {
        uart2_tx_data_buf[i] = 0;
              
              switch(cb_mode)
              {
                
                case 0: //stadard type
                case 7: //신관(클럽동)     
                case 8: //신관(클럽동) B Type
                case 9: //신관(클럽동) C Type    
                case 10: //신관(클럽동) D Type           
                case 11: //신관(클럽동) E Type     
                case 12: //신관(클럽동) F,F-1 Type      
                case 13: //신관(클럽동) G,H Type
                case 14: //신관(클럽동) I Type
                case 15: //신관(클럽동) J Type
                  
                if((room_data_buf[16] & 0x03) != fcu_back_buf[3]){ //FCU BACK ROUTINE
                uart2_tx_data_buf[i] |= 0xc0;
                fcu_back_buf[3] = room_data_buf[16] & 0x03;
                f_fcu_timer_3 = 0;
                fcu_timer_3 = 0;
              }      
                if(f_fcu_timer_3){       
                  switch(room_data_buf[16] & 0x03)
                  {
                      case 0:  uart2_tx_data_buf[i] |= 0xc0; break;//P_rly_9 = 1; P_rly_10 = 1; break; //OFF
                      
                      case 1:  uart2_tx_data_buf[i] &= 0x3f; break;//P_rly_9 = 0; P_rly_10 = 0; break; //L
                      
                      case 2:  uart2_tx_data_buf[i] |= 0x80; break;//P_rly_9 = 1; P_rly_10 = 0; break; //M
                                    
                      case 3:  uart2_tx_data_buf[i] |= 0x40; break;//P_rly_9 = 0; P_rly_10 = 1; break; //H
                  }    
                }
                
              // ****** 난방 , 냉방 VALVE ****** //
              if(room_data_buf[20] & 0x04) //냉방 
              {
                if((room_data_buf[16]  & 0x03) == 0){       //냉방 밻브 close
                  uart2_tx_data_buf[i] |= 0x20;
                  uart2_tx_data_buf[i] |= 0x08;  
               }
                else{
                  uart2_tx_data_buf[i] |= 0x20;
                  uart2_tx_data_buf[i] |= 0x04;    
                }
              }
              else  //난방
              {
                if((room_data_buf[16]  & 0x03) == 0){       //난방 밻브 close
                  uart2_tx_data_buf[i] |= 0x08;
                  uart2_tx_data_buf[i] |= 0x20;
                }
                else{
                  uart2_tx_data_buf[i] |= 0x08;
                  uart2_tx_data_buf[i] |= 0x10;
                }
              }                
                        
          break;
          
        case 1: //spa suite (4~9 floor type)
        case 2: // spa suite ( 10 floor type )
        case 3: // p-suite ( 16 좌 type )
        case 4: // p-suite ( 17 좌 type)
        case 5: //p-suite ( 17 좌 type )
        case 6: //p-suite ( 17 좌 type )
            
            if((room_data_buf[16] & 0x03) != fcu_back_buf[0]){ //FCU BACK ROUTINE
            uart2_tx_data_buf[i] |= 0xc0;
            fcu_back_buf[0] = room_data_buf[16] & 0x03;
            f_fcu_timer = 0;
            fcu_timer = 0;
          }      
            if(f_fcu_timer){       
              switch(room_data_buf[16] & 0x03)
              {
                  case 0:  uart2_tx_data_buf[i] |= 0xc0; break;//P_rly_9 = 1; P_rly_10 = 1; break; //OFF
                  
                  case 1:  uart2_tx_data_buf[i] &= 0x3f; break;//P_rly_9 = 0; P_rly_10 = 0; break; //L
                  
                  case 2:  uart2_tx_data_buf[i] |= 0x80; break;//P_rly_9 = 1; P_rly_10 = 0; break; //M
                                
                  case 3:  uart2_tx_data_buf[i] |= 0x40; break;//P_rly_9 = 0; P_rly_10 = 1; break; //H
              }    
            }
        
          // ****** 난방 , 냉방 VALVE ****** //
          if(room_data_buf[20] & 0x04) //냉방 
          {
            if((room_data_buf[16]  & 0x03) == 0){       //냉방 밻브 close
              uart2_tx_data_buf[i] |= 0x20;
              uart2_tx_data_buf[i] |= 0x08;  
           }
            else{
              uart2_tx_data_buf[i] |= 0x20;
              uart2_tx_data_buf[i] |= 0x04;    
            }
          }
          else  //난방
          {
            if((room_data_buf[16]  & 0x03) == 0){       //난방 밻브 close
              uart2_tx_data_buf[i] |= 0x08;
              uart2_tx_data_buf[i] |= 0x20;
            }
            else{
              uart2_tx_data_buf[i] |= 0x08;
              uart2_tx_data_buf[i] |= 0x10;
            }
          }                
          
          break;
        }
    
        ++i;
  }
  if(index_bit & 0x08) // FCU EX RELAY 2
  {
    
     uart2_tx_data_buf[i] = 0;
            
    switch(cb_mode)
    {
    case 9: //클럽동 C Type
    case 10: //클럽동 D Type      
    case 11://클럽동 E Type
    case 12: //클럽동 F,F-1 Type
    case 13: //클럽동 G,H Type
    case 14: //클럽동 I Type 
    case 15: //클럽동 J Type      
      

      
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x80;  ///SPARE1
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x40;  //SPARE2
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x20;  //SPARE3     
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x10;  //SPARE4
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x08;  //SPARE5
        if(room_data_buf[45] & 0x01) uart2_tx_data_buf[i] |= 0x04;  //SPARE6      
        
      break;
      
      case 1: //spa suite (4~9 floor type)
      case 2: //spa suite ( 10 floor type )
      case 3: //p-suite ( 16층 좌 type )
      case 4: //p-suite ( 17층 좌 type )
      case 5:  //p-suite ( 17 좌 type )
      case 6: //p-suite ( 17 좌 type )
        
        if((room_data_buf[17] & 0x03) != fcu_back_buf[1]){ //FCU BACK ROUTINE
        uart2_tx_data_buf[i] |= 0xc0;
        fcu_back_buf[1] = room_data_buf[17] & 0x03;
        f_fcu_timer_2 = 0;
        fcu_timer_2 = 0;
      }      
        if(f_fcu_timer_2){       
          switch(room_data_buf[17] & 0x03)
          {
              case 0:  uart2_tx_data_buf[i] |= 0xc0; break;//P_rly_9 = 1; P_rly_10 = 1; break; //OFF

              case 1:  uart2_tx_data_buf[i] &= 0x3f; break;//P_rly_9 = 0; P_rly_10 = 0; break; //L

              case 2:  uart2_tx_data_buf[i] |= 0x80; break;//P_rly_9 = 1; P_rly_10 = 0; break; //M

              case 3:  uart2_tx_data_buf[i] |= 0x40; break;//P_rly_9 = 0; P_rly_10 = 1; break; //H
          }    
        }
        
      // ****** 난방 , 냉방 VALVE ****** //
      if(room_data_buf[21] & 0x04) //냉방 
      {
        if((room_data_buf[17]  & 0x03) == 0){       //냉방 밻브 close
          uart2_tx_data_buf[i] |= 0x20; 
          uart2_tx_data_buf[i] |= 0x08;  
       }
        else{
          uart2_tx_data_buf[i] |= 0x20; 
          uart2_tx_data_buf[i] |= 0x04;          
        }
      }
      else  //난방
      {
        if((room_data_buf[17]  & 0x03) == 0){       //난방 밻브 close
          uart2_tx_data_buf[i] |= 0x08;
          uart2_tx_data_buf[i] |= 0x20;
        }
        else{
          uart2_tx_data_buf[i] |= 0x08;
          uart2_tx_data_buf[i] |= 0x10;
        }
      }  

      break;
    }
    
      ++i;
  }
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
}

void ex_input_data_set(unsigned char index, unsigned char index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = 0x80 + index;        //Device bit
  uart2_tx_data_buf[i++] = exinput_toggle_bit[index][0];
  uart2_tx_data_buf[i++] = exinput_toggle_bit[index][1];
  uart2_tx_data_buf[i++] = exinput_toggle_bit[index][2];
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
   timer_cnt = 0;
}

void ex_485_send(unsigned char index)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART2_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart2_tx_data_buf[i++] = 0x90 + index;        //Device bit
  uart2_tx_data_buf[i++] = room_data_buf[49]; //Key 상태
  uart2_tx_data_buf[i++] = (room_data_buf[40] & 0x0c);  //DND,MUR 전달(연동 1층 2층)
  
  for(j=1; j<i; ++j) crc += uart2_tx_data_buf[j];
  crc ^= 0x55;
  uart2_tx_data_buf[i++] = crc;                             //CRC
  uart2_tx_data_buf[0] = i;                                 //Length
  
  f_uart2_data_send = 1;
  uart2_tx_length = 0;
  uart2_tx_backup = i;
  timer_cnt = 0;  
}

/* UART2 data send check routine */
void UART2_tx_check(void)
{
  if(f_uart2_data_send){
    if(++timer_cnt > 45)
    {
      timer_cnt = 1001;
      if(USART_GetFlagStatus(USART2, USART_FLAG_TC) !=RESET) {
      if(uart2_tx_length == 0){
        USART2->DR = ((uart2_tx_data_buf[uart2_tx_length++] & (u16)0x01FF) | (u16)0x0100);
      }
      else{
        if(uart2_tx_length < uart2_tx_backup){
          USART2->DR = (uart2_tx_data_buf[uart2_tx_length++] & (u16)0x00FF);
        }
        else{
          P_UART2_DIR = 0;
          f_uart2_data_send = 0;
          sub_call_timer = 18;
        }
      }
      }
    }
  }
}
//=======================================================================
void UART2_control_proc(void) 
{
//u8 ix;
  unsigned char sub_device_kind;

  if(f_uart2_send_time){
    f_uart2_send_time = 0;
    
    if(f_volume_control)  volume_jig_data_set();
    else{
      if(f_common_send_time){ 
        common_data_set();  
        f_common_send_time = 0; 
      }
      else{
        if(b_sub_device_table[cb_mode][id_number] == 0xff) id_number = 0;
        sub_device_kind = b_sub_device_table[cb_mode][id_number++];
        switch(sub_device_kind){
          case B_KEY_SENSOR_0:
            key_sendor_data_set(0,0x01);
            break;  
          case B_TEMP_SENSOR_1:
            Thermo_sensor_data_set(0,0x01);
            break;  
          case B_TEMP_SENSOR_2:
            Thermo_sensor_data_set(1,0x02);
            break;  
          case B_TEMP_SENSOR_3:
            Thermo_sensor_data_set(2,0x04);
            break;  
          case B_TEMP_SENSOR_4:
            Thermo_sensor_data_set(3,0x08);
            break;  
          case B_NIGHT_TABLE_1:
            night_table_data_set(0,0x01);
            break;  
          case B_NIGHT_TABLE_2:
            night_table_data_set(1,0x02);
            break;  
          case B_LIGHT_SWITCH_1:
            light_switch_data_set(0,0x01);
            break;  
          case B_LIGHT_SWITCH_2:
            light_switch_data_set(1,0x02);
            break;  
          case B_LIGHT_SWITCH_3:
            light_switch_data_set(2,0x04);
            break;  
          case B_LIGHT_SWITCH_4:
            light_switch_data_set(3,0x08);
            break;  
          case B_LIGHT_SWITCH_5:
            light_switch_data_set(4,0x10);
            break; 
          case B_LIGHT_SWITCH_6:
            light_switch_data_set(5,0x20);
            break;                       
            
          case B_DIMMER:
            dimmer_data_set();
            break;
            
          case B_AUDIO:
            audio_data_set();
            break;
            
          case B_CHIME_IND:
            chime_ind_data_set();
            break;
            
          case B_EX_RELAY_1:
            ex_relay_data_set(0,0x01);
            break;            
            
        case B_EX_RELAY_2:
            ex_relay_data_set(1,0x02);
            break;
            
        case B_EX_RELAY_3:
            ex_relay_data_set(2,0x04);
            break;

        case B_EX_RELAY_4:
            ex_relay_data_set(3,0x08);
            break;            
  
        case B_EXINPUT:
            ex_input_data_set(0,0x01);
            break;
            
        case B_EX_485:
            ex_485_send(0x00);
            break;
            
        }
      }
    }
  }
	uart2_frame_proc();
}

void UART2_rx_check(void)
{
  unsigned char rx_crc=0;
  unsigned char i;
  
  if(f_uart2_frame_rx_ok){
    f_uart2_frame_rx_ok = 0;
      
    for(i=1; i<uart2_rxd_buf[0]-1; ++i) rx_crc += uart2_rxd_buf[i];
    rx_crc ^= 0x55;
    if(rx_crc == (uart2_rxd_buf[uart2_rxd_buf[0]-1])){
      switch(uart2_rxd_buf[1] & 0xf0){
        case 0x10:    //Thermo sensor
          led3 ^= 1;
          thermo_sensor_check(uart2_rxd_buf[1] & 0x03);
          break;
        case 0x20:    //Night Table
          night_table_check(uart2_rxd_buf[1] & 0x03);
          break;
        case 0x30:    //Light switch
          light_switch_check(uart2_rxd_buf[1] & 0x0f);
          break;
        case 0x40:    //Dimmer module
          dimmer_check(uart2_rxd_buf[1] & 0x03);
          break;
        case 0x50:    //Service Plate
            chime_ind_check(uart2_rxd_buf[1] & 0x03);    
          break;
        case 0x60:    //Key sensor
          key_sensor_rx_check(uart2_rxd_buf[1] & 0x03);
          break;
        case 0x70: //receive remote
          break;
        case 0xc0:
          if(f_volume_control)
          {
            volume_data_check();
          }
          break;
          
        case 0x80: // ex input
            exinput_check(uart2_rxd_buf[1] & 0x03);
          break;
          
        case 0x90: // ex 485
            ex_485_receive();
          break;
      }
      sub_call_timer = 15;
   }

  }
}

//
//============= Volume set jig data check routine ===============
//
void volume_data_check(void)
{
  if(uart2_rxd_buf[2] == 0xca){
    if(uart2_rxd_buf[5] & 0x01){    //alram volume
      f_alram_on = 1;
      if(room_data_buf[80] != uart2_rxd_buf[4]){
        digital_volume_execution(1,room_data_buf[80]);
        room_data_buf[80] = uart2_rxd_buf[4];
      }
    }
    else{                           //chime volume
      f_alram_on = 0;
      if((jig_switch_toggle_bit & 0x01) != (uart2_rxd_buf[6] & 0x01)){
        room_data_buf[79] = uart2_rxd_buf[3];
        digital_volume_execution(0,room_data_buf[79]);
        if(!f_chime_keep){
          chime_sq = 0;
          f_chime_sq_timer = 1;
          f_chime_keep = 1;
        }
      }
    }
    jig_switch_toggle_bit = uart2_rxd_buf[6];
  }
}

//
//============= Thermo sensor receive data check routine ==============
//
void thermo_sensor_check(unsigned char index)
{
  if((uart2_rxd_buf[8] & 0x80) == 0){
    f_ts_power_on[index] = 0;
    if((uart2_rxd_buf[2] & 0x7f) <= 70){    //Air set temp
        if(room_data_buf[0+index] != uart2_rxd_buf[2])
        {
            room_data_buf[16+index] |= 0x80;
            room_data_buf[0+index] = uart2_rxd_buf[2];
            room_data_buf[99] |= 0x02; 
            
            //if(index == 0 && cb_mode != 3 && cb_mode != 6)
            //{
            if(cb_mode != 5 && cb_mode != 3) //18층 좌 , 19층 좌
            {
              if((cb_mode == 1 && index != 1) || (cb_mode == 2 && index != 1) || cb_mode == 0 || cb_mode == 4 || cb_mode == 6 || cb_mode == 7
                 || cb_mode == 8 || cb_mode == 9 || cb_mode == 10 || cb_mode == 11 || cb_mode == 12 || cb_mode == 13 || cb_mode == 14 || cb_mode == 15)
              ts_air_set_temp_from_PC[index] = uart2_rxd_buf[2];
              ts_set_temp_send_count[index] = 3;              
            //}
            }
        }
    }
    if((uart2_rxd_buf[3] & 0x7f) <= 70){    //Air Check temp
      room_data_buf[4+index] = uart2_rxd_buf[3];
      f_check_temp_change = 1;
//    room_data_buf[99] |= 0x02;            //현재온도 변경시 온도 Data 전송
    }
    if((room_data_buf[4+index] & 0x7f) > room_data_buf[76]){   //이상온도 비교
      if(++em_temp_count[index] > 3){                 //이상온도 3회 이상 발생?
        em_temp_count[index] = 0;
        room_data_buf[42] |= (0x10 << index);         //이상 온도 발생 flag set
        //room_data_buf[45] |= 0x80;    //Fire sign ON
      }
    }
    else{
      //room_data_buf[42] &= (0xef << index);           //이상 온도 발생 flag clear
      switch(index)
      {
      case 0:
          room_data_buf[42] &= 0xef; //온도센서 1 em clear
        break;
      case 1:
          room_data_buf[42] &= 0xdf; //온도센서 2 em clear
        break;
      case 2:
          room_data_buf[42] &= 0xbf; //온도센서 3 em clear
        break;
      case 3:
          room_data_buf[42] &= 0x7f; //온도센서 4 em clear
        break;
      }
      em_temp_count[index] = 0;
      //room_data_buf[45] &= 0x7f;    //Fire sign OFF
    }
    
    if((uart2_rxd_buf[4] & 0x7f) <= 70){    //Floor set temp
      if(room_data_buf[8+index] != uart2_rxd_buf[4]){
        room_data_buf[20+index] |= 0x80;
        room_data_buf[8+index] = uart2_rxd_buf[4];
        room_data_buf[99] |= 0x04;
        
        ts_floor_set_temp_from_PC[0] = uart2_rxd_buf[4];
        ts_set_temp_send_count[0] = 3;        
      }
    }
    if((uart2_rxd_buf[5] & 0x7f) <= 70){    //Floor Check temp
      //room_data_buf[12+index] = uart2_rxd_buf[5];
      //f_check_temp_change_2 = 1;
    }
    if((uart2_rxd_buf[6] & 0x7f) <= 99){    //Humidity
      room_data_buf[31+index] = uart2_rxd_buf[6];
    }
    
    if(ts_first_power_on[index]){ 
      if(room_data_buf[40] & 0x02)
      {
      //First power on than only bit save
        if(ts_switch_toggle_bit[index] != uart2_rxd_buf[7]){    //TS switch push ?
          if((ts_switch_toggle_bit[index] & 0x01) != (uart2_rxd_buf[7] & 0x01)){    //TS on/off switch change(key push)
            room_data_buf[16+index] ^= 0x04;    //TS Air on/off toggle
            nt_switch_bit_check(K_fcu_auto_manual,index);
            room_data_buf[99] |= 0x02;
            room_data_buf[20+index] |= 0x08;    //Auto mode flag set
            room_data_buf[16+index] |= 0x80;
          }
          if((ts_switch_toggle_bit[index] & 0x02) != (uart2_rxd_buf[7] & 0x02)){    //TS floor mode on/off switch change(key push)
            room_data_buf[16+index] ^= 0x20;    //TS Floor on/off toggle
            room_data_buf[99] |= 0x04;
            room_data_buf[20+index] |= 0x80;
          }
          if((ts_switch_toggle_bit[index] & 0x10) != (uart2_rxd_buf[7] & 0x10)) room_data_buf[24+index] ^= 0x08;    //TS FaC, DoC
          if((ts_switch_toggle_bit[index] & 0x20) != (uart2_rxd_buf[7] & 0x20)) ts_speed_execution(index);          //TS local speed execution
          
          if((ts_switch_toggle_bit[index] & 0x40) != (uart2_rxd_buf[7] & 0x40)){  //Temp control mode(air,floor mode)
          
            
            
              if(room_data_buf[74] == 2){   //환절기 모드일 경우만 적용한다.
              if(room_data_buf[83+index] >= 2) room_data_buf[83+index] = 0;   //0=auto, 1=warm, 2=cool
              else ++room_data_buf[83+index];
              room_data_buf[54+index] = 0;   //Auto mode
              room_data_buf[20+index] |= 0x80;
              switch(room_data_buf[83+index]){
                case 0: break;
                case 1:
                  room_data_buf[20+index] |= 0x04;    //냉방
                  if((room_data_buf[0+index] & 0x7f) >= (room_data_buf[4+index] & 0x7f)){
                    ts_air_set_temp_from_PC[index] = (room_data_buf[4+index] & 0x7f) - 2;
                    if(ts_air_set_temp_from_PC[index] < 18) ts_air_set_temp_from_PC[index] = 18;
                    ts_set_temp_send_count[index] = 0;
                  }
                  break;
                case 2:
                  room_data_buf[20+index] &= 0xfb;    //난방
                  
                  if((room_data_buf[0+index] & 0x7f) <= (room_data_buf[4+index] & 0x7f)){
                    ts_air_set_temp_from_PC[index] = (room_data_buf[4+index] & 0x7f) + 2;
                    if(ts_air_set_temp_from_PC[index] > 30) ts_air_set_temp_from_PC[index] = 30;
                    ts_set_temp_send_count[index] = 0;
                  }
                  break;
              }
              
              
            }
          }
          if(ts_switch_toggle_bit[index] & 0x80){               //Set temp CB execution?
            //if((ts_switch_toggle_bit[index] & 0x04) != (uart2_rxd_buf[8] & 0x04)) ts_temp_up_excution(index,uart2_rxd_buf[9]);
            //if((ts_switch_toggle_bit[index] & 0x08) != (uart2_rxd_buf[8] & 0x08)) ts_temp_down_excution(index,uart2_rxd_buf[9]);
          }
        }   
      }
    }
    ts_switch_toggle_bit[index] = uart2_rxd_buf[7];
    ts_first_power_on[index] = 1;
  }
  else f_ts_power_on[index] = 1;
}

void ts_speed_execution(unsigned char index)   
{
  if(room_data_buf[16+index] & 0x04){
    if(room_data_buf[54+index] != 0){
      if(++room_data_buf[54+index] >= 4) room_data_buf[54+index] = 1;
    }
    else{
      room_data_buf[54+index] = room_data_buf[16+index] & 0x03;
      if(++room_data_buf[54+index] >= 4) room_data_buf[54+index] = 1;
    }
    room_data_buf[16+index] |= 0x80;
    room_data_buf[20+index] &= 0xf7;    //Manual mode flag set
    room_data_buf[99] |= 0x02;          //Speed 변경시 온도 상태 전송
  }
  else room_data_buf[54+index] = 0;   //Auto mode
}

void ts_temp_up_excution(unsigned char index, unsigned char ts_mode)
{
  if(ts_mode & 0x01){   //Floor mode
    if(room_data_buf[8+index] & 0x80){
      room_data_buf[8+index] &= 0x7f;
      if((((room_data_buf[29] & 0xf0)>>4)+25) > (room_data_buf[8+index] & 0x80)) ++room_data_buf[8+index];
    }
    else{
      if((((room_data_buf[29] & 0xf0)>>4)+25) > (room_data_buf[8+index] & 0x80)) room_data_buf[8+index] |= 0x80;
    }
  }
  else{                 //Air mode
    if(room_data_buf[0+index] & 0x80){
      room_data_buf[0+index] &= 0x7f;
      if((((room_data_buf[28] & 0xf0)>>4)+25) > (room_data_buf[0+index] & 0x80)) ++room_data_buf[0+index];
    }
    else{
      if((((room_data_buf[28] & 0xf0)>>4)+25) > (room_data_buf[0+index] & 0x80)) room_data_buf[0+index] |= 0x80;
    }
  }
}

void ts_temp_down_excution(unsigned char index, unsigned char ts_mode)
{
  if(ts_mode & 0x01){   //Floor mode
    if(room_data_buf[8+index] & 0x80){
      room_data_buf[8+index] &= 0x7f;
    }
    else{
      if(((room_data_buf[29] & 0x0f)+25) < (room_data_buf[8+index] & 0x80)){
        --room_data_buf[8+index];
        room_data_buf[8+index] |= 0x80;
      }
    }
  }
  else{                 //Air mode
    if(room_data_buf[0+index] & 0x80){
      room_data_buf[0+index] &= 0x7f;
    }
    else{
      if(((room_data_buf[28] & 0x0f)+25) < (room_data_buf[0+index] & 0x80)){
        --room_data_buf[0+index];
        room_data_buf[0+index] |= 0x80;
      }
    }  }
}

//
//================ Night Table receive data check routine ===============
//

void night_table_check(unsigned char index)
{
  if(f_first_nt_power_on[index]){
    if(room_data_buf[40] & 0x02){   //Guest IN?
      if((nt_switch_toggle_bit[index][0] & 0x01) != (uart2_rxd_buf[3] & 0x01)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][0],index);
      if((nt_switch_toggle_bit[index][0] & 0x02) != (uart2_rxd_buf[3] & 0x02)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][1],index);
      if((nt_switch_toggle_bit[index][0] & 0x04) != (uart2_rxd_buf[3] & 0x04)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][2],index);
      if((nt_switch_toggle_bit[index][0] & 0x08) != (uart2_rxd_buf[3] & 0x08)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][3],index);
      if((nt_switch_toggle_bit[index][0] & 0x10) != (uart2_rxd_buf[3] & 0x10)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][4],index);
      if((nt_switch_toggle_bit[index][0] & 0x20) != (uart2_rxd_buf[3] & 0x20)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][5],index);
      if((nt_switch_toggle_bit[index][0] & 0x40) != (uart2_rxd_buf[3] & 0x40)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][6],index);
      if((nt_switch_toggle_bit[index][0] & 0x80) != (uart2_rxd_buf[3] & 0x80)) nt_switch_bit_check(b_switch_kind_table[cb_mode][0][7],index);
      
      if((nt_switch_toggle_bit[index][1] & 0x01) != (uart2_rxd_buf[4] & 0x01)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][0],index);
      if((nt_switch_toggle_bit[index][1] & 0x02) != (uart2_rxd_buf[4] & 0x02)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][1],index);
      if((nt_switch_toggle_bit[index][1] & 0x04) != (uart2_rxd_buf[4] & 0x04)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][2],index);
      if((nt_switch_toggle_bit[index][1] & 0x08) != (uart2_rxd_buf[4] & 0x08)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][3],index);
      if((nt_switch_toggle_bit[index][1] & 0x10) != (uart2_rxd_buf[4] & 0x10)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][4],index);
      if((nt_switch_toggle_bit[index][1] & 0x20) != (uart2_rxd_buf[4] & 0x20)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][5],index);
      if((nt_switch_toggle_bit[index][1] & 0x40) != (uart2_rxd_buf[4] & 0x40)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][6],index);
      if((nt_switch_toggle_bit[index][1] & 0x80) != (uart2_rxd_buf[4] & 0x80)) nt_switch_bit_check(b_switch_kind_table[cb_mode][1][7],index);
      
      if((nt_switch_toggle_bit[index][2] & 0x01) != (uart2_rxd_buf[5] & 0x01)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][0],index);
      if((nt_switch_toggle_bit[index][2] & 0x02) != (uart2_rxd_buf[5] & 0x02)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][1],index);
      if((nt_switch_toggle_bit[index][2] & 0x04) != (uart2_rxd_buf[5] & 0x04)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][2],index);
      if((nt_switch_toggle_bit[index][2] & 0x08) != (uart2_rxd_buf[5] & 0x08)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][3],index);
      if((nt_switch_toggle_bit[index][2] & 0x10) != (uart2_rxd_buf[5] & 0x10)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][4],index);
      if((nt_switch_toggle_bit[index][2] & 0x20) != (uart2_rxd_buf[5] & 0x20)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][5],index);
      if((nt_switch_toggle_bit[index][2] & 0x40) != (uart2_rxd_buf[5] & 0x40)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][6],index);
      if((nt_switch_toggle_bit[index][2] & 0x80) != (uart2_rxd_buf[5] & 0x80)) nt_switch_bit_check(b_switch_kind_table[cb_mode][2][7],index);
    }
  }
  else f_first_nt_power_on[index] = 1;
  
  nt_switch_toggle_bit[index][0] = uart2_rxd_buf[3];
  nt_switch_toggle_bit[index][1] = uart2_rxd_buf[4];
  nt_switch_toggle_bit[index][2] = uart2_rxd_buf[5];
  
  nt_status[index] = uart2_rxd_buf[6];
  if(nt_status[index] & 0x10){
    if(!f_alram_on){
      f_alram_on = 1;
      digital_volume_execution(0,room_data_buf[80]);
    }
  }
  else f_alram_on = 0;
  
  if(index == 0){
    if(nt_status[0] & 0x80) room_data_buf[82] |= 0x08;
    else room_data_buf[82] &= 0xf7;
  }
  
//--- 반얀트리 호텔 나이트 테이블 온도센서 역할 처리 ---  
  
    if((uart2_rxd_buf[7] & 0x7f) <= 70){    //Air set temp
      if(room_data_buf[0+index] != uart2_rxd_buf[7]){
            ts_air_set_temp_from_PC[0+index] = uart2_rxd_buf[7];
            ts_set_temp_send_count[0+index] = 3;           
        room_data_buf[16+index] |= 0x80;
        room_data_buf[0+index] = uart2_rxd_buf[7];
        room_data_buf[99] |= 0x02;
      }
    }
    
    if(room_data_buf[40] & 0x02){   //Guest IN?
      if(nt_switch_toggle_bit[index][3] != uart2_rxd_buf[8]){    //TS switch push ?
        
        if((nt_switch_toggle_bit[index][3] & 0x01) != (uart2_rxd_buf[8] & 0x01)){
          
          room_data_buf[16+index] ^= 0x04;    //TS Air on/off toggle
          nt_switch_bit_check(K_fcu_auto_manual,index);
          room_data_buf[99] |= 0x02;
          room_data_buf[20+index] |= 0x08;    //Auto mode flag set
          room_data_buf[16+index] |= 0x80;
          
        }              
        
        //if((nt_switch_toggle_bit[0][3] & 0x10) != (uart2_rxd_buf[8] & 0x10)) room_data_buf[24] ^= 0x08;    //TS FaC, DoC
        if((nt_switch_toggle_bit[index][3] & 0x20) != (uart2_rxd_buf[8] & 0x20)) ts_speed_execution(index);          //TS local speed execution

      } 
    }  
  nt_switch_toggle_bit[index][3] = uart2_rxd_buf[8];
}

void nt_switch_bit_check(unsigned char key_code, unsigned char index)
{
   unsigned char i,j=0;
        
        
  switch(key_code)
  {
    case K_stand_master: 
          
        switch(cb_mode)
        {
        case 0: //stadard type
              
             if((room_data_buf[45] & 0x1e) || (room_data_buf[46] & 0x01))
             {
                room_data_buf[45] &= 0xe1; 
                room_data_buf[46] &= 0xfe;
             }
             else
             {
                room_data_buf[45] |= 0x1e;
                room_data_buf[46] |= 0x01;
             }
             
             break;
             
        case 1: // spa suit (4~9 floor type)
          
                switch(index)
                {
                case 0: //Night table
                case 1: //LIGHT NO.2
                  
                   if((room_data_buf[45] & 0x06) || (room_data_buf[46] & 0x01))
                   {
                      room_data_buf[45] &= 0xf9;
                      room_data_buf[46] &= 0xfe;
                   }
                   else
                   {
                      room_data_buf[45] |= 0x06;
                      room_data_buf[46] |= 0x01;
                   }                    
                  
                  break;
                  
                case 3: //LIGHT NO. 4
                  
                    if((room_data_buf[45] & 0xf0) || (room_data_buf[46] & 0x02))
                   {
                      room_data_buf[45] &= 0x0f; 
                      room_data_buf[46] &= 0xfd;
                   }
                   else
                   {
                      room_data_buf[45] |= 0xf0;
                      room_data_buf[46] |= 0x02;
                   }                        
                  
                  break;
                }
          
             break;
             
        case 2: // spa suite ( 10,11 floor )
                
                switch(index)
                {
                case 0: //Night table M
                case 2: //침실 M
                  
                    if((room_data_buf[45] & 0x3e) || (room_data_buf[46] & 0x01))
                    {
                        room_data_buf[45] &= 0xc1;
                        room_data_buf[46] &= 0xfe;
                    }
                    else
                    {
                        room_data_buf[45] |= 0x3e;
                        room_data_buf[46] |= 0x01;
                    }
                  
                  break;
                  
                case 4: //거실 M
                  
                    if((room_data_buf[45] & 0xc0) || (room_data_buf[47] & 0x03) || (room_data_buf[46] & 0x06))
                    {
                        room_data_buf[45] &= 0x3f;
                        room_data_buf[47] &= 0xfc;
                        room_data_buf[46] &= 0xf9;
                    }
                    else
                    {
                        room_data_buf[45] |= 0xc0;
                        room_data_buf[47] |= 0x03;
                        room_data_buf[46] |= 0x06;
                    }
                  
                  break;
                  
                  case 8:  //거실M
                    
                    if((room_data_buf[45] & 0x40) || (room_data_buf[47] & 0x03) || (room_data_buf[46] & 0x02))
                    {
                        room_data_buf[45] &= 0xbf;
                        room_data_buf[47] &= 0xfc;
                        room_data_buf[46] &= 0xfd;
                    }
                    else
                    {
                        room_data_buf[45] |= 0x40;
                        room_data_buf[47] |= 0x03;
                        room_data_buf[46] |= 0x02;
                    }                    
                    
                  break;
                }
              
          
              break;
              
        case 3: // p-suite ( 18층 좌 )
          
                switch(index)
                {
                  case 0: //LIGHT NO.1
                    
                      if((room_data_buf[45] & 0xfe) || (room_data_buf[47] & 0x01) || (room_data_buf[46] & 0x01))
                      {
                          room_data_buf[45] &= 0x01;
                          room_data_buf[46] &= 0xfe;
                          room_data_buf[47] &= 0xfe;
                      }
                      else
                      {
                          room_data_buf[45] |= 0xfe;
                          room_data_buf[46] |= 0x01;
                          room_data_buf[47] |= 0x01;    
                      }
                    
                    break;
                    
                  case 2: //LIGHT NO. 2
                  
                      if((room_data_buf[45] & 0xfe) || (room_data_buf[47] & 0x01) || (room_data_buf[46] & 0x01))
                      {
                          room_data_buf[45] &= 0x01;
                          room_data_buf[46] &= 0xfe;
                          room_data_buf[47] &= 0xfe;
                      }
                      else
                      {
                          room_data_buf[45] |= 0xfe;
                          room_data_buf[46] |= 0x01;
                          room_data_buf[47] |= 0x01;    
                      }                    
                    
                    break;
                    
                  case 3: //LIGHT NO. 3
                  
                      if((room_data_buf[47] & 0x0e) || (room_data_buf[45] & 0x60))
                      {
                        room_data_buf[47] &= 0xf1;
                        room_data_buf[45] &= 0x9f;
                      }
                      else 
                      {
                        room_data_buf[47] |= 0x0e;
                        room_data_buf[45] |= 0x60;
                      }
                  
                    break;
                }
          
              break;

        case 4: // p-suite ( 19층 좌 )
          
             switch(index)
             {
               case 1: //LIGHT NO. 1
                 
                 if((room_data_buf[45] & 0x0e) || (room_data_buf[46] & 0x01))
                 {
                    room_data_buf[45] &= 0xf1; 
                    room_data_buf[46] &= 0xfe;
                 }
                 else
                 {
                    room_data_buf[45] |= 0x0e;
                    room_data_buf[46] |= 0x01;
                 }
                 
                 break;
                 
             case 5: //LIGHT NO. 6
               
                      if(room_data_buf[45] & 0x70) room_data_buf[45] &= 0x8f;
                      else room_data_buf[45] |= 0x70;               
               
                break;
                
             case 7: //LIGHT NO. 8
               
                      if((room_data_buf[45] & 0x80) || (room_data_buf[47] & 0x01) || (room_data_buf[46] & 0x02))
                      {
                          room_data_buf[45] &= 0x7f;
                          room_data_buf[47] &= 0xfe;
                          room_data_buf[46] &= 0xfd;
                      }
                      else
                      {
                          room_data_buf[45] |= 0x80;
                          room_data_buf[47] |= 0x01;
                          room_data_buf[46] |= 0x02;
                      }
               
               break;
             }
             
             break;        
             
        case 5: //P_suite ( 18층 우 )
          
          switch(index)
          {
             case 0:
                
               if((room_data_buf[45] & 0x2e) || (room_data_buf[46] & 0x01))
               {
                  room_data_buf[45] &= 0xd1; 
                  room_data_buf[46] &= 0xfe;
               }
               else
               {
                  room_data_buf[45] |= 0x2e;
                  room_data_buf[46] |= 0x01;
               }     
                
             break;
             
             case 3:
                
               if(room_data_buf[47] & 0x1e)
               {
                  room_data_buf[47] &= 0xe1; 
                  //room_data_buf[45] &= 0x1e;
               }
               else
               {
                  room_data_buf[47] |= 0x1e;
                  //room_data_buf[45] |= 0xe1;
               }     
                
             break;
          }
          
            break;
          
         case 6: //19층 우 Type 
       
                switch(index)
                {
                case 3: //LIGHT NO. 1
                      
                      if((room_data_buf[45] & 0x0e)|| (room_data_buf[46] &= 0x01))
                      {
                          room_data_buf[45] &= 0xf1;
                          room_data_buf[46] &= 0xfe;
                      }
                      else
                      {
                        room_data_buf[45] |= 0x0e;
                        room_data_buf[46] |= 0x01;
                      }
                      
                  break;
                  
                 case 6: //LIGHT NO. 2
                   
                      if(room_data_buf[45] & 0x60)
                          room_data_buf[45] &= 0x9f;
                      else
                        room_data_buf[45] |= 0x60;
                      
                   break;
                   
                case 9: //LIGHT NO. 3
                  
                      if((room_data_buf[47] & 0x0e)|| (room_data_buf[46] &= 0x02))
                      {
                          room_data_buf[47] &= 0xf1;
                          room_data_buf[46] &= 0xfd;
                      }
                      else
                      {
                        room_data_buf[47] |= 0x0e;
                        room_data_buf[46] |= 0x02;
                      } 
                  
                    break;
                }    
         
         break;
         
        case 7: //신관(클럽동) A Type
          
            switch(index)
            {
            case 0:
                if(room_data_buf[45] & 0x0e) room_data_buf[45] &= 0xf1;
                else room_data_buf[45] |= 0x0e;
                break;
                
            case 2:
                if(room_data_buf[45] & 0x60) room_data_buf[45] &= 0x9f;
                else room_data_buf[45] |= 0x60;
                break;
                
            case 5:
            case 7:              
                if(room_data_buf[47] & 0x0f) room_data_buf[47] &= 0xf0;
                else room_data_buf[47] |= 0x0f;
                break;
                
              /*  if(room_data_buf[47] & 0x30) room_data_buf[47] &= 0xcf;
                else room_data_buf[47] |= 0x30;
                break;*/
                
             case 9:
                if((room_data_buf[47] & 0xc0) && (room_data_buf[48] & 0x01)) 
                {
                  room_data_buf[47] &= 0x3f;
                  room_data_buf[48] &= 0xfe;
                }
                else 
                {
                  room_data_buf[47] |= 0xc0;
                  room_data_buf[48] |= 0x01;
                }
               break;
            }
          
          break;

       case 8: //신관(클럽동) B Type
         
                switch(index)
                {
                case 1:
                      
                      if(room_data_buf[45] & 0x0e) room_data_buf[45] &= 0xf1;
                      else room_data_buf[45] |= 0x0e;                          
                  
                  break;
                  
                case 3:
                      
                      if((room_data_buf[45] & 0xc0) || (room_data_buf[47] & 0x07))
                      {
                          room_data_buf[45] &= 0x3f;
                          room_data_buf[47] &= 0xf8;
                      }
                      else
                      {
                          room_data_buf[45] |= 0xc0;
                          room_data_buf[47] |= 0x07;
                      }
                      
                  break;
                }
         
         break;       
         
        case 9: //신관(클럽동) C Type
          
                switch(index)
                {
                case 1:
                      
                      if(room_data_buf[45] & 0x06) room_data_buf[45] &= 0xf9;
                      else room_data_buf[45] |= 0x06;                          
                      
                  break;
                  
                case 3:
                  
                      if(room_data_buf[45] & 0x70) room_data_buf[45] &= 0x8f;
                      else room_data_buf[45] |= 0x70;                          
                  
                  break;
                  
                case 4: 
                  
                      if((room_data_buf[45] & 0x80) || (room_data_buf[47] & 0x03))
                      {
                          room_data_buf[45] &= 0x7f;
                          room_data_buf[47] &= 0xfc;
                      }
                      else
                      {
                          room_data_buf[45] |= 0x80;
                          room_data_buf[47] |= 0x03;
                      }
                  
                  break;
                }
          
         break;
         
        case 10: //신관(클럽동) D Type
          
          switch(index)
          {
          case 1:
            
              if(room_data_buf[45] & 0x0e) room_data_buf[45] &= 0xf1;
              else room_data_buf[45] |= 0x0e;
              
              break;
              
          case 2: 
            
              if(room_data_buf[45] & 0xf0) room_data_buf[45] &= 0x0f;
              else room_data_buf[45] |= 0xf0;               
            
              break;
          }
          
          break;
          
        case 11: //신관(클럽동) E Type
          
          switch(index)
          {
          case 1:
            
            if(room_data_buf[45] & 0x0e) room_data_buf[45] &= 0xf1;
            else room_data_buf[45] |= 0x0e;
            
            break;
            
          case 3: 
            
            if(room_data_buf[45] & 0x70) room_data_buf[45] &= 0x8f;
            else room_data_buf[45] |= 0x70;            
            
            break;
          }
          
          break;
          
        case 12: //신관(클럽동) F,F-1 Type
          
            switch(index)
            {
            case 1:
              
              if(room_data_buf[45] & 0x06) room_data_buf[45] &= 0xf9;
              else room_data_buf[45] |= 0x06;
              
              break;
              
            /*case 2:
              
              if(room_data_buf[45] & 0xf0) room_data_buf[45] &= 0x0f;
              else room_data_buf[45] |= 0xf0;
              
              break;*/              
              
            case 5:
              
              if(room_data_buf[47] & 0x38) room_data_buf[47] &= 0xc7;
              else room_data_buf[47] |= 0x38;
              
              break;
              
            case 2:  
            case 6:
              
              if((room_data_buf[47] & 0xc0) || (room_data_buf[48] & 0x03))
              {
                  room_data_buf[47] &= 0x3f;
                  room_data_buf[48] &= 0xfc;
              }
              else
              {
                  room_data_buf[47] |= 0xc0;
                  room_data_buf[48] |= 0x03;
              }
              
              break;
            }
          
          break;
          
        case 13: //신관(클럽동) G Type
          
            switch(index)
            {
            case 1:
              
              if(room_data_buf[45] & 0x0e) room_data_buf[45] &= 0xf1;
              else room_data_buf[45] |= 0x0e;              
              
              break;
              
            case 3:
              
              if(room_data_buf[45] & 0x70) room_data_buf[45] &= 0x8f;
              else room_data_buf[45] |= 0x70;                     
              
              break;
              
            case 4:
              
              if((room_data_buf[45] & 0x80) || (room_data_buf[47] & 0x07))
              {
                  room_data_buf[45] &= 0x7f;
                  room_data_buf[47] &= 0xf8;
              }
              else
              {
                  room_data_buf[45] |= 0x80;
                  room_data_buf[47] |= 0x07;
              }              
              
              break;
            }
          
          break;
          
        case 14: //신관(클럽동) I Type
          
            switch(index)
            {
            case 2:
              
                if(room_data_buf[45] & 0x1a) room_data_buf[45] &= 0xe5;
                else room_data_buf[45] |= 0x1a;                
                
              break;
              
            case 4:
              
                if(room_data_buf[45] & 0xe0) room_data_buf[45] &= 0x1f;
                else room_data_buf[45] |= 0xe0;
              
              break;
              
            case 5:
              
                if(room_data_buf[47] & 0x0f) room_data_buf[47] &= 0xf0;
                else room_data_buf[47] |= 0x0f;
                
              break;
            }
          
          break;
          
        case 15: //신관(클럽동) J Type
          
            switch(index)
            {
            case 2: 
              
                if(room_data_buf[45] & 0x1e) room_data_buf[45] &= 0xe1;
                else room_data_buf[45] |= 0x1e;                   
              
              break;
            
            case 4:
            
                if(room_data_buf[45] & 0xe0) room_data_buf[45] &= 0x1f;
                else room_data_buf[45] |= 0xe0;                
            
              break;
            
          case 6:
              
                if(room_data_buf[47] & 0x07) room_data_buf[47] &= 0xf8;
                else room_data_buf[47] |= 0x07;
            
              break;
            }
          
          break;
          
        }
           
      break;
      
    case K_rest_master:
      
      break;
      
      
      case K_rest_master_2:
        
      
      break;
      
    case K_stand_1:   
              
       switch(cb_mode)
       {
          case 0: //standard type
                
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;
                      
                      break;  
              
              break;
              
          case 1:  //spa suite (4~9floor type)
         
                switch(index)
                {
                case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;
                  
                  break;
                  
                case 3: //LIGHT NO. 4
                      
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                      
                  
                  break;
                }
              
            
              break;
              
       case 2: //spa suite ( 10 floor type )
         
                switch(index)
                {
                case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;
                  
                  break;
                  
                case 1: //LIGHT NO. 2
                  
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                      
                  
                  break;
                  
                case 4: //LIGHT NO. 5
                case 8: 
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
                    
                  break;
                }          
         
         break;
         
       case 3: //P - suite ( 16층 좌 )
                
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                    
                      
                  break;
                  
                case 2: // LIGHT NO. 3
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                          
                  
                  break;
                  
                case 3: // LIGHT NO. 4
                      
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;
                  
                  break;
                }
         
         break;
         
       case 4: //P_suite ( 17층 좌 )
         
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                    
                      
                  break;
                  
                case 4: // LIGHT NO. 4
                  
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                        
                  
                  break;
                  
                case 6: // LIGHT NO. 7
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                             
                  
                  break;
                }                
         
         break;
         
       case 5: //P_suite ( 18층 우 )
         
                switch(index)
                {
                case 0: //LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                    
                      
                  break;
                  
                 case 2: //LIGHT NO. 2
                   
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                         
                      
                   break;
                   
                case 3: //LIGHT NO. 3
                  
                       if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                       else room_data_buf[47] |= 0x02;                     
                  
                    break;
                }              
         
         break;
       
       case 6: //19층 우 Type 
       
                switch(index)
                {
                case 0: //LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                    
                      
                  break;
                  
                 case 5: //LIGHT NO. 2
                   
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                         
                      
                   break;
                   
                case 7: //LIGHT NO. 3
                  
                       if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                       else room_data_buf[47] |= 0x02;
                       
                    break;
                }    
         
         break;
         
       case 7: //신관 (클럽동) A Type
                
                switch(index)
                {
                case 0:  
                  
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                       
                  
                  break;
                  
                case 1: 
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;
                  
                  break;
                  
                case 3: 
                case 6: 
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;
                  
                  break;
                  
                case 4:
                  
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                      
                  
                  break;
                 
                  
                  /*    if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
                      else room_data_buf[47] |= 0x10;                         
                  
                  break;*/
                  
                case 8:
                  
                      if(room_data_buf[47] & 0x40) room_data_buf[47] &= 0xbf;
                      else room_data_buf[47] |= 0x40;                          
                  
                  break;
                  
                case 10:
                      
                      if(room_data_buf[48] & 0x02) room_data_buf[48] &= 0xfd;
                      else room_data_buf[48] |= 0x02;                       
                      
                  break;
                }
         
         break;
         
       case 8: //신관(클럽동) B Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                          
                  
                  break;
                  
                case 2:
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                        
                  
                  break;
                  
                case 3:
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                        
                  
                  break;               
                  
                case 4:
                      
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                            
                      
                  break;
                }
         
         break;
         
       case 9: //신관(클럽동) C Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                          
                  
                  break;
                  
                case 2:
                  
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                             
                  
                  break;
                  
                case 4:
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;
                  
                  break;
                }
         
         break;
         
       case 10: //신관(클럽동) D Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                          
                        
                  break;
                  
                case 2: 
                      
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                         
                      
                  break;
                  
                case 3: 
                      
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                         
                      
                  break;                  
                }
         
         break;
         
       case 11: //신관(클럽동) E Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                          
                  
                  break;
                  
                case 2: 
                  
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                         
                  
                  break;
                  
                case 4: 
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;
                  
                  break;
                }
         
         break;
         
       case 12: //신관(클럽동) F,F-1 Type
         
                switch(index)
                {
                case 0:

                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;
                  
                  break;
                  
                /*case 2:
                  
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                          
                  
                  break;*/
                  
                case 3:
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                        
                  
                  break;
                  
                case 4: 
                  
                      if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                      else room_data_buf[47] |= 0x08;                         
                  
                  break;
                  
                case 2:  
                case 6: 
                  
                      if(room_data_buf[47] & 0x40) room_data_buf[47] &= 0xbf;
                      else room_data_buf[47] |= 0x40;                             
                  
                  break;
                }
         
         break;
         
        case 13: //신관(클럽동) G Type
          
            switch(index)
            {
            case 0:
              
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;              
              
              break;
              
            case 2:
              
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                      
              
              break;
              
            case 4:
              
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
              
              break;
            }
          
          break;         
          
       case 14: //신관(클럽동) I Type
         
          switch(index)
          {
          case 0:
            
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                
            
            break;
            
          case 1: 
            
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                        
            
            break;
            
          case 3:
            
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                      
            
            break;
            
          case 5:
            
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                          
            
            break;
            
          case 6:
                      
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;
            
            break;
          }
         
          break;
          
       case 15: //신관(클럽동) J Type
         
          switch(index)
          {
          case 0:
            
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                
            
            break;
            
          case 1:
            
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                            
                
            break;
            
          case 3:
            
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                          
            
            break;
            
        case 5:
            
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                         
            
            break;
          }
          
          break;
         
       }
              
      break;
      
    case K_stand_2:  
      
        switch(cb_mode)
        {
          case 0:
                  
                        if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                        else room_data_buf[45] |= 0x04;

                break;    
                
          case 1:  //spa suite (4~9floor type)
         
                switch(index)
                {
                case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;
                  
                  break;
                  
                case 3: //LIGHT NO. 4
                      
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                      
                  
                  break;     
                }
            
            
              break;       
              
       case 2: //spa suite ( 10 floor type )
         
                switch(index)
                {
                case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;
                  
                  break;
                  
                case 1: //LIGHT NO. 2
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                      
                  
                  break;
                  
                case 4: //LIGHT NO. 5
                case 8:                  
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
                    
                  break;                                    
                }          
         
         break;         
         
       case 3: //P - suite ( 16층 좌 )
                
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                 
                      
                  break;            
                  
                case 2: //LIGHT NO. 3
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
                  
                  break;
                  
                case 3: //LIGHT NO. 4
                  
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                        
                  
                  break;
                }
                
         break;         
         
       case 4: //P - suite ( 17층 좌 )
         
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                 
                      
                  break;           
                  

                case 4: //LIGHT NO. 4
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                       
                  
                  break;
                  
                case 6: // LIGHT NO. 7
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                             
                  
                  break;                  
                }
         
         break;        
         
        case 5: //P_suite ( 16층 우 )
          
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                 
                      
                  break;       
                  
                case 2: // LIGHT NO. 2 
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                           
                  
                  break;
                  
                case 3: // LIGHT NO. 3
                  
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                         
                  
                  break;
                }                    
                
          break;
          
          case 6: //19층 우 Type 
       
                switch(index)
                {
                case 0: //LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                    
                      
                  break;
                  
                 case 5: //LIGHT NO. 2
                   
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
                      
                   break;
                   
                case 7: //LIGHT NO. 3
                  
                       if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                       else room_data_buf[47] |= 0x04;
                  
                    break;
                }    
         
         break;
         
        case 7: //신관 (클럽동) A Type
          
                  switch(index)
                  {
                  case 0:
                    
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;     
                      
                      break;
                      
                  case 1: 
                    
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;  
                    
                    break;      
                    
                case 3: 
                case 6:                  
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;
                  
                  break;           
                  
                case 4:
                  
                      if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                      else room_data_buf[47] |= 0x08;                      
                  
                  break;         
                  

                    
                     /* if(room_data_buf[47] & 0x20) room_data_buf[47] &= 0xdf;
                      else room_data_buf[47] |= 0x20;                           
                      
                  break;*/
                  
                case 8:
                      
                      if(room_data_buf[47] & 0x80) room_data_buf[47] &= 0x7f;
                      else room_data_buf[47] |= 0x80;                        
                      
                  break;      
                  
                case 10:
                  
                      if(room_data_buf[48] & 0x04) room_data_buf[48] &= 0xfb;
                      else room_data_buf[48] |= 0x04;                       
                  
                  break;                   
                  }
          
          break;
          
       case 8: //신관(클럽동) B Type
                
                switch(index)
                {
                case 0:
                      
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                          
                      
                  break;
                  
                case 2:
                      
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                        
                      
                  break;         
                  
                case 3:
                      
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
                      
                  break;                   
                }
         
         break;     
         
       case 9: //신관(클럽동) C Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                         
                  
                  break;
                  
                case 2:
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                              
                  
                  break;   
                  
                case 4:
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;
                  
                  break;                  
                }
         
         break;    
         
       case 10: //신관(클럽동) D Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                          
                        
                  break;
                  
                case 2: 
                      
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                         
                      
                  break;    
                  
                case 3: 
                      
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                         
                      
                  break;                         
                }
         
         break;   
         
       case 11: //신관(클럽동) E Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                     
                  
                  break;
                  
                case 2: 
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                        
                      
                  break;     
                }
         
         break;    
         
       case 12: //신관(클럽동) F,F-1 Type
         
                switch(index)
                {
                case 0:

                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;       
                  
                  break;
                  
                /*case 2: 
                  
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                        
                      
                  break;*/                     
                  
                case 3:
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                               
                  
                  break;    
                  
                case 4: 
                  
                      if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
                      else room_data_buf[47] |= 0x10;                         
                  
                  break;     
                  
                case 2:  
                case 6: 
                  
                      if(room_data_buf[47] & 0x80) room_data_buf[47] &= 0x7f;
                      else room_data_buf[47] |= 0x80;                             
                  
                  break;                  
                }
         
         break;  
         
        case 13: //신관(클럽동) G Type
          
            switch(index)
            {
            case 0:
              
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;          
              
              break;
              
            case 2:
              
                      if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
                      else room_data_buf[45] |= 0x20;                    
              
              break;    
              
            case 4:
              
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                           
              
              break;              
            }
          
          break;   
          
       case 14: //신관(클럽동) I Type
          
          switch(index)
          {
          case 0:
            
                      if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                      else room_data_buf[45] |= 0x02;                    
            
            break;
            
          case 1: 
            
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                        
            
            break;           
            
          case 3:
            
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                      
            
            break;  
            
          case 5:
            
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                        
            
            break;            
          }
          
          break;          
          
       case 15: //신관(클럽동) J Type
         
          switch(index)
          {
          case 0:
            
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                
            
            break;
            
          case 1:
            
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                            
                      
            break;     
            
          case 3:
            
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
            
            break;   
            
        case 5:
            
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                             
            
            break;            
          }
          
          break;          
        }
                
      break;
      
    case K_stand_3: 
                
      switch(cb_mode)
      { 
          case 0: //standard type
                        
                        if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                        else room_data_buf[45] |= 0x08;
                        
                        break;        
                        
          case 1:  //spa suite (4~9floor type)
                
                switch(index)
                {
                /*case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;
                  
                  break;*/
                  
                case 3: //LIGHT NO. 4
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                      
                      
                  break;                             
                }
            
            
              break;       
              
       case 2: //spa suite ( 10,11 floor type )
                
                switch(index)
                {
                case 0: //LIGHT NO.1

                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;
                  
                  break;
                  
                case 4: //LIGHT NO. 5
                case 8:                  
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                         
                    
                  break;                      
                }          
         
         break;       
         
       case 3: //P - suite ( 16층 좌 )
         
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                
                      
                  break;
                  
                case 2: //LIGHT NO. 3
                      
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
                  
                  break;

                case 3: // LIGHT NO. 4
                      
                      if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                      else room_data_buf[47] |= 0x08;
                  
                  break;                  
                }
         
         break;               
         
       case 4: //P - suite ( 17층 좌 )
                
                switch(index)
                {
                case 0: // LIGHT NO. 1
                case 3: // LIGHT NO. 4
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                
                      
                  break;         
                  
                case 4: //LIHGT NO. 5
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                           
                  
                  break;
                  
                case 6: // LIGHT NO. 7
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                             
                  
                  break;                     
                }
         
         break;     
         
      case 5: //P_suite ( 16층 우 )
        
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                
                      
                  break;           

                case 2: //LIGHT NO. 2
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                       
                      
                  break;
                  
                case 3: //LIGHT NO. 3
                  
                      if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                      else room_data_buf[47] |= 0x08;                          
                  
                  break;
                }          
        
         break;
         
         case 6: //19층 우 Type 
       
                switch(index)
                {
                case 1: //LIGHT NO. 1
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                    
                      
                  break;
                  
                 case 5: //LIGHT NO. 2
                   
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
                      
                   break;
                   
                case 7: //LIGHT NO. 3
                  
                       if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                       else room_data_buf[47] |= 0x08;
                  
                    break;
                }    
         
         break;
         
      case 7: //신관(클럽동) A Type
        
                  switch(index)
                  {
                  case 0:
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;  
                      
                      break;
                      
                  case 1:
                      
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                        
                      
                    break;
                    
                  case 8:
                  
                      if(room_data_buf[48] & 0x01) room_data_buf[48] &= 0xfe;
                      else room_data_buf[48] |= 0x01;                       
                  
                  break;                                     
                  }
                  
        break;
        
       case 8: //신관(클럽동) B Type
         
                switch(index)
                {
                case 0:
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                          
                  
                  break;
                  
                case 3:
                  
                      if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                      else room_data_buf[47] |= 0x01;                         
                  
                  break;
                }
         
         break;                
        
       case 9: //신관(클럽동) C Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;                            
                  
                  break;
                  
                case 2:
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                              
                  
                  break;     
                  
                case 4:
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;
                  
                  break;                         
                }
         
         break;       
         
       case 10: //신관(클럽동) D Type
         
                switch(index)
                {
                case 0:
                  
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                          
                        
                  break;
                  
                case 2: 
                      
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
                      
                  break;                         
                }
         
         break;      

       case 11: //신관(클럽동) E Type
         
                switch(index)
                {
                case 0:
                      
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;                         
                      
                  break;
                  
                case 2: 
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                         
                      
                  break;                      
                }
         
         break;      
         
       case 12: //신관(클럽동) F,F-1 Type
         
                switch(index)
                {
                case 0:

                      if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                      else room_data_buf[45] |= 0x04;     
                  
                  break;
                  
              /*  case 2:
                  
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                             
                  
                  break;*/   

                case 3:
                  
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                               
                  
                  break;    
                  
                case 4: 
                  
                      if(room_data_buf[47] & 0x20) room_data_buf[47] &= 0xdf;
                      else room_data_buf[47] |= 0x20;                         
                  
                  break;          
                  
                case 2:
                case 6: 
                  
                      if(room_data_buf[48] & 0x01) room_data_buf[48] &= 0xfe;
                      else room_data_buf[48] |= 0x01;                             
                  
                  break;                      
                }
         
         break;         
         
        case 13: //신관(클럽동) G Type
          
            switch(index)
            {
            case 0:
              
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;          
              
              break;
              
            case 2:
              
                      if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
                      else room_data_buf[45] |= 0x40;                  
              
              break;              
              
            case 4:
              
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                             
              
              break;                       
            }
          
          break;       
          
      case 14: //신관(클럽동) I Type
        
        switch(index)
        {
          case 3:
            
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                      
            
            break;       
            
          case 5:
            
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                          
            
            break;                   
        }
            
            break;        
        
      case 15: //신관(클럽동) J Type
        
        switch(index)
        {
          case 3:
            
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                            
            
            break;          
            
        case 5:
            
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                             
            
            break;                        
        }
        
          break;
      }
      
       break;          
      
    case K_stand_4:   
      
      switch(cb_mode)
      {
      case 0: //standard type
              
              if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
              else room_data_buf[45] |= 0x10;    
              
        break;
              
      case 1: //spa suite(4~9floor type)
              
              if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
              else room_data_buf[45] |= 0x80;          
        
        break;
        
      case 2: //spa suite ( 10 floor )
                
               if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
               else room_data_buf[47] |= 0x02;                                       
        
        break;
        
       case 3: //P - suite ( 16층 좌 )
         
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                    if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                    else room_data_buf[45] |= 0x10;                
                    
                  break;
                  
                case 2: // LIGHT NO. 3
                    
                    if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                    else room_data_buf[47] |= 0x01;
                  
                  break;
                  
                case 3: // LIGHT NO. 4
                  
                      if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
                      else room_data_buf[47] |= 0x10;
                  
                  break;                      
                }
         
         break;  
         
      case 5: //P_suite ( 16층 우 )
        
                switch(index)
                {
                case 0: // LIGHT NO. 1
                      
                    if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                    else room_data_buf[45] |= 0x10;                
                    
                  break;     
                  
                case 3: //LIGHT NO. 2
                  
                    if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
                    else room_data_buf[47] |= 0x10;                    
                  
                  break;
                }          
        
         break;
         
          case 6: //19층 우 Type 
       
                switch(index)
                {
                case 1: //LIGHT NO. 1
                case 2:
                      
                      if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
                      else room_data_buf[45] |= 0x10;                    
                      
                  break;
                  
                case 8: //LIGHT NO. 3
                  
                       if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
                       else room_data_buf[47] |= 0x10;
                       
                    break;
                }    
         
         break;
         
        case 7: //신관(클럽동) A Type
        
                      if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                      else room_data_buf[45] |= 0x08;           
        
        break;
        
       case 8: //신관(클럽동) B Type
         
                switch(index)
                {
                case 3:
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;                         
                  
                  break;
                }
         
         break;            
         
      case 9: //신관(클럽동) C Type
        
                switch(index)
                {
                case 4:
                  
                      if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
                      else room_data_buf[47] |= 0x02;
                  
                  break;  
                }
        
        break;
        
      case 10: //신관(클럽동) D Type
        
                switch(index)
                {
                case 2: 
                      
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                         
                      
                  break; 
                }
        
        break;
        
       case 12: //신관(클럽동) F,F-1 Type
         
                switch(index)
                {
                /*case 2:
                  
                      if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                      else room_data_buf[45] |= 0x80;                             
                  
                  break;*/  
                  
                case 2:
                case 6: 
                  
                      if(room_data_buf[48] & 0x02) room_data_buf[48] &= 0xfd;
                      else room_data_buf[48] |= 0x02;                             
                      
                  break;                     
                }
         
         break;     
         
        case 13: //신관 클럽동 G Type
          
          switch(index)
          {
            case 4:
              
                      if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
                      else room_data_buf[47] |= 0x04;                             
                      
              break;    
          }
          
        break;
        
      case 14: //신관 클럽동(I Type)
        
          switch(index)
          {
          case 5:
            
                      if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
                      else room_data_buf[47] |= 0x08;                          
                      
            break;             
          }
        
        break;
      }
              
    break;
      
    case K_stand_5:   
              
              if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
              else                         room_data_buf[45] |= 0x20; 
              
      break;
      
    case K_stand_6:   
              
              if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
              else                         room_data_buf[46] |= 0x01; 
              
      break;
      
    case K_stand_7:   

              if(room_data_buf[46] & 0x02) room_data_buf[46] &= 0xfd;
              else                         room_data_buf[46] |= 0x02; 
              
      break;
      
    case K_stand_8:
              
              if(room_data_buf[46] & 0x04) room_data_buf[46] &= 0xfb;
              else                         room_data_buf[46] |= 0x04;
              
      break;
      
    case K_stand_9:
              
              if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
              else                         room_data_buf[47] |= 0x02;
              
              scene_mode[1] = 0;
              
      break;
      
    case K_stand_10:

              if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
              else                         room_data_buf[47] |= 0x04;      
              
              scene_mode[1] = 0;
              
      break;
      
    case K_stand_11:
    
              if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
              else                         room_data_buf[47] |= 0x08;       
          
              scene_mode[1] = 0;
              
      break;      
      
  case K_stand_12 :
    
              if(room_data_buf[47] & 0x10) room_data_buf[47] &= 0xef;
              else                         room_data_buf[47] |= 0x10;        
    
              scene_mode[1] = 0;
              
    break;
 
  case K_stand_13 :
       
              if(room_data_buf[47] & 0x20) room_data_buf[47] &= 0xdf;
              else                         room_data_buf[47] |= 0x20;               
    
    break;         
    
    case K_bed_master:  

        switch(cb_mode)
        {
        case 4: // p-suite ( 19 - 좌 )
          
            switch(index)
            {
            case 0:
              
                 if((room_data_buf[45] & 0x0e) || (room_data_buf[46] & 0x01))
                 {
                    room_data_buf[45] &= 0xf1; 
                    room_data_buf[46] &= 0xfe;
                 }
                 else
                 {
                    room_data_buf[45] |= 0x0e;
                    room_data_buf[46] |= 0x01;
                 }              
              
              break;
              
            case 1:
              
                      if((room_data_buf[45] & 0x80) || (room_data_buf[47] & 0x01) || (room_data_buf[46] & 0x02))
                      {
                          room_data_buf[45] &= 0x7f;
                          room_data_buf[47] &= 0xfe;
                          room_data_buf[46] &= 0xfd;
                      }
                      else
                      {
                          room_data_buf[45] |= 0x80;
                          room_data_buf[47] |= 0x01;
                          room_data_buf[46] |= 0x02;
                      }              
              
              break;
            }
          
          break;
          
        case 6: // p-suite ( 19 - 우 )
          
          switch(index)
          {
          case 0:
            
            if((room_data_buf[45] & 0x0e) || (room_data_buf[46] & 0x01))
            {
                room_data_buf[45] &= 0xf1;
                room_data_buf[46] &= 0xfe;
            }
            else
            {
                room_data_buf[45] |= 0x0e;
                room_data_buf[46] |= 0x01;
            }
            
            break;
            
          case 1:
             
            if((room_data_buf[47] & 0x0e) || (room_data_buf[46] & 0x02))
            {
                room_data_buf[47] &= 0xf1;
                room_data_buf[46] &= 0xfd;
            }
            else
            {
                room_data_buf[47] |= 0x0e;
                room_data_buf[46] |= 0x02;
            }
            
            break;
          }
          break;
          
        case 7: //클럽동(신관) A Type
          
          if(room_data_buf[47] & 0x0f) room_data_buf[47] &= 0xf0;
          else room_data_buf[47] |= 0x0f;
          
          break;
          
        case 8: //클럽동(신관) B Type
          
          if(room_data_buf[45] & 0x3e) room_data_buf[45] &= 0xc1;
          else room_data_buf[45] |= 0x3e;
          
          break;
          
        case 9: //클럽동(신관) C Type
          
          if(room_data_buf[45] & 0x06) room_data_buf[45] &= 0xf9;
          else room_data_buf[45] |= 0x06;
          
          break;
          
        case 10: //클럽동(신관) D Type
          
            if((room_data_buf[45] & 0x0e) || (room_data_buf[47] & 0x03))
            {
                room_data_buf[45] &= 0xf1;
                room_data_buf[47] &= 0xfc;
            }
            else
            {
                room_data_buf[45] |= 0x0e;
                room_data_buf[47] |= 0x03;
            }          
          
          break;
          
        case 11: //클럽동(신관) E Type
        case 13: //클럽동(신관) G,H Type
          
          if(room_data_buf[45] & 0x7e) room_data_buf[45] &= 0x81;
          else room_data_buf[45] |= 0x7e;
          
          break;
          
        case 12: //클럽동(신관) F,F-1 Type
          
          if((room_data_buf[45] & 0x06) || (room_data_buf[47] & 0x07))
          {
            room_data_buf[45] &= 0xf9;
            room_data_buf[47] &= 0xf8;
          }
          else
          {
            room_data_buf[45] |= 0x06;
            room_data_buf[47] |= 0x07;
          }
          
          break;
          
        case 15: //클럽동(신관) J Type          
          
          if(room_data_buf[45] & 0x1e) room_data_buf[45] &= 0xe1;
          else room_data_buf[45] |= 0x1e;
          
          break;
          
        case 14: //클럽동(신관) I Type
         
          if(room_data_buf[45] & 0x1a) room_data_buf[45] &= 0xe5;
          else room_data_buf[45] |= 0x1a;           
          
          break;
        }
      
      break;
      
    case K_wait:
              
              room_data_buf[40] ^= 0x80; //wait on,off
              
      break;
      
  case K_night_stand_1:
        
        switch(cb_mode)
        {
          case 1: //spa suite(4~9 floor type)
              
              if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
              else room_data_buf[46] |= 0x01;
              
            break;
            
          case 2: //spa suite ( 10,11 floor type )
            
              if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
              else room_data_buf[45] |= 0x10;
            
            break;
            
        case 4: //p-suite ( 19 - 좌)
         
              switch(index)
              {
              case 0:
                
                  if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
                  else room_data_buf[45] |= 0x02;
                
                break;
                
              case 1:
                
                  if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                  else room_data_buf[45] |= 0x80;
                  
                break;
              }
          
            break;
            
        case 6: //p-suite ( 19층 우 )
          
            switch(index)
            {
            case 0:
              
              if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
              else      room_data_buf[45]  |= 0x02;
              
              break;
              
            case 1:
              
              if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
              else      room_data_buf[47] |= 0x02;
              
              break;
            }
            
            break;
            
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
              else room_data_buf[47] |= 0x01;
          
            break;
            
        case 8: //클럽동(신관) B Type
        case 9: //클럽동(신관) C Type
        case 10: //클럽동(신관) D Type
        case 11: //클럽동(신관) E Type
        case 12: //클럽동(신관) F,F-1 Type
        case 13: //클럽동(신관) G Type
        case 14: //클럽동(신관) I Type
        case 15: //클럽동(신관) J Type                    
          
          if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
          else room_data_buf[45] |= 0x02;
          
          break;            
        }
      
      break;
      
  case K_night_stand_2:
    
        switch(cb_mode)
        {
          case 2: //spa suite(10,11 floor type)
            
              if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
              else room_data_buf[45] |= 0x20;              
            
            break;
            
        case 4: //p-suite ( 19 - 좌 )
         
              switch(index)
              {
              case 0:
                
                  if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
                  else room_data_buf[45] |= 0x04;
                
                break;
                
              case 1:
                
                  if(room_data_buf[45] & 0x80) room_data_buf[45] &= 0x7f;
                  else room_data_buf[45] |= 0x80;
                  
                break;
              }
          
            break;
      
        case 6: //p-suite ( 19 - 우 )
          
            switch(index)
            {
            case 0:
              
              if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
              else      room_data_buf[45]  |= 0x04;
              
              break;
              
            case 1:
              
              if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
              else      room_data_buf[47] |= 0x04;
              
              break;
            }
            
            break;
            
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
              else room_data_buf[47] |= 0x02;
          
            break;  
            
        case 8: //클럽동(신관) B Type
        case 10: //클럽동(신관) D Type
        case 11: //클럽동(신관) E Type          
        case 12: //클럽동(신관) F,F-1 Type    
        case 13: //클럽동(신관) G Type               
        case 15: //클럽동(신관) J Type                    
          
          if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
          else room_data_buf[45] |= 0x04;
          
          break;           
          
        case 9: //클럽동(신관) C Type
        case 14: //클럽동(신관) I Type          
          
          if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
          else room_data_buf[45] |= 0x02;
          
          break;         
        }
   
      break;
      
  case K_night_stand_3:
    
        switch(cb_mode)
        {
          case 2: //spa suite(10,11 floor type)
              
              if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
              else room_data_buf[46] |= 0x01;              
            
            break;
            
        case 4: //p-suite ( 19 - 좌)
         
              switch(index)
              {
              case 0:
                
                  if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                  else room_data_buf[45] |= 0x08;
                
                break;
                
              case 1:
                
                  if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                  else room_data_buf[47] |= 0x01;
                  
                break;
              }
          
            break;   
        
        case 6: //p-suite ( 19 - 우 )
          
            switch(index)
            {
            case 0:
              
              if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
              else      room_data_buf[45]  |= 0x08;
              
              break;
              
            case 1:
              
              if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
              else      room_data_buf[47] |= 0x08;
              
              break;
            }
            
            break;      
            
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
              else room_data_buf[47] |= 0x04;
          
            break;   
            
        case 8: //클럽동(신관) B Type
        case 10: //클럽동(신관) D Type
        case 11: //클럽동(신관) E Type   
        case 13: //클럽동(신관) G Type        
        case 14: //클럽동(신관) I Type
        case 15: //클럽동(신관) J Type                    
          
          if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
          else room_data_buf[45] |= 0x08;
          
          break;                  
          
        case 9: //신관(클럽동) C Type
        case 12: //클럽동(신관) F,F-1 Type  
          
          if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
          else room_data_buf[45] |= 0x04;          
          
          break;    
          
          
        }      
   
      break;      
      
  case K_night_stand_4:
    
      switch(cb_mode)
      {
      case 4: //p-suite ( 19 - 좌 )
        
          switch(index)
          {
              case 0:
                
                  if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
                  else room_data_buf[45] |= 0x08;
                
                break;
                
              case 1:
                
                  if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
                  else room_data_buf[47] |= 0x01;
                  
                break;
        
       case 6: //p-suite ( 19 - 우 )
          
            switch(index)
            {
            case 0:
              
              if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
              else      room_data_buf[45]  |= 0x08;
              
              break;
              
            case 1:
              
              if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
              else      room_data_buf[47] |= 0x08;
              
              break;
            }
            
            break;         
          }
        
        break;     
                
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
              else room_data_buf[47] |= 0x08;
          
            break;  
            
        case 8: //클럽동(신관) B Type
        case 11: //클럽동(신관) E Type
        case 13: //클럽동(신관) G Type          
          
          if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
          else room_data_buf[45] |= 0x10;
          
          break;
          
        case 9: //신관(클럽동) C Type

          if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
          else room_data_buf[45] |= 0x04;          
          
          break;
          
      case 10: //신관(클럽동) D Type
      case 12: //클럽동(신관) F,F-1 Type     
        
          if(room_data_buf[47] & 0x01) room_data_buf[47] &= 0xfe;
          else room_data_buf[47] |= 0x01;            
          
          break;
          
        case 14: //클럽동(신관) I Type
        case 15: //클럽동(신관) J Type                    

          if(room_data_buf[45] & 0x08) room_data_buf[45] &= 0xf7;
          else room_data_buf[45] |= 0x08;          
          
          break;
      }
    
    break;
    
  case K_night_stand_5:
    
      switch(cb_mode)
      {
      case 4: //p-suite ( 19 - 좌 )
        
          switch(index)
          {
              case 0:
                
                  if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
                  else room_data_buf[46] |= 0x01;
                
                break;
                
              case 1:
                
                  if(room_data_buf[46] & 0x02) room_data_buf[46] &= 0xfd;
                  else room_data_buf[46] |= 0x02;
                  
                break;
          }
        
        break;
        
        case 6: //p-suite ( 19 - 우 )
          
            switch(index)
            {
            case 0:
              
              if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
              else      room_data_buf[46]  |= 0x01;
              
              break;
              
            case 1:
              
              if(room_data_buf[46] & 0x02) room_data_buf[46] &= 0xfd;
              else      room_data_buf[46] |= 0x02;
              
              break;            
          }
        
        break;    
        
              
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
              else room_data_buf[47] |= 0x04;
          
            break;  
            
        case 8: //클럽동(신관) B Type
        case 11: //클럽동(신관) E Type
        case 13: //클럽동(신관) G Type        
          
          if(room_data_buf[45] & 0x20) room_data_buf[45] &= 0xdf;
          else room_data_buf[45] |= 0x20;
          
          break;          
          
        case 9: //클럽동(신관) C Type
          
              if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
              else room_data_buf[45] |= 0x04;          
          
          break;
          
      case 10: //신관(클럽동) D Type
      case 12: //클럽동(신관) F,F-1 Type         
        
          if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
          else room_data_buf[47] |= 0x02;            
        
          break;          
          
      case 14: //클럽동(신관) I Type
      case 15: //클럽동(신관) J Type                  
          
          if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
          else room_data_buf[45] |= 0x10;
        
          break;
      }      
    
    break;
    
  case K_night_stand_6:
    
      switch(cb_mode)
      {
      case 4: //p-suite ( 19 - 좌 )
        
          switch(index)
          {
              case 0:
                
                  if(room_data_buf[46] & 0x01) room_data_buf[46] &= 0xfe;
                  else room_data_buf[46] |= 0x01;
                
                break;
                
              case 1:
                
                  if(room_data_buf[46] & 0x02) room_data_buf[46] &= 0xfd;
                  else room_data_buf[46] |= 0x02;
                  
                break;
          }
        
                 
        break;
        
        case 7: //신관(클럽동) A Type
          
              if(room_data_buf[47] & 0x08) room_data_buf[47] &= 0xf7;
              else room_data_buf[47] |= 0x08;            
          
            break;
        case 8: //클럽동(신관) B Type          
          
              if(room_data_buf[47] & 0x20) room_data_buf[47] &= 0xdf;
              else room_data_buf[47] |= 0x20;
          
            break;      
          
        case 9: //클럽동(신관) C Type
              
              if(room_data_buf[45] & 0x04) room_data_buf[45] &= 0xfb;
              else room_data_buf[45] |= 0x04;          
          
          break;     
          
      case 10: //신관(클럽동) D Type
        
          if(room_data_buf[47] & 0x02) room_data_buf[47] &= 0xfd;
          else room_data_buf[47] |= 0x02;            
        
          break;      
          
      case 11: //신관(클럽동) E Type
      case 13: //클럽동(신관) G Type        
        
              if(room_data_buf[45] & 0x40) room_data_buf[45] &= 0xbf;
              else room_data_buf[45] |= 0x40;          
        
          break;
          
      case 12: //클럽동(신관) F,F-1 Type     
        
          if(room_data_buf[47] & 0x04) room_data_buf[47] &= 0xfb;
          else room_data_buf[47] |= 0x04;   
        
          break;
          
      case 14: //클럽동(신관) I Type
      case 15: //클럽동(신관) J Type            
          
          if(room_data_buf[45] & 0x10) room_data_buf[45] &= 0xef;
          else room_data_buf[45] |= 0x10;
        
          break;          
      }    
    
    break;
      
    case K_exchanger:   //전열교환기
      //if(room_data_buf[46] & 0x04) room_data_buf[46] &= 0xfb;
      //else room_data_buf[46] |= 0x04;
      break;
    case K_entry_lamp:    //입구등
      //if(room_data_buf[45] & 0x02) room_data_buf[45] &= 0xfd;
      //else{
      //  room_data_buf[45] |= 0x02;
       // timer_1sec_entry = 0;
      //  enterance_lamp_delay_timer = 0;
      //  f_enterance_lamp_delay = 0;
      //}
      break;
      
    case K_dim_step_up : 
          if(dimmer_level[0] > 0x60) dimmer_level[0] = 0x60;
          else dimmer_level[0] += 20;
    break;
    
    case K_dim_step_down : 
          if((dimmer_level[0]) < 20) dimmer_level[0] = 0;
          else dimmer_level[0] -= 20;
    break;
    
    case K_scene_1:
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[0][i];
      }
      room_data_buf[45] &= 0x01;
      room_data_buf[45] |= 0x0a;       
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[0] = 1;
      break;
      
    case K_scene_2:
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[1][i];
      }
      room_data_buf[45] &= 0x01;
      room_data_buf[45] |= 0x38;       
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[0] = 2;
      break;
      
    case K_scene_3:
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[2][i];
      }
      room_data_buf[45] &= 0x01;
      room_data_buf[45] |= 0x80;

      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[0] = 3;
      break;
      
    case K_scene_4:
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[3][i];
      }
      room_data_buf[45] &= 0x01;
      room_data_buf[45] |= 0x4f;      
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[0] = 4;
      break;
      
    case K_scene_5:
      
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[0][i];
      }
      room_data_buf[47] &= 0x01;
      room_data_buf[47] |= 0x0e;      
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[1] = 1;      
      
      break;

    case K_scene_6:

      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[1][i];
      }
      room_data_buf[47] &= 0x01;
      room_data_buf[47] |= 0x10;      
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[1] = 2;            
      
      break;

    case K_scene_7:
      
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[2][i];
      }
      room_data_buf[47] &= 0x01;
      room_data_buf[47] |= 0x06;      
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[1] = 3;        
      
      break;

    case K_scene_8:
      
      for(i=0; i<16; ++i){
        dimmer_level[i] = b_scene_table[3][i];
      }
      room_data_buf[47] &= 0x01;
      room_data_buf[47] |= 0x0a;      
      dimmer_apply_bit |= 0x3f;
      dim_apply_count = 0;
      scene_mode[1] = 4;   
      
      break;      
      
    case K_radio_onoff:
      if(room_data_buf[36+index] & 0x08) room_data_buf[36+index] &= 0xf7;
      else room_data_buf[36+index] |= 0x08;
      room_data_buf[43] |= 0x80;
      break;
        
    case K_channel_up:
      if(room_data_buf[36+index] & 0x08){
        if((room_data_buf[36+index] & 0x07) == 0x07) room_data_buf[36+index] &= 0xf8;
        else ++room_data_buf[36+index];
      }
      break;
      
    case K_channel_down:
      if(room_data_buf[36+index] & 0x08){
        if((room_data_buf[36+index] & 0x07) == 0x00) room_data_buf[36+index] |= 0x07;
        else --room_data_buf[36+index];
      }
      break;
      
    case K_volume_up:
      if(room_data_buf[36+index] & 0x08){
        for(i=0; i<10; ++i){
          if(audio_volume[0] <= b_volume_step[i]){
            if(audio_volume[0] != 0xfe) audio_volume[0] = b_volume_step[i+1];
            break;
          }
        }
//        if(++audio_volume[0] >= 0xfe) audio_volume[0] = 0xfe;
//        if((audio_volume[0] + 50) >= 0xfe) audio_volume[0] = 0xfe;
//        else audio_volume[0] += 50;
      }
      break;
    case K_volume_down:
      if(room_data_buf[36+index] & 0x08){
        for(i=0; i<10; ++i){
          if(audio_volume[0] <= b_volume_step[i]){
            if(audio_volume[0] != 0x02) audio_volume[0] = b_volume_step[i-1];
            break;
          }
        }
//        if(audio_volume[0] > 2) --audio_volume[0];
//        if(audio_volume[0] > 52) audio_volume[0] -= 50;
//        else audio_volume[0] = 2;
      }
      break;
      
    case K_volume_mute :
      if(room_data_buf[36+index] & 0x40)
      {
        room_data_buf[36+index] &= 0xbf;
        room_data_buf[43] &= 0x7f;
      }
      else 
      {
        room_data_buf[36+index] |= 0x40;
        room_data_buf[43] |= 0x80;
      }
      break;    
   case K_fcu_onoff: 
      
      if(!ondol_set_flag[0])
      {
        room_data_buf[16] ^= 0x04;  room_data_buf[16] |= 0x80;  //room_data_buf[54+index] &= 0xf8;   
        if(room_data_buf[16] & 0x04) room_data_buf[20] |= 0x08; //auto
        else room_data_buf[20] &= 0xf7; //manual
      }
      else
      {
        room_data_buf[16] ^= 0x20; room_data_buf[20] |= 0x80;
        if(room_data_buf[16] & 0x20) room_data_buf[20] |= 0x08; //auto
        else room_data_buf[20] &= 0xf7; //manual        
      }
      
      break;
      
    case K_fcu_onoff_2: room_data_buf[17] ^= 0x04;  room_data_buf[17] |= 0x80;  //room_data_buf[54+index] &= 0xf8;   
      if(room_data_buf[17] & 0x04) room_data_buf[21] |= 0x08; //auto
      else room_data_buf[21] &= 0xf7; //manual
      break;
      
    case K_fcu_low:   
        room_data_buf[16+index] |= 0x04;  room_data_buf[16+index] |= 0x80;  room_data_buf[54+index] &= 0xf8;   room_data_buf[54+index] |= 0x01;  
        room_data_buf[20+index] &= 0xf7; //manual
      break;
      
    case K_fcu_mid:   
        room_data_buf[16+index] |= 0x04;  room_data_buf[16+index] |= 0x80;  room_data_buf[54+index] &= 0xf8;   room_data_buf[54+index] |= 0x02;  
        room_data_buf[20+index] &= 0xf7; //manual
      break;
      
    case K_fcu_high:     
        room_data_buf[16+index] |= 0x04;  room_data_buf[16+index] |= 0x80;  room_data_buf[54+index] &= 0xf8;   room_data_buf[54+index] |= 0x03;  
        room_data_buf[20+index] &= 0xf7; //manual
      break;    
      
    case K_fcu_auto_manual: room_data_buf[54+index] &= 0xf8;  room_data_buf[16+index] |= 0x80; break;
      
    case K_dnd: control_count = 0; dnd_mur_control_flag = 1; dnd_mur_execution('D','T'); break;
    case K_mur: control_count = 0; dnd_mur_control_flag = 1; dnd_mur_execution('M','T'); break;
    case K_call_wait:
    case K_emergency:
    case K_chime_push:
      if(!f_chime_keep){
        f_chime_keep = 1;
        chime_sq = 0;
        f_chime_sq_timer = 1;
        digital_volume_execution(0,room_data_buf[79]);
      }      
      break;
      
    case K_dim_onoff_1:
      if(dimmer_level[0] != 0) dimmer_level[0] = 0;
      else dimmer_level[0] = 99;
      dimmer_apply_bit |= 0x01;
      dim_apply_count = 0;
      scene_mode[0] = 0;
      break;
      
    case K_dim_onoff_2:
      if(dimmer_level[1] != 0) dimmer_level[1] = 0;
      else dimmer_level[1] = 99;
      dimmer_apply_bit |= 0x02;
      dim_apply_count = 0;
      scene_mode[0] = 0;
      break;

    case K_dim_onoff_3:
      if(dimmer_level[2] != 0) dimmer_level[2] = 0;
      else dimmer_level[2] = 99;
      dimmer_apply_bit |= 0x04;
      dim_apply_count = 0;
      scene_mode[0] = 0;
      break;

    case K_dim_onoff_4:
      if(dimmer_level[3] != 0) dimmer_level[3] = 0;
      else dimmer_level[3] = 99;
      dimmer_apply_bit |= 0x08;
      dim_apply_count = 0;
      scene_mode[0] = 0;
      break;
  
    case K_dim_master_0:
      for(i=0; i<16; ++i){
        if(b_dim_master_point_table[0][i] != 0xff){
          if(dimmer_level[b_dim_master_point_table[0][i]] != 0) j = 1;
        }
        break;
      }
      
      for(i=0; i<16; ++i){
        if(j){
          if(b_dim_master_point_table[0][i] != 0xff){ dimmer_level[b_dim_master_point_table[0][i]] = 0; room_data_buf[50+index] = 0x00; }
          else break;
        }
        else{
          if(b_dim_master_point_table[0][i] != 0xff){ dimmer_level[b_dim_master_point_table[0][i]] = 99; room_data_buf[50+index] = 0x1b; }
          else break;
        }
      }
      break;      
//--------------------------------------------      
          
      
      

  }
  
}

//
//=============== Light switch receive data check routine ==============================
//
void light_switch_check(unsigned char index)
{
  if(f_first_ls_power_on[index])
  {
      if(room_data_buf[40] & 0x02){   //Guest IN?
        if((light_switch_toggle_bit[index][0] & 0x01) != (uart2_rxd_buf[3] & 0x01)) 
        {
          nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][0],index);
        }
        if((light_switch_toggle_bit[index][0] & 0x02) != (uart2_rxd_buf[3] & 0x02)) 
        {
          nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][1],index);
        }
        if((light_switch_toggle_bit[index][0] & 0x04) != (uart2_rxd_buf[3] & 0x04)) 
        {
          nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][2],index);
        }
        if((light_switch_toggle_bit[index][0] & 0x08) != (uart2_rxd_buf[3] & 0x08)) 
        {
          nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][3],index);
        }
        if((light_switch_toggle_bit[index][0] & 0x10) != (uart2_rxd_buf[3] & 0x10)) nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][4],index);
        if((light_switch_toggle_bit[index][0] & 0x20) != (uart2_rxd_buf[3] & 0x20)) nt_switch_bit_check(b_ls_switch_kind_table[cb_mode][index][5],index);
      }
  }
  else f_first_ls_power_on[index] = 1;
  
  light_switch_toggle_bit[index][0] = uart2_rxd_buf[3];
}

void exinput_check(unsigned char index)
{
  if(f_first_input_power_on[index])
  {
      if(room_data_buf[40] & 0x02){   //Guest IN?
        if((exinput_toggle_bit[index][0] & 0x01) != (uart2_rxd_buf[2] & 0x01)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][0],index);
        if((exinput_toggle_bit[index][0] & 0x02) != (uart2_rxd_buf[2] & 0x02)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][1],index);
        if((exinput_toggle_bit[index][0] & 0x04) != (uart2_rxd_buf[2] & 0x04)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][2],index);
        if((exinput_toggle_bit[index][0] & 0x08) != (uart2_rxd_buf[2] & 0x08)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][3],index);
        if((exinput_toggle_bit[index][0] & 0x10) != (uart2_rxd_buf[2] & 0x10)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][4],index);
        if((exinput_toggle_bit[index][0] & 0x20) != (uart2_rxd_buf[2] & 0x20)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][5],index);
        if((exinput_toggle_bit[index][0] & 0x40) != (uart2_rxd_buf[2] & 0x40)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][6],index);
        if((exinput_toggle_bit[index][0] & 0x80) != (uart2_rxd_buf[2] & 0x80)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][7],index);
        
        if((exinput_toggle_bit[index][1] & 0x01) != (uart2_rxd_buf[3] & 0x01)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][8],index);
        if((exinput_toggle_bit[index][1] & 0x02) != (uart2_rxd_buf[3] & 0x02)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][9],index);
        if((exinput_toggle_bit[index][1] & 0x04) != (uart2_rxd_buf[3] & 0x04)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][10],index);
        if((exinput_toggle_bit[index][1] & 0x08) != (uart2_rxd_buf[3] & 0x08)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][11],index);
        if((exinput_toggle_bit[index][1] & 0x10) != (uart2_rxd_buf[3] & 0x10)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][12],index);
        if((exinput_toggle_bit[index][1] & 0x20) != (uart2_rxd_buf[3] & 0x20)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][13],index);
        if((exinput_toggle_bit[index][1] & 0x40) != (uart2_rxd_buf[3] & 0x40)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][14],index);
        if((exinput_toggle_bit[index][1] & 0x80) != (uart2_rxd_buf[3] & 0x80)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][15],index);

        if((exinput_toggle_bit[index][2] & 0x01) != (uart2_rxd_buf[4] & 0x01)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][16],index);
        if((exinput_toggle_bit[index][2] & 0x02) != (uart2_rxd_buf[4] & 0x02)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][17],index);
        if((exinput_toggle_bit[index][2] & 0x04) != (uart2_rxd_buf[4] & 0x04)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][18],index);
        if((exinput_toggle_bit[index][2] & 0x08) != (uart2_rxd_buf[4] & 0x08)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][19],index);
        if((exinput_toggle_bit[index][2] & 0x10) != (uart2_rxd_buf[4] & 0x10)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][20],index);
        if((exinput_toggle_bit[index][2] & 0x20) != (uart2_rxd_buf[4] & 0x20)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][21],index);
        if((exinput_toggle_bit[index][2] & 0x40) != (uart2_rxd_buf[4] & 0x40)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][22],index);
        if((exinput_toggle_bit[index][2] & 0x80) != (uart2_rxd_buf[4] & 0x80)) nt_switch_bit_check(b_exinput_switch_kind_table[cb_mode][index][23],index);
        
        }
     }
    else f_first_input_power_on[index] = 1;
     
    exinput_toggle_bit[index][0] = uart2_rxd_buf[2];  
    exinput_toggle_bit[index][1] = uart2_rxd_buf[3];  
    exinput_toggle_bit[index][2] = uart2_rxd_buf[4];  
}

//========================= dimmer receive data check routine =============================
//
void dimmer_check(unsigned char index)
{
  if(f_first_dimmer_power_on[index])
  {
    
  }
  else f_first_dimmer_power_on[index] = 1;
     
  dimmer_switch_toggle_bit[index] = uart2_rxd_buf[4];
}
  
void chime_ind_check(unsigned char index)
{
  if(f_first_ci_power_on[index])
  {
    if((chime_ind_toggle_bit[index] & 0x01) != (uart2_rxd_buf[3] & 0x01))
    {
      if(!f_chime_keep)
      {
        f_chime_keep = 1;
        chime_sq = 0;
        f_chime_sq_timer = 1;
        digital_volume_execution(0,room_data_buf[79]);
        
        handy_sign_timer = 0;
        //if(room_data_buf[40] & 0x02) // 입실일때
        //{
          room_data_buf[48] |= 0x80;
          f_chime_sign = 1;
        //}
        
      }
    }
  }
  else f_first_ci_power_on[index] = 1;
   
  chime_ind_toggle_bit[index] = uart2_rxd_buf[3]; 
}

void chime_ex_sub_check(void)
{
      if(!f_chime_keep)
      {
        f_chime_keep = 1;
        chime_sq = 0;
        f_chime_sq_timer = 1;
        digital_volume_execution(0,room_data_buf[79]);
        
        handy_sign_timer = 0;
        if(room_data_buf[40] & 0x02) // 입실일때
        {
          room_data_buf[48] |= 0x80;
          f_chime_sign = 1;
        }
        
      } 
}

//
//=============== Key sensor receive data check routine =================
//
void key_sensor_rx_check(unsigned char index)
{
  if(f_first_ks_power_on[index])
  {
    if(room_data_buf[40] & 0x02) //재실인가 ??
    {
      if((ks_switch_toggle_bit[index] & 0x01) != (uart2_rxd_buf[3] & 0x01))
      {
        nt_switch_bit_check(K_dnd, index);
      }
      if((ks_switch_toggle_bit[index] & 0x02) != (uart2_rxd_buf[3] & 0x02))
      {
        nt_switch_bit_check(K_mur, index);
      }
    }
  }
  else f_first_ks_power_on[index] = 1; 
  
  ks_switch_toggle_bit[index] = uart2_rxd_buf[3];
  
  if(dip_switch_buf[0] & 0x40){   //통신형 K/S 사용하는가?
    f_ks_rx_ok = 1;
    ks_rx_timer = 0;
    ks_key_status = uart2_rxd_buf[4];
    f_ks_error = 0;
  }
  
}

void ex_485_receive(void)
{
    if((cb_mode == 4) || (cb_mode == 6)) //19층 (좌,우 Type) - Key sensor 없음.
    {
      if(key_status_data != uart2_rxd_buf[2])
      {
          key_status_data = uart2_rxd_buf[2];
          
          switch(key_status_data)
          {
          case 0x00: //공실
             ks_key_status = 0x00;
             break;
              
          case 0x01: //청소중
             ks_key_status = 0x10;
             break;
            
          case 0x02: //청소완료     
             ks_key_status = 0x20;
             break;
             
          case 0x03: //재실
             ks_key_status = 0x30;
             break;
          }
      }
    }
    
    if(!dnd_mur_control_flag) //제어 상태가아닐때 수신받는다..
    {
      if((room_data_buf[40] & 0x0c) != (uart2_rxd_buf[3] & 0x0c))
      {
          room_data_buf[40] = uart2_rxd_buf[3]; //DND, MUR 동기화
      }
    } ////////////////////////////////////
    else if(++control_count > 5) //제어후 5회 이상 수신 보낸뒤 다시 재수신 진행
    {
        control_count = 0;
        dnd_mur_control_flag = 0;
    }
}

//=======================================================================
//=======================================================================

