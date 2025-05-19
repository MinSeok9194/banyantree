/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		:   
processor	  : STM32F103xx
compiler		: IAR 6.41A Compiler 
program BY 	: H.H.Hwang
date 			  : 2013.	  .
copy right 	: Plus - H.
===========================================================================*/

#define __UART3_H__

#include "stm32f10x_lib.h"
#include "main.h"

//========================================================================
/* UART3_initial	:								              */												
//========================================================================
void UART3_initial(void) {
NVIC_InitTypeDef NVIC_InitStructure;
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

	 /* Enable the USART3 Pins Partial Software Remapping */
//	GPIO_PinRemapConfig(GPIO_Remap_USART3, DISABLE);  

	/* Enable GPIOB and USART3 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	 /* Enable USART3 clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        
	/* Configure USART3 Tx (PB10) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3 Rx (PB11) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

/* USART3 configuration ------------------------------------------------------*/
  /* USART3 configured as follow:
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
// USART_InitStructure.USART_CPOL = USART_CPOL_Low;
//  USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
//  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;
  
  /* Configure the USART3 */
  USART_Init(USART3, &USART_InitStructure);
  
 /* Enable the USART Transmoit interrupt: this interrupt is generated when the 
   USART1 transmit data register is empty */  
//  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
  
/* Enable the USART Receive interrupt: this interrupt is generated when the 
   USART3 receive data register is not empty */
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  
  /* Enable USART3 */
  USART_Cmd(USART3, ENABLE);
  
  /* Enable the USART3 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


void UART3_control_proc(void)
{
    unsigned char sub_device_kind;
    
          if(f_uart3_data_send_ok)
          {
              f_uart3_data_send_ok = 0;
              
      if(f_common_send_time_2){ 
        common_data_set_2();  
        f_common_send_time_2 = 0; 
      }
      else{
        if(b_sub_device_table_2[cb_mode][id_number_2] == 0xff) id_number_2 = 0;
        sub_device_kind = b_sub_device_table_2[cb_mode][id_number_2++];
        switch(sub_device_kind){
          
          case B_LIGHT_SWITCH_1:
            light_switch_data_set_uart3(0,0x01);
            break;  
          case B_LIGHT_SWITCH_2:
            light_switch_data_set_uart3(1,0x02);
            break;  
          case B_LIGHT_SWITCH_3:
            light_switch_data_set_uart3(2,0x04);
            break;  
          case B_LIGHT_SWITCH_4:
            light_switch_data_set_uart3(3,0x08);
            break;  
          case B_LIGHT_SWITCH_5:
            light_switch_data_set_uart3(4,0x10);
            break;  
          case B_LIGHT_SWITCH_6:
            light_switch_data_set_uart3(5,0x20);
            break;      
          case B_LIGHT_SWITCH_7:
            light_switch_data_set_uart3(6,0x40);
            break; 
          case B_LIGHT_SWITCH_8:
            light_switch_data_set_uart3(7,0x80);
            break;
          case B_LIGHT_SWITCH_9:
            light_switch_data_set_uart3(8,0x100);
            break;
          case B_LIGHT_SWITCH_10:
            light_switch_data_set_uart3(9,0x200);
            break; 
            
          case B_LIGHT_SWITCH_11:
            light_switch_data_set_uart3(10,0x400);
            break;         

          case B_LIGHT_SWITCH_12:
            light_switch_data_set_uart3(11,0x800);
            break;         

          case B_LIGHT_SWITCH_13:
            light_switch_data_set_uart3(12,0x1000);
            break;         
          
          case B_LIGHT_SWITCH_14:
            light_switch_data_set_uart3(13,0x2000);
            break;      
            
        case B_LIGHT_SWITCH_15:
            light_switch_data_set_uart3(14,0x4000);
            break;
            
         case B_LIGHT_SWITCH_16:
            light_switch_data_set_uart3(15,0x8000);
            break;           
        }
      }              
          }
          
          UART3_tx_check();
          UART3_rx_check();
}
void light_switch_data_set_uart3(unsigned char index, unsigned int index_bit)
{
  unsigned char crc = 0;
  unsigned char i=0,j;
  
  P_UART3_DIR = 1;
  ++i;                                                        //Length point(아래에서 처리)
  uart3_tx_data_buf[i++] = index + 0x30;                      //Device ID
  
  if(index_bit & 0x01) 
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {
      
    case 0: //stadard Type (침실 3구)
    case 2: //spa suite (10 floor type)
    case 4: //spa-suite ( 19층 좌 Type )
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x04;  
          
          break;
          
    case 1: //spa suite (4~9 floor type)      
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x04;           
          
          break;
          
    case 3: //p-suite ( 18층 좌 Type )
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0xfe) == 0) && ((room_data_buf[47] & 0x01) == 0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 5: //P_suite ( 18층 우 )
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0x0e) == 0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x04;          
          
          break;
          
    case 6: //19층 우
    case 15: //신관(클럽동) J Type
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 14: //신관(클럽동) I Type

          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x04;          
          
          break;
          
    case 7: //신관(클럽동) A Type
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x10;
          if((room_data_buf[45] & 0x0e) == 0) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 9: //신관(클럽동) C Type

          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x04;  
      
      break;
          
    case 8: //신관(클럽동) B Type
    case 10: //신관(클럽동) D Type
    case 11: //신관(클럽동) E Type
//    case 12: //신관(클럽동) F,F-1 Type
    case 13: //신관(클럽동) G,H Type
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x04;  

          break;
          
    case 12: //신관(클럽동) F,F-1 Type
      
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x04;        
      
          break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);
  }
  if(index_bit & 0x02) 
  {
    uart3_tx_data_buf[i] = 0;    
    
    switch(cb_mode)
    {
    case 0: //stadard Type (Master)
          
          if(((room_data_buf[45] & 0x1e) ==0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 4: //p - suite ( 19 좌 type )
          
          if(((room_data_buf[45] & 0x0e) ==0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02;          
          
          break;
         
    case 1: //spa suite (4~9 floor type)
           
           if(((room_data_buf[45] & 0x06) ==0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02; 
           
           break;
          
    case 2: //spa suite (10 floor type)
      
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x04;
      
          break;
          
    case 3: //p-suite ( 18층 좌 Type )
    case 5: //p_suite ( 18층 우 Type )
            
            if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          
          break;     
          
    case 6:
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x02;
          //if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 7:  //신관(클럽동) A Type
          
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;   
          
          break;
          
    case 9: //신관(클럽동) C Type
      
          if((room_data_buf[45] & 0x06) == 0) uart3_tx_data_buf[i] |= 0x02;        
      
      break;
          
    case 8: //신관(클럽동) B Type
    case 10: //신관(클럽동) D Type
    case 11: //신관(클럽동) E Type      
//    case 12: //신관(클럽동) F,F-1 Type   
    case 13: //신관(클럽동) G,H Type      
          
          if((room_data_buf[45] & 0x0e) == 0) uart3_tx_data_buf[i] |= 0x02;
          
          break;          
          
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type      
          
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 12: //신관(클럽동) F,F-1 Type
          
          if((room_data_buf[45] & 0x06) == 0) uart3_tx_data_buf[i] |= 0x02;
      
          break;
      
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);
  }
  if(index_bit & 0x04) 
  {
    uart3_tx_data_buf[i] = 0;

    switch(cb_mode)
    {
    case 0: //stadard Type (화장실 1구)
    case 1: //spa suite (4~9 floor type)  
    case 4: //p-suite ( 19층 좌 Type )
          
          if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 2: //spa suite (10 floor type)
    
          if(((room_data_buf[45] & 0x3e) == 0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 3: //p-suite ( 18층 좌 Type ) -> 4구 M -> 2구 변경
             
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;
          //if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x02;  
          //if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x10;
          //if(((room_data_buf[45] & 0xfe) == 0) && ((room_data_buf[47] & 0x01) == 0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x04;      
          
          break;            
          
    case 5: //p_suite ( 18층 우 Type )
          
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x02; 
          break;
          
    case 6:
      
      if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x02;
      
      break;
      
    case 7: //신관 (클랍동) A Type
      
      if((room_data_buf[45] & 0x60) == 0) uart3_tx_data_buf[i] |= 0x02;
      
      break;
      
    case 8: //신관(클럽동) B Type
      
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x04;            
          
          break;     
          
    case 9: //신관(클럽동) C Type
      
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;             
          
          break;
          
    case 10: //신관(클럽동) D Type
    //case 12: //신관(클럽동) F,F-1 Type           
          
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x10;
          if((room_data_buf[45] & 0xf0) == 0) uart3_tx_data_buf[i] |= 0x04;              
          
          break;
          
    case 11: //신관(클럽동) E Type
    case 13: //신관(클럽동) G Type
          
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;          
          
          break;
          
    case 15: //신관(클럽동) J Type 
          
          if((room_data_buf[45] & 0x1e) == 0) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 14: //신관(클럽동) I Type
          
          if((room_data_buf[45] & 0x1a) == 0) uart3_tx_data_buf[i] |= 0x02;          
      
          break;
          
    case 12: //신관(클럽동) F,F-1 Type
      
          if(room_data_buf[47] & 0x40) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x80) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[48] & 0x01) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[48] & 0x02) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[47] & 0xc0) == 0) && ((room_data_buf[48] & 0x03) == 0)) uart3_tx_data_buf[i] |= 0x04;        
      
      break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);      
  }
  if(index_bit & 0x08)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {
    case 0: //stadard Type (복도 4구)
          
          if(room_data_buf[45] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(((room_data_buf[45] & 0x1e) ==0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x04;     
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x10;
          
          break;
          
    case 1: //spa suite (4~9 floor type)
      
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x08;
          if(((room_data_buf[45] & 0xf0) ==0) && ((room_data_buf[46] & 0x02) == 0)) uart3_tx_data_buf[i] |= 0x04;     
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x10;      
          
          break;
          
    case 2: //spa suite (10 floor type)
      
          if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          
          break;

    case 3: //p-suite ( 16 좌 type )
          
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[47] & 0x0e) == 0) && ((room_data_buf[45] & 0x60) == 0)) uart3_tx_data_buf[i] |= 0x04;     
      
      break;
          
    case 4: //p-suite ( 17 좌 type )
          
          if(room_data_buf[45] & 0x08) uart3_tx_data_buf[i] |=0x02;          
          
          break;
          
   case 5: //p_suite ( 18층 우 Type )
          
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[47] & 0x10) uart3_tx_data_buf[i] |= 0x10;
          if((room_data_buf[47] & 0x1e) == 0) uart3_tx_data_buf[i] |= 0x04;        
          
          break;
          
    case 6:
      
      if(((room_data_buf[45] & 0x0e) == 0) && ((room_data_buf[46] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02;
      
      break;
      
    case 7:  //신관(클럽동) A Type
          
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x04;
          
      break;
      
    case 8: //신관(클럽동) B Type
      
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x08;           
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x02;      
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0xc0) == 0) && ((room_data_buf[47] & 0x07) == 0)) uart3_tx_data_buf[i] |= 0x04;
          
          break;          
          
    case 9: //신관(클럽동) C Type
          
          if((room_data_buf[45] & 0x70) == 0) uart3_tx_data_buf[i] |= 0x02;           
          
          break;          
          
    case 10: //신관(클럽동) D Type
          
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;           
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x04;                
          
          break;
          
    case 11: //신관(클럽동) E Type 
    case 13: //신관(클럽동) G Type
          
          if((room_data_buf[45] & 0x70) == 0) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 12: //신관(클럽동) F,F-1 Type
          
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;           
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x02;    
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x04; 
          
          break;      
          
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type
      
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x04;          
          
          break;          
    }    
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);      
  }  
  if(index_bit & 0x10)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {
    case 0: //stadard Type (wait)
        
        if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
    
      break;
      
    case 1: //spa suite (4~9 floor type)
        
        if(room_data_buf[46] & 0x02) uart3_tx_data_buf[i] |= 0x02;        
        
      break;
      
    case 2: //spa suite (10 floor type)
        
        if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x01;
        if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x08;
        if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x02;
        if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x10;
        if(((room_data_buf[45] & 0xc0) == 0) && ((room_data_buf[46] & 0x06)==0) && ((room_data_buf[47] & 0x03)==0)) uart3_tx_data_buf[i] |= 0x04;
        
      break;
    
        
      case 3: // spa suite ( 16 (좌) floor type )
      
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;     
      
        break;      
      
    case 4: //p-suite ( 17 좌 type )
      
          if(room_data_buf[45] & 0x10) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;          
          
      break;
      
      case 6: //spa suite (10 floor type)
      
          if(room_data_buf[46] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          
          break;
  
      case 5:
  
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;
     
    case 7:  //신관(클럽동) A Type
          
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x04;
          
      break;      
      
    case 8: //신관(클럽동) B Type
          
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x02;          
          
      break;
      
    case 9: //신관(클럽동) C Type

          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x08;           
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x02;      
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0x80) == 0) && ((room_data_buf[47] & 0x03) == 0)) uart3_tx_data_buf[i] |= 0x04;        
      
      break;
      
      
    case 13: //신관(클럽동) G Type
      
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x08;           
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x02;      
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0x80) == 0) && ((room_data_buf[47] & 0x07) == 0)) uart3_tx_data_buf[i] |= 0x04;      
          
      break;
      
    case 10: //신관(클럽동) D Type
      
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;           
      
      break;
      
    case 11: //신관(클럽동) E Type
      
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
      break;
          
    case 12: //신관(클럽동) F,F-1 Type
          
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x01;           
          if(room_data_buf[47] & 0x10) uart3_tx_data_buf[i] |= 0x02;    
          if(room_data_buf[47] & 0x20) uart3_tx_data_buf[i] |= 0x04; 
          
          break; 
          
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type
          
          if((room_data_buf[45] & 0xe0) == 0) uart3_tx_data_buf[i] |= 0x02;
          
          break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);      
  }    
  if(index_bit & 0x20)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {
      case 1: //spa suite (4~9 floor type)
        
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;
        
      case 2: // spa suite (10 floor type)
          
          if(room_data_buf[46] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[46] & 0x04) uart3_tx_data_buf[i] |= 0x04;
          
        break;
        
      case 4: //p-suite ( 17 좌 Type )
      
          if((room_data_buf[45] & 0x70) == 0) uart3_tx_data_buf[i] |= 0x02;
          
        break;
        
    case 6:
          
          if(room_data_buf[45] & 0x20) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x04;
          //if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x04;
          
          break;
          
    case 7: //신관(클럽동) A Type
      
          if((room_data_buf[47] & 0x0f) == 0) uart3_tx_data_buf[i] |= 0x02; 
            
          break;
          
    case 8: //신관(클럽동) B Type
    case 9: //신관(클럽동) C Type
    case 11://신관(클럽동) E Type
    case 13: //신관(클럽동) G Type
      
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;         
          
          break;
          
    case 12: //신관(클럽동) F,F-1 Type
          
          if((room_data_buf[47] & 0x38) == 0) uart3_tx_data_buf[i] |= 0x02;           
          
          break;           
          
    case 14: //신관(클럽동) I Type
      
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x08;           
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x02;      
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x10;
          if((room_data_buf[47] & 0x0f) == 0) uart3_tx_data_buf[i] |= 0x04;             
          
          break;
          
    case 15: //신관(클럽동) J Type
      
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x02;           
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x04;              
      
          break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);     
  }
  if(index_bit & 0x40)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    { 
      case 2: // spa suite (10 floor type)
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;
        
      case 4: //p-suite (17 좌 Type )
          
          if(room_data_buf[45] & 0x80) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x04;
          //if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x04;           
          
        break;
        
        case 6:
          
          if((room_data_buf[45] & 0x60)==0) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
        case 7: //신관(클럽동) A Type
          
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x04; 
          
          break;
      
    case 12: //신관(클럽동) F,F-1 Type           
          
          if(room_data_buf[47] & 0x40) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x80) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[48] & 0x01) uart3_tx_data_buf[i] |= 0x02;  
          if(room_data_buf[48] & 0x02) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[47] & 0xc0) == 0) && ((room_data_buf[48] & 0x03) == 0)) uart3_tx_data_buf[i] |= 0x04;              
          
          break;      
          
    case 14: //신관(클럽동) I Type
          
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 15: //신관(클럽동) J Type
      
          if((room_data_buf[47] & 0x07) == 0) uart3_tx_data_buf[i] |= 0x02;
      
          break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);     
  }
  if(index_bit & 0x80)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    { 
      case 2: // spa suite (10 floor type)
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
            
        break;
          
      case 4: //spa suite ( 17층 좌 Type )
          
          if(((room_data_buf[45] & 0x80) == 0) && ((room_data_buf[47] & 0x01) == 0) && ((room_data_buf[46] & 0x02) == 0)) uart3_tx_data_buf[i] |= 0x02;
          
        break;
        
        case 6: //19층 우
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x04) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x04;
          break;
          
        case 7: //신관(클럽동) A Type
          if((room_data_buf[47] & 0x0f) == 0) uart3_tx_data_buf[i] |= 0x02;           
          break;
      
        case 12: //신관(클럽동) F,F-1 Type
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
          break;
          
    case 14: //신관(클럽동) I Type
    case 15: //신관(클럽동) J Type
        
        if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
      
        break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);     
  }  
  if(index_bit & 0x100)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {  
      case 2: //spa suite ( 11~12층)
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[45] & 0x40) uart3_tx_data_buf[i] |= 0x08;
          if(room_data_buf[47] & 0x01) uart3_tx_data_buf[i] |= 0x02;
          if(room_data_buf[47] & 0x02) uart3_tx_data_buf[i] |= 0x10;
          if(((room_data_buf[45] & 0x40) == 0) && ((room_data_buf[46] & 0x02)==0) && ((room_data_buf[47] & 0x03)==0)) uart3_tx_data_buf[i] |= 0x04;          
          break;
          
      case 4: //spa suite ( 17층 좌 Type )
          if(room_data_buf[46] & 0x02) uart3_tx_data_buf[i] |= 0x02;
          break;
          
       case 6: //19층 우
          if(room_data_buf[47] & 0x08) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x10) uart3_tx_data_buf[i] |= 0x04;
          break;
          
        case 7: //신관(클럽동) A Type
          if(room_data_buf[47] & 0x40) uart3_tx_data_buf[i] |= 0x01;
          if(room_data_buf[47] & 0x80) uart3_tx_data_buf[i] |= 0x02;           
          if(room_data_buf[48] & 0x01) uart3_tx_data_buf[i] |= 0x04; 
          break; 
          
        case 12: //신관(클럽동) F,F-1 Type
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
          break;    
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);       
  }
  if(index_bit & 0x200)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {  
    case 2: //spa suite ( 11층~12층 )
      if(room_data_buf[46] & 0x02) uart3_tx_data_buf[i] |= 0x02;
      break;
      
    case 4: //spa suite ( 17층 좌 Type )
      if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;   
      break;
      
    case 6:
      if(((room_data_buf[47] & 0x0e) == 0) && ((room_data_buf[46] & 0x02) == 0)) uart3_tx_data_buf[i] |= 0x02;
      break;
      
    case 7: //신관(클럽동) A Type
      if(((room_data_buf[47] & 0xc0) == 0) && ((room_data_buf[48] & 0x01) == 0)) uart3_tx_data_buf[i] |= 0x02;      
      break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);       
  }
  if(index_bit & 0x400)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {  
      case 4: //spa suite ( 17층 좌 Type )
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;
        
    case 6:
      
      if(room_data_buf[46] & 0x02) uart3_tx_data_buf[i] |= 0x02;
      
      break;
      
    case 7: //신관(클럽동) A Type
      if(room_data_buf[48] & 0x02) uart3_tx_data_buf[i] |= 0x01;
      if(room_data_buf[48] & 0x04) uart3_tx_data_buf[i] |= 0x04;
      break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);       
  }
  
  if(index_bit & 0x800)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {  
      case 6: 
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;

      case 7: //신관(클럽동) A Type
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
        break;
    }
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);  
  }
   
  if(index_bit & 0x1000)
  {
    uart3_tx_data_buf[i] = 0;
    
    switch(cb_mode)
    {  
      case 6: 
          
          if(room_data_buf[40] & 0x80) uart3_tx_data_buf[i] |= 0x02;
          
        break;
    }
    
    ++i;
    uart3_tx_data_buf[i++] = light_switch_toggle_bit_2[index][0];
    uart3_tx_data_buf[i++] = ls_control_data_read(index);       
  }
  
  
  uart3_tx_data_buf[i++] = cb_mode;                         //cb_mode check
  for(j=1; j<i; ++j) crc += uart3_tx_data_buf[j];
  crc ^= 0x55;
  uart3_tx_data_buf[i++] = crc;                             //CRC
  uart3_tx_data_buf[0] = i;                                 //Length
  
  f_uart3_data_send = 1;
  uart3_tx_length = 0;
  uart3_tx_backup = i;
  timer_cnt_2 = 0;
}

void common_data_set_2(void)
{
  
  unsigned char crc = 0;
  unsigned char i = 0,j;
  unsigned char temp=0;
    
  P_UART3_DIR = 1;
  ++i;
  uart3_tx_data_buf[i++] = 0xff;                        //Device ID
  uart3_tx_data_buf[i++] = 0xff;                        //Device ID
  
  if(room_data_buf[40] & 0x01) temp |= 0x01;            //Check IN 
  if(room_data_buf[40] & 0x02) temp |= 0x02;            //Guest IN
  if(room_data_buf[40] & 0x10) temp |= 0x04;            //Window
  if(room_data_buf[20] & 0x04) temp |= 0x08;            //Cool mode
  if(room_data_buf[40] & 0x04) temp |= 0x10;            //DND
  if(room_data_buf[40] & 0x08) temp |= 0x20;            //MUR
  if(room_data_buf[82] & 0x08) temp |= 0x80;            //현재조도상태 1=밝게하라(밝다),0=어둡게하라(어둡다)
  uart3_tx_data_buf[i++] = temp;
  uart3_tx_data_buf[i++] = room_data_buf[82];           //조도 조절법, 복귀시간 및 현재 상태
  
  for(j=1; j<i; ++j) crc += uart3_tx_data_buf[j];
  crc ^= 0x55;
  uart3_tx_data_buf[i++] = crc;                             //CRC
  uart3_tx_data_buf[0] = i;                                 //Length
  
  f_uart3_data_send = 1;
  uart3_tx_length = 0;
  uart3_tx_backup = i;
  timer_cnt_2 = 0;
  
}

/* UART3 data send check routine */
void UART3_tx_check(void)
{
  if(f_uart3_data_send){
    if(++timer_cnt_2 > 45)
    {
      timer_cnt_2 = 1001;
      if(USART_GetFlagStatus(USART3, USART_FLAG_TC) !=RESET) {
      if(uart3_tx_length == 0){
        USART3->DR = ((uart3_tx_data_buf[uart3_tx_length++] & (u16)0x01FF) | (u16)0x0100);
      }
      else{
        if(uart3_tx_length < uart3_tx_backup){
          USART3->DR = (uart3_tx_data_buf[uart3_tx_length++] & (u16)0x00FF);
        }
        else{
          P_UART3_DIR = 0;
          f_uart3_data_send = 0;
          f_uart3_data_send_time = 18;
        }
      }
      }
    }
  }
}

//
//========== light sw routine =============
//
void UART3_rx_check(void)
{
  unsigned char rx_crc=0;
  unsigned char i;
  
  if(f_uart3_frame_rx_ok)
  {
    f_uart3_frame_rx_ok = 0;
      
      for(i=1; i<uart3_rxd_buf[0]-1; ++i) rx_crc += uart3_rxd_buf[i];
      rx_crc ^= 0x55;
      if(rx_crc == (uart3_rxd_buf[uart3_rxd_buf[0]-1])){
        switch(uart3_rxd_buf[1] & 0xf0){          
          case 0x30:    //Light switch
            light_switch_check_2(uart3_rxd_buf[1] & 0x0F);
            break;        
        }
        f_uart3_data_send_time = 18;
     }
      
  }
}

void light_switch_check_2(unsigned char index)
{
  if(f_first_ls_power_on_2[index])
  {
      if(room_data_buf[40] & 0x02) //Guest IN?
      {  
          if((light_switch_toggle_bit_2[index][0] & 0x01) != (uart3_rxd_buf[3] & 0x01)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][0],index);
          if((light_switch_toggle_bit_2[index][0] & 0x02) != (uart3_rxd_buf[3] & 0x02)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][1],index);
          if((light_switch_toggle_bit_2[index][0] & 0x04) != (uart3_rxd_buf[3] & 0x04)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][2],index);
          if((light_switch_toggle_bit_2[index][0] & 0x08) != (uart3_rxd_buf[3] & 0x08)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][3],index);
          if((light_switch_toggle_bit_2[index][0] & 0x10) != (uart3_rxd_buf[3] & 0x10)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][4],index);
          if((light_switch_toggle_bit_2[index][0] & 0x20) != (uart3_rxd_buf[3] & 0x20)) nt_switch_bit_check(b_ls_switch_kind_table_7_uart3[cb_mode][index][5],index);   
      }
  }
  else f_first_ls_power_on_2[index] = 1;
  
  light_switch_toggle_bit_2[index][0] = uart3_rxd_buf[3];
}
