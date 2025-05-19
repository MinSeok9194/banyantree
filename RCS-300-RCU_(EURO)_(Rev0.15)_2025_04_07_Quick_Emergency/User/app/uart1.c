/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		:   
processor	  : STM32F103xx
compiler		: IAR 6.41A Compiler 
program BY 	: H.H.Hwang
date 			  : 2013.	  .
copy right 	: Plus - H.
===========================================================================*/


#define __UART1_H__

#include "stm32f10x_lib.h"
#include "main.h"

//=======================================================================
u8 u1ctrl =7, u1txinx;
void uart1_frame_proc() {

  UART1_tx_check();
  
/*  
	switch(u1ctrl) {
		case 0:
			if(dtx.ldad !=dtx.svad) { 
				USART1->DR =dtx.buf[dtx.ldad] & (u16)0x01FF;
				u1ctrl++;
			}
			led.s.txd =0;
			watchdog.s.b5 =1;					// watchdog setup
			Extm.s.u1c =10;
			break;
			
		case 1:
			if(USART_GetFlagStatus(USART1, USART_FLAG_TC) !=RESET) {
				if(++dtx.ldad >=sizeof(dtx.buf)) dtx.ldad =0;
				if(dtx.ldad !=dtx.svad) {
					led.s.txd++;
					USART1->DR = (dtx.buf[dtx.ldad] & (u16)0x01FF);
					Extm.s.u1c =10;
				}
				else u1ctrl =0;
			}
			else if(!Extm.s.u1c) u1ctrl =7;			
			break;

// powerup start
		case 7:
			UART1_initial();						// UART1 initial
			hyper_clear();
			sprintf(xdebug, "%c[1;0H%c[1;%xm%c[1;%xm", 0x1b, 0x1b, cyan,  0x1b, _black); mm;

			if(zin.msel &0x80) __xcprintf("Uart1 Initial !!!\r\n");
			u1_init_f =1;
			
		default:		u1ctrl =0;		break;
 	}
*/  
}


/* UART1 data arrange routine */
void UART1_data_set(void)
{
/*  
  if((room_data_buf[99] & 0x07) == 0x07)  cb_data_send('R');
  else if((room_data_buf[99] & 0x07) == 0x03) cb_data_send('t');
  else if((room_data_buf[99] & 0x07) == 0x02) cb_data_send('T');
  else if((room_data_buf[99] & 0x07) == 0x04) cb_data_send('F');
  else cb_data_send('S');
*/
  
  if((room_data_buf[99] & 0x07) == 0x07)  cb_data_send('R');
  else if((room_data_buf[99] & 0x07) == 0x03) cb_data_send('t');
  else if(room_data_buf[99] & 0x02) cb_data_send('T');
  else if(room_data_buf[99] & 0x04) cb_data_send('F');
  else cb_data_send('S');
  
  
}

void cb_data_send(unsigned char send_cmd)
{
  unsigned char crc = 0;
  unsigned char i=0;
  unsigned char j;
  
  P_UART1_DIR = 1;
  
  uart1_tx_data_buf[i++] = ' ';                                 //Preamble
  uart1_tx_data_buf[i++] = '{';                                 //STX
  ++i;                                                          //Length

  uart1_tx_data_buf[i++] = main_id_buf;                         //ID
  
  switch(send_cmd){
    case 'S':   //Standard data send
      uart1_tx_data_buf[i++] = 'S';                                 //Command
      uart1_tx_data_buf[i++] = cb_basic_data(0);                    //Basic data
      if(f_clean_finish)
      {
        f_clean_finish = 0;
        room_data_buf[49] = 2;
      }
      uart1_tx_data_buf[i++] = room_data_buf[49];                   //Key status
      uart1_tx_data_buf[i++] = room_data_buf[88];                   //Sub device comm status
      uart1_tx_data_buf[i++] = room_data_buf[89];                   //Sub device comm status
      //uart1_tx_data_buf[i++] = lg_air_error_code[0];                //LG aircon error code 0
      //uart1_tx_data_buf[i++] = lg_air_error_code[1];                //LG aircon error code 1
      break;
      
    case 't':   //Standard data + Air temp data send
      room_data_buf[99] &= 0xfc;
      uart1_tx_data_buf[i++] = 't';                                 //Command
      uart1_tx_data_buf[i++] = cb_basic_data(0);                    //Basic data
      if(f_clean_finish)
      {
        f_clean_finish = 0;
        room_data_buf[49] = 2;
      }      
      uart1_tx_data_buf[i++] = room_data_buf[49];                   //Key status
      uart1_tx_data_buf[i++] = room_data_buf[88];                   //Sub device comm status
      uart1_tx_data_buf[i++] = room_data_buf[89];                   //Sub device comm status
      
      uart1_tx_data_buf[i++] = air_temp_control_status(0);          //Air temp control status
      uart1_tx_data_buf[i++] = air_temp_control_status(1);
      uart1_tx_data_buf[i++] = air_temp_control_status(2);
      uart1_tx_data_buf[i++] = air_temp_control_status(3);
      uart1_tx_data_buf[i++] = air_temp_control_status(4);
      uart1_tx_data_buf[i++] = air_temp_control_status(5);
      uart1_tx_data_buf[i++] = air_temp_control_status(6);
      uart1_tx_data_buf[i++] = air_temp_control_status(7);
      
      uart1_tx_data_buf[i++] = room_data_buf[0];                    //Air Set temp
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      uart1_tx_data_buf[i++] = room_data_buf[0];
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      
      uart1_tx_data_buf[i++] = room_data_buf[4];                    //Air Check temp
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      uart1_tx_data_buf[i++] = room_data_buf[4];
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      break;
      
    case 'T':   //Air temp data send
      room_data_buf[99] &= 0xfd;
      uart1_tx_data_buf[i++] = 'T';                                 //Command
      uart1_tx_data_buf[i++] = air_temp_control_status(0);          //Air temp control status
      uart1_tx_data_buf[i++] = air_temp_control_status(1);
      uart1_tx_data_buf[i++] = air_temp_control_status(2);
      uart1_tx_data_buf[i++] = air_temp_control_status(3);
      uart1_tx_data_buf[i++] = air_temp_control_status(4);
      uart1_tx_data_buf[i++] = air_temp_control_status(5);
      uart1_tx_data_buf[i++] = air_temp_control_status(6);
      uart1_tx_data_buf[i++] = air_temp_control_status(7);
      
      uart1_tx_data_buf[i++] = room_data_buf[0];                    //Air Set temp
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      uart1_tx_data_buf[i++] = room_data_buf[0];
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      
      uart1_tx_data_buf[i++] = room_data_buf[4];                    //Air Check temp
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      uart1_tx_data_buf[i++] = room_data_buf[4];
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      break;
      
    case 'F':   //Floor temp data send
      room_data_buf[99] &= 0xfb;
      uart1_tx_data_buf[i++] = 'F';                                 //Command
      uart1_tx_data_buf[i++] = floor_temp_control_status(0);          //floor temp control status
      uart1_tx_data_buf[i++] = floor_temp_control_status(1);
      uart1_tx_data_buf[i++] = floor_temp_control_status(2);
      uart1_tx_data_buf[i++] = floor_temp_control_status(3);
      uart1_tx_data_buf[i++] = floor_temp_control_status(4);
      uart1_tx_data_buf[i++] = floor_temp_control_status(5);
      uart1_tx_data_buf[i++] = floor_temp_control_status(6);
      uart1_tx_data_buf[i++] = floor_temp_control_status(7);
      
      uart1_tx_data_buf[i++] = room_data_buf[8];                    //Floor Set temp
      uart1_tx_data_buf[i++] = room_data_buf[9];
      uart1_tx_data_buf[i++] = room_data_buf[10];
      uart1_tx_data_buf[i++] = room_data_buf[11];  
      uart1_tx_data_buf[i++] = room_data_buf[8];
      uart1_tx_data_buf[i++] = room_data_buf[9];
      uart1_tx_data_buf[i++] = room_data_buf[10];
      uart1_tx_data_buf[i++] = room_data_buf[11];
      
      uart1_tx_data_buf[i++] = room_data_buf[12];                    //Floor Check temp
      uart1_tx_data_buf[i++] = room_data_buf[13];
      uart1_tx_data_buf[i++] = room_data_buf[14];
      uart1_tx_data_buf[i++] = room_data_buf[15];
      uart1_tx_data_buf[i++] = room_data_buf[12];
      uart1_tx_data_buf[i++] = room_data_buf[13];
      uart1_tx_data_buf[i++] = room_data_buf[14];
      uart1_tx_data_buf[i++] = room_data_buf[15];
      break;
      
    case 'R':
      room_data_buf[99] &= 0xfc;
      uart1_tx_data_buf[i++] = 't';                                 //Command
      uart1_tx_data_buf[i++] = cb_basic_data(0);                    //Basic data
      if(f_clean_finish)
      {
        f_clean_finish = 0;
        room_data_buf[49] = 2;
      }      
      uart1_tx_data_buf[i++] = room_data_buf[49];                   //Key status
      uart1_tx_data_buf[i++] = room_data_buf[88];                   //Sub device comm status
      uart1_tx_data_buf[i++] = room_data_buf[89];                   //Sub device comm status
      
      uart1_tx_data_buf[i++] = air_temp_control_status(0);          //Air temp control status
      uart1_tx_data_buf[i++] = air_temp_control_status(1);
      uart1_tx_data_buf[i++] = air_temp_control_status(2);
      uart1_tx_data_buf[i++] = air_temp_control_status(3);
      uart1_tx_data_buf[i++] = air_temp_control_status(4);
      uart1_tx_data_buf[i++] = air_temp_control_status(5);
      uart1_tx_data_buf[i++] = air_temp_control_status(6);
      uart1_tx_data_buf[i++] = air_temp_control_status(7);
      
      uart1_tx_data_buf[i++] = room_data_buf[0];                    //Air Set temp
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      uart1_tx_data_buf[i++] = room_data_buf[0];
      uart1_tx_data_buf[i++] = room_data_buf[1];
      uart1_tx_data_buf[i++] = room_data_buf[2];
      uart1_tx_data_buf[i++] = room_data_buf[3];
      
      uart1_tx_data_buf[i++] = room_data_buf[4];                    //Air Check temp
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      uart1_tx_data_buf[i++] = room_data_buf[4];
      uart1_tx_data_buf[i++] = room_data_buf[5];
      uart1_tx_data_buf[i++] = room_data_buf[6];
      uart1_tx_data_buf[i++] = room_data_buf[7];
      
      uart1_tx_data_buf[i++] = room_data_buf[8];                    //Floor Set temp
      uart1_tx_data_buf[i++] = room_data_buf[9];
      uart1_tx_data_buf[i++] = room_data_buf[10];
      uart1_tx_data_buf[i++] = room_data_buf[11];
      uart1_tx_data_buf[i++] = room_data_buf[8];
      uart1_tx_data_buf[i++] = room_data_buf[9];
      uart1_tx_data_buf[i++] = room_data_buf[10];
      uart1_tx_data_buf[i++] = room_data_buf[11];
      
      uart1_tx_data_buf[i++] = room_data_buf[12];                    //Floor Check temp
      uart1_tx_data_buf[i++] = room_data_buf[13];
      uart1_tx_data_buf[i++] = room_data_buf[14];
      uart1_tx_data_buf[i++] = room_data_buf[15];
      uart1_tx_data_buf[i++] = room_data_buf[12];
      uart1_tx_data_buf[i++] = room_data_buf[13];
      uart1_tx_data_buf[i++] = room_data_buf[14];
      uart1_tx_data_buf[i++] = room_data_buf[15];
      
      break;
  }
  uart1_tx_data_buf[2] = i+1;
  for(j=1; j<i; ++j) crc += uart1_tx_data_buf[j];
  crc ^= 0x55;
  uart1_tx_data_buf[i++] = crc;                             //CRC
  uart1_tx_data_buf[i++] = '}';                             //ETX
  f_uart1_data_send = 1;
  uart1_tx_length = 0;
  uart1_tx_backup = i;  
 
}

unsigned char air_temp_control_status(unsigned char index)
{
  unsigned char return_data = 0;
  
  return_data = room_data_buf[16+index] & 0x0f;             //Speed & on/off status & use
  if(room_data_buf[20+index] & 0x04) return_data |= 0x10;   //냉난상태
  return_data &= 0x9f;  
  return_data |= ((room_data_buf[20+index] & 0x03) << 5);   //제어상태
  return(return_data);
}

unsigned char floor_temp_control_status(unsigned char index)
{
  unsigned char return_data = 0;
  
  if(room_data_buf[16+index] & 0x10) return_data |= 0x01;   //valve 상태              
  if(room_data_buf[16+index] & 0x20) return_data |= 0x04;   //바닥 switch on/off status
  if(room_data_buf[16+index] & 0x40) return_data |= 0x08;   //바닥센서 사용 유/무                            
  return_data |= ((room_data_buf[20+index] & 0x30) << 1);   //제어상태
  
  return(return_data);
}

unsigned char cb_basic_data(unsigned char index)
{
  unsigned char return_data = 0;
      
  if(room_data_buf[index+60] & 0x01) return_data |= 0x01;       //한실(1) or 양실
  if((room_data_buf[42] >> index) & 0x01) return_data |= 0x02;  //비상상태
  if((room_data_buf[42] >> index) & 0x30) return_data |= 0x04;  //이상온도
  if(room_data_buf[40] & 0x04)  return_data |= 0x08;            //DND ?
  if(room_data_buf[40] & 0x08)  return_data |= 0x10;            //MUR ?
  if(room_data_buf[40] & 0x40)  return_data |= 0x20;            //Door ?
  if(room_data_buf[40] & 0x10)  return_data |= 0x40;            //Windows open ?
  if(room_data_buf[41] & 0x01)  return_data |= 0x80;            //예냉/예열 가동 여부
  return(return_data);
}

/* UART1 data send check routine */
void UART1_tx_check(void)
{
  if(f_uart1_data_send){
	  if(USART_GetFlagStatus(USART1, USART_FLAG_TC) !=RESET) {
      if(uart1_tx_length < uart1_tx_backup){
        USART1->DR = (uart1_tx_data_buf[uart1_tx_length++] & (u16)0x01FF);
      }
      else{
        P_UART1_DIR = 0;
        f_uart1_data_send = 0;
      }
    }
  }
}

//=======================================================================
/* UART1_initial	:								              */												
//=======================================================================
void UART1_initial(void){
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

  /* RS485 UART1 Direction port define */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Enable GPIOA and USART1 clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
  
  /* Configure USART1 Tx (PA9) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

/* USART1 configuration ------------------------------------------------------*/
  /* USART1 configured as follow:
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
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_InitStructure.USART_Clock = USART_Clock_Disable;
//  USART_InitStructure.USART_CPOL = USART_CPOL_Low;
//  USART_InitStructure.USART_CPHA = USART_CPHA_2Edge;
//  USART_InitStructure.USART_LastBit = USART_LastBit_Disable;

  /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

/* Enable the USART Transmoit interrupt: this interrupt is generated when the 
   USART1 transmit data register is empty */  
//  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

/* Enable the USART Receive interrupt: this interrupt is generated when the 
   USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);

  /* Enable the USART1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


//=======================================================================
u8 uart1ctrl =15;
void UART1_control_proc(void) {
//u8 ix; 
  unsigned char crc_buf = 0;
  unsigned char i;


  if(f_uart1_frame_rx_ok){
    f_uart1_frame_rx_ok = 0;
    for(i=0; i<uart1_rxd_buf[1]-2; ++i) crc_buf += uart1_rxd_buf[i];
    crc_buf ^= 0x55;
    if(crc_buf == (uart1_rxd_buf[uart1_rxd_buf[1]-2])){
      if(main_id_buf == uart1_rxd_buf[2]){
        main_rxd_check();
        led2 ^= 1;
        UART1_data_set();
      }
      else{
        if(uart1_rxd_buf[2] == 0xff){
          main_rxd_check();
        }
      }

    }
  }

	uart1_frame_proc();

}


void main_rxd_check(void)
{
  unsigned char temp_index;
  
  switch(uart1_rxd_buf[3]){
    case 'G':   //Basic data & Group o
      if((uart1_rxd_buf[4] & 0x01) != (room_data_buf[40] & 0x01)){
        if((room_data_buf[40] & 0x02) == 0){
          room_data_buf[16] |= 0x24;
          room_data_buf[17] |= 0x24;
          room_data_buf[18] |= 0x24;
          room_data_buf[19] |= 0x24;
          
          room_data_buf[40] &= 0xfb;  //DND clear
          
          if((room_data_buf[85] != 0 ) && (room_data_buf[85] != 0xff)){   //기준온도 대기 //h@20140711
            ts_air_set_temp_from_PC[0] = room_data_buf[85];
            ts_air_set_temp_from_PC[1] = room_data_buf[85];
            ts_air_set_temp_from_PC[2] = room_data_buf[85];
            ts_air_set_temp_from_PC[3] = room_data_buf[85];
          }
        
          if((room_data_buf[86] != 0 ) && (room_data_buf[86] != 0xff)){   //기준온도 바닥 //h@20140711
            ts_floor_set_temp_from_PC[0] = room_data_buf[86];
            ts_floor_set_temp_from_PC[1] = room_data_buf[86];
            ts_floor_set_temp_from_PC[2] = room_data_buf[86];
            ts_floor_set_temp_from_PC[3] = room_data_buf[86];
          }
         
        }       
      }
      
      if(uart1_rxd_buf[4] & 0x01) room_data_buf[40] |= 0x01;  //check IN
      else room_data_buf[40] &= 0xfe;                         //check OUT
      if(uart1_rxd_buf[4] & 0x02) room_data_buf[40] |= 0x20;  //Message ON
      else room_data_buf[40] &= 0xdf;                         //Message OFF
      //if(uart1_rxd_buf[4] & 0x04) room_data_buf[41] |= 0x02;  //심야제어 ON
      //else room_data_buf[41] &= 0xfd;                         //심야제어 OFF
      room_data_buf[41] &= 0xfd;                         //심야제어 OFF
      if(uart1_rxd_buf[4] & 0x08) room_data_buf[41] |= 0x01;  //예냉.예열 ON
      else room_data_buf[41] &= 0xfe;                         //예냉.예열 OFF
      room_data_buf[74] = ((uart1_rxd_buf[4] & 0x70) >> 4);   //냉.난.환절기 모드
      
      main_comm_rx_ok = 1;
      
      if(room_data_buf[74] == 0){
        room_data_buf[20] &= 0xfb;    //난방
        room_data_buf[21] &= 0xfb;    //난방
        room_data_buf[22] &= 0xfb;    //난방
        room_data_buf[23] &= 0xfb;    //난방
      }
      else if(room_data_buf[74] == 1){
        room_data_buf[20] |= 0x04;    //냉방
        room_data_buf[21] |= 0x04;    //냉방
        room_data_buf[22] |= 0x04;    //냉방
        room_data_buf[23] |= 0x04;    //냉방
      }
      //else if(room_data_buf[74] == 2){  //환절기 모드
      
      if((uart1_rxd_buf[5] <= 99) || (uart1_rxd_buf[5] == 0xff)){
        if(uart1_rxd_buf[5] == 0) room_data_buf[81] = 0xff;
        else room_data_buf[81] = uart1_rxd_buf[5];                  //Group No
      }
      break;
      
    case 'C':   //Basic data
      if(uart1_rxd_buf[4] & 0x01) room_data_buf[40] |= 0x01;  //check IN
      else room_data_buf[40] &= 0xfe;                         //check OUT
      if(uart1_rxd_buf[4] & 0x02) room_data_buf[40] |= 0x20;  //Message ON
      else room_data_buf[40] &= 0xdf;                         //Message OFF
      //if(uart1_rxd_buf[4] & 0x04) room_data_buf[41] |= 0x02;  //심야제어 ON
      //else room_data_buf[41] &= 0xfd;                         //심야제어 OFF
      room_data_buf[41] &= 0xfd;                         //심야제어 OFF
      if(uart1_rxd_buf[4] & 0x08) room_data_buf[41] |= 0x01;  //예냉.예열 ON
      else room_data_buf[41] &= 0xfe;                         //예냉.예열 OFF
      room_data_buf[74] = ((uart1_rxd_buf[4] & 0x70) >> 4);   //냉.난.환절기 모드
      main_comm_rx_ok = 1;

      if(room_data_buf[74] == 0){
        room_data_buf[20] &= 0xfb;    //난방
        room_data_buf[21] &= 0xfb;    //난방
        room_data_buf[22] &= 0xfb;    //난방
        room_data_buf[23] &= 0xfb;    //난방
      }
      else if(room_data_buf[74] == 1){
        room_data_buf[20] |= 0x04;    //냉방
        room_data_buf[21] |= 0x04;    //냉방
        room_data_buf[22] |= 0x04;    //냉방
        room_data_buf[23] |= 0x04;    //냉방
      }
      //else if(room_data_buf[74] == 2){  //환절기 모드
      //}
      break;
      
    case 'T':   //대기 설정온도 개별 변경
      temp_index = 5;
      if(uart1_rxd_buf[4] & 0x01){
        ts_air_set_temp_from_PC[0] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[0] = 0;
      }
      if(uart1_rxd_buf[4] & 0x02){
        ts_air_set_temp_from_PC[1] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[1] = 0;
      }
      if(uart1_rxd_buf[4] & 0x04){
        ts_air_set_temp_from_PC[2] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[2] = 0;
      }
      if(uart1_rxd_buf[4] & 0x08){
        ts_air_set_temp_from_PC[3] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[3] = 0;
      }
      break;
    case 't':   //대기 설정온도 전체 변경
       ts_air_set_temp_from_PC[0] = uart1_rxd_buf[5];
       ts_air_set_temp_from_PC[1] = uart1_rxd_buf[6];
       ts_air_set_temp_from_PC[2] = uart1_rxd_buf[7];
       ts_air_set_temp_from_PC[3] = uart1_rxd_buf[8];
       ts_set_temp_send_count[0] = 0;
       ts_set_temp_send_count[1] = 0;
       ts_set_temp_send_count[2] = 0;
       ts_set_temp_send_count[3] = 0;
      break;
    case 'F':   //바닥 설정온도 개별 변경
      temp_index = 5;
      if(uart1_rxd_buf[4] & 0x01){
        ts_floor_set_temp_from_PC[0] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[0] = 0;
      }
      if(uart1_rxd_buf[4] & 0x02){
        ts_floor_set_temp_from_PC[1] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[1] = 0;
      }
      if(uart1_rxd_buf[4] & 0x04){
        ts_floor_set_temp_from_PC[2] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[2] = 0;
      }
      if(uart1_rxd_buf[4] & 0x08){
        ts_floor_set_temp_from_PC[3] = uart1_rxd_buf[temp_index++];
        ts_set_temp_send_count[3] = 0;
      }
      break;
    case 'f':   //바닥 설정온도 전체 변경
       ts_floor_set_temp_from_PC[0] = uart1_rxd_buf[5];
       ts_floor_set_temp_from_PC[1] = uart1_rxd_buf[6];
       ts_floor_set_temp_from_PC[2] = uart1_rxd_buf[7];
       ts_floor_set_temp_from_PC[3] = uart1_rxd_buf[8];
       ts_set_temp_send_count[0] = 0;
       ts_set_temp_send_count[1] = 0;
       ts_set_temp_send_count[2] = 0;
       ts_set_temp_send_count[3] = 0;
      break;
    case 'O':   //온도센서 ON/OFF 원격 제어
      if(uart1_rxd_buf[4] & 0x01){
        if(uart1_rxd_buf[5] & 0x01) room_data_buf[16] |= 0x04;  
        else room_data_buf[16] &= 0xfb;  
        room_data_buf[16] |= 0x80;  
      }
      if(uart1_rxd_buf[4] & 0x02){
        if(uart1_rxd_buf[5] & 0x02) room_data_buf[17] |= 0x04;  
        else room_data_buf[17] &= 0xfb;  
        room_data_buf[17] |= 0x80; 
      }
      if(uart1_rxd_buf[4] & 0x04){
        if(uart1_rxd_buf[5] & 0x04) room_data_buf[18] |= 0x04;  
        else room_data_buf[18] &= 0xfb;  
        room_data_buf[18] |= 0x80;  
      }
      if(uart1_rxd_buf[4] & 0x08){
        if(uart1_rxd_buf[5] & 0x08) room_data_buf[19] |= 0x04;  
        else room_data_buf[19] &= 0xfb;  
        room_data_buf[19] |= 0x80; 
      }
      
      if(uart1_rxd_buf[6] & 0x01){
        if(uart1_rxd_buf[7] & 0x01) room_data_buf[16] |= 0x20;  
        else room_data_buf[16] &= 0xdf;  
        room_data_buf[20] |= 0x80;
      }
      if(uart1_rxd_buf[6] & 0x02){
        if(uart1_rxd_buf[7] & 0x02) room_data_buf[17] |= 0x20;  
        else room_data_buf[17] &= 0xdf;  
        room_data_buf[21] |= 0x80;
      }
      if(uart1_rxd_buf[6] & 0x04){
        if(uart1_rxd_buf[7] & 0x04) room_data_buf[18] |= 0x20;  
        else room_data_buf[18] &= 0xdf;  
        room_data_buf[22] |= 0x80;
      }
      if(uart1_rxd_buf[6] & 0x08){
        if(uart1_rxd_buf[7] & 0x08) room_data_buf[19] |= 0x20;  
        else room_data_buf[19] &= 0xdf;  
        room_data_buf[23] |= 0x80;
      }      
      
      room_data_buf[99] |= 0x02;
      room_data_buf[99] |= 0x04;
      break;
      
    case 'P':   //공통 Data
      room_data_buf[70] = (uart1_rxd_buf[4] & 0x0f) * 10; //외출온도차
      room_data_buf[75] = (uart1_rxd_buf[4] & 0xf0) >> 4; //냉.난 전환온도차
      room_data_buf[76] = uart1_rxd_buf[5];               //이상경보온도
      if(room_data_buf[81] == 0xff){
        if(uart1_rxd_buf[6] != 0xff) room_data_buf[71] = uart1_rxd_buf[6];               //대기난방 공실기준온도
        if(uart1_rxd_buf[7] != 0xff) room_data_buf[72] = uart1_rxd_buf[7];                //대기냉방 공실기준온도
        if(uart1_rxd_buf[8] != 0xff) room_data_buf[73] = uart1_rxd_buf[8];               //바닥 공실기준온도
      }
      room_data_buf[28] = uart1_rxd_buf[9];               //대기온도센서 상.하한온도
      room_data_buf[29] = uart1_rxd_buf[10];              //바닥온도센서 상.하한온도
      if(uart1_rxd_buf[11] <= 0x99) room_data_buf[90] = uart1_rxd_buf[11];              //년
      if(uart1_rxd_buf[12] <= 0x12) room_data_buf[91] = uart1_rxd_buf[12];              //월
      if(uart1_rxd_buf[13] <= 0x31) room_data_buf[92] = uart1_rxd_buf[13];              //일
      if(uart1_rxd_buf[14] <= 0x23) room_data_buf[93] = uart1_rxd_buf[14];              //시
      if(uart1_rxd_buf[15] <= 0x59) room_data_buf[94] = uart1_rxd_buf[15];              //분
      room_data_buf[77] = uart1_rxd_buf[16];              //화장실 유지시간 H
      room_data_buf[78] = uart1_rxd_buf[17];              //화장실 유지시간 L
      entry_lamp_delay_time = (room_data_buf[77] * 0x100) + room_data_buf[78];
      if((uart1_rxd_buf[18] != 0) && (uart1_rxd_buf[18] != 0xff)){
        room_data_buf[82] &= 0x08;
        room_data_buf[82] |= (uart1_rxd_buf[18] & 0xf7);            //조도 조절법
      }
      if((uart1_rxd_buf[19] == '0') || (uart1_rxd_buf[19] == '1')){   //온도센서 온도표시방법
        if(uart1_rxd_buf[19] == '0')  room_data_buf[41] &= 0xf7;  //현재온도표시
        else room_data_buf[41] |= 0x08;  //설정온도표시
        f_temp_dsp_kind_rx = 1;
      }
      
      if((uart1_rxd_buf[20] != 0) && (uart1_rxd_buf[20] != 0xff)){
        room_data_buf[85] = b_decimal_to_hex_table[uart1_rxd_buf[20]];    //기준온도 대기   //h@20140711
      }
      if((uart1_rxd_buf[21] != 0) && (uart1_rxd_buf[21] != 0xff)){
        room_data_buf[86] = b_decimal_to_hex_table[uart1_rxd_buf[21]];    //기준온도 바닥   //h@20140711
      }
      
      break;
      
    case 'I':   //Chime & alram volume
      room_data_buf[79] = uart1_rxd_buf[4];
      room_data_buf[80] = uart1_rxd_buf[5];
      break;
      
    case 'i':   //Group data(room_data_buf[81] == Group 일때만 적용한다.)
      if(uart1_rxd_buf[4] == room_data_buf[81])
      {
        if(uart1_rxd_buf[5] != 0xff) room_data_buf[72] = uart1_rxd_buf[5];               //대기냉방 공실기준온도
        if(uart1_rxd_buf[6] != 0xff) room_data_buf[71] = uart1_rxd_buf[6];               //대기난방 공실기준온도
        if(uart1_rxd_buf[7] != 0xff) room_data_buf[73] = uart1_rxd_buf[7];               //바닥 공실기준온도
      }
      break;

    case '$':   //보정온도                                                //h@20140704 =>
      if((uart1_rxd_buf[4] & 0x0f) <= 0x04){
        if(uart1_rxd_buf[5] != 0xff){
          bojung_temp[uart1_rxd_buf[4] & 0x0f] = uart1_rxd_buf[5];
        }
      }
      break;                                                              //<= h@20140704
      
    case 'R':   //All data 전송
      room_data_buf[99] = 0x07;
      break;
    
    
    case 'n':   //DND On-line control
      if(uart1_rxd_buf[4] == '1') dnd_mur_execution('D','N');
      else if(uart1_rxd_buf[4] == '0') dnd_mur_execution('N','F');
      break;
      
    case 'o':   //MUR On-line control
      if(uart1_rxd_buf[4] == '1') dnd_mur_execution('M','N');
      else if(uart1_rxd_buf[4] == '0') dnd_mur_execution('N','F');
      break;      
  }

}

//=======================================================================
//=======================================================================

