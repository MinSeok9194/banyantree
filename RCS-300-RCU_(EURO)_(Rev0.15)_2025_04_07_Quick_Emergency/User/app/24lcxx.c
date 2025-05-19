/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program		  : HDPVR FRONT PANEL 
processor 	  : STM32F101xx
compiler		  : IAR 4.41A Compiler 
program BY	  : YN.Kim
date			  : 2008.	 .
copy right	  : Insem Inc.
============================================================================*/
#define __24CXX_H__

#include "stm32f10x_lib.h"
#include "24lcxx.h"

#ifdef I2C_INTERRUPT_USE
//=======================================================================
/* M24LCXX_initial	:		        						      				   */												
//=======================================================================
void M24LCXX_initial(void){

	I2C_DeInit(I2C1);  

	/* Enable the i2c1 Pins Partial Software Remapping */
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);  

	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO, ENABLE);  
	/* I2C1 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = Slave_Address;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = i2c_clk_speed;       

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);   

	/* Enable I2C1 event and buffer interrupts */
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);  

	/* Configure and enable I2C1 event interrupt -------------------------------*/
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
  /* Configure and enable I2C1 error interrupt -------------------------------*/  
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);
}

void i2c_pagewrite (u8 slave_addr, u16 StartAddress, u8* Data, u16 len) {

	Direction = Transmitter;
	i2c1.i2c_rw =0;
	i2c1.slave_address =Slave_Address;
	i2c1.start_address =StartAddress;
	i2c1.i2cbuf =Data;
	i2c1.len =len;
	i2c1.flag.s.busy =1;

	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);  
	I2C_GenerateSTART(I2C1, ENABLE);
}

u8 i2c_pageread(u8 slave_addr, u16 StartAddress, u8* Data, u16 len) {

	if(i2c1.flag.s.stby ==0) { i2c1.flag.s.stby =1;
		Direction = Transmitter;
		i2c1.i2c_rw =1;
		i2c1.slave_address =Slave_Address;
		i2c1.start_address =StartAddress;
		i2c1.i2cbuf =Data;
		i2c1.len =len;
		i2c1.flag.s.busy =1;
		
		I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
		I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);  
		I2C_GenerateSTART(I2C1, ENABLE);

		return 0;
	}
	else {
		if(i2c1.flag.s.busy) return 0;
		else { i2c1.flag.s.stby =0;
			return 1;
		}
	}			
}

#else
//=======================================================================
/* M24LCXX_ByteWrite	:									   */												
//=======================================================================
void i2c_bytewrite (u8 slave_addr, s32 StartAddress, u8 Data) {
u8 ix;
u16 err_cnt = 0;

	M24LCXX_initial();			// iic initial

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);  

	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){ 
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }

	if(StartAddress !=-1) {
		/* Send EEPROM address for write */
		I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Transmitter);

			/* Test on EV6 and clear it */
//		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    err_cnt = 0;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
      if(++err_cnt > 100){
        eep_err_status = 'E';
        break;
      }
    }  

#if defined(ADDRESS_2BYTE)
		/* Send the EEPROM's internal address to write to */
		I2C_SendData(I2C1, (u8)(StartAddress>>8));
		/* Test on EV8 and clear it */
//		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    err_cnt = 0;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      if(++err_cnt > 100){
        eep_err_status = 'E';
        break;
      }
    }  
#endif

		/* Send the EEPROM's internal address to write to */
		I2C_SendData(I2C1, (u8)StartAddress);
		/* Test on EV8 and clear it */
//		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    err_cnt = 0;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
      if(++err_cnt > 100){
        eep_err_status = 'E';
        break;
      }
    }  
	}
	else {
		/* Send EEPROM address for write */
		I2C_Send7bitAddress(I2C1, slave_addr , I2C_Direction_Transmitter);

			/* Test on EV6 and clear it */
//		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    err_cnt = 0;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
      if(++err_cnt > 100){
        eep_err_status = 'E';
        break;
      }
    }  
	}
	
	/* Send the byte to be written */
	I2C_SendData(I2C1, Data); 

	/* Test on EV8 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }  

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	for(ix=0; ix<50; ix++); 			// must be delay

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, DISABLE);  
}

//=======================================================================
/* M24LCXX_PageWrite	:									   */												
//=======================================================================
u8 i2c_pagewrite (u8 slave_addr, s32 StartAddress, u8* Data, u16 len) {
u16 ix;

	M24LCXX_initial();			// iic initial

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);  

	/* Send STRAT condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  

	if(StartAddress !=-1) {
		/* Send EEPROM address for write */
		I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Transmitter);

		/* Test on EV6 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
		  
#if defined(ADDRESS_2BYTE)
		/* Send the EEPROM's internal address to write to */
		I2C_SendData(I2C1, (u8)(StartAddress>>8));
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
#endif

		/* Send the EEPROM's internal address to write to */
		I2C_SendData(I2C1, (u8)StartAddress);
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}
	else {
		/* Send EEPROM address for write */
		I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);

		/* Test on EV6 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	
	
	for(ix=0; ix<len; ix++){
		/* Send the byte to be written */
		I2C_SendData(I2C1, Data[ix]); 
		
		/* Test on EV8 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	for(ix=0; ix<50; ix++); 			// must be delay

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, DISABLE);  
   
   return 0;
}

//=======================================================================
/* M24LCXX_ByteRead	:									   */												
//=======================================================================
u8 i2c_byteread(u8 slave_addr, u32 StartAddress) {
u8 Data;
u16 ix;
u16 err_cnt = 0;

  eep_err_status = 'O';
	M24LCXX_initial();			// iic initial

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }  

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

#if defined(ADDRESS_2BYTE)
	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2C1, (u8)(StartAddress>>8));  
	/* Test on EV8 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }
#endif

	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2C1, (u8)StartAddress);  
	/* Test on EV8 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }

	/* Send EEPROM address for read */
	I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }
   

	I2C_AcknowledgeConfig(I2C1, DISABLE);  

//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));		 
  err_cnt = 0;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
    if(++err_cnt > 100){
      eep_err_status = 'E';
      break;
    }
  }
  
	Data =I2C_ReceiveData(I2C1);

	/* Send STOP Condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	for(ix=0; ix<50; ix++);			 // must be delay

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, DISABLE);

	return Data;      
}

//=======================================================================
/* M24LCXX_PageRead	:									   */												
//=======================================================================
u8 i2c_pageread(u8 slave_addr, u32 StartAddress, u8* Data, u16 len) {
u16 ix;

	M24LCXX_initial();			// iic initial

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for write */
	I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

#if defined(ADDRESS_2BYTE)
	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2C1, (u8)(StartAddress>>8));  
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
#endif

	/* Send the EEPROM's internal address to write to */
	I2C_SendData(I2C1, (u8)StartAddress);  
	/* Test on EV8 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	/* Send STRAT condition a second time */  
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send EEPROM address for read */
	I2C_Send7bitAddress(I2C1, slave_addr |(u8)(StartAddress>>15), I2C_Direction_Receiver);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	I2C_AcknowledgeConfig(I2C1, ENABLE);  

	for(ix=0; ix<len; ix++) {
		if(ix==(len-1)) I2C_AcknowledgeConfig (I2C1, DISABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));		 
		Data[ix] =I2C_ReceiveData(I2C1);
	}

	/* Send STOP Condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	for(ix=0; ix<50; ix++);			 // must be delay

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, DISABLE);

	return 1;      
}

//=======================================================================
/* M24LCXX_initial	:		        						      */												
//=======================================================================
void M24LCXX_initial(void) {
GPIO_InitTypeDef GPIO_InitStructure;
I2C_InitTypeDef  I2C_InitStructure; 

	I2C_DeInit(I2C1);  

	/* Enable the i2c1 Pins Partial Software Remapping */
//	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);  

	/* Enable GPIOB clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);  
	/* I2C1 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* Configure I2C1 pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = Slave_Address;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = i2c_clk_speed;       

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure); 
	
}
#endif
//=======================================================================
//=======================================================================



