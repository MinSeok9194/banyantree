/*============================(C) COPYRIGHT 2008 Insem Inc.=========================
program		  : HDPVR FRONT PANEL 
processor	  : STM32F101xx
compiler 	  : IAR 4.41A Compiler 
program BY	  : YN.Kim
date			  : 2008.	 .
copy right	  : Insem Inc.
============================================================================*/
#ifndef __24CXX_H__x
#define __24CXX_H__x

//=======================================================================
#define EEP_24LC02_USE
//#define EEP_24LC32_USE

//=======================================================================
#define i2c_clk_speed 	350000
#define Slave_Address 	0xA0						// slave address
#define Transmitter     0x00
#define Receiver        0x01

//=======================================================================
#if defined (EEP_24LC02_USE)
	#define ADDRESS_1BYTE
	#define Full_Address			0x0100
	#define eeprom_pagelength	8 

#elif defined (EEP_24LC32_USE)
	#define ADDRESS_2BYTE
	#define Full_Address			0x1000
	#define eeprom_pagelength	8
#else
  #error Unknow Device
#endif

#if defined(ADDRESS_1BYTE)
typedef struct {
	u8 slave_address;
	u8 start_address;
	u8 i2c_rw;
	u8 *i2cbuf;
	u16 len;
	union {
		u8 byte;
		struct {
			unsigned busy	:1;
			unsigned stby	:1;
		}s;
	}flag;
}__i2c1;

#elif defined(ADDRESS_2BYTE)
typedef struct {
	u8 slave_address;
	u16 start_address;
	u8 i2c_rw;
	u8 *i2cbuf;
	u16 len;
	union {
		u8 byte;
		struct {
			unsigned busy	:1;
			unsigned stby	:1;
		}s;
	}flag;
}__i2c1;

#else
  #error Address Range Undifine
#endif

//=======================================================================
//=======================================================================
//=======================================================================
//=======================================================================
#ifdef I2C_INTERRUPT_USE
void M24LCXX_initial(void);
void i2c_pagewrite (u8 slave_addr, u32StartAddress, u8* Data, u16 len);
u8 i2c_pageread(u8 slave_addr, u32 StartAddress, u8* Data, u16 len);

//=======================================================================
#ifdef __24CXX_H__
#define iiEX

iiEX vu8 Direction = Transmitter;

#else
#define iiEX extern 
iiEX vu8 Direction;
#endif

iiEX u8 addr_len;
iiEX __i2c1 i2c1;
iiEX vu16 i2c_txinx, i2c_rxinx;


////----////
#else
void i2c_bytewrite (u8 slave_addr, s32 StartAddress, u8 Data);
u8 i2c_byteread(u8 slave_addr, u32 StartAddress);
u8 i2c_pagewrite (u8 slave_addr, s32 StartAddress, u8* Data, u16 len);
u8 i2c_pageread(u8 slave_addr, u32 StartAddress, u8* Data, u16 len);
void M24LCXX_initial(void);

//=======================================================================
#ifdef __24CXX_H__
#define iiEX
#else
#define iiEX extern 
#endif

#endif
#endif
iiEX u8 eep_err_status;

//=======================================================================


//=======================================================================
//=======================================================================

