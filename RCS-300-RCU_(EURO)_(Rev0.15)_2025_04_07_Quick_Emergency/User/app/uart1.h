/*=====================(C) COPYRIGHT 2008 Insem Inc.=========================
program 		: HDPVR FRONT PANEL 
processor	: STM32F101xx
compiler		: IAR 4.41A Compiler 
program BY 	: YN.Kim
date 			: 2008.	  .
copy right 	: Insem Inc.
=======================================================================*/
void UART1_control_proc(void);
void UART1_initial(void);

u8 u1_byteload(void);
void __lineclr(u8 len);
void screen_clear(void);
void __printf(char *str);
void __xcprintf(const char *str);
void _xdprintf(char *str);
void __xdprintf(char *str);
void factory_initial(void);
void hyper_clear(void);
void __linefeed(void);
void UART1_tx_check(void);
void UART1_data_set(void);
unsigned char cb_basic_data(unsigned char index);
void cb_data_send(unsigned char send_cmd);
unsigned char air_temp_control_status(unsigned char index);
unsigned char floor_temp_control_status(unsigned char index);
void main_rxd_check(void);



//=======================================================================

#define P_UART1_DIR		  GpioA->Bit.b8

//=======================================================================

typedef enum {
	black =0x30,
	red,
	green,
	yellow,
	blue,
	magenta,
	cyan,
	white
}_foreground;

typedef enum {
	_black =0x40,
	_red,
	_green,
	_yellow,
	_blue,
	_magenta,
	_cyan,
	_white
}_background;

typedef enum {
	NUL,
	SOH,
	STX,
	ETX,
	EOT,
	ENQ,
	ACK,
	BEL,
	BS,
	TAB,
	LF,
	VT,
	FF,
	CR,
	SO,
	SI,
	DLE,
	DC1,
	DC2,
	DC3,
	DC4,
	NAK,
	SYN,
	ETB,
	XCAN,
	EM,
	SUB,
	ESC,
	FS,
	GS,
	RS,
	US,
	SPACE,
}__control_code;

//=======================================================================
typedef struct {
	u16 ldad;
	u16 svad;
	u8 buf[64];
}__ring;

typedef struct {
	u16 ldad;
	u16 svad;
	u8 buf[4*1024];
}__trx;

typedef struct {
	u8 slen;
	u8 msel;
	u8 menumax;
	u8 input[20];
}__zin;

typedef enum {
	_password =0x31,
	_beep,
	_debug,
	_factory_initial,
	_ethernet_config,
	
}_select;

//=======================================================================

#ifdef __UART1_H__
#define u1EX 
u1EX char xdebug[256];

const char* menu[] = {
	"1. Password change\r\n",
	"2. Beep sound\r\n",
	"3. Debug mode\r\n",
	"4. Factory Initial\r\n",
	"5. Ethernet configuration\r\n",

	".",			// end point
};

const u8 selno[] = {
	_password,
	_beep,
	_debug,
	_factory_initial,
	_ethernet_config,
};
extern u16 wdtcnt;


const char* ethernet_menu[] = {
	"1. Server IP address       ",
	"2. Server port number      ",
	"3. IP configuration mathod ",
	"4. Local IP                ",
	"5. Subnet mask             ",
	"6. Gateway                 ",
	"7. Local port number       ",
	"8. Baudrate                ",
	"9. Operation mode          ",
	" ",

	"D. Download     U. Upload     S. Save ",
	
	".",			// end point
};

const char* bps_list[] = {
	"115200",
	"115200",
	"57600 ",
	"38400 ",
	"19200 ",
	"9600  ",
	"4800  ",
	"2400  ",
	"1200  ",
};
const char* op_mode[] = {
	"Server",
	"Client",
	"",
	"",
};

#define eth_size 9
const char* eth_initial[] ={
	"211.214.213.30",
	"5000",
	"1",
	"211.214.213.31",
	"255.255.255.0",
	"211.214.213.1",
	"5000",
	"1",
	"1",
	
	".",
};

const u8 eselno[] = {
	'1','2','3','4','5','6','7','8','9','a','b','c',
};







const u8 b_hex_to_decimal_table[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
  0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31,
  0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
  0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63,
  0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
  0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95,
  0x96, 0x97, 0x98, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99 
};

const u8 b_decimal_to_hex_table[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
  0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee
};


const u16 hextbl1[] =    {0, 1,     2,     3,     4,     5,      6,      7,      8,      9,     0x0f,  0x0f,  0x0f,  0x0f,  0x0f,  0x0f};
const u16 hextbl10[] =   {0, 0x0a,  0x14,  0x1e,  0x28,  0x32,   0x3c,   0x46,   0x50,   0x5a,  0xf0,  0xf0,  0xf0,  0xf0,  0xf0,  0xf0};
const u16 hextbl100[] =  {0, 0x64,  0xc8,  0x12c, 0x190, 0x1f4,  0x258,  0x2bc,  0x320,  0x384, 0xf00, 0xf00, 0xf00, 0xf00, 0xf00, 0xf00};
const u16 hextbl1000[] = {0, 0x3e8, 0x7d0, 0xbb8, 0xfa0, 0x1388, 0x1770, 0x1b58, 0x1f40, 0x2328,0xf000,0xf000,0xf000,0xf000,0xf000,0xf000};














#else
#define u1EX extern
u1EX char xdebug[256];

#endif

u1EX __trx dtx;
u1EX __ring ring;
u1EX u8 spdata, rcnt, rdinx;
u1EX __zin zin;
u1EX u8 update[20][20];
u1EX u8 ethcnt, bk_ethcnt ;

u1EX u8 uart1_tx_length;
u1EX u8 uart1_tx_backup;
u1EX u8 uart1_tx_data_buf[100];

u1EX u8 uart1_rxding_point;
u1EX u8 uart1_rxding_buf[100];
u1EX u8 uart1_rxd_buf[100];
u1EX u8 ts_mode_set;
u1EX u8 tnt_page_mode[10];
u1EX u8 tnt_toggle_bit[][20][20];
u1EX u8 language_code;
u1EX u8 dimmer_apply_bit;
u1EX u8 scene_code;
u1EX u8 tnt_toggle_power_on[10];
u1EX u8 dim_apply_count;
u1EX u8 scene_mode[2];
u1EX u8 tnt_chime_send_buf;
u1EX u8 dimmer_level[16];
u1EX u8 dimmer_step_level[16];
u1EX u8 audio_volume[4];

u1EX u8 chime_ind_toggle_bit[4];
u1EX u8 f_first_ci_power_on[4];

u1EX u8 language_code;
u1EX u8 dimmer_apply_bit;
u1EX u8 scene_code;
u1EX u8 dim_apply_count;
u1EX u8 tnt_chime_send_buf;

u1EX u8 tnt_heartbeat;

u1EX u8 tnt_page_mode_buf;
//=======================================================================
u1EX __dwbits uart1;
#define fsend_f			uart1.s.b0
#define u1_init_f			uart1.s.b1
#define firstdp_f 		uart1.s.b3
#define Wdtdis_f			uart1.s.b4
#define eth_download_f 	uart1.s.b5
#define eth_upload_f 	uart1.s.b6
#define complete_f 	uart1.s.b7
#define esc_f 	uart1.s.b8

u1EX __dwbits flag1;
#define f_uart1_data_send			flag1.s.b0
#define f_uart1_com_start			flag1.s.b1
#define f_uart1_frame_rx_ok		flag1.s.b2

u1EX __dwbits flag2;
#define main_comm_rx_ok 			flag2.s.b0
#define f_temp_dsp_kind_rx		        flag2.s.b1
#define f_chime_send                            uart2_flag2.s.b2
#define f_language_rx                           uart2_flag2.s.b3
//=======================================================================

//=======================================================================

