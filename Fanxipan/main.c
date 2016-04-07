//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************
/* USER ...*/
#include "defines.h"
#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_watchdog.h"
#include "attributes.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_ethernet.h"
#include "tm_stm32f4_exti.h"
#include "tm_stm32f4_fatfs.h"
#include "tm_stm32f4_rtc.h"
#include "stm32f4xx_rcc.h"
#include "tm_stm32f4_id.h"
#include "tm_stm32f4_bkpsram.h"
#include "tcp_echoclient.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

//******************************************************************************
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "croutine.h"
//******************************************************************************

void vConfigure(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vEthernet(void *pvParameters);
void vMain(void *pvParameters);
static void printf_d(char *str);

/* Fatfs object */
FATFS FatFs;
/* File object */
FIL fil;
FRESULT fres;
DSTATUS disk_initialize (BYTE drv);
char ACM[8];

//extern uint8_t tcp_active_connections;
/*Khoi LCD*/
	uint8_t customChar[] = {
		0x00,	/*  xxx 11111 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00,	/*  xxx 10001 */
		0x00	/*  xxx 11111 */
	};
unsigned char flag_LCD = 0;
char buffer_lcd[32];				//ghi du lieu de xuat ra lcd
char buffer_dtmf[32];				//ghi tam du lieu dtmf
char str_dtmf[32];					//thanh ghi luu du lieu dtmf 
unsigned char add_str_dtmf=0;// cho phep ghi du lieu dtmf vao thanh ghi
unsigned char lcd_menu		=0;// giao dien setup chinh
unsigned char lcd_menu10	=0;// giao dien setup readcar
unsigned char lcd_menu11	=0;
unsigned char lcd_menu12	=0;
unsigned char lcd_menu13	=0;
	
unsigned char lcd_menu20	=0;// giao dien setup TCPIP
unsigned char lcd_menu21	=0;
unsigned char lcd_menu22	=0;
unsigned char lcd_menu23	=0;
unsigned char lcd_menu24	=0;
	
unsigned char lcd_menu30	=0;// giao dien setup relay
unsigned char lcd_menu31	=0;
unsigned char lcd_menu32	=0;
unsigned char lcd_menu33	=0;
unsigned char lcd_menu34	=0;

unsigned char lcd_menu40	=0;// giao dien showsetup
unsigned char lcd_menu41	=0;
unsigned char lcd_menu42	=0;
unsigned char lcd_menu43	=0;

unsigned char lcd_menu50	=0;// giao dien lua chon thiet bi
unsigned char lcd_menu51	=0;
unsigned char lcd_menu52	=0;
unsigned char lcd_menu53	=0;
/*Khoi COM*/
char 					str[64];			// dung cho rs232
char 					BufferCom1[64];	//com1
char 					BufferCom2[64];	//com2
char 					BufferCom3[64];	//com3
/*Khoi Rtc*/
TM_RTC_Time_t datatime;				// cau truc thoi gian
static void 	updatime(uint8_t h,uint8_t m,uint8_t s,uint8_t d,uint8_t mon,uint8_t year);
unsigned char fag_updatime=0;	// cho phep update theo chu ki
static void 	Sttup(char * buffer);	//tach du lieu gui ve truong stt
static void 	Timeup(char * buffer);//tach du lieu gui ve truong time
/*Khoi SW*/
void 					read_sw_add(void);		//doc gia tri switch 8 bit luu bien
unsigned char value_dip =0; /* value DIP switch*/
/*Khoi Ethernet*/
char 					data_tcp[128];		//thanh ghi luu du lieu de truyen di
uint16_t 			requests_count = 1;
uint8_t 			car_read=0;				//So lan gui di khong nhan lon hon 3 autoreset IC
uint8_t 				macaddress[64][6]	=	{ // Co dinh
															{0x00,0xE0,0x69,0xF0,0x00,0x01}, 	//1
															{0x00,0xE0,0x69,0xF0,0x00,0x02},	
															{0x00,0xE0,0x69,0xF0,0x00,0x03},
															{0x00,0xE0,0x69,0xF0,0x00,0x04},
															{0x00,0xE0,0x69,0xF0,0x00,0x05},	//5
															{0x00,0xE0,0x69,0xF0,0x00,0x06},
															{0x00,0xE0,0x69,0xF0,0x00,0x07},
															{0x00,0xE0,0x69,0xF0,0x00,0x08},
															{0x00,0xE0,0x69,0xF0,0x00,0x09},
															{0x00,0xE0,0x69,0xF0,0x00,0x0A},	//10
															{0x00,0xE0,0x69,0xF0,0x00,0x0B},
															{0x00,0xE0,0x69,0xF0,0x00,0x0C},
															{0x00,0xE0,0x69,0xF0,0x00,0x0D},
															{0x00,0xE0,0x69,0xF0,0x00,0x0E},
															{0x00,0xE0,0x69,0xF0,0x00,0x0F},	//15
															{0x00,0xE0,0x69,0xF0,0x00,0x10},
															{0x00,0xE0,0x69,0xF0,0x00,0x11},
															{0x00,0xE0,0x69,0xF0,0x00,0x12},
															{0x00,0xE0,0x69,0xF0,0x00,0x13},
															{0x00,0xE0,0x69,0xF0,0x00,0x14},	//20
															{0x00,0xE0,0x69,0xF0,0x00,0x15},
															{0x00,0xE0,0x69,0xF0,0x00,0x16},
															{0x00,0xE0,0x69,0xF0,0x00,0x17},
															{0x00,0xE0,0x69,0xF0,0x00,0x18},	
															{0x00,0xE0,0x69,0xF0,0x00,0x19}, 	//25
															{0x00,0xE0,0x69,0xF0,0x00,0x1A},	
															{0x00,0xE0,0x69,0xF0,0x00,0x1B},
															{0x00,0xE0,0x69,0xF0,0x00,0x1C},
															{0x00,0xE0,0x69,0xF0,0x00,0x1D},	
															{0x00,0xE0,0x69,0xF0,0x00,0x1E},	//30
															{0x00,0xE0,0x69,0xF0,0x00,0x1F},
															{0x00,0xE0,0x69,0xF0,0x00,0x20},
															{0x00,0xE0,0x69,0xF0,0x00,0x21},
															{0x00,0xE0,0x69,0xF0,0x00,0x22},	
															{0x00,0xE0,0x69,0xF0,0x00,0x23},	//35
															{0x00,0xE0,0x69,0xF0,0x00,0x24},
															{0x00,0xE0,0x69,0xF0,0x00,0x25},
															{0x00,0xE0,0x69,0xF0,0x00,0x26},
															{0x00,0xE0,0x69,0xF0,0x00,0x27},	
															{0x00,0xE0,0x69,0xF0,0x00,0x28},	//40
															{0x00,0xE0,0x69,0xF0,0x00,0x29},
															{0x00,0xE0,0x69,0xF0,0x00,0x2A},
															{0x00,0xE0,0x69,0xF0,0x00,0x2B},
															{0x00,0xE0,0x69,0xF0,0x00,0x2C},	
															{0x00,0xE0,0x69,0xF0,0x00,0x2D},	//45
															{0x00,0xE0,0x69,0xF0,0x00,0x2E},
															{0x00,0xE0,0x69,0xF0,0x00,0x2F},
															{0x00,0xE0,0x69,0xF0,0x00,0x30},
															{0x00,0xE0,0x69,0xF0,0x00,0x31},	
															{0x00,0xE0,0x69,0xF0,0x00,0x32},	//50
															{0x00,0xE0,0x69,0xF0,0x00,0x33},
															{0x00,0xE0,0x69,0xF0,0x00,0x34},
															{0x00,0xE0,0x69,0xF0,0x00,0x35},	
															{0x00,0xE0,0x69,0xF0,0x00,0x36},	
															{0x00,0xE0,0x69,0xF0,0x00,0x37},	//55
															{0x00,0xE0,0x69,0xF0,0x00,0x38},
															{0x00,0xE0,0x69,0xF0,0x00,0x39},
															{0x00,0xE0,0x69,0xF0,0x00,0x3A},	
															{0x00,0xE0,0x69,0xF0,0x00,0x3B},	
															{0x00,0xE0,0x69,0xF0,0x00,0x3C},	//60
															{0x00,0xE0,0x69,0xF0,0x00,0x3D},
															{0x00,0xE0,0x69,0xF0,0x00,0x3E},
															{0x00,0xE0,0x69,0xF0,0x00,0x3F},	
															{0x00,0xE0,0x69,0xF0,0x00,0x40},	//64											
															};
uint8_t 				ipadress[4]			=	{10,24,12,101};	// khi chua cau hinh 
uint8_t 				getwayadress[4]	=	{10,24,12,1};		// khi chua cau hinh
uint8_t 				netmaskadress[4]=	{255,255,255,0};	// khi chua cau hinh
uint8_t					serveradress[4]	=	{10,24,12,11};	// khi chua cau hinh
uint16_t				portadress 			= 8866;							// khi chua cau hinh
unsigned int 		status_sv				=0;									//lenh dk tu server la 0 1 2
unsigned char 	flag_status_sv	=0;									//cho phep nhan lenh dk tu server.
unsigned char 	malenh_sv				=0;									// ma lenh 1 va 2
unsigned int 		giadichvu				=0;				
unsigned int 		luotdi					=0;
unsigned int  	sodutk					=0;
TM_TCPCLIENT_t *tcpclient;
/*Khoi process*/
unsigned char  	Process_1				=0;		// xu li day len PC nhung.
unsigned char  	Process_2				=0;		// xu li day len server.
/*Khoi RFID*/
unsigned char 	flag_RFID 			=0;	/*Send SelectCard */		
unsigned char 	flag_RFID1			=0;
unsigned char 	flag_RFID2			=0;
unsigned char 	flag_wiegand		=0;
unsigned char 	flag_wiegand_26	=0;
unsigned char 	flag_wiegand_34	=0;
char 						SelectCard[4]		={0xBA,0x02,0x01,0xB9};									// code hex dau doc the da nang
char 						Carpull[7]			={0x7F,0x40,0x01,0x20,0x0D,0xC0,0x00};	// code hex nuot the
char 						Carpush[7]			={0x7F,0x40,0x01,0x21,0x8D,0xC5,0x00};	// code hex nha the
char 						IDCAR1[7]				={0x00,0x00,0x00,0x00,0x00,0x00,0x00};	// luu gia tri IDCAR tu dau doc da nang
char 						UID1[30];
char 						UID1_save[30];
char 						IDCAR2[10];
/*Khoi Action*/
void 						ProcessAction(void);
void 						Serveprocess(void);
/*Khoi Wait*/	
void 						WaitPC(unsigned int t);
unsigned char 	flag_PC=0;		// doi pc phan hoi va xu li xong du lieu nhan veroi moi bat
unsigned char 	flag_SV=0;		// doi server phan hoi va xu li xong du lieu nhan ve roi moi bat
/*Khoi VIP*/
char *vip_id[2] = {		// ma the vip
	"FCB1BE49", //
	"CCC5C749", //
//	"00000000",	//
//	"00000000",	//
	};
/*time out*/
unsigned int 			timeout					=0;	// thoi gian active relay
/* Khoi Ban phim */
unsigned char 		express					=0;			// gia tri nhan duoc tu ban phim
unsigned char 		flag_dtmf 			=0;	
/*Khoi 485*/
int LEDStatus											=0;						// the hien trang thai cua led hien thi
unsigned char flag_485 						=0;	// xu li trong ham chinh duoc set theo timer
/*Khoi flag*/
unsigned char flag_R10 						=0;	// active relay 1
unsigned char flag_R20 						=0;	// active relay 2
unsigned char flag_R11 						=0;	// active relay 3
unsigned char flag_R21 						=0;	// active relay 4
unsigned char flag_test						=0;	// su dung khi gui lenh test tu cong com 
unsigned char flag_serve					=0;	// su dung khi nhan lenh tu server
unsigned char flag_pass						=0; // xac nhan di qua
unsigned char flag_passed					=0;	// xac nhan di qua roi
unsigned char flag_vip						=0;	// su kien vip
unsigned char flag_config					=1;	// cho phep cau hinh thiet bi
unsigned char flag_lcdset					=0;	// cho phep hien thi lcd man configure
unsigned char flag_ok							=0;	// cho phep hien thi lcd man configure
unsigned char flag_default				=0; // cho phep doc du lieu sdram
static int check_vip(char *name);

/* Khoi OUTPUT*/
static int turn_on_dk1(void);
static int turn_off_dk1(void);

static int	turn_on_dk2(void);
static int turn_off_dk2(void);

static int	turn_on_dk3(void);
static int turn_off_dk3(void);

static int	turn_on_dk4(void);
static int turn_off_dk4(void);
/*Khoi thoi gian RELAY*/
int timerdk1 =0;
int timerdk2 =0;
int timerdk3 =0;
int timerdk4 =0;

int timer_dk1 =0;
int timer_dk2 =0;
int timer_dk3 =0;
int timer_dk4 =0;	
/*Khoi timer*/
#define NUM_TIMERS 10
xTimerHandle xTimers[ NUM_TIMERS ];
long lExpireCounters[ NUM_TIMERS ] = { 0 };
void vTimerCallback1( xTimerHandle  pxTimer );
void vTimerCallback2( xTimerHandle  pxTimer );	// dau doc da nang
void vTimerCallback3( xTimerHandle  pxTimer ); 	// 34bit Weigang
void vTimerCallback4( xTimerHandle  pxTimer );	// 34bit Weigang
void vTimerCallback5( xTimerHandle  pxTimer ); 	// 26bit Weigang
void vTimerCallback6( xTimerHandle  pxTimer );	// 26bit Weigang
/*Khoi Wiegand*/ 
uint8_t 				flag_finish_1 = 1;
uint8_t 				flag_finish_2 = 1;
__IO uint64_t 	card_1 = 0;
__IO uint8_t  	count_1 = 0;
__IO uint64_t 	card_2 = 0;
__IO uint8_t  	count_2 = 0;
uint8_t 				crc_26bit(uint32_t allnum_bit,uint32_t wiegand);
uint8_t 				crc_34bit(uint32_t allnum_bit,uint64_t wiegand);

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
const signed char timer_xx[] = "Timer";
uint8_t mac_address[6];
uint8_t i;
			/* Free and total space */

//******************************************************************************
int main(void)
{	
          xTimers[1] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (500/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)1,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback1     // Each timer calls the same callback when it expires.
                                      );		
          xTimers[2] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (150/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)2,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback2     // Each timer calls the same callback when it expires.
                                      );
          xTimers[3] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (10/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)3,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback3     // Each timer calls the same callback when it expires.
                                      );
          xTimers[4] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (10/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)4,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback4     // Each timer calls the same callback when it expires.
                                      );
          xTimers[5] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (30/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)5,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback5     // Each timer calls the same callback when it expires.
                                      );
          xTimers[6] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (30/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)6,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback6     // Each timer calls the same callback when it expires.
                                      );																					
					if( xTimers[1] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[1], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}
					if( xTimers[2] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[2], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}			
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );	
		/* Initialize system */
	SystemInit();
	/* Initialize backup SRAM */
	TM_BKPSRAM_Init();
	/*Setup dau doc the*/
//	TM_BKPSRAM_Write8(1,0);	// 0 la 26bit va 34 bit, 1 la 26bit, 2 la 34bit.
//	TM_BKPSRAM_Write8(2,0);	// 0 la 26bit va 34 bit, 1 la 26bit, 2 la 34bit.
	if(TM_BKPSRAM_Read8(1)==2){
					if( xTimers[3] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[3], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}		
}
	if(TM_BKPSRAM_Read8(2)==2){
					if( xTimers[4] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[4], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}		
}
	if(TM_BKPSRAM_Read8(1)==1){
					if( xTimers[5] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[5], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}		
}
	if(TM_BKPSRAM_Read8(2)==1){
					if( xTimers[6] != NULL )
          {
              // Start the timer.  No block time is specified, and even if one was
              // it would be ignored because the scheduler has not yet been
              // started.
              if( xTimerStart( xTimers[6], 0 ) != pdPASS )
              {
                  // The timer could not be set into the Active state.
              }
						}			
}
	TM_BKPSRAM_Write8(3,0);	// 0 la khong su dung, 1 co su dung.
	/*Setup giao tiep RS232*/
	TM_BKPSRAM_Write8(4,0);	// 0 la khong su dung, 1 co su dung.
	/*Setup giao tiep 485*/
	TM_BKPSRAM_Write8(5,0);	// 0 la 9600, 1 la 115200.
	/*Setup MicroSDcar*/
	TM_BKPSRAM_Write8(6,0); // 0 la khong luu, 1 la co luu du lieu
	/* init OUTPUT*/
	TM_GPIO_Init(RELAY_DK1_PORT, RELAY_DK1_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK2_PORT, RELAY_DK2_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK3_PORT, RELAY_DK3_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK4_PORT, RELAY_DK4_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	/* init OUTPUT*/	
	STM_EVAL_LEDInit(LED_ORANGE);
		/*init SWADD*/
	TM_GPIO_Init(ADD_BIT0_PORT, ADD_BIT0_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);	
	TM_GPIO_Init(ADD_BIT1_PORT, ADD_BIT1_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT2_PORT, ADD_BIT2_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT3_PORT, ADD_BIT3_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT4_PORT, ADD_BIT4_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT5_PORT, ADD_BIT5_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT6_PORT, ADD_BIT6_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
	TM_GPIO_Init(ADD_BIT7_PORT, ADD_BIT7_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium);
/* Initialize USART6 at 115200 baud, TX: PC6, RX: PC7 , COM 1 - RFID1 gan cong tac nguon*/ 
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);
/* Initialize USART3 at 115200 baud, TX: PD8, RX: PD9 ,	COM 2 -PC gan ethernet*/
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
/* Initialize USART1 at 115200 baud, TX: PA9, RX: PA10, CONG 485 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 9600);
	if (!TM_RTC_Init(TM_RTC_ClockSource_Internal)) {
		/* RTC was first time initialized */
		/* Do your stuff here */
		/* eg. set default time */
	}
	/* Set wakeup interrupt every 1 second */
	TM_RTC_Interrupts(TM_RTC_Int_30s);
/* doc gia tri cong tac 8 bit de lay che do setup or run*/ 
	read_sw_add();
	if(TM_BKPSRAM_Read8(0)==1){
	flag_default=0;
	timeout=TM_BKPSRAM_Read8(22);
	}
	else flag_default=1;
//flag_config=0;

/*Task Create Begin*/
	xTaskCreate( vConfigure, (const signed char*)"Configure using DTMF", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vMain, (const signed char*)"Main run ...", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, (const signed char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vEthernet, (const signed char*)"Ethernet Running ..", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	vTaskStartScheduler();
}
//******************************************************************************
//******************************************************************************
void vConfigure(void *pvParameters)
{
int 					ipadress1[4]			=	{192,168,1,188};
int 					getwayadress1[4]	=	{192,168,1,1};
int						serveradress1[4]	=	{192,168,1,250};
int						portadress1 			= 8866;	
int						timeout1					=1;
int						wg1=0;
int						wg2=0;
	lcd_menu=1;
	TM_WATCHDOG_Reset();
	while(1)
	{
	if(flag_config==1){
	if(flag_lcdset==1&&add_str_dtmf==0){
		TM_HD44780_CursorOn();
		flag_lcdset=0;
		if(lcd_menu){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'&&lcd_menu20==0&&lcd_menu30==0&&lcd_menu40==0&&lcd_menu50==0){
			
			lcd_menu10=1;
			lcd_menu=0;
		}
		if(express=='2'&&lcd_menu10==0&&lcd_menu30==0&&lcd_menu40==0&&lcd_menu50==0){
			
			lcd_menu20=1;
			lcd_menu=0;
		}
		if(express=='3'&&lcd_menu10==0&&lcd_menu20==0&&lcd_menu40==0&&lcd_menu50==0){
			
			lcd_menu30=1;
			lcd_menu=0;
		}
		if(express=='4'&&lcd_menu10==0&&lcd_menu20==0&&lcd_menu30==0&&lcd_menu50==0){
			
			lcd_menu40=1;
			lcd_menu=0;
		}
		if(express=='5'&&lcd_menu10==0&&lcd_menu20==0&&lcd_menu30==0&&lcd_menu40==0){
			
			lcd_menu50=1;
			lcd_menu=0;
		}
		}
		sprintf(buffer_lcd,"Main Setup TIS8\n\r");
		strcat(buffer_lcd,"1.Card....4.Show\r");
		strcat(buffer_lcd,"2.TCPIP...5.Type\r");
		strcat(buffer_lcd,"3.OutPut..6.OK");
		
		}
		if(lcd_menu10){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'){
			
			lcd_menu11=1;
			lcd_menu10=0;
		}
		if(express=='2'){
			
			lcd_menu12=1;
			lcd_menu10=0;
		}
		if(express=='3'){
			
			lcd_menu13=1;
			lcd_menu10=0;
		}
		}
		sprintf(buffer_lcd,"SetReadCard\n\r");
		strcat(buffer_lcd,"1.Weigand 1\n\r");
		strcat(buffer_lcd,"2.Weigand 2\n\r");
		strcat(buffer_lcd,"3.RS232");
		
		}
		if(lcd_menu11){
		sprintf(buffer_lcd,"SetWeigand1\n\r");
		strcat(buffer_lcd,"1.Weigand 26bit\n\r");
		strcat(buffer_lcd,"2.Weigand 34bit\n\r");
		strcat(buffer_lcd,"1or2->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu12){
		sprintf(buffer_lcd,"SetWeigand2\n\r");
		strcat(buffer_lcd,"1.Weigand 26bit\n\r");
		strcat(buffer_lcd,"2.Weigand 34bit\n\r");
		strcat(buffer_lcd,"1or2->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu13){
		sprintf(buffer_lcd,"SetRS232\n\r");
		strcat(buffer_lcd,"1.Baud 9600\n\r");
		strcat(buffer_lcd,"2.Baud 115200\n\r");
		strcat(buffer_lcd,"1or2->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu20){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'){
			
			lcd_menu21=1;
			lcd_menu20=0;
		}
		if(express=='2'){
			
			lcd_menu22=1;
			lcd_menu20=0;
		}
		if(express=='3'){
			
			lcd_menu23=1;
			lcd_menu20=0;
		}
		if(express=='4'){
			
			lcd_menu24=1;
			lcd_menu20=0;
		}
		}
		sprintf(buffer_lcd,"1.IP\n\r");
		strcat(buffer_lcd,"2.IPserver\n\r");
		strcat(buffer_lcd,"3.Port\n\r");
		strcat(buffer_lcd,"4.DefaulGetway");
		
		}
		if(lcd_menu21){
		sprintf(buffer_lcd,"SetIPdevice\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu22){
		sprintf(buffer_lcd,"SetIPServer\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu23){
		sprintf(buffer_lcd,"SetPort\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);	
		}
		if(lcd_menu24){
		sprintf(buffer_lcd,"SetGetway\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu30){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'){
			
			lcd_menu31=1;
			lcd_menu30=0;
		}
		if(express=='2'){
			
			lcd_menu32=1;
			lcd_menu30=0;
		}
		if(express=='3'){
			
			lcd_menu33=1;
			lcd_menu30=0;
		}
		if(express=='4'){
			
			lcd_menu34=1;
			lcd_menu30=0;
		}
		}
		sprintf(buffer_lcd,"1.RS232-485\n\r");
		strcat(buffer_lcd,"2.Timeout relay\n\r");
		strcat(buffer_lcd,"3.TCPServer\n\r");
		strcat(buffer_lcd,"4.Timeoutprocess");
		
		}
		if(lcd_menu31){
		sprintf(buffer_lcd,"SetRS232-485\n\r");
		strcat(buffer_lcd,"1.Yes\n\r");
		strcat(buffer_lcd,"2.No\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);		
		}
		if(lcd_menu32){
		sprintf(buffer_lcd,"SetTimeout-Relay\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu33){
		sprintf(buffer_lcd,"TCPServer\n\r");
		strcat(buffer_lcd,"1.Yes\n\r");
		strcat(buffer_lcd,"2.No\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);		
		}
		if(lcd_menu34){
		sprintf(buffer_lcd,"Time-Process\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);		
		}
		if(lcd_menu40){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'){
			
			lcd_menu41=1;
			lcd_menu40=0;
		}
		if(express=='2'){
			
			lcd_menu42=1;
			lcd_menu40=0;
		}
		if(express=='3'){
			
			lcd_menu43=1;
			lcd_menu40=0;
		}
		}
		sprintf(buffer_lcd,"Show select\n\r");
		strcat(buffer_lcd,"1.IP,Server\n\r");
		strcat(buffer_lcd,"2.Card-Port\n\r");
		strcat(buffer_lcd,"3.Relay-DefaulGW");		
		}
		if(lcd_menu41){
			ipadress[0]=TM_BKPSRAM_Read8(7);
			ipadress[1]=TM_BKPSRAM_Read8(8);
			ipadress[2]=TM_BKPSRAM_Read8(9);
			ipadress[3]=TM_BKPSRAM_Read8(10);
			serveradress[0]=TM_BKPSRAM_Read8(11);
			serveradress[1]=TM_BKPSRAM_Read8(12);
			serveradress[2]=TM_BKPSRAM_Read8(13);
			serveradress[3]=TM_BKPSRAM_Read8(14);
			sprintf(buffer_lcd,"IP Device\n\r%d.%d.%d.%d\n\rServer\n\r%d.%d.%d.%d",ipadress[0],ipadress[1],ipadress[2],ipadress[3],serveradress[0],serveradress[1],serveradress[2],serveradress[3]);
		}
		if(lcd_menu42){
		portadress=TM_BKPSRAM_Read16(16);
		wg1=TM_BKPSRAM_Read8(1);
		wg2=TM_BKPSRAM_Read8(2);
		if((wg1==1)&&(wg2==1)){
		sprintf(buffer_lcd,"ReadCard 26bit\n\r\\n\rPort\n\r%d",portadress);	
		}else{
		if((wg1==1)&&(wg2==2)){
		sprintf(buffer_lcd,"Read26bit-34bit\n\r\\n\rPort\n\r%d",portadress);	
		}else{
		if((wg1==2)&&(wg2==2)){
		sprintf(buffer_lcd,"ReadCard 34bit\n\r\\n\rPort\n\r%d",portadress);	
		}else{
		if((wg1==2)&&(wg2==1)){
		sprintf(buffer_lcd,"Read34bit-26bit\n\r\\n\rPort\n\r%d",portadress);	
		}
		else
			sprintf(buffer_lcd,"NOTWiegand%d%d\n\rPort\n\r%d",wg1,wg2,portadress);
	}
	}
	}
	}
		if(lcd_menu43){
		timeout=TM_BKPSRAM_Read8(22);
		getwayadress[0]=TM_BKPSRAM_Read8(18);
		getwayadress[1]=TM_BKPSRAM_Read8(19);
		getwayadress[2]=TM_BKPSRAM_Read8(20);
		getwayadress[3]=TM_BKPSRAM_Read8(21);
		sprintf(buffer_lcd,"TimeoutRelay\n\r%d\n\rDefault Gw\n\r%d.%d.%d.%d",timeout,getwayadress[0],getwayadress[1],getwayadress[2],getwayadress[3]);
		}
		if(lcd_menu50){
		if(flag_dtmf==1){
		flag_dtmf=0;
		TM_HD44780_Clear();
		if(express=='1'){
			
			lcd_menu51=1;
			lcd_menu50=0;
		}
		if(express=='2'){
			
			lcd_menu52=1;
			lcd_menu50=0;
		}
		if(express=='3'){
			
			lcd_menu53=1;
			lcd_menu50=0;
		}
		}
		sprintf(buffer_lcd,"Device-Type\n\r");
		strcat(buffer_lcd,"1.Tayquay\n\r");
		strcat(buffer_lcd,"2.Swing\n\r");
		strcat(buffer_lcd,"3.......");		
		}
		TM_HD44780_Puts(0,0,buffer_lcd);
		sprintf(buffer_lcd,"");
	}

	if(flag_dtmf==1){
		if(add_str_dtmf==1){
			flag_dtmf=0;
			TM_HD44780_Clear();
			if(lcd_menu11==1)TM_HD44780_Puts(0,0,"Wiegand1");
			if(lcd_menu12==1)TM_HD44780_Puts(0,0,"Wiegand2");
			if(lcd_menu13==1)TM_HD44780_Puts(0,0,"Baudrate");
			if(lcd_menu21==1)TM_HD44780_Puts(0,0,"IPdevice");
			if(lcd_menu22==1)TM_HD44780_Puts(0,0,"IPServer");
			if(lcd_menu23==1)TM_HD44780_Puts(0,0,"Port");
			if(lcd_menu24==1)TM_HD44780_Puts(0,0,"Default GW");
			if(lcd_menu32==1)TM_HD44780_Puts(0,0,"TimeoutRelay");
			strcat(str_dtmf,buffer_dtmf);
			sprintf(buffer_dtmf,"");
			TM_HD44780_Puts(0,1,str_dtmf);
		}
			if(express=='#'){
			if((lcd_menu10==1)||(lcd_menu20==1)||(lcd_menu30==1)||(lcd_menu40==1)||(lcd_menu50==1)){
				lcd_menu=1;lcd_menu10=0;lcd_menu20=0;lcd_menu30=0;lcd_menu40=0;lcd_menu50=0;
				}
			if(lcd_menu11==1){
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d#",&wg1);
				TM_BKPSRAM_Write8(1,wg1);
				sprintf(str_dtmf,"");
				lcd_menu10=1;
				lcd_menu11=0;
				lcd_menu=0;
				}
			if(lcd_menu12==1){
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d#",&wg2);
				TM_BKPSRAM_Write8(2,wg2);
				sprintf(str_dtmf,"");
				lcd_menu10=1;
				lcd_menu12=0;
				lcd_menu=0;
				}
			if(lcd_menu13==1){
				printf_d(str_dtmf);
				lcd_menu10=1;
				lcd_menu13=0;
				lcd_menu=0;
				}
			if(lcd_menu21==1){	// ip device
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d.%d.%d.%d#",&ipadress1[0],&ipadress1[1],&ipadress1[2],&ipadress1[3]);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write8(7,ipadress1[0]);		//IP1 device
				TM_BKPSRAM_Write8(8,ipadress1[1]); 		//IP2 device
				TM_BKPSRAM_Write8(9,ipadress1[2]);		//IP3 device
				TM_BKPSRAM_Write8(10,ipadress1[3]);		//IP4 device				
				sprintf(str,"%x.%x.%x.%x",ipadress[0],ipadress[1],ipadress[2],ipadress[3]);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu21=0;
				lcd_menu=0;
			}
			if(lcd_menu22==1){	// ip server
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d.%d.%d.%d#",&serveradress1[0],&serveradress1[1],&serveradress1[2],&serveradress1[3]);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write8(11,serveradress1[0]);		//IP1 server
				TM_BKPSRAM_Write8(12,serveradress1[1]); 	//IP2 server
				TM_BKPSRAM_Write8(13,serveradress1[2]);		//IP3 server
				TM_BKPSRAM_Write8(14,serveradress1[3]);		//IP4 server				
				sprintf(str,"%x.%x.%x.%x",serveradress[0],serveradress[1],serveradress[2],serveradress[3]);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu22=0;
				lcd_menu=0;
			}
			if(lcd_menu24==1){	// default gw
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d.%d.%d.%d#",&getwayadress1[0],&getwayadress1[1],&getwayadress1[2],&getwayadress1[3]);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write8(18,getwayadress1[0]);		//IP1 server
				TM_BKPSRAM_Write8(19,getwayadress1[1]); 	//IP2 server
				TM_BKPSRAM_Write8(20,getwayadress1[2]);		//IP3 server
				TM_BKPSRAM_Write8(21,getwayadress1[3]);		//IP4 server				
				sprintf(str,"%x.%x.%x.%x",getwayadress1[0],getwayadress1[1],getwayadress1[2],getwayadress1[3]);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu24=0;
				lcd_menu=0;
			}
			if(lcd_menu23==1){	// port
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%4d#",&portadress1);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write16(16,portadress1);		//IP1 device		
				sprintf(str,"%d",portadress1);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu23=0;
				lcd_menu=0;
			}
			if(lcd_menu32==1){	// Time out
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d#",&timeout1);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write8(22,timeout1);		//IP1 device		
				sprintf(str,"%d",timeout1);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu30=1;
				lcd_menu32=0;
				lcd_menu=0;
			}
			if(lcd_menu31==1){
				lcd_menu30=1;
				lcd_menu31=0;
				lcd_menu=0;
				}
			if(lcd_menu32==1){
				lcd_menu30=1;
				lcd_menu32=0;
				lcd_menu=0;
				}
			if(lcd_menu33==1){
				lcd_menu30=1;
				lcd_menu33=0;
				lcd_menu=0;
				}
			if(lcd_menu34==1){
				printf_d(str_dtmf);
				sscanf(str_dtmf,"%d#",&timeout1);
					/*Setup giao tiep TCPIP*/
				TM_BKPSRAM_Write8(23,timeout1);		//IP1 device		
				sprintf(str,"%d",timeout1);
				printf_d(str);
				sprintf(str_dtmf,"");
				lcd_menu30=1;
				lcd_menu34=0;
				lcd_menu=0;
				}
			if(lcd_menu41==1){
				lcd_menu40=1;
				lcd_menu41=0;
				lcd_menu=0;
				}
			if(lcd_menu42==1){
				lcd_menu40=1;
				lcd_menu42=0;
				lcd_menu=0;
				}
			if(lcd_menu43==1){
				lcd_menu40=1;
				lcd_menu43=0;
				lcd_menu=0;
				}
			if(lcd_menu51==1){
				lcd_menu50=1;
				lcd_menu51=0;
				lcd_menu=0;
				}
			if(lcd_menu52==1){
				lcd_menu50=1;
				lcd_menu52=0;
				lcd_menu=0;
				}
			if(lcd_menu53==1){
				lcd_menu50=1;
				lcd_menu53=0;
				lcd_menu=0;
				}			
			add_str_dtmf=0;
		}
			if(express=='6'){
		if(lcd_menu==1)
		TM_BKPSRAM_Write8(0,1);
	}
	}
	TM_WATCHDOG_Reset();
	}
}
}
void vLedBlinkRed(void *pvParameters)
{
	int kk=0;
	TM_WATCHDOG_Reset();
	while(1)
	{
		if(tcpclient->state==CLIENT_CONNECTED){
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(100 / portTICK_RATE_MS );
		}else{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(50 / portTICK_RATE_MS );
		}
	if(flag_485){
		kk++;
		if(kk>6){
			if(car_read>3) 
			{
			NVIC_SystemReset();
			car_read=0;
			}
			kk=0;
		}	
	}
	else kk=0;
	TM_WATCHDOG_Reset();		
	}
}

void vEthernet(void *pvParameters)
{	
	if(flag_config==0){
	//device
	if(flag_default==0){
	ipadress[0]=TM_BKPSRAM_Read8(7);
	ipadress[1]=TM_BKPSRAM_Read8(8);
	ipadress[2]=TM_BKPSRAM_Read8(9);
	ipadress[3]=TM_BKPSRAM_Read8(10);

	// server
	serveradress[0]=TM_BKPSRAM_Read8(11);
	serveradress[1]=TM_BKPSRAM_Read8(12);
	serveradress[2]=TM_BKPSRAM_Read8(13);
	serveradress[3]=TM_BKPSRAM_Read8(14);
	// Default getway
	getwayadress[0]=TM_BKPSRAM_Read8(18);
	getwayadress[1]=TM_BKPSRAM_Read8(19);
	getwayadress[2]=TM_BKPSRAM_Read8(20);
	getwayadress[3]=TM_BKPSRAM_Read8(21);
	}
	ipadress[3]=ipadress[3]+value_dip;
	if (TM_ETHERNET_Init(macaddress[value_dip],ipadress, getwayadress, netmaskadress) == TM_ETHERNET_Result_Ok) {
	printf_d("TM_ETHERNET_Init OK \n");
	}
//Put string to LCD
		TM_HD44780_Clear();	
		TM_HD44780_Puts(0, 0, "----TIS8-PRO----CreartebyR&D-TIS"); /* 0 dong 1, 1 dong 2*/
		TM_HD44780_Puts(0, 2, "Welcom-->>TIS-OS");
		sprintf(buffer_lcd,"Time out OS:%d", timeout);
		TM_HD44780_Puts(0, 3,buffer_lcd);
		vTaskDelay(1000);
		TM_HD44780_Clear();
	/* Print MAC address to user */
		sprintf(buffer_lcd,"ACM:%s ->IP\n\r%d.%d.%d.%d\n\r%02X-%02X-%02X\n\r%02X-%02X-%02X",
		ACM,
		TM_ETHERNET_GetLocalIP(0),
		TM_ETHERNET_GetLocalIP(1),
		TM_ETHERNET_GetLocalIP(2),
		TM_ETHERNET_GetLocalIP(3),
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
		TM_HD44780_Puts(0, 0,buffer_lcd);
		vTaskDelay(1000);
		TM_HD44780_Clear();
		sprintf(buffer_lcd,"GW:%d.%d.%d.%d\n\rNet%d.%d.%d.%d\n\rSV:%d.%d.%d.%d\r\nN\r\nPORT:%d",
		TM_ETHERNET_GetGateway(0),
		TM_ETHERNET_GetGateway(1),
		TM_ETHERNET_GetGateway(2),
		TM_ETHERNET_GetGateway(3),
		TM_ETHERNET_GetNetmask(0),
		TM_ETHERNET_GetNetmask(1),
		TM_ETHERNET_GetNetmask(2),
		TM_ETHERNET_GetNetmask(3),
		serveradress[0],
		serveradress[1],
		serveradress[2],
		serveradress[3],
		portadress
	);
	TM_HD44780_Puts(0, 0,buffer_lcd);
	vTaskDelay(1000);
	flag_ok=1;
}
	/* Reset watchdog */
	TM_WATCHDOG_Reset();
	while(1)
	{
		/* Update ethernet, call this as fast as possible */
		TM_ETHERNET_Update();
		/* Reset watchdog */
		TM_WATCHDOG_Reset();
	}
}

void vMain(void *pvParameters)
{
			/* Init LCD*/
		TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
//Initialize LCD 16 cols x 4 rows
		TM_HD44780_Init(16,4);
//Save custom character on location 0 in LCD
		TM_HD44780_CreateChar(0, &customChar[0]);
		/* init DTMF*/
		TM_GPIO_Init(DTMF_BIT0_PORT, DTMF_BIT0_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
		TM_GPIO_Init(DTMF_BIT1_PORT, DTMF_BIT1_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
		TM_GPIO_Init(DTMF_BIT2_PORT, DTMF_BIT2_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
		TM_GPIO_Init(DTMF_BIT3_PORT, DTMF_BIT3_PIN, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_Low);
		TM_EXTI_Attach(DFMF_BIT4_PORT, DTMF_BIT4_PIN, TM_EXTI_Trigger_Rising);		
	/*init interrup INPUT*/
		TM_EXTI_Attach(W2_D0_PORT, W2_D0_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W2_D1_PORT, W2_D1_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W1_D1_PORT, W1_D1_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W1_D0_PORT, W1_D0_PIN, TM_EXTI_Trigger_Rising);
	/*init interrup OPT*/
		TM_EXTI_Attach(OPT_PORT,OPT_PIN1, TM_EXTI_Trigger_Falling);
//		TM_EXTI_Attach(OPT_PORT,OPT_PIN2, TM_EXTI_Trigger_Falling);
/* int DIR 485 set = send , reset = recvice*/ 
		TM_GPIO_Init(CCU_DIR_PORT, CCU_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
		TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);
	if(flag_config==0){	
		printf_d("Init main ...\n");
		read_sw_add();
		sprintf(ACM,"ACM%03d",value_dip);
	//timeout = 1;
		memset(str,'\0',0);
		flag_wiegand=0;	
		flag_RFID1=0;
		flag_serve=0;
		flag_test=0;
		updatime(01,01,01,01,01,15);
// configure role mo rong
		turn_off_dk3();
		turn_off_dk4();
		flag_RFID1=0;
		flag_wiegand=0;
		flag_pass=0;
		flag_485=0;
	}
	else
	printf_d("Init Setup ...\n");
	/*end by duc*/
	TM_WATCHDOG_Reset();
	while(1)
	{
	if(flag_config==0){
/*process server control*/
/*process 485*/
	if(flag_485){
	flag_485=0;
	if(flag_ok){
	if(flag_pass!=0){
	flag_pass++;
	if(flag_pass>20) flag_pass=0;
	}
	if(flag_pass!=0){
		if(tcpclient->state==1)sprintf(buffer_lcd,"Device:%s..%d\rConnected.......\rG:Open.Luot:%04d",ACM,tcpclient->state,luotdi);
		else sprintf(buffer_lcd,									"Device:%s..%d\rNotConnect......\rG:Open.Luot:%04d",ACM,tcpclient->state,luotdi);
		TM_HD44780_Puts(0,0,buffer_lcd);
	}
	else{
		if(tcpclient->state==1)sprintf(buffer_lcd,"Device:%s..%d\rConnected.......\rG:CloseLuot:%04d\rID:.............",ACM,tcpclient->state,luotdi);
		else sprintf(buffer_lcd,									"Device:%s..%d\rNotConnect......\rG:CloseLuot:%04d\rID:.............",ACM,tcpclient->state,luotdi);
		TM_HD44780_Puts(0,0,buffer_lcd);
	}	
	if(malenh_sv==2){
	malenh_sv=0;
	TM_HD44780_Clear();
	sprintf(buffer_lcd,"GiaDv:%d\n\rSodutk:%d\n\rLuotdi:%d",giadichvu,sodutk,luotdi);
	TM_HD44780_Puts(0, 0,buffer_lcd);
		}
	}
}
/*xu li khi co su kien nhan the*/
if((flag_RFID1==1)	&&(flag_pass==0)){			
		Process_1=1;
		Process_2=1;
		IDCAR1[0]=BufferCom1[4];
		IDCAR1[1]=BufferCom1[5];
		IDCAR1[2]=BufferCom1[6];
		IDCAR1[3]=BufferCom1[7];
		IDCAR1[4]=BufferCom1[8];
		IDCAR1[5]=BufferCom1[9];
		IDCAR1[6]=BufferCom1[10];
		
		if(BufferCom1[1]==0x08)	
			{
			sprintf(UID1,"%02X%02X%02X%02X",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3]);
			}
		if(BufferCom1[1]==0x0B) 
			{
			sprintf(UID1,"%02X%02X%02X%02X%02X%02X%02X",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3],IDCAR1[4],IDCAR1[5],IDCAR1[6]);
			}
		if(check_vip(UID1)){
			flag_SV=1;
			flag_R10=1;
			timerdk1 =0;
			flag_R21=1;
			timerdk4 =0;
			Process_1=0;
			Process_2=0;
			flag_vip=1;
		}
		else{
		if(Process_1){
			if(UID1 != NULL){
				sprintf(UID1_save,"%s",UID1);
				TM_HD44780_Puts(0,3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		if(Process_2){
			if(UID1 != NULL){
				car_read++;
				sprintf(data_tcp,"(1,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
				TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);
				sprintf(UID1_save,"%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
				memset(data_tcp,NULL,0);
			}
		}
		}
		WaitPC(300);
		flag_RFID1=0;
		
		if(flag_PC){
			ProcessAction();
			WaitPC(10);
		}
		else Process_1=0;
		
		if(flag_SV){ // duoc bat len trong luc cho doi.
			flag_SV=0;
			Serveprocess();
			WaitPC(10);
		}
		else Process_2=0;		
		flag_RFID1=0;	
		flag_test=0;
		flag_serve=0;
	}

if((flag_wiegand==1)&&(flag_pass==0)){
		Process_1=1;
		Process_2=1;
		if(flag_wiegand_34){
		sprintf(UID1,"%02X%02X%02X%02X",IDCAR1[3],IDCAR1[2],IDCAR1[1],IDCAR1[0]);
		flag_wiegand_34=0;
		}
		if(flag_wiegand_26){
		sprintf(UID1,"%s",IDCAR2);
		flag_wiegand_26=0;
		}
		if(check_vip(UID1)){
			flag_SV=1;
			flag_R10=1;
			timerdk1 =0;
			flag_R21=1;
			timerdk4 =0;
			Process_1=0;
			Process_2=0;
			flag_vip=1;
		}
		else{
		if(Process_1){
			if(UID1 != NULL){
				sprintf(UID1_save,"%s",UID1);
				TM_HD44780_Puts(0, 3,UID1_save);
				memset(UID1,NULL,0);
			}
		}
		if(Process_2){
			if(UID1 != NULL){
				car_read++;
				sprintf(data_tcp,"(1,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
				TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);
				sprintf(UID1_save,"%s",UID1);
				TM_HD44780_Puts(0,3,UID1_save);
				memset(UID1,NULL,0);
				memset(data_tcp,NULL,0);
			}
		}
		}
		WaitPC(300);
		flag_wiegand=0;
		
		if(flag_PC){
			ProcessAction();
			WaitPC(10);
		}
		else Process_1=0;
		
		if(flag_SV){ // duoc bat len trong luc cho doi.
			flag_SV=0;
			Serveprocess();
			WaitPC(10);
		}
		else Process_2=0;		
		flag_wiegand=0;	
		flag_test=0;
		flag_serve=0;
}
if(flag_serve==1){
	flag_serve=0;
	Serveprocess();
}	
if(flag_test==1){
	flag_test=0;
	ProcessAction();
}	
/* Quan li thoi gian cho tung su kien relay actived*/
timer_dk1 = timerdk1/2;	
if (timer_dk1 >= timeout){
			turn_off_dk1();
			flag_R10 =0;
			timerdk1=0;
			timer_dk1=0;
			flag_RFID1=0;
			flag_wiegand=0;
			Process_1=0;
			Process_2=0;
			WaitPC(50);
		}
timer_dk2 = timerdk2/2;
if (timer_dk2 >= timeout){
			turn_off_dk2();
			flag_R20 =0;
			timerdk2=0;
			timer_dk2=0;
			flag_RFID1=0;
			flag_wiegand=0;
			Process_1=0;
			Process_2=0;
			WaitPC(50);
		}
timer_dk3 = timerdk3;
if (timer_dk3 >= 4){
			turn_off_dk3();
			flag_R11 =0;
			timerdk3=0;
			timer_dk3=0;
			if(LEDStatus==0) {Process_1=0;Process_2=0;WaitPC(50);}
		}
timer_dk4 = timerdk4;
if (timer_dk4 >= 4){
			turn_off_dk4();
			flag_R21 =0;
			timer_dk4=0;
			timerdk4=0;
			if(LEDStatus==0) {Process_1=0;Process_2=0;WaitPC(50);}
		}
		TM_WATCHDOG_Reset();
	}
	}
}
//******************************************************************************
/* Tran duc code*/
static void  	printf_d(char *str){
	TM_USART_Puts(USART3,str);
	//vTaskDelay(5);
}
void 					ProcessAction(void){
		if(strncmp(BufferCom3,"R10",3)==0) // mo relay 1
	{
		flag_R10=1;
		timerdk1 =0;
	}
		if(strncmp(BufferCom3,"R20",3)==0) // mo relay 2
	{
		flag_R20=1;
		timerdk2 =0;
	}
		if(strncmp(BufferCom3,"R11",3)==0) // nuot the mo relay 3
	{
		flag_R11=1;
		timerdk3 =0;
	}
		if(strncmp(BufferCom3,"R21",3)==0) // nha the mo relay 4
	{
		flag_R21=1;
		timerdk4 =0;
	}
		if(strncmp(BufferCom3,"R00",3)==0) // TEST
	{
	TM_USART_Puts(USART3,"OK");
	}
		if(flag_R10)
	{		// active relay 1
		turn_on_dk1();
	}
		if(flag_R20)
	{	// active relay 2
		turn_on_dk2();
	}
		if(flag_R11)
	{	// active relay 3
		turn_on_dk3();
	}
		if(flag_R21)
	{	// active relay 4
		turn_on_dk4();
	}
	
/*Update time*/

	if(flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R11) timer_dk3 = timerdk3;

	if(flag_R21) timer_dk4 = timerdk4;
}

void 					Serveprocess(){
	if(flag_status_sv){
	flag_status_sv=0;
	if(	status_sv==1)				//mo relay 1 va 3
	{
		flag_R10=1;
		timerdk1 =0;
		flag_R11=1;
		timerdk3 =0;
	}
		if(status_sv==2)				//mo relay 1 va 4
	{
		flag_R10=1;
		timerdk1 =0;
		flag_R21=1;
		timerdk4 =0;		
	}
		if(status_sv==3)				// mo relay 3
	{
		flag_R11=1;
		timerdk3 =0;		
	}	
		if(status_sv==0)				// mo relay 4
	{
		flag_R21=1;
		timerdk4 =0;		
	}
}
		if(flag_R10)
	{		// active relay 1
		turn_on_dk1();
	}
		if(flag_R20)
	{	// active relay 2
		turn_on_dk2();
	}
		if(flag_R11)
	{	// active relay 3
		//turn_on_dk3();
	}
		if(flag_R21)
	{	// active relay 4
		turn_on_dk4();
	}
	
/*Update time*/

	if(flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R11) timer_dk3 = timerdk3;

	if(flag_R21) timer_dk4 = timerdk4;
}
void 					WaitPC(unsigned int t){
	unsigned long int n=0;
	while(n<=t){
	vTaskDelay(10);
	if(flag_PC==1||flag_SV==1)
		{
			break;
		}
	n++;
	}
}
void 					TM_USART3_ReceiveHandler(uint8_t c) {	// PC
	static uint8_t cnt=0;
	if(c=='R')cnt=0;
	BufferCom3[cnt]=c;
	if(cnt==2){flag_test=1;} // recive 3 ki tu.
	if((cnt==2)&&(Process_1==1))flag_PC=1;
	if(cnt<48)cnt++;
	else cnt =0;
}
void 					TM_USART6_ReceiveHandler(uint8_t c) {	// RFID1
	static uint8_t cnt=0;
	if(c==0xBD)cnt=0;
	BufferCom1[cnt]=c;
	if((BufferCom1[3]==0x00)&&(cnt==BufferCom1[3]+1)&&cnt)
	{
		if(Process_1==0 && Process_2==0 && flag_pass ==0){
			flag_RFID1=1; // cho phep xu li du lieu xi tien trinh da ok
			}
			memset(BufferCom1,'\0',0);
	}
	if(cnt<48)cnt++;
}
uint16_t 			TM_ETHERNETCLIENT_CreateHeadersCallback(TM_TCPCLIENT_t* connection, char* buffer, uint16_t buffer_length) {
	sprintf(buffer,"%s",data_tcp);
	return strlen(buffer);
}

void 					TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection, uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	static int i=0;
	static const char *s2 = "(,)";
	static char *buffer1;
	static char *token;
	memset(buffer1,0,0);
	buffer1=(char*)buffer;
	token = strtok(buffer1,s2);
	car_read=1;
   for (i=0;i<6;i++) 
   {	
			switch(i){
				case 0: {
					sprintf(str,"%s,",token);
					malenh_sv=atoi(str);
					printf_d(str);
				}break;
				case 1: {
					sprintf(str,"%s,",token); 
					printf_d(str);
				}break;
				case 2: {
					sprintf(str,"%s,",token); 	
					printf_d(str);
				}break;
				case 3: {
					sprintf(str,"%s,",token);//cong wiegand mac dinh 1 
					printf_d(str);
				}break;
				case 4: {										// Ma lenh dieu khien tu server.
					sprintf(str,"%s,",token); 
					printf_d(str);
					Sttup(str);
				}break;	
				case 5: {										// datetime server.
					sprintf(str,"%s",token); 
					printf_d(str);
					Timeup(str);
				}break;
				default:break;
			}
			token = strtok(NULL, s2);	
   }
	memset(buffer1,0,0);
	memset(buffer,0,0);
	if(malenh_sv==1){
	flag_serve=1;				// xu li trong ham chinh.
	flag_SV=1;
	flag_status_sv=1;
	}
	if(malenh_sv==2){
		flag_passed=0;
	}
	connection->headers_done = 1;
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectedCallback(TM_TCPCLIENT_t* connection) {
	/* We are connected */
	sprintf(str,"Connected to %s\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
	/* We are disconnected, done with connection */
	if (success) {
	sprintf(str,"Connection %s successfully.Active%d\n", connection->name, *connection->active_connections_count);
	printf_d(str);
	}
	/* Increase number of requests */
	tcpclient=connection;
	requests_count++;
}

void 					TM_ETHERNETCLIENT_ErrorCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"An error occured on connection %s\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionStartedCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"Connection %s has started\n", connection->name);
	printf_d(str);
	tcpclient=connection;
}
void 					TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	
	if (dhcp) {
		/* IP set with DHCP */
		sprintf(str,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
	} else {
		/* Static IP */
		sprintf(str,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		printf_d(str);
	}
	
	/* Print MAC address to user */
	sprintf(str,"MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);
	sprintf(str,"MAC: %02X-%02X-%02X-%02X-%02X-%02X\n",
		TM_ETHERNET_GetMACAddr(0),
		TM_ETHERNET_GetMACAddr(1),
		TM_ETHERNET_GetMACAddr(2),
		TM_ETHERNET_GetMACAddr(3),
		TM_ETHERNET_GetMACAddr(4),
		TM_ETHERNET_GetMACAddr(5)
	);




	/* Print 100M link status, 1 = 100M, 0 = 10M */
	sprintf(str,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	printf_d(str);
//	TM_USART_Puts(USART6,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	sprintf(str,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
	printf_d(str);
//	TM_USART_Puts(USART6,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
}


void 					TM_ETHERNET_LinkIsDownCallback(void) {
	/* This function will be called when ethernet cable will not be plugged */
	/* It will also be called on initialization if connection is not detected */
}

void 					TM_ETHERNET_LinkIsUpCallback(void) {
	/* Cable has been plugged in back, link is detected */
	/* I suggest you that you reboot MCU here */
	/* Do important stuff before */
}
void 					TM_ETHERNET_SystemResetCallback(void) {
	if (TM_ETHERNET_Init(macaddress[value_dip],ipadress, getwayadress, netmaskadress) == TM_ETHERNET_Result_Ok) {
	printf_d("RE_ETHERNET_Init OK \n");
	}
}
void 					TM_DELAY_1msHandler(void) {
	/* Time update for ethernet, 1ms */
	/* Add 1ms time for ethernet */
	TM_ETHERNET_TimeUpdate(1);
}
void 					TM_ETHERNET_DHCPStartCallback(void) {
	/* Print to user */
	sprintf(str,"DHCP has started with assigning IP address\n");
	printf_d(str);
}
void 					vTimerCallback1( xTimerHandle  pxTimer ){
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		flag_485=1;
		flag_lcdset=1;	
		TM_RTC_GetDateTime(&datatime, TM_RTC_Format_BIN);
		if(flag_R10){
			timerdk1++;
		}
		if(flag_R20){ 
			timerdk2++;	
		}
		if(flag_R11){ 
			timerdk3++;
		}
		if(flag_R21){
			timerdk4++;
		}
 }
void 					vTimerCallback2( xTimerHandle  pxTimer ){		// 
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		//TM_USART_Puts(USART6,SelectCard);
 }
void 					vTimerCallback3( xTimerHandle  pxTimer ){ 	// Read wiegand 34 1. 10ms
		static uint32_t countTimeOut1 = 0;
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		
		 if(count_1 > 0){
			 countTimeOut1++;
		 }
		 if(crc_34bit(count_1,card_1)){
			 uint32_t card_buf = 0;			 
				card_1 >>= 1;
				card_1 &= 0xFFFFFFFF;
				card_buf = (uint32_t)(card_1&0xFFFFFFFF);
				IDCAR1[0] = (uint8_t)(card_buf);
				IDCAR1[1] = (uint8_t)(card_buf >> 8);
				IDCAR1[2] = (uint8_t)(card_buf >> 16);
				IDCAR1[3] = (uint8_t)(card_buf >> 24);
				countTimeOut1 = 0;
				card_1 = 0;
				count_1 = 0;
				flag_finish_1 = 1;
				//if(flag_pass==0)
				//{
					flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
					flag_wiegand_34=1;
				//}
			}		 
				if(countTimeOut1 >= 5){ // 50ms ( gioi han thoi gian nhan du lieu 
			 flag_finish_1 = 0;
			 countTimeOut1 = 0;
			 card_1 = 0;
			 count_1 = 0;			 
			 flag_finish_1 = 1;
		 }			
 }
void 					vTimerCallback4( xTimerHandle  pxTimer ){ 	// Read wiegand 34 2. 10ms
		static uint32_t countTimeOut2 = 0;
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		

		 if(count_2 > 0){
			 countTimeOut2++;
		 }
		 if(crc_34bit(count_2,card_2)){
			 uint32_t card_buf = 0;			 
				card_2 >>= 1;
				card_2 &= 0xFFFFFFFF;
				card_buf = (uint32_t)(card_2&0xFFFFFFFF);
				IDCAR1[0] = (uint8_t)(card_buf);
				IDCAR1[1] = (uint8_t)(card_buf >> 8);
				IDCAR1[2] = (uint8_t)(card_buf >> 16);
				IDCAR1[3] = (uint8_t)(card_buf >> 24);
				countTimeOut2 = 0;
				card_2 = 0;
				count_2 = 0;
				flag_finish_2 = 1;
				//if(flag_pass==0)
				//{
					flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
					flag_wiegand_34=1;
				//}
			}		 
				if(countTimeOut2 >= 5){ // 50ms ( gioi han thoi gian nhan du lieu 
			 flag_finish_2 = 0;
			 countTimeOut2 = 0;
			 card_2 = 0;
			 count_2 = 0;			 
			 flag_finish_2 = 1;
		 }			
 }
void 					vTimerCallback5( xTimerHandle  pxTimer ){		// Read wiegang 26 1. 10ms
		long lArrayIndex;
		static uint32_t countTimeOut1 = 0;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
			if(count_1 > 0){
			 countTimeOut1++;
		 }
		//TM_USART_Puts(USART6,SelectCard);
		 if(crc_26bit(count_1,card_1)){
			 //uint32_t card_buf = 0;			 
			 card_1 >>= 1;
			 card_1 &= 0xFFFFFF;
			 card_1 = (uint32_t)(card_1&0xFFFFFF);	
				sprintf(IDCAR2,"%03u%05u",(uint32_t)card_1>>16,(uint32_t)card_1&0xFFFF);
				printf_d(IDCAR2);
				countTimeOut1 = 0;
				card_1 = 0;
				count_1 = 0;
				flag_finish_1 = 1;
				//if(flag_pass==0)
				//{
				flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
				flag_wiegand_26=1;
				//}
		 }		 
				if(countTimeOut1 >= 5){ // 50ms ( gioi han thoi gian nhan du lieu 
			 flag_finish_1 = 0;
			 countTimeOut1 = 0;
			 card_1 = 0;
			 count_1 = 0;			 
			 flag_finish_1 = 1;
		 }				
 }
void 					vTimerCallback6( xTimerHandle  pxTimer ){		// Read wiegang 26 2. 10ms
		long lArrayIndex;
		static uint32_t countTimeOut2 = 0;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
			if(count_2 > 0){
			 countTimeOut2++;
		 }
		 if(crc_26bit(count_2,card_2)){
			 //uint32_t card_buf = 0;			 
			 card_2 >>= 1;
			 card_2 &= 0xFFFFFF;
			 card_2 = (uint32_t)(card_2&0xFFFFFF);	
				sprintf(IDCAR2,"%03u%05u",(uint32_t)card_2>>16,(uint32_t)card_2&0xFFFF);
				printf_d(IDCAR2);
				card_2 = 0;
				count_2 = 0;
				flag_finish_2 = 1;
				countTimeOut2 = 0;
				//if(flag_pass==0)
				//{
				flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
				flag_wiegand_26=1;
				//}
		 }		 
			if(countTimeOut2 >= 5){ // 50ms ( gioi han thoi gian nhan du lieu 
			 flag_finish_2 = 0;
			 countTimeOut2 = 0;
			 card_2 = 0;
			 count_2 = 0;			 
			 flag_finish_2 = 1;
		 }				
 }
void 					TM_EXTI_Handler(uint16_t GPIO_Pin) {				// Interrupt ext
	/* Handle external line 0 interrupts */
	if (GPIO_Pin == DTMF_BIT4_PIN) {
		
		if(TM_GPIO_GetInputPinValue(DTMF_BIT0_PORT, DTMF_BIT0_PIN)==0){ // Q1  =0
			if(TM_GPIO_GetInputPinValue(DTMF_BIT1_PORT, DTMF_BIT1_PIN)==0){//  Q2 =0
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0 Q321=0 
						express = 'D';
					else express = '8';
						}
				else{ // Q1 =0,Q2=0,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '4';
					else express = '#';
						}
			}
			else //Q1=0,Q2=1,
			{
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0  
						express = '2';
					else express = '0';
						}
				else{ // Q1 =0,Q2=1,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '6';
					else 	express = 'B';
				}
			}
		}
		else{					//Q1=1
				if(TM_GPIO_GetInputPinValue(DTMF_BIT1_PORT, DTMF_BIT1_PIN)==0){//  Q2 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
						if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0 
						express = '1';
						else express = '9';
						}
					else{ // Q1 =1,Q2=0,Q3=1
				if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
								express = '5';
					else express = 'A';
					}
				}
				else //Q1=1,Q2=1,
				{
				if(TM_GPIO_GetInputPinValue(DTMF_BIT2_PORT, DTMF_BIT2_PIN)==0){// Q3 =0
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)// Q4 =0  
						express = '3';
					else express = '.';
						}
				else{ // Q1 =1,Q2=1,Q3=1
					if(TM_GPIO_GetInputPinValue(DTMF_BIT3_PORT, DTMF_BIT3_PIN)==0)
						express = '7';
					else express = 'C';
				}
			}
		}
		flag_dtmf=1;
		sprintf(buffer_dtmf,"%c",express);
		//strcat(buffer_lcd,buffer_dtmf);
	}
	if (GPIO_Pin == W1_D0_PIN) { // run W1 D0
		if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		count_1++;	
	}
	}
	if (GPIO_Pin == W1_D1_PIN) {	// run W1 D1
	if(flag_finish_1&&(LEDStatus==0)){
		card_1 <<= 1;
		card_1 |= 1;
		count_1++;	
	}
		
	}
	if (GPIO_Pin == W2_D0_PIN) {
		if(flag_finish_2&&(LEDStatus==0)){
		card_2 <<= 1;
		count_2++;	
	}
	}
	if (GPIO_Pin == W2_D1_PIN) {
	if(flag_finish_2&&(LEDStatus==0)){
		card_2 <<= 1;
		card_2 |= 1;
		count_2++;	
	}
	}

	if (GPIO_Pin == OPT_PIN1) { // ngat cac cong 
	if((flag_pass!=0)&&(flag_vip==0)){
	sprintf(data_tcp,"(2,%s,%s,1,,%d-%d-%d/%d-%d-%d)",ACM,UID1_save,datatime.hours,datatime.minutes,datatime.seconds,datatime.date,datatime.month,datatime.year);
	vTaskDelay(5);
	TM_ETHERNETCLIENT_Connect("server",serveradress[0],serveradress[1],serveradress[2],serveradress[3],portadress,&requests_count);	
	flag_passed=1;
	turn_on_dk3();
	}
	timerdk1=(timeout*2);
	timerdk2=(timeout*2);
	flag_vip=0;
	flag_pass=0;
	flag_RFID1=0;
	flag_wiegand=0;
}
}
void 					TM_RTC_RequestHandler() {										// Interrupt rtc
	fag_updatime=1;
	turn_off_dk3();
	turn_off_dk4();
}
void 					read_sw_add(void){													// Read switch 8 bit
	unsigned int sw_add[8];
	if(GPIO_ReadInputDataBit(ADD_BIT0_PORT,ADD_BIT0_PIN)==0) sw_add[0] = 1;
			else sw_add[0] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT1_PORT,ADD_BIT1_PIN)==0) sw_add[1] = 1;
			else sw_add[1] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT2_PORT,ADD_BIT2_PIN)==0) sw_add[2] = 1;
			else sw_add[2] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT3_PORT,ADD_BIT3_PIN)==0) sw_add[3] = 1;
			else sw_add[3] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT4_PORT,ADD_BIT4_PIN)==0) sw_add[4] = 1;
			else sw_add[4] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT5_PORT,ADD_BIT5_PIN)==0) sw_add[5] = 1;
			else sw_add[5] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT6_PORT,ADD_BIT6_PIN)==0) sw_add[6] = 1;
			else sw_add[6] = 0;
	if(GPIO_ReadInputDataBit(ADD_BIT7_PORT,ADD_BIT7_PIN)==0) sw_add[7] = 1;
			else sw_add[7] = 0;

	value_dip = 1*sw_add[5]+2*sw_add[4]+4*sw_add[3]+8*sw_add[2]+16*sw_add[1]+32*sw_add[0];
	if(sw_add[6]) timeout = 10;
	else timeout =1;
	if(sw_add[7]) flag_config = 0;
	 
}
static int 		turn_on_dk1(void){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=1;
	flag_pass=1;
	return 1;
	}
	else return 0;
}
static int 		turn_off_dk1(void){
	TM_GPIO_SetPinLow(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 0;
}
static int		turn_on_dk2(void){
	if(!LEDStatus){
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=2;
	return 1;
	}
	else return 0;
}
static int 		turn_off_dk2(void){
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 0;
}
static int		turn_off_dk3(void){
	TM_GPIO_SetPinHigh(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
static int 		turn_on_dk3(void){
	TM_GPIO_SetPinLow(RELAY_DK3_PORT,RELAY_DK3_PIN);
	TM_USART_Puts(USART3,Carpull);
	return 0;
}
static int		turn_off_dk4(void){
	TM_GPIO_SetPinHigh(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
static int 		turn_on_dk4(void){
	TM_GPIO_SetPinLow(RELAY_DK4_PORT,RELAY_DK4_PIN);
	TM_USART_Puts(USART3,Carpush);
	return 0;
}
static int 		check_vip(char * name){											// check IDcar
	static int i =0;
	for(i=0;i<2;i++){
	if(strstr(name,vip_id[i]))
		return 1;
	}
	return 0;
}
static void 	updatime(uint8_t h,uint8_t m,uint8_t s,uint8_t d,uint8_t mon,uint8_t year){ // Up datetime 
			datatime.hours = h;
			datatime.minutes = m;
			datatime.seconds = s;
			datatime.year = year;
			datatime.month = mon;
			datatime.date = d;
			/* Set new time */
			TM_RTC_SetDateTime(&datatime, TM_RTC_Format_BIN);
}
static void 	Sttup(char * buffer){ 		
		static int 				i=0;	// thong tin tk
		static unsigned int  			j=0;	// so du tk
		static unsigned int  			k=0;	// gia dich vu
		static int 				h=0;	// so luot khach di qua
		static int 				l=0;	//
		sscanf(buffer,"%d-%d-%d-%d-%d",&i,&j,&k,&h,&l);
		status_sv=l;
		luotdi=h;
		sodutk=j;
		giadichvu=k;
}
static void 	Timeup(char * buffer){
		static int h, m, s, d, mon, year;
		sscanf(buffer,"%d-%d-%d/%d-%d-%d",&h,&m,&s,&d,&mon,&year);
		updatime(h,m,s,d,mon,year);
}
uint8_t 			crc_26bit(uint32_t allnum_bit,uint32_t wiegand){
	uint32_t wiegand1_tam = 0;
	uint32_t a = 0, b = 0;
	uint32_t icount = 0;
	
	if((wiegand & 0xFC000000) != 0)
		return 0;
	if(allnum_bit != 26)
		return 0;
	wiegand1_tam = wiegand;
	for(icount = 1;icount <= 12;icount++){
		if((wiegand1_tam >>= 1)& 1)
			b++;
	}
	b = (b%2)?0:1;  // CRC le
	for(icount = 1;icount <= 12;icount++){
		if((wiegand1_tam >>= 1)& 1)
			a++;
	}
	a %= 2;	        // CRC chan
	
	wiegand1_tam = wiegand;
	if((a == wiegand1_tam >> 25) && (b == (wiegand1_tam & 1)))
		return 1;
	else
		return 0;
}
uint8_t 			crc_34bit(uint32_t allnum_bit,uint64_t wiegand){
	uint64_t wiegand1_tam = 0;
	uint32_t a = 0, b = 0;
	uint32_t icount = 0;
	
	if(allnum_bit != 34)
		return 0;
	wiegand1_tam = wiegand;
	for(icount = 1;icount <= 16;icount++){
		if((wiegand1_tam >>= 1)& 1)
			b++;
	}
	b = (b%2)?0:1;  // CRC le
	for(icount = 1;icount <= 16;icount++){
		if((wiegand1_tam >>= 1)& 1)
			a++;
	}
	a %= 2;	        // CRC chan
	
	wiegand1_tam = wiegand;
	if((a == wiegand1_tam >> 33) && (b == (wiegand1_tam & 1)))
		return 1;
	else
		return 0;
}
