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
static void debug_test(char *str);
static void push_detex(char *str);
static void push_led();
static void push_ccu();
/* Fatfs object */
FATFS FatFs;
/* File object */
FIL fil;
FRESULT fres;
DSTATUS disk_initialize (BYTE drv);
char ZCU[8];

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
unsigned char flag_config=1;
unsigned char flag_default=0;
unsigned char flag_lcdset=0;
unsigned char flag_LCD = 0;
char buffer_lcd[64];				//ghi du lieu de xuat ra lcd
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
/*Khoi 485*/
static void 	send_485(int i);						// su dung cong nao
static void 	recv_485(int i);				
/*Khoi SW*/
void 					read_sw_add(void);		//doc gia tri switch 8 bit luu bien
unsigned char value_dip =0; /* value DIP switch*/
unsigned char timeout=0;
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
TM_TCPCLIENT_t *tcpclient;
char  			Sensortex[30][7]={	
{0xB9,0x5F,0xB1,0xB9,0x7D,0x7F,0xF5},
{0xB9,0xBE,0xAD,0xB9,0x7C,0x7F,0xD5},
{0xB9,0xAF,0xA9,0xB9,0x7B,0x7F,0xEA},
{0xB9,0xBD,0xA5,0xB9,0x7A,0x7F,0xD4},
{0xB9,0x5E,0xA1,0xB9,0x79,0x7F,0xFA},
{0xB9,0xBC,0x9D,0xB9,0x78,0x7F,0xD3},
{0xB9,0xD7,0x99,0xB9,0x77,0x7F,0xE9},
{0xB9,0xBB,0x95,0xB9,0x76,0x7F,0xD2},
{0xB9,0x5D,0x91,0xB9,0x75,0x7F,0xF4},
{0xB9,0xBA,0x8D,0xB9,0x74,0x7F,0xD1},
{0xB9,0xAE,0x89,0xB9,0x73,0x7F,0xE8},
{0xB9,0xB9,0x85,0xB9,0x72,0x7F,0xD0},
{0xB9,0x5C,0x81,0xB9,0x71,0x7F,0xFE},
{0xB9,0xB8,0x7D,0xB9,0x70,0x7F,0xCF},
{0xB9,0xEB,0x79,0xB9,0x6F,0x7F,0xE7},
{0xB9,0xB7,0x75,0xB9,0x6E,0x7F,0xCE},
{0xB9,0x5B,0x71,0xB9,0x6D,0x7F,0xF3},
{0xB9,0xB6,0x6D,0xB9,0x6C,0x7F,0xCD},
{0xB9,0xAD,0x69,0xB9,0x6B,0x7F,0xE6},
{0xB9,0xB5,0x65,0xB9,0x6A,0x7F,0xCC},
{0xB9,0x5A,0x61,0xB9,0x69,0x7F,0xF9},
{0xB9,0xB4,0x5D,0xB9,0x68,0x7F,0xCB},
{0xB9,0xD6,0x59,0xB9,0x67,0x7F,0xE5},
{0xB9,0xB3,0x55,0xB9,0x66,0x7F,0xCA},
{0xB9,0x59,0x51,0xB9,0x65,0x7F,0xF2},
{0xB9,0xB2,0x4D,0xB9,0x64,0x7F,0xC9},
{0xB9,0xAC,0x49,0xB9,0x63,0x7F,0xE4},
{0xB9,0xB1,0x45,0xB9,0x62,0x7F,0xC8},
{0xB9,0x58,0x41,0xB9,0x7F,0x7F,0xEB},
{0xB9,0xBF,0xB5,0xB9,0x7E,0x7F,0xD6},
};
/*Khoi process*/
unsigned char  	Process_1				=0;		// xu li day len PC nhung.
unsigned char  	Process_2				=0;		// xu li day len server.
/*Khoi Wait*/	
void 						WaitPC(unsigned int t);
unsigned char 	flag_PC=0;		// doi pc phan hoi va xu li xong du lieu nhan veroi moi bat
unsigned char 	flag_SV=0;		// doi server phan hoi va xu li xong du lieu nhan ve roi moi bat
/* Khoi Ban phim */
unsigned char 		express					=0;			// gia tri nhan duoc tu ban phim
unsigned char 		flag_dtmf 			=0;	
/*Khoi 485*/
int LEDStatus											=0;						// the hien trang thai cua led hien thi
unsigned char flag_485 						=0;	// xu li trong ham chinh duoc set theo timer
/*Khoi detex*/
void 	show_detex();
char 	detex[64];
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
	/* init OUTPUT*/
	TM_GPIO_Init(RELAY_DK1_PORT, RELAY_DK1_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK2_PORT, RELAY_DK2_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK3_PORT, RELAY_DK3_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK4_PORT, RELAY_DK4_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	/* init OUTPUT*/	
	STM_EVAL_LEDInit(LED_ORANGE);
	TM_GPIO_Init(BUZZER_PORT,BUZZER_PIN,TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);	
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
	vTaskDelay(1000);
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
		sprintf(buffer_lcd,"ZCU Configure\n\r");
		strcat(buffer_lcd,"1.ZCU.4.Show\n\r");
		strcat(buffer_lcd,"2.SenSor.\n\r");
		strcat(buffer_lcd,"3.6.OK");		
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
		sprintf(buffer_lcd,"Set ZCU\n\r");
		strcat(buffer_lcd,"1.Com 1\n\r");
		strcat(buffer_lcd,"2.Com 2\n\r");
		strcat(buffer_lcd,"3.Com 3");
		
		}
		if(lcd_menu11){
		sprintf(buffer_lcd,"SetUpCom1\n\r");
		strcat(buffer_lcd,"1.\n\r");
		strcat(buffer_lcd,"2.\n\r");
		strcat(buffer_lcd,"1or2->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu12){
		sprintf(buffer_lcd,"SetUpCom2\n\r");
		strcat(buffer_lcd,"1.\n\r");
		strcat(buffer_lcd,"2.\n\r");
		strcat(buffer_lcd,"1or2->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu13){
		sprintf(buffer_lcd,"SetUpCom3\n\r");
		strcat(buffer_lcd,"1.\n\r");
		strcat(buffer_lcd,"2.\n\r");
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
		sprintf(buffer_lcd,"SetupSenSor\n\r");
		strcat(buffer_lcd,"1.Size\n\r");
		strcat(buffer_lcd,"2.DoCao\n\r");
		strcat(buffer_lcd,"3.");
		
		}
		if(lcd_menu21){
		sprintf(buffer_lcd,"NhapSoLuong\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu22){
		sprintf(buffer_lcd,"Nhap DoCao\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu23){
		sprintf(buffer_lcd,"Notset23\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);	
		}
		if(lcd_menu24){
		sprintf(buffer_lcd,"Notset24\n\r");
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
		sprintf(buffer_lcd,"1.\n\r");
		strcat(buffer_lcd,"2.\n\r");
		strcat(buffer_lcd,"3.\n\r");
		strcat(buffer_lcd,"4.");
		
		}
		if(lcd_menu31){
		sprintf(buffer_lcd,"Notset31\n\r");
		strcat(buffer_lcd,"1.Yes\n\r");
		strcat(buffer_lcd,"2.No\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);		
		}
		if(lcd_menu32){
		sprintf(buffer_lcd,"NotSet32\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);
		}
		if(lcd_menu33){
		sprintf(buffer_lcd,"Notset33\n\r");
		strcat(buffer_lcd,"1.Yes\n\r");
		strcat(buffer_lcd,"2.No\n\r");
		strcat(buffer_lcd,"->#Ok");
		add_str_dtmf=1;
		memset(str_dtmf,0,0);
		memset(buffer_lcd,0,0);		
		}
		if(lcd_menu34){
		sprintf(buffer_lcd,"Notset34\n\r");
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
		sprintf(buffer_lcd,"Show Select\n\r");
		strcat(buffer_lcd,"1.SenSor\n\r");
		strcat(buffer_lcd,"2.Baudrate\n\r");
		strcat(buffer_lcd,"3.DoCaoSensor");		
		}
		if(lcd_menu41){
		}
		if(lcd_menu42){
	}
		if(lcd_menu43){
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
			if(lcd_menu11==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu12==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu13==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu21==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu22==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu23==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu24==1)TM_HD44780_Puts(0,0,"");
			if(lcd_menu32==1)TM_HD44780_Puts(0,0,"");
			strcat(str_dtmf,buffer_dtmf);
			sprintf(buffer_dtmf,"");
			TM_HD44780_Puts(0,1,str_dtmf);
		}
			if(express=='#'){
			if((lcd_menu10==1)||(lcd_menu20==1)||(lcd_menu30==1)||(lcd_menu40==1)||(lcd_menu50==1)){
				lcd_menu=1;lcd_menu10=0;lcd_menu20=0;lcd_menu30=0;lcd_menu40=0;lcd_menu50=0;
				}
			if(lcd_menu11==1){	//
				debug_test(str_dtmf);
				sprintf(str_dtmf,"");
				lcd_menu10=1;
				lcd_menu11=0;
				lcd_menu=0;
				}
			if(lcd_menu12==1){	//
				debug_test(str_dtmf);
				lcd_menu10=1;
				lcd_menu12=0;
				lcd_menu=0;
				}
			if(lcd_menu13==1){	//
				debug_test(str_dtmf);
				lcd_menu10=1;
				lcd_menu13=0;
				lcd_menu=0;
				}
			if(lcd_menu21==1){	//
				debug_test(str_dtmf);
				debug_test(str);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu21=0;
				lcd_menu=0;
			}
			if(lcd_menu22==1){	//
				debug_test(str_dtmf);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu22=0;
				lcd_menu=0;
			}
			if(lcd_menu24==1){	//
				debug_test(str_dtmf);
				sprintf(str_dtmf,"");
				lcd_menu20=1;
				lcd_menu24=0;
				lcd_menu=0;
			}
			if(lcd_menu23==1){	//
				lcd_menu20=1;
				lcd_menu23=0;
				lcd_menu=0;
			}
			if(lcd_menu32==1){	// 
				lcd_menu30=1;
				lcd_menu32=0;
				lcd_menu=0;
			}
			if(lcd_menu31==1){	//
				lcd_menu30=1;
				lcd_menu31=0;
				lcd_menu=0;
				}
			if(lcd_menu32==1){	//
				lcd_menu30=1;
				lcd_menu32=0;
				lcd_menu=0;
				}
			if(lcd_menu33==1){	//
				lcd_menu30=1;
				lcd_menu33=0;
				lcd_menu=0;
				}
			if(lcd_menu34==1){	//
				lcd_menu30=1;
				lcd_menu34=0;
				lcd_menu=0;
				}
			if(lcd_menu41==1){	//
				lcd_menu40=1;
				lcd_menu41=0;
				lcd_menu=0;
				}
			if(lcd_menu42==1){	//
				lcd_menu40=1;
				lcd_menu42=0;
				lcd_menu=0;
				}
			if(lcd_menu43==1){	//
				lcd_menu40=1;
				lcd_menu43=0;
				lcd_menu=0;
				}
			if(lcd_menu51==1){	//
				lcd_menu50=1;
				lcd_menu51=0;
				lcd_menu=0;
				}
			if(lcd_menu52==1){	//
				lcd_menu50=1;
				lcd_menu52=0;
				lcd_menu=0;
				}
			if(lcd_menu53==1){	//
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

		//	NVIC_SystemReset(); reset mach
	TM_WATCHDOG_Reset();		
	}
}

void vEthernet(void *pvParameters)
{	
	debug_test("TM_ETHERNET_Init OK \n");
		vTaskDelay(1000);
//Put string to LCD
		TM_HD44780_Clear();	
		//TM_HD44780_Puts(0, 0, "----TIS8-PRO----CreartebyR&D-TIS"); /* 0 dong 1, 1 dong 2*/
		//TM_HD44780_Puts(0, 2, "Welcom-->>TIS-OS");
		//TM_HD44780_Puts(0, 3,buffer_lcd);
		vTaskDelay(1000);
	/* Reset watchdog */
	TM_WATCHDOG_Reset();
	while(1)
	{

	}
}

void vMain(void *pvParameters)
{
		int ik=0;
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
/* int DIR 485 set = send , reset = recvice*/ 
		TM_GPIO_Init(CCU_DIR_PORT, CCU_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
		recv_485(1);
	if(flag_config==0){	
		debug_test("Init main ...\n");
		read_sw_add();
		sprintf(ZCU,"ACM%03d",value_dip);
		memset(str,'\0',0);
		updatime(01,01,01,01,01,15);
	}
	else
	debug_test("Init Setup ...\n");
	/*end by duc*/
	TM_WATCHDOG_Reset();
	while(1)
	{
	if(flag_config==0){
	if(flag_485==1){
	flag_485=0;
	for(ik=0;i<2;i++){
	push_detex(Sensortex[i]);
	//push_detex("TEST");
	recv_485(1);
	}
	
	}
	TM_WATCHDOG_Reset();
	}
	}
}

//******************************************************************************
static void 	debug_test(char *str){
	//
}
static void 	push_detex(char *str){
	//
	send_485(1);
	TM_USART_Puts(USART1,str);
}
static void 	push_led(char *str){
	//
}
static void 	push_ccu(char *str){
	//
}
static void 	send_485(int i){

		TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);

}
static void 	recv_485(int i){

		TM_GPIO_SetPinLow(CCU_DIR_PORT,CCU_DIR_PIN);

}
void 					TM_USART3_ReceiveHandler(uint8_t c) {	// PC
	static uint8_t cnt=0;
}
void 					TM_USART6_ReceiveHandler(uint8_t c) {	// RFID1
	static uint8_t cnt=0;
}
void 					TM_USART1_ReceiveHandler(uint8_t c) {	// 485
	static uint8_t cnt=0;
	TM_USART_Putc(USART3,c);
}
uint16_t 			TM_ETHERNETCLIENT_CreateHeadersCallback(TM_TCPCLIENT_t* connection, char* buffer, uint16_t buffer_length) {
	//sprintf(buffer,"%s",data_tcp);
	return strlen(buffer);
}

void 					TM_ETHERNETCLIENT_ReceiveDataCallback(TM_TCPCLIENT_t* connection, uint8_t* buffer, uint16_t buffer_length, uint16_t total_length) {
	connection->headers_done = 1;
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectedCallback(TM_TCPCLIENT_t* connection) {
	/* We are connected */
	sprintf(str,"Connected to %s\n", connection->name);
	debug_test(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionClosedCallback(TM_TCPCLIENT_t* connection, uint8_t success) {
	/* We are disconnected, done with connection */
	if (success) {
	sprintf(str,"Connection %s successfully.Active%d\n", connection->name, *connection->active_connections_count);
	debug_test(str);
	}
	/* Increase number of requests */
	tcpclient=connection;
	requests_count++;
}

void 					TM_ETHERNETCLIENT_ErrorCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"An error occured on connection %s\n", connection->name);
	debug_test(str);
	tcpclient=connection;
}

void 					TM_ETHERNETCLIENT_ConnectionStartedCallback(TM_TCPCLIENT_t* connection) {
	/* Print to user */
	sprintf(str,"Connection %s has started\n", connection->name);
	debug_test(str);
	tcpclient=connection;
}
void 					TM_ETHERNET_IPIsSetCallback(uint8_t ip_addr1, uint8_t ip_addr2, uint8_t ip_addr3, uint8_t ip_addr4, uint8_t dhcp) {
	/* Called when we have valid IP, it might be static or DHCP */
	
	if (dhcp) {
		/* IP set with DHCP */
		sprintf(str,"IP: %d.%d.%d.%d assigned by DHCP server\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		debug_test(str);
	} else {
		/* Static IP */
		sprintf(str,"IP: %d.%d.%d.%d; STATIC IP used\n", ip_addr1, ip_addr2, ip_addr3, ip_addr4);
		debug_test(str);
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
	debug_test(str);
//	TM_USART_Puts(USART6,"Link 100M: %d\n", TM_ETHERNET.speed_100m);
	/* Print duplex status: 1 = Full, 0 = Half */
	sprintf(str,"Full duplex: %d\n", TM_ETHERNET.full_duplex);
	debug_test(str);
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
	debug_test("RE_ETHERNET_Init OK \n");
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
	debug_test(str);
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
 }
void 					vTimerCallback2( xTimerHandle  pxTimer ){		// 
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
 }
void 					vTimerCallback3( xTimerHandle  pxTimer ){ 	// Read wiegand 34 1. 10ms

		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;			
 }
void 					vTimerCallback4( xTimerHandle  pxTimer ){ 	// Read wiegand 34 2. 10ms
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;			
 }
void 					vTimerCallback5( xTimerHandle  pxTimer ){		// Read wiegang 26 1. 10ms
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;				
 }
void 					vTimerCallback6( xTimerHandle  pxTimer ){		// Read wiegang 26 2. 10ms
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;				
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
}
void 					TM_RTC_RequestHandler() {										// Interrupt rtc
	fag_updatime=1;
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
	if(sw_add[7]) flag_config = 0; 
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
}
static void 	Timeup(char * buffer){
		static int h, m, s, d, mon, year;
		sscanf(buffer,"%d-%d-%d/%d-%d-%d",&h,&m,&s,&d,&mon,&year);
		updatime(h,m,s,d,mon,year);
}
void 	show_detex(){
	int i=0;
	TM_HD44780_Clear();
	sprintf(buffer_lcd,"");
	for(i=0;1<64;i++){
		if(detex[i]==0) strcat(buffer_lcd,"!");
		if(detex[i]==1) strcat(buffer_lcd,"X");
		if(detex[i]==2) strcat(buffer_lcd,"0");
	}
	TM_HD44780_Puts(0,0,buffer_lcd);
}

