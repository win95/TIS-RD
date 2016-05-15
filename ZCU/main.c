//******************************************************************************
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "discoveryf4utils.h"
//******************************************************************************
/* USER test...*/
#include "defines.h"
#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_watchdog.h"
#include "attributes.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_exti.h"
#include "tm_stm32f4_rtc.h"
#include "stm32f4xx_rcc.h"
#include "tm_stm32f4_id.h"
#include "tm_stm32f4_bkpsram.h"
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
extern char data_Detect[17];
void StackDETECT(void);
unsigned char flag_detect;
uint16_t addDevice=4; // dia chi cua mach
#define TIME_WHAT_LOOP_DET 5//ms
#define DETinZCU    64 

void sendDataWhatLopDET(char addressDET);
void saveInfoDetect(char dataDetect[17],char addressDET,char value);
char getInfoDetect(char dataDetect[17],char addressDET);
void WhatLoopDetect(void);
void reset_Data_Detect(void);
char ZCU[8];
uint32_t nCountTimeOutRe = 0; // dung de rut ngan thoi gian phai cho` de gui lenh sang 1 detect khac (TIM2_IRQHandler)
// Qua trinh hoat dong cua ZCU chia lam 2 giai doan chinh
// neu bang 0 thi chua bat dau 1 quy trinh nao ca
// 1 : Cap nhat du lieu tu DETECT
// 2 : tinh toan dua tren du lieu cap nhat va hien thi ra LCD
uint8_t flagState3Round = 1; 
char data_Detect[17]; // bien luu tru trang thai cua detect
char FlagComple;
void StackLCD(void);

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

#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
const signed char timer_xx[] = "Timer";
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
                                          (100/ portTICK_RATE_MS),     // The timer period in ticks.
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
/* Initialize USART6 at 9600 baud, TX: PC6, RX: PC7 , COM 1 - RFID1 gan cong tac nguon*/ 
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 9600);
/* Initialize USART3 at 9600 baud, TX: PD8, RX: PD9 ,	COM 2 -PC gan ethernet*/
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
/* Initialize USART1 at 4800 baud, TX: PA9, RX: PA10, CONG 485 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 4800);
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
	send_485(1);
/*Task Create Begin*/
//	xTaskCreate( vConfigure, (const signed char*)"Configure using DTMF", 
//		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
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
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(100 / portTICK_RATE_MS );
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
		TM_HD44780_Puts(0, 0, "----TIS8-PRO----CreartebyR&D-TIS"); /* 0 dong 1, 1 dong 2*/
		TM_HD44780_Puts(0, 2, "Welcom-->>TIS-OS");
		TM_HD44780_Puts(0, 3,buffer_lcd);
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
	flagState3Round = 1;
	push_detex("\r\n");
	while(1)
	{
		StackDETECT();
		//push_detex("\r\n");
		//StackLCD();
	}
}

//******************************************************************************
static void 	debug_test(char *str){
	//
	vTaskDelay(10);
}
static void 	push_detex(char *str){
	//
	send_485(1);
	TM_USART_Puts(USART1,str);
	vTaskDelay(5);
	recv_485(1);
}
static void 	push_led(char *str){
	//
	vTaskDelay(10);
}
static void 	push_ccu(char *str){
	//
	vTaskDelay(10);
}
static void 	send_485(int i){

		TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);

}
static void 	recv_485(int i){

		TM_GPIO_SetPinLow(CCU_DIR_PORT,CCU_DIR_PIN);

}
void 					TM_USART3_ReceiveHandler(uint8_t c) {	// LED
	static uint8_t cnt=0;
}
void 					TM_USART1_ReceiveHandler(uint8_t c) {	// detex
	static unsigned char data_rmc = 0;		
	static unsigned char rx_buffer[15] = "";
	static unsigned char FLAG = 0;
	//data_rmc = c;
	TM_USART_Putc(USART6,c);
//		if(data_rmc != 0)
//		{
//		if((FLAG == 0) && (((data_rmc >= 1) && (data_rmc <= 63)) || (data_rmc == 0xE1) || (data_rmc == 0xE2)))
//		{
//			static unsigned char FLAG_ARR = 0;
//			if((data_rmc >= 1) && (data_rmc <= 63)){
//				FLAG_ARR++;
//				rx_buffer[0] = data_rmc;
//			}else if(FLAG_ARR == 1){
//				FLAG_ARR = 0;
//				rx_buffer[1] = data_rmc;
//				FLAG = 1;
//			}else{
//				FLAG = 0;
//				FLAG_ARR = 0;
//			}
//			
//		}
//		}else{
//			FLAG = 0;
//		}
//		
//		if(FLAG == 1) 		
//		{
//			if(rx_buffer[0] >= 1 && rx_buffer[0] <= 63)
//			{
//					//1 : Co xe, 2 : khong co xe, 3 : khong co tin hieu. O la` bi loi~
//					saveInfoDetect(data_Detect,rx_buffer[0],(rx_buffer[1]==0xE2)?1:2);
//			//		if(IN_BIT7) // neu bit thu 7 trong switch ON 		
//			//		{	
////						  printf("\r\nDetect TAU : %s>\r\nnCountTimeOutRe=%d\r\n",rx_buffer,nCountTimeOutRe);
////						  printf("Detect ADD: %u, ZCU ADD:%u \r\n",rx_buffer[0],getAdd());
////							printf("data_Detect: %s\r\n",data_Detect);
//			//				((rx_buffer[1]==0xE2)?printf("Co xe do\r\n\r\n"):printf("Ko co xe do\r\n\r\n"));
//			//		}				
//					// da nhan du lieu, hoi vong ZCU khac				
//					nCountTimeOutRe = TIME_WHAT_LOOP_DET + 1; 
//			}
//			FLAG = 0;
//		}
}
void 					TM_USART6_ReceiveHandler(uint8_t c) {	// CCU
	static unsigned char el_rmc = 0;    
	static unsigned char reset = 0;    
	static char data_rmc = 0;		
	static char rx_buffer[15] = "";
	static unsigned char rx_wr_index_rmc = 0;
	static char send_buffer[30] = "";

		 if(data_rmc == '/')  
		 {		
					if(el_rmc == 3)
					{
							el_rmc = 0;
							rx_wr_index_rmc = 0;			
							for(reset = 0;reset < 15;reset++)
							{
								rx_buffer[reset] = 0;
							}
					}									
					el_rmc  = 1;
		 }
		 else if((el_rmc == 1)&&(data_rmc == 'C'))
		 {
					el_rmc = 2;     
		 }
		 else if(el_rmc < 2)
		 {
					el_rmc = 0;
		 }
		 else if(el_rmc == 2)  
		 {
					el_rmc = 3;
		 }                                                       
		 else if((el_rmc == 3)&&(data_rmc == '>')) 		
		 {
			  rx_buffer[rx_wr_index_rmc] = 0;
				//printf("DK:%d,%s\r\n",rx_wr_index_rmc,rx_buffer);			 
				if((rx_wr_index_rmc != 2) && (rx_wr_index_rmc != 11) && (rx_wr_index_rmc != 10)) 
				{
						el_rmc = 0;
						rx_wr_index_rmc = 0;			
						for(reset = 0;reset < 15;reset++)
						{
							rx_buffer[reset] = 0;		
						}
						return;
				}
				if(addDevice == ((rx_buffer[0]-'0')*10 + (rx_buffer[1]-'0')))
				{
						if(rx_wr_index_rmc == 2) // hoi du lieu cua ZCU
						{
								data_Detect[16] = 0;
								sprintf(send_buffer,"/C%02d%s>",addDevice,data_Detect); 	
								//USART4putsf(send_buffer);	// debug
								//USART1putsf(send_buffer); // day ra CCU
						}
						if((rx_wr_index_rmc == 11) || (rx_wr_index_rmc == 10)) // Nhan thong tin phan hoi tu CCU va 
						{
								sprintf(send_buffer,"/%s>",rx_buffer+2);
								//USART3putsf(send_buffer);		// day ra bang led	
								//USART4putsf(send_buffer);			//debug
						}			
				}
				el_rmc = 0;
				rx_wr_index_rmc = 0;					
				for(reset = 0;reset < 15;reset++)
				{
						rx_buffer[reset] = 0;
				}
				return;
		 }
		 if(el_rmc == 3)
		 {
					if((((data_rmc >= 48) && (data_rmc < 58)) || (data_rmc == 'P')) && (rx_wr_index_rmc < 13))
					{
							rx_buffer[rx_wr_index_rmc++] = data_rmc;
					}
					else
					{
							el_rmc = 0;
							rx_wr_index_rmc = 0;			
							for(reset = 0;reset < 15;reset++)
						  {
									rx_buffer[reset] = 0;
							}
					}
			}
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
		flag_detect=1;
	
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

/*******************************************************************************
* Function Name  : sendDataWhatLopDET
* Description    : Gui du lieu toi cac DET
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void sendDataWhatLopDET(char addressDET)
{
	char buffData[10] = "";
	if(addressDET == 62) return;
	sprintf(buffData,"%c%c%c%c",0xA3,addressDET,0x01,(0xA3+addressDET));
	push_detex(buffData);
}
/*******************************************************************************
* Function Name  : saveInfoDetect
* Description    : luu tru du lieu vao mang data_Detect khi biet dia chi cua detect va gia tri 1 hay 0 hoac ?
* Input          : mang luu tru cua detect, dia chi cua detect, gia tri 
* Output         : None
* Return         : None
* Attention		 : None
1 : Co xe, 2 : khong co xe, 3 : khong co tin hieu
value nhan gia tri 1,2 hoac 3
*******************************************************************************/
void saveInfoDetect(char dataDetect[17],char addressDET,char value)
{
	char bstt = 0;
	char buffD = 0;
	char stt = addressDET;	
	if((addressDET < 1) || (addressDET > 63) || (value < 1) || (value > 3))
		return;
	bstt = (addressDET - 1) / 4;
	buffD = *(dataDetect + bstt);	
	if(stt%4 == 0) 
	{
			stt = 4; 
	}else
		{
				stt %= 4;
		}	
	switch(stt)
	{
			case 1: 
			{
					buffD &= 0x3F;
					buffD |= (value << 6);
			} break;
			case 2: 
			{
					buffD &= 0xCF;
					buffD |= (value << 4);
			} break;	
			case 3: 
			{
					buffD &= 0xF3;
					buffD |= (value << 2);		
			} break;	
			case 4: 
			{
					buffD &= 0xFC;
					buffD |= value;						
			} break;		
			default : return;				
	}
	*(dataDetect + bstt) = buffD;
	dataDetect[15] |=  0x03; // dam bao rang vi tri 64 cua detect luon luon mang gia tri 3
	dataDetect[16]  =  0;
}
/*******************************************************************************
* Function Name  : getInfoDetect
* Description    : lay thong tin cua detect
* Input          : noi luu tru du lieu va` dia chi cua detect
* Output         : None
* Return         : None
* Attention		 : None
1 : Co xe, 2 : khong co xe, 3 : khong co tin hieu. O la` bi loi~
value nhan gia tri 1,2 hoac 3
*******************************************************************************/
char getInfoDetect(char dataDetect[17],char addressDET)
{
	char bstt = 0;
	char buffD = 0;
	char stt = addressDET;	
	if((addressDET < 1) || (addressDET > 63))
		return 0;
	bstt = (addressDET - 1) / 4;
	buffD = *(dataDetect + bstt);	
	if(stt%4 == 0) 
	{
			stt = 4; 
	}else
		{
				stt %= 4;
		}	
		switch(stt)
		{
			case 1: 
			{
					buffD = (uint8_t) (buffD >> 6);
			} break;
			case 2: 
			{
					buffD = (uint8_t) (buffD << 2);
					buffD = (uint8_t) (buffD >> 6);
			} break;	
			case 3: 
			{
					buffD = (uint8_t) (buffD << 4);
					buffD = (uint8_t) (buffD >> 6);			
			} break;	
			case 4: 
			{
					buffD = (uint8_t) (buffD << 6);
					buffD = (uint8_t) (buffD >> 6);						
			} break;		
			default : return 0;
		}
		return buffD;
}
/*******************************************************************************
* Function Name  : getInfoDetect
* Description    : lay thong tin cua detect
* Input          : noi luu tru du lieu va` dia chi cua detect
* Output         : None
* Return         : None
* Attention		 : None
1 : Co xe, 2 : khong co xe, 
3 : khong co tin hieu. O la` bi loi~
value nhan gia tri 1,2 hoac 3
*******************************************************************************/
uint32_t sumLostDetect(char dataDetect[17])
{
	uint32_t i = 0, sum = 0;
	for(i=1;i<64;i++)
	{
		if(getInfoDetect(dataDetect,i) == 2)
			sum++;
	}
	return sum;
}
/*******************************************************************************
* Function Name  : sendDataWhatLopDET
* Description    : Gui du lieu toi cac DET
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void WhatLoopDetect(void)
{
	static uint8_t flagOne = 0; // co trang thai lan dau de bo qua qua' trinh lan dau tien cap nhat gia tri so lan hoi khong phan hoi
	static uint32_t nCount = 0;
	static uint8_t nCountAdd = 1;	
//  if(flagState3Round==1) // Buoc 1 : hoi vong trang thai cua cac detect trong ZCU
//  {
//		 	if((nCount++ > TIME_WHAT_LOOP_DET) || nCountTimeOutRe++ > TIME_WHAT_LOOP_DET+5)
//				if(nCount++ > TIME_WHAT_LOOP_DET)
//		{
					// luc vao kiem tra lenh nay, nhung ZCU[0] khong dung nen neu co gan thi cung khong bi anh huong toi
					// du lieu da ghi
//					if(flagOne) // bo qua lan dau tien khi vao chuong trinh nay
//					{
//							if(nCount > TIME_WHAT_LOOP_DET) // hoi ma khong nhan duoc phan hoi ve
//							{
//									if(nCountAdd != 1)
//											saveInfoDetect(data_Detect,nCountAdd-1,3);
//									else 
//											saveInfoDetect(data_Detect,DETinZCU-1,3);
//							}
//					}else flagOne = 1;
					// Chi gui lenh hoi du lieu sang ZCU ke tiep trong 2 truong hop
					// 1 : het thoi gian TIME_WHAT_LOOP_ZCU ( cau hinh trong defineall.h
					// 2 : Bien nCountTimeOutRe tang bang voi thoi gian TIME_WHAT_LOOP_ZCU
					sendDataWhatLopDET(nCountAdd);
					nCount = 0;
					nCountTimeOutRe = 0; // bat dau tinh thoi gian time out 
					if(++nCountAdd >= DETinZCU) // mac dinh la moi CCU quan ly 63 ZCU
					{
						FlagComple = 1; // khi nao nhang co len bang 1 thi hien thi LCD
						nCountAdd = 1;
						flagState3Round = 1; // chuyen sang buoc 2: tinh toan va day du lieu 
					}
//				}
//		}
}
/*******************************************************************************
* Function Name  : StackDETECT
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void StackDETECT(void)
{
		if(flag_detect == 1) // hoi vong detect 
		{
				//IWDG_ReloadCounter(); // reset WDT
				flagState3Round = 1;
				WhatLoopDetect();
				//TM_USART_Puts(USART3,"TST");
				flag_detect = 0;
		}	
}

/*******************************************************************************
* Function Name  : StackLCD
* Description    : Hien thi thong tin len LCD
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
1 : Co xe, 2 : khong co xe, 3 : khong co tin hieu. O la` bi loi~
value nhan gia tri 1,2 hoac 3
*******************************************************************************/
void StackLCD(void)
{
		char disPlay[64] = "";
		char iCountLCD = 0;		
		char numberLost = 0;
//	if((flag_lcdset==1) && (FlagComple == 1)) // tinh thoi gian 2s de hien thi LCD
//	if(FlagComple == 1) // cu moi lan quet xong 1 zone thi cap nhat du lieu len LCD
//	{
		FlagComple = 0;
		flag_lcdset=0;
				for(iCountLCD=0;iCountLCD<63;iCountLCD++)
				{
						switch(getInfoDetect(data_Detect,iCountLCD+1))
						{
								case 1: 
									disPlay[iCountLCD] = '1'; 
									break;
								case 2: 
									disPlay[iCountLCD] = '0';
									numberLost++;
									break;
								case 3: 
									disPlay[iCountLCD] = '?'; 
									break;
								default : 
									disPlay[iCountLCD] = '?';
									break;
						}
				}
				disPlay[61] = disPlay[62];  
				disPlay[62] = numberLost/10+48;
				disPlay[63] = numberLost%10+48;
				TM_HD44780_Puts(0,0,disPlay);
				TM_HD44780_Puts(0,1,disPlay+16);
				TM_HD44780_Puts(0,2,disPlay+32);
				TM_HD44780_Puts(0,3,disPlay+48);
//		}
}
void reset_Data_Detect(void)
{
	char i;
	for(i=0;i<16;i++)
	{
			data_Detect[i] = 255;
	}
	data_Detect[16] = 0;
}