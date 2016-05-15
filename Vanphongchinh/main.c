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
void vMain(void *pvParameters);
void vLedBlinkRed(void *pvParameters);
void vSend(void *pvParameters);
unsigned int 	 	biendem=0;	
unsigned char 	flag_RFID = 0;	/*Send SelectCard */		
unsigned char 	flag_RFID1=0;
unsigned char 	flag_out=1;
unsigned char 	flag_wiegand		=0;
unsigned char 	flag_wiegand_26	=0;
unsigned char 	flag_wiegand_34	=0;
unsigned char 	flag_R					=0;
unsigned char 	flag_O					=0;
unsigned char   send_pc=0;
unsigned char   send_pc_ok=1;
uint8_t cnt=0;
char *vip_id[2] = {
	"8C BD 9B 4D",
	"FC 23 D9 93",
	};
char SelectCard[4]= {0xBA,0x02,0x01,0xB9};       
char IDCAR1[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
char IDCAR2[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int LEDStatus=0;
char str[50];
uint8_t BufferCom1[50];
char BufferCom2[50];
char BufferCom3[50];
char UID1[50];

/*Khoi LCD*/
	uint8_t customChar[] = {
		0x1F,	/*  xxx 11111 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x11,	/*  xxx 10001 */
		0x1F	/*  xxx 11111 */
	};
/*time out*/
unsigned int timeout_relay 	= 0	;
unsigned int timeout_relay1 = 0	;
unsigned int timeout_relay2 = 0	;
unsigned int timeout_relay3 = 0	;
unsigned int timeout_relay4 = 0	;
unsigned char flag_485 = 0;
unsigned char flag_debug = 0;
unsigned char flag_rst = 0;
unsigned char flag_rff = 0;
unsigned char flag_R10 =0;
unsigned char flag_R20 =0;
unsigned char flag_R11 =0;
unsigned char flag_R21 =0;
unsigned char flag_test=0;
int check_vip(char * name);
/* Khoi OUTPUT*/

static int 	turn_on_dk1(int t);
static int 	turn_off_dk1();

static int	turn_on_dk2(int t);
static int 	turn_off_dk2();

static int	turn_on_dk3();
static int 	turn_off_dk3();

static int	turn_on_dk4();
static int 	turn_off_dk4();

unsigned int timerdk1 =0;
unsigned int timerdk2 =0;
unsigned int timerdk3 =0;
unsigned int timerdk4 =0;

unsigned int timer_dk1 =0;
unsigned int timer_dk2 =0;
unsigned int timer_dk3 =0;
unsigned int timer_dk4 =0;	
/*Khoi Wiegand*/ 
uint8_t 				flag_finish_1 = 1;
uint8_t 				flag_finish_2 = 1;
__IO uint64_t 	card_1 = 0;
__IO uint8_t  	count_1 = 0;
__IO uint64_t 	card_2 = 0;
__IO uint8_t  	count_2 = 0;
uint8_t 				crc_26bit(uint32_t allnum_bit,uint32_t wiegand);
uint8_t 				crc_34bit(uint32_t allnum_bit,uint64_t wiegand);
void ProcessAction(void);
/*Khoi timer*/
#define NUM_TIMERS 6
xTimerHandle xTimers[ NUM_TIMERS ];
long lExpireCounters[ NUM_TIMERS ] = { 0 };
void vTimerCallback1( xTimerHandle  pxTimer );
void vTimerCallback2( xTimerHandle  pxTimer );	// dau doc da nang
void vTimerCallback3( xTimerHandle  pxTimer ); 	// 34bit Weigang
void vTimerCallback4( xTimerHandle  pxTimer );	// 26bit Weigang
void vTimerCallback5( xTimerHandle  pxTimer );	// 26bit Weigang
unsigned long int wait_time=0;
#define STACK_SIZE_MIN	128	/* usStackDepth	- the stack size DEFINED IN WORDS.*/
#define wait_define 300 // 6s
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
                                          (150/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)2,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback2     // Each timer calls the same callback when it expires.
                                      );
          xTimers[3] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (20/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)3,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback3     // Each timer calls the same callback when it expires.
                                      );
          xTimers[4] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (20/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)4,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback4     // Each timer calls the same callback when it expires.
                                      );
          xTimers[5] = xTimerCreate(timer_xx,         // Just a text name, not used by the kernel.
                                          (20/ portTICK_RATE_MS),     // The timer period in ticks.
                                          pdTRUE,         // The timers will auto-reload themselves when they expire.
                                          (void*)5,     // Assign each timer a unique id equal to its array index.
                                          vTimerCallback5     // Each timer calls the same callback when it expires.
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
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );	

	/* Initialize system */
	SystemInit();
	/* Enable watchdog, 4 seconds before timeout */
	if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_8s)) {
		/* Report to user */
		//printf("Reset occured because of Watchdog\n");
	}	
	TM_EXTI_Attach(OPT_PORT, OPT_PIN1, TM_EXTI_Trigger_Falling);
	/* init OUTPUT*/	
	STM_EVAL_LEDInit(LED_ORANGE);
	/* init OUTPUT*/
	TM_GPIO_Init(RELAY_DK1_PORT, RELAY_DK1_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK2_PORT, RELAY_DK2_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK3_PORT, RELAY_DK3_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(RELAY_DK4_PORT, RELAY_DK4_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	TM_GPIO_Init(BUZZER_PORT, BUZZER_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
	
	/* Initialize USART6 at 115200 baud, TX: PC6, RX: PC7 , COM 1 - RFID1 gan cong tac nguon*/ 
	TM_USART_Init(USART6, TM_USART_PinsPack_1, 115200);
/* Initialize USART3 at 115200 baud, TX: PD8, RX: PD9 ,	COM 2 -PC gan ethernet*/
	TM_USART_Init(USART3, TM_USART_PinsPack_3, 9600);
/* Initialize USART1 at 115200 baud, TX: PA9, RX: PA10, CONG 485 */
	TM_USART_Init(USART1, TM_USART_PinsPack_1, 9600);
/* int DIR 485 set = send , reset = recvice*/ 
	TM_GPIO_Init(CCU_DIR_PORT, CCU_DIR_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
	TM_GPIO_SetPinHigh(CCU_DIR_PORT,CCU_DIR_PIN);
//flag_config=0;

/*Task Create Begin*/
	xTaskCreate( vMain, (const signed char*)"Main run ...", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, (const signed char*)"Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vSend, (const signed char*)"Send du lieu ...", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	vTaskStartScheduler();
}
//******************************************************************************
//******************************************************************************
void vMain(void *pvParameters)
{
			/*init interrup INPUT*/
		TM_EXTI_Attach(W1_D0_PORT, W1_D0_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W1_D1_PORT, W1_D1_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W2_D1_PORT, W2_D1_PIN, TM_EXTI_Trigger_Rising);
		TM_EXTI_Attach(W2_D0_PORT, W2_D0_PIN, TM_EXTI_Trigger_Rising);
	/* Init LCD*/
		TM_GPIO_Init(HD44780_RW_PORT, HD44780_RW_PIN, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
		TM_GPIO_SetPinLow(HD44780_RW_PORT,HD44780_RW_PIN);
		timeout_relay = 10;
		memset(str,'\0',0);
    //Initialize LCD 20 cols x 4 rows
    TM_HD44780_Init(16, 4);
    //Save custom character on location 0 in LCD
    TM_HD44780_CreateChar(0, &customChar[0]);    
		TM_HD44780_Clear();
		TM_HD44780_Puts(0, 0,"----TIS8 PRO----");
		flag_RFID1=0;
		flag_wiegand=0;
		flag_test=0;
	// configure role mo rong
		turn_off_dk3();
		turn_off_dk4();
		TM_WATCHDOG_Reset();
	while(1)
	{

/*xu li khi co su kien nhan the*/
if(flag_RFID1==1){			
		if((BufferCom1[1]==0x08)&&(BufferCom1[8]==0x01)&&(BufferCom1[9]!=0x00))
			{
				IDCAR1[0]=BufferCom1[4];
				IDCAR1[1]=BufferCom1[5];
				IDCAR1[2]=BufferCom1[6];
				IDCAR1[3]=BufferCom1[7];
				sprintf(UID1,"%02X %02X %02X %02X",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3]);
				}
			else
			if(BufferCom1[1]==0x0B) 
			{
				IDCAR1[0]=BufferCom1[4];
				IDCAR1[1]=BufferCom1[5];
				IDCAR1[2]=BufferCom1[6];
				IDCAR1[3]=BufferCom1[7];
				IDCAR1[4]=BufferCom1[8];
				IDCAR1[5]=BufferCom1[9];
				IDCAR1[6]=BufferCom1[10];
				vTaskDelay(10);
				sprintf(UID1,"%02X %02X %02x %02X %02X %02X %02X",IDCAR1[0],IDCAR1[1],IDCAR1[2],IDCAR1[3],IDCAR1[4],IDCAR1[5],IDCAR1[6]);
				vTaskDelay(10);
			}			
			if(send_pc==0) 
			{
				if(check_vip(UID1)){
				flag_test=1;
				flag_O=1;
				flag_R10=1;
				flag_R21=1;
				timerdk4 =0;
				timerdk1 =0;
				}
				else{
				send_pc=1;
				send_pc_ok=0;
				}
			}
	flag_RFID1=0;
	
	}
if(flag_wiegand==1){
		flag_wiegand=0;
		if(flag_wiegand_26==1){
		sprintf(UID1,"%s",IDCAR2);
		flag_wiegand_26=0;
		TM_USART_Puts(USART3,UID1);	
		}
		biendem=0;
		}		
if(flag_test==1){
	ProcessAction();
	flag_test=0;
}
/* Quan li thoi gian cho tung su kien relay actived*/
timer_dk1 = timerdk1/2;	
if (timer_dk1 >= timeout_relay1){
			turn_off_dk1();
			flag_R10 =0;
			timerdk1=0;
			timer_dk1=0;
		}
timer_dk2 = timerdk2/2;
if (timer_dk2 >= timeout_relay2){
			turn_off_dk2();
			flag_R20 =0;
			timerdk2=0;
			timer_dk2=0;
		}
timer_dk3 = timerdk3;
if (timer_dk3 >= 1){
			turn_off_dk3();
			flag_R11 =0;
			timerdk3=0;
			timer_dk3=0;
		}
timer_dk4 = timerdk4;
if (timer_dk4 >= 1){
			turn_off_dk4();
			flag_R21 =0;
			timer_dk4=0;
			timerdk4=0;
		}
		TM_WATCHDOG_Reset();
}
}
void vLedBlinkRed(void *pvParameters)
{
	while(1){
/*process 485*/
	if(flag_485){
	if(timeout_relay1>99) timeout_relay1=timeout_relay;
	if(timeout_relay2>99) timeout_relay2=timeout_relay;
	flag_485=0;
	if(LEDStatus==0) TM_USART_Puts(USART1, "/LED000>\r\n");
	if(LEDStatus==1) TM_USART_Puts(USART1, "/LED001>\r\n");
	if(LEDStatus==2) TM_USART_Puts(USART1, "/LED002>\r\n");
	sprintf(str,"time %2d %2d %2d",timer_dk1,timer_dk2,timeout_relay);
	TM_HD44780_Puts(0,1,str);
		/*xu li 1 tien trinh hoan chinh*/
	if(send_pc==0) 
	{TM_HD44780_Puts(0, 2,"Wait for Card");
	TM_HD44780_Puts(0, 3,"...........     ");
	}
		else 
		{TM_HD44780_Puts(0, 2,"Waiting PC...");
		TM_HD44780_Puts(0, 3,UID1);
		}
	}
if(biendem>50){
	NVIC_SystemReset();
	biendem=0;
}
		if(flag_wiegand){
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(100/ portTICK_RATE_MS );
		}else{
		STM_EVAL_LEDToggle(LED_ORANGE);
		vTaskDelay(300/ portTICK_RATE_MS );
		}
}
}
void vSend(void *pvParameters){
	while(1){
	if(send_pc==1)
	{	
		if(strlen(UID1)>5)
		{
		TM_USART_Puts(USART3,UID1);
		TM_GPIO_SetPinHigh(BUZZER_PORT,BUZZER_PIN);
		vTaskDelay(30);
		TM_GPIO_SetPinLow(BUZZER_PORT,BUZZER_PIN);
		send_pc_ok=1;
		}
		if(UID1!=NULL) memset(UID1,0,0);
	}
	}
}
//******************************************************************************
/* Tran duc code*/
void ProcessAction(void)
{
	int wg,time;
if(flag_R==1)
	{
	flag_R=0;	
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
	
		if(strncmp(BufferCom3,"R00",3)==0)	// TEST MACH
	{
		flag_debug=1;
	}
		if(strncmp(BufferCom3,"RST",3)==0)	// Reset mach
	{
		flag_rst=1;
	}
		if(strncmp(BufferCom3,"RFF",3)==0)	// Reset mach
	{
		flag_rff=1;
	}
		if(flag_rff){
		flag_rff=0;
		timerdk1=(timeout_relay*2);
		timerdk2=(timeout_relay*2);		
	}
		if(flag_rst){
		flag_rst=0;
		NVIC_SystemReset();	
	}
		if(flag_debug){
		flag_debug=0;
		TM_USART_Puts(USART3,"OK");		
	}
		if(flag_R10){		// active relay 1
		turn_on_dk1(0);
	}
		if(flag_R20){	// active relay 2
		turn_on_dk2(0);
	}
		if(flag_R11){	// active relay 3
		turn_on_dk3();
	}
		if(flag_R21){	// active relay 4
		turn_on_dk4();
	}
	memset(BufferCom2,0,0);
}	
if(flag_O==1){
	flag_O=0;
	sscanf(BufferCom2,"<%d,ON,%d>",&wg,&time);
	memset(BufferCom3,0,0);
	if(wg==1){
		flag_R10 =1;
		wg=0;
		//turn_off_dk2();
	}
	if(wg==2)
	{
		flag_R20 =1;
		wg=0;
		//turn_off_dk1();
	}
		if(flag_R10){		// active relay 1
		turn_on_dk1(time);		
	}
		if(flag_R20){	// active relay 2
		turn_on_dk2(time);
	}
		if(flag_R11){	// active relay 3
		turn_on_dk3();
		timerdk3=1;
	}
		if(flag_R21){	// active relay 4
		turn_on_dk4();
		timerdk4=1;
	}
}
/*Update time*/

	if(flag_R10) timer_dk1 = timerdk1/2;

	if(flag_R20) timer_dk2 = timerdk2/2;

	if(flag_R11 ) timer_dk3 = timerdk3;

	if(flag_R21 ) timer_dk4 = timerdk4;
}

void 					TM_USART3_ReceiveHandler(uint8_t c) {	// PC
	static uint8_t cnt1=0;
	static uint8_t cnt2=0;
	if(c=='R')cnt1=0;
	if(c==0x3C)cnt2=0;
	BufferCom3[cnt1]=c;
	BufferCom2[cnt2]=c;
	if(cnt1==2){
		cnt1=0;
		flag_test=1;
		flag_R=1;
	}
	if((cnt2==7)&&(c==0x3E)){
		cnt2=0;
		flag_test=1;
		flag_O=1;

	}
	if((cnt2==7)&&(c==0x3E)){
		cnt2=0;
		flag_O=1;
	}
	if(cnt1<4)cnt1++;
	else cnt1 =0;
	if(cnt2<8)cnt2++;
	else cnt2 =0;
}
void 					TM_USART6_ReceiveHandler(uint8_t c) {	// RFID1
BufferCom1[cnt]=c;
if((cnt==4)&&(BufferCom1[1]==0x03)) flag_out=1;
if((cnt==9)&&(BufferCom1[0]==0xBD)&&(BufferCom1[1]==0x08)&&(BufferCom1[2]==0x01)&&(send_pc_ok==1)&&(flag_out==1)) {
	flag_RFID1=1;
	flag_out=0;
	wait_time=0;
}
if((cnt==12)&&(BufferCom1[0]==0xBD)&&(BufferCom1[1]==0x0B)&&(BufferCom1[2]==0x01)&&(send_pc_ok==1)) flag_RFID1=1;
cnt++;
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
void 					vTimerCallback2( xTimerHandle  pxTimer ){	// mail
	long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		 TM_USART_Puts(USART6,SelectCard);
	   cnt=0;
 }
void 					vTimerCallback3( xTimerHandle  pxTimer ){ 	// Read wiegand 1. 10ms
		static uint32_t countTimeOut1 = 0;
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
		 if(send_pc!=0) send_pc++;
		 if(send_pc==99) 
		 {
			 send_pc_ok=1;
			 send_pc=0;
			 memset(UID1,0,50);
		 }
		 if(count_1 > 0){
			 countTimeOut1++;
		 }
		 if(crc_26bit(count_1,card_1)){ 
			card_1 >>= 1;
			card_1 &= 0xFFFFFF;
			card_1 = (uint32_t)(card_1&0xFFFFFF);	
			sprintf(IDCAR2,"1,%03u%05u",(uint32_t)card_1>>16,(uint32_t)card_1&0xFFFF);
			countTimeOut1 = 0;
			card_1 = 0;
			count_1 = 0;
			flag_finish_1 = 1;
			flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
			flag_wiegand_26=1;
		 }
		
 }
void 					vTimerCallback4( xTimerHandle  pxTimer ){		// Read wiegang 2. 10ms
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
		 if(crc_26bit(count_2,card_2)){	 
			card_2 >>= 1;
			card_2 &= 0xFFFFFF;
			card_2 = (uint32_t)(card_2&0xFFFFFF);	
			sprintf(IDCAR2,"2,%03u%05u",(uint32_t)card_2>>16,(uint32_t)card_2&0xFFFF);
			countTimeOut2 = 0;
			card_2 = 0;
			count_2 = 0;
			flag_finish_2 = 1;
			flag_wiegand=1; // cho phep xu li du lieu xi tien trinh da ok
			flag_wiegand_26=1;
		 }			
 }
void 					vTimerCallback5( xTimerHandle  pxTimer ){	// mail
		long lArrayIndex;
     /* Optionally do something if the pxTimer parameter is NULL. */
     configASSERT( pxTimer );

     /* Which timer expired? */
     lArrayIndex = ( long ) pvTimerGetTimerID( pxTimer );

     /* Increment the number of times that pxTimer has expired. */
     lExpireCounters[ lArrayIndex ] += 1;
			if(wait_time==wait_define){
			flag_out=1;
			}
			wait_time++;
 }
void 					TM_EXTI_Handler(uint16_t GPIO_Pin) {			// Interrupt ext
	if (GPIO_Pin == W1_D0_PIN) { // run W1 D0
		if(flag_finish_1){
		card_1 <<= 1;
		count_1++;	
	}
	}
	if (GPIO_Pin == W1_D1_PIN) {	// run W1 D1
	if(flag_finish_1){
		card_1 <<= 1;
		card_1 |= 1;
		count_1++;	
	}	
	}
	if (GPIO_Pin == W2_D0_PIN) {
	if(flag_finish_2){
		card_2 <<= 1;
		count_2++;	
	}
	}
	if (GPIO_Pin == W2_D1_PIN) {
	if(flag_finish_2){
		card_2 <<= 1;
		card_2 |= 1;
		count_2++;	
	}
	}

	if (GPIO_Pin == OPT_PIN1) { // ngat cac cong 
	timerdk1=(timeout_relay1*2);
	timerdk2=(timeout_relay2*2);
	flag_wiegand=0;
}
	biendem++;
}
void 					TM_RTC_RequestHandler() {									// Interrupt rtc
	turn_off_dk3();
	turn_off_dk4();
}
static int 	turn_on_dk1(int t){
	TM_GPIO_SetPinHigh(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=1;
	if(t!=0)timeout_relay1=t;
		else timeout_relay1 =timeout_relay;
	return 1;
}
static int 	turn_off_dk1(){
	TM_GPIO_SetPinLow(RELAY_DK1_PORT,RELAY_DK1_PIN);
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=0;
	return 1;
}
static int	turn_on_dk2(int t){
	TM_GPIO_SetPinHigh(RELAY_DK2_PORT,RELAY_DK2_PIN);
	LEDStatus=2;
	if(t!=0)timeout_relay2=t;
		else timeout_relay2 =timeout_relay;
	return 1;
}
static int 	turn_off_dk2(){
	TM_GPIO_SetPinLow(RELAY_DK2_PORT,RELAY_DK2_PIN);
	return 1;
}
static int	turn_off_dk3(){
	TM_GPIO_SetPinHigh(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
static int 	turn_on_dk3(){
	TM_GPIO_SetPinLow(RELAY_DK3_PORT,RELAY_DK3_PIN);
	return 0;
}
static int	turn_off_dk4(){
	TM_GPIO_SetPinHigh(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
static int 	turn_on_dk4(){
	TM_GPIO_SetPinLow(RELAY_DK4_PORT,RELAY_DK4_PIN);
	return 0;
}
int 	check_vip(char * name){
	int i =0;
	for(i=0;i<2;i++){
	if(strstr(name,vip_id[i]))
		return 1;
	}
	return 0;
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
