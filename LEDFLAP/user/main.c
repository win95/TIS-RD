/**
  ******************************************************************************
  * @file    GPIO_Toggle\main.c
  * @author  MCD Application Team
  * @version  V2.2.0
  * @date     30-September-2014
  * @brief   This file contains the main function for GPIO Toggle example.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/**
  * @addtogroup GPIO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */

#define DATA_GPIO_PORT  (GPIOC)
#define DATA_GPIO_PINS  (GPIO_PIN_2)

#define CLKR_GPIO_PORT  (GPIOC)
#define CLKR_GPIO_PINS  (GPIO_PIN_3)

#define CLKG_GPIO_PORT  (GPIOC)
#define CLKG_GPIO_PINS  (GPIO_PIN_4)
//ouput pin
#define ROW1_GPIO_PORT  (GPIOC)
#define ROW1_GPIO_PINS  (GPIO_PIN_6)
#define ROW2_GPIO_PORT  (GPIOC)
#define ROW2_GPIO_PINS  (GPIO_PIN_7)
#define ROW3_GPIO_PORT  (GPIOG)
#define ROW3_GPIO_PINS  (GPIO_PIN_0)
#define ROW4_GPIO_PORT  (GPIOG)
#define ROW4_GPIO_PINS  (GPIO_PIN_1)
#define ROW5_GPIO_PORT  (GPIOE)
#define ROW5_GPIO_PINS  (GPIO_PIN_3)
#define ROW6_GPIO_PORT  (GPIOE)
#define ROW6_GPIO_PINS  (GPIO_PIN_2)
#define ROW7_GPIO_PORT  (GPIOE)
#define ROW7_GPIO_PINS  (GPIO_PIN_1)
#define ROW8_GPIO_PORT  (GPIOE)
#define ROW8_GPIO_PINS  (GPIO_PIN_0)
//input pin
#define RED_GPIO_PORT  (GPIOE)
#define RED_GPIO_PINS  (GPIO_PIN_7)
#define GREEN_GPIO_PORT  (GPIOE)
#define GREEN_GPIO_PINS  (GPIO_PIN_6)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t DisplayBuff[8][2];
uint8_t GreenEnable=0;
uint8_t RedEnable=0;
/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
static void IWDG_Config(void);
/* Private functions ---------------------------------------------------------*/
void InitialPort(void)
{
  GPIO_Init(DATA_GPIO_PORT, DATA_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(CLKR_GPIO_PORT, CLKR_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(CLKG_GPIO_PORT, CLKG_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  
  GPIO_Init(ROW1_GPIO_PORT, ROW1_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW2_GPIO_PORT, ROW2_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW3_GPIO_PORT, ROW3_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW4_GPIO_PORT, ROW4_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW5_GPIO_PORT, ROW5_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW6_GPIO_PORT, ROW6_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW7_GPIO_PORT, ROW7_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(ROW8_GPIO_PORT, ROW8_GPIO_PINS, GPIO_MODE_OUT_PP_HIGH_SLOW);
  
  GPIO_Init(RED_GPIO_PORT, RED_GPIO_PINS, GPIO_MODE_IN_PU_NO_IT);
  GPIO_Init(GREEN_GPIO_PORT, GREEN_GPIO_PINS, GPIO_MODE_IN_PU_NO_IT);
}
void ResetPort(void)
{
  GPIO_WriteLow(DATA_GPIO_PORT, DATA_GPIO_PINS);
  GPIO_WriteLow(CLKR_GPIO_PORT, CLKR_GPIO_PINS);
  GPIO_WriteLow(CLKG_GPIO_PORT, CLKG_GPIO_PINS);
  GPIO_WriteLow(ROW1_GPIO_PORT,ROW1_GPIO_PINS);
  GPIO_WriteLow(ROW2_GPIO_PORT,ROW2_GPIO_PINS);
  GPIO_WriteLow(ROW3_GPIO_PORT,ROW3_GPIO_PINS);
  GPIO_WriteLow(ROW4_GPIO_PORT,ROW4_GPIO_PINS);
  GPIO_WriteLow(ROW5_GPIO_PORT,ROW5_GPIO_PINS);
  GPIO_WriteLow(ROW6_GPIO_PORT,ROW6_GPIO_PINS);
  GPIO_WriteLow(ROW7_GPIO_PORT,ROW7_GPIO_PINS);
  GPIO_WriteLow(ROW8_GPIO_PORT,ROW8_GPIO_PINS);
};
/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

void UpdateDisplay(int16_t value)
{
  uint8_t i,j,d;
  for(j=0;j<8;j++)
    {
    d=DisplayBuff[j][0];  //data for RED color   
    for(i=0;i<8;i++)
      {
        if(d&0x01==0x01)GPIO_WriteHigh(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        else GPIO_WriteLow(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        
        if(RedEnable==0)GPIO_WriteLow(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        
        GPIO_WriteLow(CLKR_GPIO_PORT, (GPIO_Pin_TypeDef)CLKR_GPIO_PINS); 
        GPIO_WriteHigh(CLKR_GPIO_PORT, (GPIO_Pin_TypeDef)CLKR_GPIO_PINS);
        GPIO_WriteLow(CLKR_GPIO_PORT, (GPIO_Pin_TypeDef)CLKR_GPIO_PINS);
        d>>=1;
      }
    d=DisplayBuff[j][1];  //data for GREEN color
    for(i=0;i<8;i++)
      {
        if(d&0x01 ==0x01)GPIO_WriteHigh(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        else GPIO_WriteLow(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        
        if(GreenEnable==0)GPIO_WriteLow(DATA_GPIO_PORT, (GPIO_Pin_TypeDef)DATA_GPIO_PINS);
        
        GPIO_WriteLow(CLKG_GPIO_PORT, (GPIO_Pin_TypeDef)CLKG_GPIO_PINS);
        GPIO_WriteHigh(CLKG_GPIO_PORT, (GPIO_Pin_TypeDef)CLKG_GPIO_PINS);
        GPIO_WriteLow(CLKG_GPIO_PORT, (GPIO_Pin_TypeDef)CLKG_GPIO_PINS);
        d>>=1;
      }
    switch(j)
      {
      case 0: GPIO_WriteHigh(ROW1_GPIO_PORT,(GPIO_Pin_TypeDef)ROW1_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW1_GPIO_PORT,(GPIO_Pin_TypeDef)ROW1_GPIO_PINS);break;
      case 1: GPIO_WriteHigh(ROW2_GPIO_PORT,(GPIO_Pin_TypeDef)ROW2_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW2_GPIO_PORT,(GPIO_Pin_TypeDef)ROW2_GPIO_PINS);break;
      case 2: GPIO_WriteHigh(ROW3_GPIO_PORT,(GPIO_Pin_TypeDef)ROW3_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW3_GPIO_PORT,(GPIO_Pin_TypeDef)ROW3_GPIO_PINS);break;
      case 3: GPIO_WriteHigh(ROW4_GPIO_PORT,(GPIO_Pin_TypeDef)ROW4_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW4_GPIO_PORT,(GPIO_Pin_TypeDef)ROW4_GPIO_PINS);break;
      case 4: GPIO_WriteHigh(ROW5_GPIO_PORT,(GPIO_Pin_TypeDef)ROW5_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW5_GPIO_PORT,(GPIO_Pin_TypeDef)ROW5_GPIO_PINS);break;
      case 5: GPIO_WriteHigh(ROW6_GPIO_PORT,(GPIO_Pin_TypeDef)ROW6_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW6_GPIO_PORT,(GPIO_Pin_TypeDef)ROW6_GPIO_PINS);break;
      case 6: GPIO_WriteHigh(ROW7_GPIO_PORT,(GPIO_Pin_TypeDef)ROW7_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW7_GPIO_PORT,(GPIO_Pin_TypeDef)ROW7_GPIO_PINS);break;
      case 7: GPIO_WriteHigh(ROW8_GPIO_PORT,(GPIO_Pin_TypeDef)ROW8_GPIO_PINS);Delay(value);
              GPIO_WriteLow(ROW8_GPIO_PORT,(GPIO_Pin_TypeDef)ROW8_GPIO_PINS);break;
      };
    }
}
void ClearDisplay(void)
{
  int8_t i;
  for(i=0;i<8;i++)
    {
    DisplayBuff[i][0]=0x00;
    DisplayBuff[i][1]=0x00;
    }
}
void FullRed(void)
{
  int8_t i;
  for(i=0;i<8;i++)
    {
    DisplayBuff[i][0]=0xFF;
    }
}
void FullGreen(void)
{
  int8_t i;
  for(i=0;i<8;i++)
    {
    DisplayBuff[i][1]=0xFF;
    }
}

void RedX(void)
{
  DisplayBuff[0][0]=0x81;
  DisplayBuff[1][0]=0x42;
  DisplayBuff[2][0]=0x24;
  DisplayBuff[3][0]=0x18;
  DisplayBuff[4][0]=0x18;
  DisplayBuff[5][0]=0x24;
  DisplayBuff[6][0]=0x42;
  DisplayBuff[7][0]=0x81;
}
void GreenRow(int mode)
{
  if(mode==1){
  DisplayBuff[0][1]=0x20;
  DisplayBuff[1][1]=0x30;
  DisplayBuff[2][1]=0x18;
  DisplayBuff[3][1]=0xFC;
  DisplayBuff[4][1]=0xFC;
  DisplayBuff[5][1]=0x18;
  DisplayBuff[6][1]=0x30;
  DisplayBuff[7][1]=0x20;
  }
  else
  {
  DisplayBuff[0][1]=0x18;
  DisplayBuff[1][1]=0x18;
  DisplayBuff[2][1]=0xDB;
  DisplayBuff[3][1]=0x7E;
  DisplayBuff[4][1]=0x3C;
  DisplayBuff[5][1]=0x18;
  DisplayBuff[6][1]=0x00;
  DisplayBuff[7][1]=0x00;
  }
}
void MoveGreenRow(int8_t mode)
{
  uint8_t i,temp;
  static int16_t n=0;
  n++;
  if(n>8)
    {
      n=0;
    if(mode ==1)
    for(i=0;i<8;i++)
      {
        if(DisplayBuff[i][1] & 0x01==0x01)
          {
           DisplayBuff[i][1]>>=1;
           DisplayBuff[i][1]|=0x80;
          }
        else DisplayBuff[i][1]>>=1;
      }
    else
      {
      temp=DisplayBuff[7][1];
      for(i=7;i>0;i--)
        {
        DisplayBuff[i][1]=DisplayBuff[i-1][1];
        }
      DisplayBuff[0][1]=temp;
      };
    };
}
void main(void)
{
  uint32_t icheck  = 0;
  uint8_t  ficheck = 0;
  /* Initialize I/Os in Output Mode */
  InitialPort();
  ResetPort();
  ClearDisplay();
  IWDG_Config();
  GreenRow(0);
  RedX();
//  FullGreen();
//  FullRed();
  while (1)
  {
    IWDG_ReloadCounter();
    if(GPIO_ReadInputPin(RED_GPIO_PORT,RED_GPIO_PINS)||GPIO_ReadInputPin(GREEN_GPIO_PORT,GREEN_GPIO_PINS)){
      if(icheck++ > 10){
        ficheck = 1;
      }
    }else {
      icheck = 0;
      ficheck = 0;
    }
    
    if(ficheck == 1)
    {
      RedX();
      GreenEnable=0;
      RedEnable=1;
    }
    else
    {
      GreenEnable=1;
      RedEnable=0;
      MoveGreenRow(0);
    }
    UpdateDisplay(500);
  }

}

/**
  * @brief Delay
  * @param nCount
  * @retval None
  */
void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

static void IWDG_Config(void)
{
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
  
  /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  
  /* IWDG counter clock: LSI/128 */
  IWDG_SetPrescaler(IWDG_Prescaler_128);
  
  /* Set counter reload value to obtain 250ms IWDG Timeout.
    Counter Reload Value = 250ms/IWDG counter clock period
                         = 250ms / (LSI/128)
                         = 0.25s / (LsiFreq/128)
                         = LsiFreq/(128 * 4)
                         = LsiFreq/512
   */
  //IWDG_SetReload((uint8_t)(LsiFreq/512));
  
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
