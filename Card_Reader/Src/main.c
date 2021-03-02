/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"
#include "rc522.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R1_PORT Keypad_R1_GPIO_Port
#define R1_PIN Keypad_R1_Pin
#define R2_PORT Keypad_R2_GPIO_Port
#define R2_PIN Keypad_R2_Pin
#define R3_PORT Keypad_R3_GPIO_Port
#define R3_PIN Keypad_R3_Pin
#define R4_PORT Keypad_R4_GPIO_Port
#define R4_PIN Keypad_R4_Pin
#define C1_PORT Keypad_C1_GPIO_Port
#define C1_PIN Keypad_C1_Pin
#define C2_PORT Keypad_C2_GPIO_Port
#define C2_PIN Keypad_C2_Pin
#define C3_PORT Keypad_C3_GPIO_Port
#define C3_PIN Keypad_C3_Pin

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t key1_test=0;
uint8_t key1=0;
uint8_t key2=0;
uint8_t birakma_flag1 =1;
uint8_t birakma_flag1_flag =0;
uint8_t buton_Deneme=0;
uint8_t ekranayazma(uint8_t datacevir);

uint8_t sifre[]={2,5,8,0,9,6};
uint8_t sifre_lenght=(sizeof(sifre)/sizeof(uint8_t));
uint8_t sifre_test[15];
uint8_t girilentus=0;
uint8_t sifrebitir=0;

void open_door (void);
void close_door(void);
void right_pasword(void);
void wrong_pasword(void);
char read_keypad (void);

uint8_t rfid_flag=1;
unsigned char CardID[5];

RTC_TimeTypeDef sTime;
RTC_DateTypeDef DateToUpdate;

uint16_t gun,ay,yil,saat,dakika,saniye;
char Time_buffer[8];
char line1_buffer[16];
char usart_buffer[50];
uint8_t usart_buffer1[50];

struct kullanici{
unsigned char RFID[5];
char Name[15];
char Surname[15];
char Time[8];
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
struct kullanici kimlik1;

kimlik2.RFID[0]=0x24;
kimlik2.RFID[1]=0x0B;
kimlik2.RFID[2]=0xFF;
kimlik2.RFID[3]=0x62;
kimlik2.RFID[4]=0x7C;
strcpy(kimlik2.Name,"Eren");
strcpy(kimlik2.Surname , "Kaplan");


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	MFRC522_Init();
  lcd_init();
	HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Stop(&htim4);
	HAL_TIM_Base_Stop(&htim5);
	HAL_TIM_Base_Stop(&htim7);
	HAL_TIM_Base_Stop(&htim6);
	
  lcd_send_cmd (0x80|0x00);
  lcd_send_string("Password or Card");
	
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("PW:");
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
if (MFRC522_Check(CardID) == MI_OK) 
	{   
				
	if (MFRC522_Compare(CardID, kimlik1.RFID) == MI_OK && rfid_flag==1)
		{ 
						
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
						
	sprintf(Time_buffer,"%2d/%2d/%2d",sTime.Hours,sTime.Minutes,sTime.Seconds);
  strcpy(kimlik1.Time,Time_buffer);
	          
	sprintf(line1_buffer,"%s %s",kimlik1.Name,kimlik1.Surname);
	lcd_send_cmd (0x80|0x00);
  lcd_send_string("                ");
	lcd_send_cmd (0x80|0x00);
  lcd_send_string(line1_buffer);
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("                ");
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("    WELCOME");
			
	sprintf(usart_buffer,"%s %s %s\n",kimlik1.Time,kimlik1.Name,kimlik1.Surname);
	for(int i=0;i<=sizeof(usart_buffer);i++){
	usart_buffer1[i]=usart_buffer[i];
	}
  HAL_UART_Transmit(&huart1,usart_buffer1,sizeof(usart_buffer),100);
	
	rfid_flag=0;
	open_door();
	}

else if  (rfid_flag==1) 
	{
	rfid_flag=0;
	lcd_send_cmd (0x80|0x00);
  lcd_send_string("                ");
	lcd_send_cmd (0x80|0x00);	
  lcd_send_string(" Undefined Card");
	close_door();
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("                ");	
	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Stop(&htim5);
	}
		}
if(CardID[0]== 0x00){
	rfid_flag=1;
		}
  HAL_Delay(1);
										 
		//2.  KOD				 
						 
	key1_test=read_keypad();   //tus birakma kontrol
if(key1_test!=0x00 && birakma_flag1_flag == 0){
	HAL_TIM_Base_Start(&htim4);
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
	birakma_flag1_flag =1;
		}
if(birakma_flag1 == 0 && key1_test == 0x00){
	birakma_flag1=1;
	buton_Deneme=1;
	birakma_flag1_flag = 0;
		}
if(key1_test != 0x00 ){
	key1=key1_test;
	key2=ekranayazma(key1);
		}
if(birakma_flag1==1 && key1 == 11 && buton_Deneme == 1)
		{
	buton_Deneme=0;
if(girilentus !=0){
	girilentus--;
		}
for(int i=girilentus; i<=14; i++)
{
	sifre_test[i]=0;
		}
	lcd_send_cmd (0x80|0x44+girilentus);
  lcd_send_string("  ");
	  }
if(birakma_flag1==1 && key1 == 12 && buton_Deneme == 1)
		{
	sifrebitir=1;
		}		
if(birakma_flag1==1 && buton_Deneme == 1){
	buton_Deneme=0;
if(key1 == 10 || key1 == 1 || key1 == 2|| key1 == 3 || key1 == 4 || key1 == 5|| key1 == 6 || key1 == 7 || key1 == 8 || key1 == 9){
	sifre_test[girilentus]=key1%10;
	girilentus++;
if(girilentus==1){
	lcd_send_cmd (0x80|0x40);
	lcd_send_string("PW:               ");
		}
	lcd_send_cmd (0x80|0x43+girilentus);
  lcd_send_data(ekranayazma(sifre_test[girilentus-1]));
		}
if(sifrebitir==1){
	sifrebitir=0;
 if(girilentus !=sifre_lenght)
		{
	girilentus=0;
	wrong_pasword();
for(int k=0; k<=14;k++){
	sifre_test[k]=0;
		}
		 }
 else
		{ 
	girilentus=0;
for(int i=0;i<=sifre_lenght-1;i++){
	if(sifre[i]!=sifre_test[i]){
	wrong_pasword();
for(int k=0; k<=14;k++){
	sifre_test[k]=0;
		}
	i=100;
		}
if(i==sifre_lenght-1){
	right_pasword();
for(int k=0; k<=14;k++){
	sifre_test[k]=0;
			}
				}
					}
				    }
		         	}
		            }		
                 }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)== 0x32F2){
  return;
}
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 22;
  sTime.Minutes = 11;
  sTime.Seconds = 5;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  DateToUpdate.Month = RTC_MONTH_DECEMBER;
  DateToUpdate.Date = 25;
  DateToUpdate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
HAL_TIM_Base_Stop(&htim4);
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */
  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7200;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 6000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_TIM_Base_Stop(&htim5);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */
  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 21000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7200;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 12000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, buzzer_Pin|GPIO_PIN_11|Keypad_R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Keypad_R2_Pin|Keypad_R3_Pin|Keypad_R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Keypad_R1_Pin */
  GPIO_InitStruct.Pin = Keypad_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Keypad_R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Keypad_R2_Pin Keypad_R3_Pin Keypad_R4_Pin */
  GPIO_InitStruct.Pin = Keypad_R2_Pin|Keypad_R3_Pin|Keypad_R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Keypad_C1_Pin */
  GPIO_InitStruct.Pin = Keypad_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Keypad_C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Keypad_C2_Pin Keypad_C3_Pin */
  GPIO_InitStruct.Pin = Keypad_C2_Pin|Keypad_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void right_pasword(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);

  lcd_send_cmd (0x80|0x00);
  lcd_send_string("                ");
	lcd_send_cmd (0x80|0x00);
  lcd_send_string("Correct Password");
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("                ");
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
	HAL_TIM_Base_Start(&htim6);
  HAL_TIM_Base_Start(&htim5);
	
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	
	sprintf(usart_buffer,"%2d/%2d/%2d Correct Pasword\n",sTime.Hours,sTime.Minutes,sTime.Seconds);
	for(int i=0;i<=sizeof(usart_buffer);i++){
	usart_buffer1[i]=usart_buffer[i];
	}
  HAL_UART_Transmit(&huart1,usart_buffer1,sizeof(usart_buffer),100);
}
void wrong_pasword(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
	
	lcd_send_cmd (0x80|0x00);
  lcd_send_string("                ");
	lcd_send_cmd (0x80|0x00);
  lcd_send_string(" Wrong Password");
	lcd_send_cmd (0x80|0x40);
  lcd_send_string("                ");
	
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start(&htim6);
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
	
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	sprintf(usart_buffer,"%2d/%2d/%2d Wrong Pasword\n",sTime.Hours,sTime.Minutes,sTime.Seconds);
	for(int i=0;i<=sizeof(usart_buffer);i++){
	usart_buffer1[i]=usart_buffer[i];
	}
  HAL_UART_Transmit(&huart1,usart_buffer1,sizeof(usart_buffer),100);
}
void open_door (void){
	HAL_TIM_Base_Start(&htim6);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);

	HAL_TIM_Base_Start(&htim5);
	HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
	
	for(int i=0; i<=14; i++)
	{
	 sifre_test[i]=0;
	}
	girilentus=0;
}

void close_door(void){
	
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
HAL_TIM_Base_Start(&htim7);
HAL_GPIO_WritePin(buzzer_GPIO_Port,buzzer_Pin,GPIO_PIN_SET);
	
							
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	sprintf(usart_buffer,"%2d/%2d/%2d Undefined Card\n",sTime.Hours,sTime.Minutes,sTime.Seconds);
	for(int i=0;i<=sizeof(usart_buffer);i++){
	usart_buffer1[i]=usart_buffer[i];
	}
  HAL_UART_Transmit(&huart1,usart_buffer1,sizeof(usart_buffer),100);
	for(int i=0; i<=14; i++)
	{
	sifre_test[i]=0;
	}
	girilentus=0;
}
uint8_t ekranayazma(uint8_t datacevir){

if(datacevir==1){
return '1';
}
if(datacevir==2){
return '2';
}
if(datacevir==3){
return '3';
}
	if(datacevir==4){
return '4';
}
	if(datacevir==5){
return '5';
}
	if(datacevir==6){
return '6';
}
	if(datacevir==7){
return '7';
}
	if(datacevir==8){
return '8';
}
	if(datacevir==9){
return '9';
}
	if(datacevir==0){
return '0';
}
	else
	{
	return 0;
	}
}

char read_keypad (void)
	{
	/* Make ROW 1 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_RESET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		birakma_flag1=0;
		return 1;
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		birakma_flag1=0;
		return 2;
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		birakma_flag1=0;
		return 3;
	}

	/* Make ROW 2 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_RESET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		birakma_flag1=0;
		return 4;
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		birakma_flag1=0;
		return 5;
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		birakma_flag1=0;
		return 6;
	}
	
	/* Make ROW 3 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_RESET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_SET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		birakma_flag1=0;
		return 7;
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		birakma_flag1=0;
		return 8;
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		birakma_flag1=0;
		return 9;
	}
		
	/* Make ROW 4 LOW and all other ROWs HIGH */
	HAL_GPIO_WritePin (R1_PORT, R1_PIN, GPIO_PIN_SET);  //Pull the R1 low
	HAL_GPIO_WritePin (R2_PORT, R2_PIN, GPIO_PIN_SET);  // Pull the R2 High
	HAL_GPIO_WritePin (R3_PORT, R3_PIN, GPIO_PIN_SET);  // Pull the R3 High
	HAL_GPIO_WritePin (R4_PORT, R4_PIN, GPIO_PIN_RESET);  // Pull the R4 High
	
	if (!(HAL_GPIO_ReadPin (C1_PORT, C1_PIN)))   // if the Col 1 is low
	{
		birakma_flag1=0;
		return 11;
	}
	
	if (!(HAL_GPIO_ReadPin (C2_PORT, C2_PIN)))   // if the Col 2 is low
	{
		birakma_flag1=0;
		return 10;
	}
	
	if (!(HAL_GPIO_ReadPin (C3_PORT, C3_PIN)))   // if the Col 3 is low
	{
		birakma_flag1=0;
		return 12;
	}
	else 
		return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
