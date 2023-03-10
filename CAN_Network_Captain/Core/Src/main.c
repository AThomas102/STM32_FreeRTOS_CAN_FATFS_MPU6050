/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "mpu6050.h"
#include "csv.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions
#include <stdlib.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId SDCardSaveTaskHandle;
osThreadId GetData1Handle;
osThreadId GetData2Handle;
osThreadId ReadCANHandle;
osThreadId GetAccelTaskHandle;
osMutexId canTelemetryMutexHandle;
/* USER CODE BEGIN PV */
MPU6050_t MPU6050;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void const * argument);
void StartSDCardSaveTask(void const * argument);
void StartGetData1(void const * argument);
void StartGetData2(void const * argument);
void StartReadCAN(void const * argument);
void StartGetAccelTask(void const * argument);

/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FATFS variables
FATFS 		FatFs;
FIL 		rwfile;
FILINFO*	filInfo;
FRESULT 	fres;                 //Result after operations
uint8_t		OVERWRITE = 1;		  // Overwrite files in the directory


// RTOS task variables
int16_t failsafe = 3000;
int16_t DefaultTask_delay = 3000;
int16_t	SDCardSave_delay = 3000;
int16_t	read_delay = 500;
int16_t	accel_read_delay = 200;
int16_t retransmit_delay = 1000;

typedef uint32_t TaskProfiler;
TaskProfiler DefaultProfiler;
uint32_t current_time;				// HAL get tick time since restart
uint32_t accel_data_time;			// change to a single value taskQueue?

// CAN variables
CAN_HandleTypeDef     	CanHandle;
CAN_RxHeaderTypeDef		RxHeader;
uint8_t               	RxData[4];	// global CAN received data
uint32_t              	TxMailbox;	// global CAN transmission mailbox
CAN_FilterTypeDef 		canfil; //CAN Bus Filter
// flag variables for CAN data received, these can be made into signals using osSignalSet()
uint8_t data1collected = 0;
uint8_t accel_data_collected = 0;
// Accelerometer measurement variables, can this be done in DMA?
float Ax, Ay, Az, Gx, Gy, Gz = 0;

//m
//char buffer[100];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  myprintf("------- CAN Captain Controller ----------\r\n");

  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan,&canfil) != HAL_OK){
	Error_Handler();
  }
  if (HAL_CAN_Start(&hcan) != HAL_OK){
	Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){
	Error_Handler();
  }


  myprintf("SD Card Connecting\r\n");
  fres = f_mount(&FatFs, "0", 1);    //1=mount now
  if (fres != FR_OK)
  {
  myprintf("No SD Card found : (%i)\r\n", fres);
  }
  else {
  myprintf("SD Card Mounted Successfully!!!\r\n");
  }
  f_mount(NULL, "0", 0);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of canTelemetryMutex */
  osMutexDef(canTelemetryMutex);
  canTelemetryMutexHandle = osMutexCreate(osMutex(canTelemetryMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SDCardSaveTask */
  osThreadDef(SDCardSaveTask, StartSDCardSaveTask, osPriorityHigh, 0, 256);
  SDCardSaveTaskHandle = osThreadCreate(osThread(SDCardSaveTask), NULL);

  /* definition and creation of GetData1 */
  osThreadDef(GetData1, StartGetData1, osPriorityBelowNormal, 0, 256);
  GetData1Handle = osThreadCreate(osThread(GetData1), NULL);

  /* definition and creation of GetData2 */
  osThreadDef(GetData2, StartGetData2, osPriorityBelowNormal, 0, 128);
  GetData2Handle = osThreadCreate(osThread(GetData2), NULL);

  /* definition and creation of ReadCAN */
  osThreadDef(ReadCAN, StartReadCAN, osPriorityNormal, 0, 128);
  ReadCANHandle = osThreadCreate(osThread(ReadCAN), NULL);

  /* definition and creation of GetAccelTask */
  osThreadDef(GetAccelTask, StartGetAccelTask, osPriorityBelowNormal, 0, 256);
  GetAccelTaskHandle = osThreadCreate(osThread(GetAccelTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  //osThreadDef(CANMailboxCheck, StartCANMailboxCheck, osPriorityNormal, 0, 128);


  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x0000020B;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SP1_CS_GPIO_Port, SP1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SP1_CS_Pin */
  GPIO_InitStruct.Pin = SP1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SP1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int dataShift = 10;
int dataPacket = 1;

void myprintf(const char *fmt, ...) {

	char *buffer = malloc(100);

	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, 100, fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000);
	free(buffer);
}

char* int_to_string(int num) {
    int len = snprintf(NULL, 0, "%.10d", num);
    char* str = malloc(len + 1);
    snprintf(str, len + 1, "%.10d", num);
    return str;
}

void printCANMessage(CAN_RxHeaderTypeDef Header, uint8_t data[]){
	char dataStr[20];
	myprintf("canMsg = 0x%lx %li", Header.StdId, Header.DLC);
	for (int i = 0; i<Header.DLC; i++)  {  // print the data
		myprintf(dataStr, " %x", data[i]);
	}
	myprintf("\r\n");
}

void print_buffer(CSV_BUFFER *buffer){
	int i, j;
	myprintf("\n");
	for (i = 0; i < buffer->rows; i++) {
		for (j = 0; j < buffer->width[i]; j++) {
			myprintf("%c%s%c%c", buffer->text_delim, buffer->field[i][j]->text, buffer->text_delim, buffer->field_delim);
		}
		myprintf("\r\n");
	}
	myprintf("\r\n");
}

// float to string
void ftstr(char* str, float num) {
	int whole = (int)num;
	int dec = (int)((num - whole) * 1000 + 0.5); // 3 decimal places
	if (dec < 0) {
	        dec = -dec;
	    }
	if (whole < 99999){
		sprintf(str, "%d.%d", whole, dec);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	CAN_TxHeaderTypeDef   TxHeader;

	// CAN default data
	TxHeader.StdId = 0x222;
	TxHeader.ExtId = 0x00;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 4;
	TxHeader.TransmitGlobalTime = DISABLE;

	uint8_t freeMailboxs;
	uint8_t data[10] = "bcad";

	/* Infinite loop */
	for(;;)
	{
		//myprintf(" -- STARTING thread DefaultTask -- \r\n");
		DefaultProfiler++;

//		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox) != HAL_OK){
//			Error_Handler();
//		}
//		freeMailboxs = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
//		myprintf("packet sent: %s\r\n", data);
//		myprintf("num free mailboxs: %i\r\n", freeMailboxs);
//		myprintf("text mailbox used: %ld\r\n", TxMailbox);

		//myprintf(" -- ENDING thread DefaultTask -- \r\n");
		osDelay(DefaultTask_delay);	// Task is put to sleep for 500ms
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSDCardSaveTask */
/**
* @brief Function implementing the SDCardSaveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSDCardSaveTask */
void StartSDCardSaveTask(void const * argument)
{
  /* USER CODE BEGIN StartSDCardSaveTask */
  uint16_t current_row = 0;
  uint8_t file_number = 1;
  TCHAR file_path[30];
  char string_value[20] = {0};

  sprintf(file_path, "0:/csv/test%hu.csv", file_number);
  /* Infinite loop */
  for(;;)
  {
	myprintf(" -- STARTING thread SDCardSave -- \r\n");

	myprintf("Mounting SD card\r\n");
		fres = f_mount(&FatFs, "0", 1);
		if (fres != FR_OK){
			myprintf("f_mount problem: %d\r\n", fres);
		}
		else if (file_number < 10){
			myprintf("SD Card Mounted!\r\n");

			if (current_row > 10 || file_number == 1){
				current_row = 0;
				do {
					sprintf(file_path, "0:/csv/test%hu.csv", file_number);
					fres = f_stat(file_path, filInfo);
					file_number++;
					if (fres != FR_OK) break;
					myprintf("filepath exists: %s\r\n",file_path);
				} while(fres != FR_NO_FILE && !OVERWRITE);
			}

			myprintf("using filepath: %s\r\n",file_path);

			// read file, else if doesnt exist create new file
			fres = f_open(&rwfile, file_path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
			if (fres != FR_OK)
			{
				myprintf("f_open problem: %d\r\n", fres);
			}
			else{
				myprintf("Creating buffer\r\n");
				CSV_BUFFER *buffer = csv_create_buffer();

				if (current_row == 0){
					// first write to the file requires the data headings to be defined
					myprintf("Writing data headings \r\n");
					csv_set_field(buffer, 0, 0, "Time (ms)");
					csv_set_field(buffer, 0, 1, "Accel x (m/s2)");
					csv_set_field(buffer, 0, 2, "Accel y (m/s2)");
					csv_set_field(buffer, 0, 3, "Accel z (m/s2)");
					csv_set_field(buffer, 0, 4, "Roll x (m/s2)");
					csv_set_field(buffer, 0, 5, "Roll y (m/s2)");
					csv_set_field(buffer, 0, 6, "Roll z (m/s2)");
					csv_set_field(buffer, 0, 7, "Temp (C)\r");
					current_row++;
				}
				else{
					myprintf("Loading csv\r\n");
					csv_load(buffer, &rwfile);
				}

				myprintf("editing csv\r\n");
				// add a row of measurement values
				if(accel_data_collected){			// set MPU6050 values
					memset(string_value, 0, sizeof(string_value));
					sprintf(string_value, "%li", accel_data_time);
					csv_set_field(buffer, current_row, 0, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Ax);
					csv_set_field(buffer, current_row, 1, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Ay);
					csv_set_field(buffer, current_row, 2, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Az);
					csv_set_field(buffer, current_row, 3, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Gx);
					csv_set_field(buffer, current_row, 4, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Gy);
					csv_set_field(buffer, current_row, 5, string_value);
					memset(string_value, 0, sizeof(string_value));
					ftstr(string_value, Gz);
					csv_set_field(buffer, current_row, 6, string_value);
					// final character placeholder
					// the \r is needed for csv_set_field termination of the row
					csv_set_field(buffer, current_row+1, 0, "end\r");
					current_row++;
				}
				print_buffer(buffer);
				csv_save(&rwfile, buffer);

				f_mount(NULL, "0", 0);

				f_close(&rwfile);
				csv_destroy_buffer(buffer);
				}
			}
		else{
			myprintf("file write completed\r\n");
		}

	// restart CAN telemetry threads
	//myprintf(" -- RESUMING thread GetData1 -- \r\n");
	osThreadResume(GetData1Handle);
	//myprintf(" -- RESUMING thread GetData2 -- \r\n");
	osThreadResume(GetData2Handle);
	// reset data collection
	//myprintf(" -- RESUMING thread GetAccel -- \r\n");
	osThreadResume(GetAccelTaskHandle);
	accel_data_collected = 0;
	myprintf(" -- ENDING thread SDCardSave -- \r\n");
	osDelay(SDCardSave_delay);
  }
  /* USER CODE END StartSDCardSaveTask */
}

/* USER CODE BEGIN Header_StartGetData1 */
/**
* @brief Function implementing the GetData1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetData1 */
void StartGetData1(void const * argument)
{
  /* USER CODE BEGIN StartGetData1 */
	CAN_TxHeaderTypeDef   TxHeader1;

	// can transmission 1
	TxHeader1.StdId = 0x201;
	TxHeader1.ExtId = 0x00;
	TxHeader1.RTR = CAN_RTR_DATA;
	TxHeader1.IDE = CAN_ID_STD;
	TxHeader1.DLC = 4;
	TxHeader1.TransmitGlobalTime = DISABLE;


	uint8_t TransmissionData1[8] = "dat1";
	uint8_t node1req[4] = {0x77, 0x6f, 0x6f, 0x66};

	/* Infinite loop */
  for(;;)
  {
	  myprintf(" -- STARTING thread GetData1 -- \r\n");
	  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
		  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader1, TransmissionData1, &TxMailbox) == HAL_OK){
			  myprintf("CAN message sent to data1\r\n");}
		  else{
			  Error_Handler();
		  }
	  }
	  osDelay(10);	// response delay
	  // continue retransmission until data1 is collected
	  // change this to wait for a flag change from ReadCAN
	  if (RxHeader.StdId == 0x206 &&
		  RxData[0] == node1req[0] &&
		  RxData[1] == node1req[1] &&
	      RxData[2] == node1req[2] &&
		  RxData[3] == node1req[3]){
		  // process data1...
		  myprintf("CAN response from node1\r\n");
		  myprintf("processing...\r\n");
		  myprintf(" -- SUSPENDING thread GetData1 -- \r\n");
		  osThreadSuspend(GetData1Handle);
	  }
	  else{
		  myprintf("no CAN response, delaying thread GetData1\r\n");
		  current_time = HAL_GetTick();
		  myprintf("current time: %lu\r\n", current_time);
		  myprintf(" -- ENDING thread GetData1 -- \r\n");
		  osDelay(retransmit_delay);
	  }

  }
  /* USER CODE END StartGetData1 */
}

/* USER CODE BEGIN Header_StartGetData2 */
/**
* @brief Function implementing the GetData2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetData2 */
void StartGetData2(void const * argument)
{
  /* USER CODE BEGIN StartGetData2 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(retransmit_delay);
  }
  /* USER CODE END StartGetData2 */
}

/* USER CODE BEGIN Header_StartReadCAN */
/**
* @brief Function implementing the ReadCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadCAN */
void StartReadCAN(void const * argument)
{
  /* USER CODE BEGIN StartReadCAN */
	/* Infinite loop */
	for(;;)
	{
		//myprintf(" -- STARTING thread ReadCAN -- \r\n");
		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)){
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);	//LED shows a CAN message read
			printCANMessage(RxHeader, RxData);

//			if (RxHeader.StdId == 0x206){
//				myprintf("data request response from node1 of id 0x206\r\n");
//				data1collected = 1;	// can be changed into thread signal/flag
//			}
		}
		//myprintf(" -- ENDING thread ReadCAN -- \r\n");
		osDelay(read_delay);
	}
  /* USER CODE END StartReadCAN */
}

/* USER CODE BEGIN Header_StartGetAccelTask */
/**
* @brief Function implementing the GetAccelTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetAccelTask */
void StartGetAccelTask(void const * argument)
{
  /* USER CODE BEGIN StartGetAccelTask */
  /* Infinite loop */
	uint8_t accel_cnt = 0;
	MPU6050_Init(&hi2c3);
  /* Infinite loop */
  for(;;)
  {
	myprintf(" -- running MPU6050 measurement task -- \r\n");
	Ax = 0;
	Ay = 0;
	Az = 0;
	Gx = 0;
	Gy = 0;
	Gz = 0;
	// take 4 accel_cnt to average out the value
	for(int i = 0; i < 4; i++){
		if (!MPU6050_Read_All(&hi2c3, &MPU6050)){
			myprintf("fetching.. ");
			Ax += MPU6050.Ax;
			Ay += MPU6050.Ay;
			Az += MPU6050.Az;
			Gx += MPU6050.Gx;
			Gy += MPU6050.Gy;
			Gz += MPU6050.Gz;
			accel_cnt++;
		}
	}
	if(accel_cnt == 4){
		myprintf(" Data collected.\r\n");
		accel_data_collected = 1;
		accel_data_time = HAL_GetTick();
		Ax += Ax/4;
		Ay += Ay/4;
		Az += Az/4;
		Gx += Gx/4;
		Gy += Gy/4;
		Gz += Gz/4;
		osThreadSuspend(GetAccelTaskHandle);
	}
	else{
		MPU6050_Init(&hi2c3);
	}
	accel_cnt = 0;
	osDelay(accel_read_delay);
	}

  /* USER CODE END StartGetAccelTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  myprintf("HAL error occurred!\r\n"); // print
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
