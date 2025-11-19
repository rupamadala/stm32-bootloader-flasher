/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void); //type casting function pointer
typedef void (*pGoToAddr)(void); //type casting function pointer
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN(a,b) ((a)<(b) ? (a):(b))
#define RX_DATA_BUFFER_LENGTH 		250
#define PAGE_BUFFER_LEN				4096
#define FLASH_BOOT_ADDRESS			(uint32_t)0x8000000
#define FLASH_APPL_ADDRESS			(uint32_t)0x8100000
#define NUM_OF_PAGES_PER_BANK		(uint8_t)256
#define FLASH_BANK1_ADDRESS			(uint32_t)0x8000000
#define FLASH_BANK2_ADDRESS			(uint32_t)0x8100000

#define DATA_INDEX					6
#define UART_ACK					0x79
#define UART_NACK					0x81

//Boot loader Host Commands
#define FLASH_ERASE_CMD 			0
#define GO_TO_ADDR_CMD 				1
#define FLASH_WRITE_MEM_CMD			2
#define FLASH_READ_CMD				3
#define END_OF_FILE_CMD				4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
bool page_break = false;
static uint8_t rx_data[RX_DATA_BUFFER_LENGTH];
uint16_t temp_buf_length;
static uint8_t temp_buf[PAGE_BUFFER_LEN];
static uint8_t page_buf[PAGE_BUFFER_LEN];
uint32_t page_start_address = 0xFFFFFFFF;   // Invalid value → forces first frame to load page
uint16_t bytes_filled = 0;
HAL_StatusTypeDef rx_buf_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void bootloader_mode();
uint8_t erase_flash_page(uint32_t Bank, uint8_t start_Page, uint32_t Num_of_Pages);
uint8_t get_page_num(uint32_t address);
void process_rx_data(uint8_t *rx_buf, uint8_t data_length);
void write_flash_page(uint32_t address, uint8_t *pg_buf);
uint8_t check_crc(uint8_t *data);
void jump_to_application();
uint8_t read_back_flash(uint64_t data, uint32_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void jump_to_application(){
	//firstly fetch application address and store in MSP local variable
	HAL_UART_DeInit(&hlpuart1);
	HAL_GPIO_DeInit(Upload_firmware_GPIO_Port, Upload_firmware_Pin);
	pFunction appl_reset_handler;
	uint32_t msp = *((volatile uint32_t*) (FLASH_APPL_ADDRESS));
	//then fetch address of application's reset_handler() and assign it to function pointer.
	appl_reset_handler = (pFunction)(*((volatile uint32_t*)(FLASH_APPL_ADDRESS + 4)));

	__disable_irq();
	//de-init RCC clock Configuration -- i.e reset it to default state
	HAL_RCC_DeInit();
	HAL_DeInit();
	SCB->VTOR = FLASH_APPL_ADDRESS;
	//Set msp register with application address
	__set_MSP(msp);
	__enable_irq();

	//Call appl reset_handler function -- i.e jumping to application
	appl_reset_handler();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	SCB->VTOR = FLASH_BOOT_ADDRESS;
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
  //MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);
if(HAL_GPIO_ReadPin(Upload_firmware_GPIO_Port, Upload_firmware_Pin) == GPIO_PIN_SET){
   bootloader_mode();
}
else{
	jump_to_application();
}
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : Upload_firmware_Pin */
  GPIO_InitStruct.Pin = Upload_firmware_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Upload_firmware_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void bootloader_mode(){
	uint8_t frame_len = 0;
	MX_LPUART1_UART_Init();
	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		//HAL_Delay(500);
		//fetch rx data
		rx_buf_status = HAL_UART_Receive(&hlpuart1, rx_data, 1, HAL_MAX_DELAY);
		if(rx_buf_status != HAL_OK){
			HAL_UART_Transmit(&hlpuart1, (uint8_t *) &rx_buf_status, 1, HAL_MAX_DELAY);
		}
		else{
			frame_len = rx_data[0];
			rx_buf_status = HAL_UART_Receive(&hlpuart1, &rx_data[1], frame_len, HAL_MAX_DELAY);
			if(rx_buf_status != HAL_OK){
				HAL_UART_Transmit(&hlpuart1, (uint8_t *) &rx_buf_status, 1, HAL_MAX_DELAY);
			}
			else{
				process_rx_data(rx_data, frame_len+1);
			}
		}
	}
}

void write_flash_page(uint32_t address, uint8_t *pg_buf){
	HAL_FLASH_Unlock();
	HAL_Delay(1);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	uint64_t data;
	for(int j=0; j<PAGE_BUFFER_LEN; j+=8){
		//Flash writes in little endian order which means LSB goes to lowest address and MSB goes to highest address
		// for eg 88 77 66 55 44 33 22 11 is a 64 bit double word, then 11(LSB) goes to address+0, 88(MSB) goes to address+7, when we use Flash_Program API.
		data = 0;
		for(int i=0;i<8;i++){
			data |= ((uint64_t) (pg_buf[j+i])) << (8*i);
		}
		HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);

		if(status != HAL_OK) {
			uint32_t err = HAL_FLASH_GetError();
			printf("Flash write error @0x%08lX, err=0x%lX\r\n", address, err);
			uint8_t tx_msg = UART_NACK;
			HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
			HAL_FLASH_Lock();
			break;
		}
		else{
			address+=8;
		}
	}
	HAL_FLASH_Lock();
}

uint8_t check_crc(uint8_t *data){
	uint8_t data_length = data[0];
	uint8_t crc = 0;
	uint8_t crc_status;
	for(int i = 1; i< (data_length); i++){
		crc += data[i];
	}
	crc &= 0xFF;

	if(crc != data[data_length]){
		crc_status = UART_NACK; //NACK
	}
	else{
		crc_status = UART_ACK; //ACK
	}

return crc_status;
}

uint8_t erase_flash_page(uint32_t Bank, uint8_t start_Page, uint32_t Num_of_Pages){
	FLASH_EraseInitTypeDef pEraseInit;
	uint32_t PageError;
	uint8_t erase_status;
	pEraseInit.Banks 		= Bank;
	pEraseInit.Page			= start_Page;
	pEraseInit.NbPages		= Num_of_Pages;
	pEraseInit.TypeErase	= FLASH_TYPEERASE_PAGES;
	HAL_FLASH_Unlock();
	HAL_Delay(1);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	// Poll BSY flag just to ensure completion
	while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)){
		// optional small wait
	}
	if(status == HAL_OK){
		erase_status = UART_ACK;
	}
	else {
		erase_status = UART_NACK;
		printf("Page Error: %lu\n",PageError);
		printf("Erase failed! Error code: %lu\r\n", HAL_FLASH_GetError());
	}
	HAL_FLASH_Lock();
return erase_status;
}

uint8_t get_page_num(uint32_t address){
	uint8_t page_num;
	if(address < FLASH_BANK2_ADDRESS){
		page_num = (address - FLASH_BANK1_ADDRESS)/(PAGE_BUFFER_LEN);
	}
	else{
		page_num = (address - FLASH_BANK2_ADDRESS)/(PAGE_BUFFER_LEN);
	}
	return page_num;
}

void process_rx_data(uint8_t *rx_buf, uint8_t rx_buf_len){
	uint8_t cmd = rx_buf[1];
	uint8_t tx_msg;
	uint32_t address = 0x0;
	uint32_t start_address = 0x0;
	uint32_t end_address = 0x0;

	switch(cmd){
	case FLASH_ERASE_CMD:
		start_address = (rx_buf[2] << 24) | (rx_buf[3] << 16) | (rx_buf[4] << 8) | (rx_buf[5]);
		end_address = (rx_buf[6] << 24) | (rx_buf[7] << 16) | (rx_buf[8] << 8) | (rx_buf[9]);

		uint8_t start_page_num = get_page_num(start_address);
		uint8_t end_page_num = get_page_num(end_address);

		if((start_address%8 == 0) || (end_address%8 == 0)){
			if(start_address < FLASH_BANK2_ADDRESS){
				if(end_address < FLASH_BANK2_ADDRESS){ //Erasing in Flash Bank 1 Only
					tx_msg = erase_flash_page(FLASH_BANK_1, start_page_num, ((end_page_num - start_page_num) + 1));
				}
				else{ //Erasing in both Flash Bank 1 and Flash Bank 2
					tx_msg = erase_flash_page(FLASH_BANK_1, start_page_num, (NUM_OF_PAGES_PER_BANK - start_page_num));
					if(tx_msg == UART_ACK){
						tx_msg = erase_flash_page(FLASH_BANK_2, 0, (end_page_num + 1));
						if(tx_msg == UART_NACK){
							HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
							break;
						}
					}
					else{
						HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
						break;
						}
				}
			}
			else{ //Erasing in Flash Bank 2
				tx_msg = erase_flash_page(FLASH_BANK_2, start_page_num, ((end_page_num - start_page_num) + 1));
				if(tx_msg == UART_NACK){
					HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
					break;
				}
			}
		}

		else{
			tx_msg = UART_NACK;
			HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
			break;
		}

		tx_msg = check_crc(rx_buf);
		HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
		break;
	case GO_TO_ADDR_CMD:
		address = (rx_buf[2] << 24) | (rx_buf[3] << 16) | (rx_buf[4] << 8) | (rx_buf[5]);
		tx_msg = UART_ACK;
		HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
		pFunction appl_reset_handler;
		uint32_t msp = *((volatile uint32_t*) (address));
		//then fetch address of application's reset_handler() and assign it to function pointer.
		appl_reset_handler = (pFunction)(*((volatile uint32_t*)(address + 4)));


		__disable_irq();
		HAL_UART_DeInit(&hlpuart1);
		HAL_GPIO_DeInit(Upload_firmware_GPIO_Port, Upload_firmware_Pin);
		// Turn ON MSI so app clock init won't fail
		RCC->CR |= RCC_CR_MSION;
		while(!(RCC->CR & RCC_CR_MSIRDY));

		//de-init RCC clock Configuration -- i.e reset it to default state
		HAL_RCC_DeInit();
		HAL_DeInit();
		SCB->VTOR = address;
		//Set msp register with application address
		__set_MSP(msp);
		__enable_irq();

		//Call appl reset_handler function -- i.e jumping to application
		appl_reset_handler();
		while(1); //bootloader freezes here forever

		break;
	case FLASH_WRITE_MEM_CMD:
		//parsing the data from frame
		address = (rx_buf[2] << 24) | (rx_buf[3] << 16) | (rx_buf[4] << 8) | (rx_buf[5]);
		uint32_t new_page_start;
		new_page_start = (address) & (~(PAGE_BUFFER_LEN - 1));
		tx_msg = check_crc(rx_buf);

		uint8_t data_length = (rx_buf[0]- 6);
		uint16_t offset = address - new_page_start;

		if(new_page_start != page_start_address){
			if(bytes_filled>0){
				write_flash_page(page_start_address, page_buf);
			}
			page_start_address = new_page_start;
			//copy old data first into page_buffer
			//Load new page
			memcpy(&page_buf[0], (uint8_t *)page_start_address, PAGE_BUFFER_LEN);
			bytes_filled = 0;

			//checks if any split frame and append data accordingly
			if(page_break){
				memcpy(&page_buf[0], temp_buf, temp_buf_length);
				bytes_filled += temp_buf_length;
				page_break = false;
				temp_buf_length = 0;
			}
		}

		//uint16_t write_index = offset + bytes_filled;
		uint16_t write_index = offset;
		uint16_t space_in_page = PAGE_BUFFER_LEN - write_index;

		if(data_length <= space_in_page){
			memcpy(&page_buf[write_index], &rx_buf[DATA_INDEX], data_length);
			bytes_filled += data_length;
		}
		else{
			memcpy(&page_buf[write_index], &rx_buf[DATA_INDEX], space_in_page);
			bytes_filled += space_in_page;
			temp_buf_length = data_length - space_in_page;
			memcpy(temp_buf, &rx_buf[DATA_INDEX + space_in_page], temp_buf_length);
			page_break = true;
		}

		if (bytes_filled >= PAGE_BUFFER_LEN) {
		    write_flash_page(page_start_address, page_buf);
			bytes_filled = 0;
		}

		HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
		break;
	case FLASH_READ_CMD:
		address = (rx_buf[2] << 24) | (rx_buf[3] << 16) | (rx_buf[4] << 8) | (rx_buf[5]);
		//Check address
		if(address%8 != 0){
			tx_msg = UART_NACK;
			HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
			break;
		}
		uint8_t read_data_buf[8];
		memcpy(read_data_buf, (uint8_t *)address, 8);
		HAL_UART_Transmit(&hlpuart1, read_data_buf, 8, HAL_MAX_DELAY);
		HAL_Delay(2);
		tx_msg = UART_ACK;
		HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
		break;
	case END_OF_FILE_CMD:
		write_flash_page(page_start_address, page_buf);
		tx_msg = UART_ACK;
		HAL_UART_Transmit(&hlpuart1, &tx_msg, 1, HAL_MAX_DELAY);
		break;
	}

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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
