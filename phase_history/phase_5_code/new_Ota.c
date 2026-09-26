/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : OTA.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// [추가] 앱 유효 표식 (Flash 맨 끝 16바이트)
// magic은 맨 마지막에 기록 -> 기록 도중 전원이 끊겨도 표식은 무효로 남음
typedef struct {
	uint32_t magic;      // META_MAGIC
	uint32_t size;       // 검증된 이미지 크기
	uint32_t crc;        // 검증된 CRC32
	uint32_t magic_inv;  // ~META_MAGIC (이중 확인)
} AppMeta;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDRESS 0x08004000 //Application 시작 주소 (Sector 1) 이게 진짜 중요함
#define CHUNK_SIZE 256 //한번에 수신하는 바이트
#define UPDATE_TIMEOUT_MS 3000 //"UPDATE" 대기 시간 (3초)
#define ACK_TIMEOUT_MS 5000  // 청크 수신 타임아웃 (5초)

// [추가] 유효 표식 위치: Sector 7 마지막 16바이트 (0x0807FFF0)
// Sector 7은 업데이트 시작 때 앱 영역과 함께 지워지므로, 새 이미지가 CRC를 통과해야만 다시 기록됨
#define META_ADDRESS 0x0807FFF0
#define META_MAGIC   0x5A5AA5A5U

// [변경] 마지막 청크의 0xFF 패딩이 표식 영역을 덮지 않도록 한 청크만큼 여유를 둠
#define MAX_APP_SIZE (((512-16) * 1024) - CHUNK_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

//함수 프로토타입 추가
void SendString(const char* str);
void JumpToApp(void);
int WaitForUpdateSignal(uint32_t timeout_ms);
HAL_StatusTypeDef EraseAppFlash(void);
HAL_StatusTypeDef WriteChunkToFlash(uint32_t address, uint8_t* data, uint32_t len);
uint32_t CalculateFlashCRC(uint32_t start_addr, uint32_t size);
int IsAppValid(void);                                        // [추가]
HAL_StatusTypeDef WriteAppMeta(uint32_t size, uint32_t crc); // [추가]

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  //LED 켜기 (부트로더 진입 표시)
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  // Step1: 업데이트 신호 대기 (3초)
  if(!WaitForUpdateSignal(UPDATE_TIMEOUT_MS)){
	  // [변경] 신호 없음 -> 검증된 앱일 때만 점프
	  if(IsAppValid()){
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  JumpToApp();
	  }

	  // [변경] 유효한 앱 없음(전송 중단·CRC 불일치·빈 영역) -> 실행하지 않고 업데이트 대기
	  // 기존: LED 점멸 무한 루프(리셋 전까지 업데이트 불가)
	  SendString("NO_APP\r\n");
	  while(!WaitForUpdateSignal(1000)){
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // 1초 주기 점멸
	  }
  }

  //Step 2: 업데이트 모드 진입
  SendString("READY\r\n");

  //Step3: 파일 크기 수신 (4바이트, little-endian)
  uint32_t total_size = 0;
  if(HAL_UART_Receive(&huart2, (uint8_t*)&total_size, 4, ACK_TIMEOUT_MS) != HAL_OK) {
	  SendString("ERR_SIZE\r\n");
	  NVIC_SystemReset();
  }

  if(total_size == 0 || total_size > MAX_APP_SIZE){
	  SendString("ERR_RANGE\r\n");
	  NVIC_SystemReset();
  }
  SendString("ACK\r\n");

  //Step4: Flash Erase (Sector 1~7, 유효 표식도 함께 지워짐)
  HAL_FLASH_Unlock();

  if(EraseAppFlash() != HAL_OK){
	  HAL_FLASH_Lock();
	  SendString("ERR_ERASE\r\n");
	  NVIC_SystemReset();
  }
  SendString("ACK\r\n");

  //Step5: 바이너리 수신 & Flash Write
  uint32_t received = 0;
  uint8_t chunk[CHUNK_SIZE];

  while(received < total_size){
	  uint32_t to_read = total_size - received;
	  if(to_read > CHUNK_SIZE) to_read = CHUNK_SIZE;

	  //256바이트 단위로 수신 (마지막 청크는 Python이 0xFF 패딩)
	  if(HAL_UART_Receive(&huart2, chunk, CHUNK_SIZE, ACK_TIMEOUT_MS) != HAL_OK){
		  HAL_FLASH_Lock();
		  SendString("ERR_RX\r\n");
		  NVIC_SystemReset(); // 표식이 없으므로 재부팅 후 점프하지 않음
	  }

	  //Flash에 기록
	  if(WriteChunkToFlash(APP_ADDRESS + received, chunk, CHUNK_SIZE) != HAL_OK) {
		  HAL_FLASH_Lock();
		  SendString("ERR_WRITE\r\n");
		  NVIC_SystemReset();
	  }

	  received += to_read;

	  //ACK 전송 (Python이 다음 청크 전송)
	  SendString("ACK\r\n");

	  //LED 토글 (진행표시)
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }

  HAL_FLASH_Lock();

  //Step6: CRC검증 (Flash에서 다시 읽어 계산)
  uint32_t expected_crc = 0;
  if(HAL_UART_Receive(&huart2, (uint8_t*)&expected_crc, 4, ACK_TIMEOUT_MS) != HAL_OK) {
	  SendString("ERR_CRC_RX\r\n");
	  NVIC_SystemReset();
  }

  uint32_t actual_crc = CalculateFlashCRC(APP_ADDRESS, total_size);

  if(actual_crc != expected_crc){
	  SendString("NACK\r\n");
	  NVIC_SystemReset(); //CRC불일치 -> 표식 없이 재부팅 -> 업데이트 대기
  }

  // [추가] Step7: CRC 통과 -> 유효 표식 기록
  HAL_FLASH_Unlock();
  if(WriteAppMeta(total_size, actual_crc) != HAL_OK){
	  HAL_FLASH_Lock();
	  SendString("ERR_META\r\n");
	  NVIC_SystemReset();
  }
  HAL_FLASH_Lock();

  //Step8 : 성공 -> 재부팅
  SendString("DONE\r\n");
  HAL_Delay(100);
  NVIC_SystemReset();
  //재부팅 후 -> 부트로더 -> 3초대기 -> 신호 없음 -> 표식·CRC 확인 -> JumpToApp -> 새 펌웨어

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 1. UART 문자열 전송 유틸리티
void SendString(const char* str)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 100);
}

// 2. Application으로 점프
typedef void (*pFunction)(void);
void JumpToApp(void)
{
	//(1) App 영역의 첫 4바이트 = Initial Stack Pointer
	uint32_t app_sp = *(volatile uint32_t*)APP_ADDRESS;

	//(2) 그 다음 4바이트 = Reset Handler 주소
	uint32_t app_entry = *(volatile uint32_t*)(APP_ADDRESS + 4);

	//(3) SP가 유효한 SRAM 범위인지 검증
	// STM32F411RE의 SRAM: 0X2000_0000 ~ 0X2002_0000 (128KB)
	if((app_sp < 0x20000000) || (app_sp > 0x20020000)) {
		return; //App 이 없거나 손상됨 -> 부트로더에 머무름
	}

	//(4) 모든 인터럽트 비활성화
	__disable_irq();

	//(5) SysTick 정지 (HAL이 사용하던 타이머)
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	//(6) Vector Table을 App 주소로 재설정
	SCB->VTOR = APP_ADDRESS;

	//(7) Stack Pointer 설정
	__set_MSP(app_sp);

	//(8) App의 Reset Handler로 점프 (돌아오지 않음)
	pFunction app_reset = (pFunction)app_entry;
	app_reset();
}

// 3. "UPDATE\n" 신호 대기
int WaitForUpdateSignal(uint32_t timeout_ms)
{
	uint8_t rx_buf[8] = {0};
	uint32_t start = HAL_GetTick();
	uint8_t idx = 0;

	while((HAL_GetTick() - start) < timeout_ms) {
		uint8_t byte;
		// 10ms 타임아웃으로 1바이트씩 수신 시도
		if (HAL_UART_Receive(&huart2, &byte, 1, 10) == HAL_OK) {
			if(byte == '\n'){
				rx_buf[idx] = '\0';
				if(strcmp((char*)rx_buf, "UPDATE")==0){
					return 1; //업데이트 요청 확인
				}
				idx = 0; //다른 문자열이면 버퍼 리셋
			} else if (idx < 7) {
				rx_buf[idx++] = byte;
			}
		}
	}
	return 0; //타임아웃 -> 정상 부팅
}


// 4. Application Flash 영역 삭제
HAL_StatusTypeDef EraseAppFlash(void)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t sector_error;

	erase.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; //2.7~3.6V
	erase.Sector = FLASH_SECTOR_1; // Sector 0 은 절대 건드리지 않음 (부트로더) (1~7까지 지우기)
	erase.NbSectors = 7;

	return HAL_FLASHEx_Erase(&erase, &sector_error);
}

// 5. 수신한 데이터를 Flash에 기록
HAL_StatusTypeDef WriteChunkToFlash(uint32_t address, uint8_t* data, uint32_t len)
{
	for(uint32_t i=0; i<len; i+=4){
		uint32_t word = *(uint32_t*)&data[i];
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+i, word) != HAL_OK){
			return HAL_ERROR;
		}
	}
	return HAL_OK;
}

// 6. CRC32 계산 (소프트웨어 구현)
// 원본 데이어와 동일한지 검증. python에 zlib.crc32()로 계산한 값과 비교할 예정
// CRC32는 구글링하면 됨
uint32_t CalculateFlashCRC(uint32_t start_addr, uint32_t size)
{
	uint32_t crc = 0xFFFFFFFF;
	uint8_t* ptr = (uint8_t*)start_addr;

	for(uint32_t i=0; i<size; i++){
		crc ^= ptr[i];
		for(int j=0; j<8; j++){
			crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
		}
	}
	return crc ^ 0xFFFFFFFF;
}

// 7. [추가] 부팅 시 앱 유효성 확인: 표식이 있고, 기록된 CRC와 현재 Flash 내용이 일치해야 통과
int IsAppValid(void)
{
	const volatile AppMeta* meta = (const volatile AppMeta*)META_ADDRESS;

	if(meta->magic != META_MAGIC || meta->magic_inv != ~META_MAGIC) {
		return 0; // 표식 없음: 전송 중단 또는 CRC 불일치로 끝난 업데이트
	}
	if(meta->size == 0 || meta->size > MAX_APP_SIZE) {
		return 0;
	}
	return (CalculateFlashCRC(APP_ADDRESS, meta->size) == meta->crc);
}

// 8. [추가] 유효 표식 기록 (CRC 통과 후에만 호출)
// magic을 가장 마지막에 기록: 도중에 전원이 끊기면 magic이 0xFFFFFFFF로 남아 무효 처리됨
HAL_StatusTypeDef WriteAppMeta(uint32_t size, uint32_t crc)
{
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, META_ADDRESS + 4,  size) != HAL_OK) return HAL_ERROR;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, META_ADDRESS + 8,  crc) != HAL_OK) return HAL_ERROR;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, META_ADDRESS + 12, ~META_MAGIC) != HAL_OK) return HAL_ERROR;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, META_ADDRESS + 0,  META_MAGIC) != HAL_OK) return HAL_ERROR;
	return HAL_OK;
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