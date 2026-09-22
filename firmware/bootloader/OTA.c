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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_ADDRESS 0x08004000 //Application 시작 주소 (Sector 1) 이게 진짜 중요함
#define CHUNK_SIZE 256 //한번에 수신하는 바이트. 
// 7/27 SRAM 128KB 중 부트로더가 쓸 수 있는 스택은 작다. RAM 사용량의 상한

#define UPDATE_TIMEOUT_MS 3000 //"UPDATE" 대기 시간 (3초)
// 별도 부팅 핀 없이 UART만으로 모드를 가르는 대가. 

#define ACK_TIMEOUT_MS 5000  // 청크 수신 타임아웃 (5초)
#define MAX_APP_SIZE ((512-16) * 1024)  //최대 App 크기 (496KB)
// 512KB=전체 Flash, 16KB=Sector 0(부트로더). 즉 앱이 쓸 수 있는 Sector 1~7의 합

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
  if(!WaitForUpdateSignal(UPDATE_TIMEOUT_MS)){ // 이 분기가 부트로더의 유일한 갈림길. 
	  //신호 없음 -> 정상 부팅, 신호 있음 -> 업데이트 모드
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  JumpToApp();

	  //JumpToApp실패 (App이 없음-그럴 일은 거의 없겠지만) -> LED 빠른 점멸로 표시
	  SendString("NO_APP\r\n");
	  while(1){
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  HAL_Delay(200);
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

  //Step4: Flash Erase
  HAL_FLASH_Unlock();

  if(EraseAppFlash() != HAL_OK){
	  HAL_FLASH_Lock();
	  SendString("ERR_ERASE\r\n");
	  NVIC_SystemReset();
  }
  SendString("ACK\r\n");

  //Step5: 바이너리 수신 & Flash Write
  uint32_t received = 0;
  uint8_t chunk[CHUNK_SIZE]; //부트로더가 쓰는 사실상 유일한 RAM 버퍼. 
  // 496KB 펌웨어를 128KB SRAM으로 처리하는 방법

  while(received < total_size){
	  uint32_t to_read = total_size - received;
	  if(to_read > CHUNK_SIZE) to_read = CHUNK_SIZE;

	  //256바이트 단위로 수신 (마지막 청크는 Python이 0xFF 패딩)
	  if(HAL_UART_Receive(&huart2, chunk, CHUNK_SIZE, ACK_TIMEOUT_MS) != HAL_OK){
		  HAL_FLASH_Lock();
		  SendString("ERR_RX\r\n");
		  NVIC_SystemReset();
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

  //Step6: CRC검증
  uint32_t expected_crc = 0;
  if(HAL_UART_Receive(&huart2, (uint8_t*)&expected_crc, 4, ACK_TIMEOUT_MS) != HAL_OK) {
	  SendString("ERR_CRC_RX\r\n");
	  NVIC_SystemReset();
  }

  uint32_t actual_crc = CalculateFlashCRC(APP_ADDRESS, total_size);
  // RAM이 아니라 Flash를 다시 읽어 계산한다. 전송 오류뿐 아니라 저장 오류까지 검증.

  if(actual_crc != expected_crc){
	  SendString("NACK\r\n");
	  NVIC_SystemReset(); //CRC불일치 -> 재부팅 -> 부트로더 재진입
  }

  //Step7 : 성공 -> 재부팅
  SendString("DONE\r\n");
  HAL_Delay(100);
  NVIC_SystemReset();
  //재부팅 후 -> 부트로더 -> 3초대기 -> 신호 없음 -> JumpToApp -> 새 펌웨어

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
  // 앱 Flash의 첫 4바이트가 왜 코드가 아니라 SP인가? 
  // -> 리셋 시 하드웨어가 하는 일을 소프트웨어로 재현
	uint32_t app_sp = *(volatile uint32_t*)APP_ADDRESS;

	//(2) 그 다음 4바이트 = Reset Handler 주소
  // 다음 4바이트 = Reset Handler 주소. 이 8바이트가 벡터 테이블의 앞머리
	uint32_t app_entry = *(volatile uint32_t*)(APP_ADDRESS + 4);

	//(3) SP가 유효한 SRAM 범위인지 검증
	// STM32F411RE의 SRAM: 0X2000_0000 ~ 0X2002_0000 (128KB)
  // F411RE의 SRAM 128KB 범위. 앱이 없으면 지워진 Flash라 0xFFFFFFFF가 읽히고 이 범위 밖 -> 점프 안 함
	if((app_sp < 0x20000000) || (app_sp > 0x20020000)) {
		return; //App 이 없거나 손상됨 -> 부트로더에 머무름
	}

	//(4) 모든 인터럽트 비활성화
  // 빼면? -> 점프 도중 인터럽트가 뜨면 아직 부트로더 벡터 테이블을 보는 상태에서 SP만 바뀐 어중간한 문맥으로 진입
	__disable_irq();

	//(5) SysTick 정지 (HAL이 사용하던 타이머)
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	//(6) Vector Table을 App 주소로 재설정
  // 가장 중요. 빼면? -> 앱은 도는데 인터럽트만 부트로더 핸들러로 가는, 원인 찾기 극도록 어려운 버그
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
  // "바이트 지우기"가 없는 이유 -> Flash 셀은 0->1 복원에 고전압이 필요해 섹터 통째로만 가능하다는 물리적 제약.
	erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; //2.7~3.6V. 
	erase.Sector = FLASH_SECTOR_1; // Sector 0 은 절대 건드리지 않음 (부트로더) (1~7까지 지우기)
  // 부트로더가 자기 자신을 지우지 않는 것이 이 시스템이 절대 벽돌이 되지 않는 이유.
	erase.NbSectors = 7; // 1부터 7까지=496KB. MAX_APP_SIZE와 같은 값

	return HAL_FLASHEx_Erase(&erase, &sector_error);
}

// 5. 수신한 데이터를 Flash에 기록
// 7/27 이 함수 호출 전에 반드시 erase가 끝나 있어야 한다.
// Flash는 1->0만 가능하므로 0xFF 상태가 아닌 곳에 쓰면 원하는 값이 안 나온다.
HAL_StatusTypeDef WriteChunkToFlash(uint32_t address, uint8_t* data, uint32_t len)
{
  // 왜 4씩 건너뛰나 -> FLASH_TYPEPROGRAM_WORD가 32비트 단위. 256바이트면 64회 호출.
	for(uint32_t i=0; i<len; i+=4){
		uint32_t word = *(uint32_t*)&data[i];
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address+i, word) != HAL_OK){
			return HAL_ERROR; // Flash 쓰기는 실패할 수 있는 연산이다. SRAM과 차이.
		}
	}
	return HAL_OK;
}

// 6. CRC32 계산 (소프트웨어 구현)
// 원본 데이터와 동일한지 검증. python에 zlib.crc32()로 계산한 값과 비교할 예정
// CRC32는 구글링하는 게 좋다
uint32_t CalculateFlashCRC(uint32_t start_addr, uint32_t size)
{
	uint32_t crc = 0xFFFFFFFF; // 초기값. -> 앞에 0이 몇 개 부어도 값이 달라지도록 만드는 장치
	uint8_t* ptr = (uint8_t*)start_addr;

	for(uint32_t i=0; i<size; i++){
		crc ^= ptr[i];
		for(int j=0; j<8; j++){
			crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1)); // 0xEDB88320은 CRC-32 표준 다항식을 비트 반사한 값(그래서 오른쪽 시프트)
		}
	}
	return crc ^ 0xFFFFFFFF;
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
