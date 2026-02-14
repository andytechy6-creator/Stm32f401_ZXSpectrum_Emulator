/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vga.h"
#include "spectrum.h"
#include "z80.h"
#include "video.h"
#include "vga_timing.h"

//#include "usbh_core.h"
#include "usbh_hid.h"
//#include "usbh_hid_keybd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REPEAT_DELAY_MS   400
#define BTN_STABLE_COUNT  3

#define BIT_A   0
#define BIT_B   1   // ×10
#define BIT_C   2   // T5 ARR
#define BIT_D   3   // ×100
#define BIT_E   4   // T3 CCR
#define BIT_F   5   // T5 CCR
#define BIT_G   6   // minus/plus toggle
#define BIT_H   7   // T3 ARR


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int multiplier = 10;
static uint8_t sign_positive = 1;




#if 0
static uint8_t btn_last_raw = 0xFF;
static uint8_t btn_stable   = 0xFF;
static uint8_t btn_counter  = 0;

static uint32_t last_press_time[8] = {0};


static const int step_table[] = { 1, 10, 100, 1000 };
static uint8_t step_index = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

static inline void update_sign_led(void);
uint8_t button_repeat(uint8_t b);
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost);
void Keyboard_HandleKey(uint8_t key);
//void getTimes(void);

//uint8_t Buttons_Get(void);
//static inline uint8_t Buttons_ReadRaw(void);
//static inline int get_multiplier(uint8_t b);
//static inline uint8_t button_repeat(uint8_t b, int bit);
//void Tuning_Update(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
#if 0
	//LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	if (USBH_HID_GetDeviceType(phost) == HID_MOUSE)
	{
		HID_MOUSE_Info_TypeDef *Mouse_Info;
		Mouse_Info = USBH_HID_GetMouseInfo(phost);
		int X_VAL = Mouse_Info->x;
		int Y_VAL = Mouse_Info->y;
		if (X_VAL > 127) X_VAL -= 255;
		if (Y_VAL > 127) Y_VAL -= 255;

		int len = sprintf (Uart_Buf, "X=%d, Y=%d, Button1=%d, Button2=%d, Button3=%d\n", X_VAL, Y_VAL, \
				                                Mouse_Info->buttons[0],Mouse_Info->buttons[1], Mouse_Info->buttons[2]);
		HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, len, 1000);

	}
#endif
	if (USBH_HID_GetDeviceType(phost) == HID_KEYBOARD)
	{
		HID_KEYBD_Info_TypeDef *Keyboard_Info;
		Keyboard_Info = USBH_HID_GetKeybdInfo(phost);
		char key = USBH_HID_GetASCIICode (Keyboard_Info);

		//int len = sprintf (Uart_Buf, "Key Pressed = %c\n", key);
		//HAL_UART_Transmit(&huart2, (uint8_t *)Uart_Buf, len, 1000);
	    if (key != 0) {
	        Keyboard_HandleKey(key);
	    }
	}
}
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  //uint16_t usb_poll_div = 0;
  getTimes();
  spectrum_init();
  vga_timing_init(); // TIM1/TIM3/TIM4 setup
  video_recompute_vertical_window();

  vga_init(); // DMA + pixel buffer setup

  vga_init_border_line();
  // Build a bar pattern directly into a packed buffer once
#if 0
  for (int i = 0; i < H_VISIBLE_PIX; i++)
  {
      uint8_t pix = (i & 0x20) ? 0x07 : 0x38;

      int r = (pix >> 0) & 3;
      int g = (pix >> 2) & 3;
      int b = (pix >> 4) & 3;

      flat[i] = VGA_PACK_RGB(r, g, b);
  }
#endif
  vga_timing_start(); // start pixel clock
  sync_to_vsync();


  //uint16_t last_line = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if (t5_cnt + margin >= activeFstart && t5_cnt < TIM5->ARR)
	  {
	      // We're close enough to active: start servicing lines
	      video_line_step();
	  }
#if 0

	  if (!LL_DMA_IsActiveFlag_TC5(DMA2) && dmaStarted)
	  {
		    LL_DMA_ClearFlag_TC5(DMA2);
		    if (repeat == swap_line)
		    {
		     swapBuffers();
		    }
		    ReArmDMA();
		    repeat ^= 1;
		    dmaStarted = 0;
      }
#endif



#if 0
	      if (GPIOA->IDR & V_BLANK_Pin)
	      {
	    	  video_update_osd_values();
	      }

	      // Only think about USB outside visible lines
	      // Poll USB once per 2 frames, near the top, outside visible area
	      //MX_USB_HOST_Process();

	      static uint16_t frame_counter = 0;

	      static uint8_t in_vblank = 0;

	      uint8_t usb_blank = (GPIOA->IDR & V_BLANK_Pin) ? 1 : 0;
	      if (usb_blank)
	      {
	    	  frame_counter++;
	      }

if (frame_counter == 2)
{
	frame_counter=0;
	      if (usb_blank)
	      {
	          // We are inside vblank: run USB as much as possible
	          MX_USB_HOST_Process();
	          in_vblank = 1;
	      }
	      else
	      {
	          // We just left vblank
	          in_vblank = 0;
	      }
}


    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
#endif
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_25, 336, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(84000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */
  LL_SPI_Enable(SPI1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM1 DMA Init */

  /* TIM1_UP Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_5);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 9;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_TI1FP1);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_GATED);
  LL_TIM_IC_SetFilter(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_FALLING);
  LL_TIM_DisableIT_TRIG(TIM1);
  LL_TIM_DisableDMAReq_TRIG(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  /* USER CODE BEGIN TIM1_Init 2 */
  LL_TIM_EnableCounter(TIM1);
  //LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableDMAReq_UPDATE(TIM1);

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 524;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 2;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerInput(TIM2, LL_TIM_TS_ITR3);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE1);
  LL_TIM_DisableIT_TRIG(TIM2);
  LL_TIM_DisableDMAReq_TRIG(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
  LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);
  LL_TIM_EnableCounter(TIM2);


  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PB10   ------> TIM2_CH3
  */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  //old arr 2580;
  //old ccr 480

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2844;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH4);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 262;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetOnePulseMode(TIM3, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM3, LL_TIM_TS_ITR3);
  LL_TIM_SetSlaveMode(TIM3, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM3);
  LL_TIM_DisableDMAReq_TRIG(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  LL_TIM_EnableCounter(TIM3);
  /* USER CODE END TIM3_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB1   ------> TIM3_CH4
  */
  GPIO_InitStruct.Pin = H_BLANK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(H_BLANK_GPIO_Port, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2666;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 160;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
  LL_TIM_EnableCounter(TIM4);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_UPDATE(TIM4);

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration
  PB6   ------> TIM4_CH1
  */
  GPIO_InitStruct.Pin = HSYNC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(HSYNC_GPIO_Port, &GPIO_InitStruct);

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1354700;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM5, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM5);
  LL_TIM_SetClockSource(TIM5, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM5, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 74700;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM5, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetOnePulseMode(TIM5, LL_TIM_ONEPULSEMODE_SINGLE);
  LL_TIM_SetTriggerInput(TIM5, LL_TIM_TS_ITR0);
  LL_TIM_SetSlaveMode(TIM5, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM5);
  LL_TIM_DisableDMAReq_TRIG(TIM5);
  LL_TIM_SetTriggerOutput(TIM5, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM5);
  /* USER CODE BEGIN TIM5_Init 2 */

  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableCounter(TIM5);

  /* USER CODE END TIM5_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM5 GPIO Configuration
  PA0-WKUP   ------> TIM5_CH1
  */
  GPIO_InitStruct.Pin = V_BLANK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(V_BLANK_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM10);

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 16694;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM10, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM10);
  LL_TIM_OC_EnablePreload(TIM10, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 63;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  LL_TIM_OC_Init(TIM10, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM10, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM10_Init 2 */
  LL_TIM_EnableCounter(TIM10);
  /* USER CODE END TIM10_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM10 GPIO Configuration
  PB8   ------> TIM10_CH1
  */
  GPIO_InitStruct.Pin = VSYNCB8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(VSYNCB8_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  TIM_InitStruct.Prescaler = 83;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 200-LL_TIM_IC_FILTER_FDIV1_N2;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM11, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM11);
  LL_TIM_OC_EnablePreload(TIM11, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 100;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM11, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM11, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM11_Init 2 */
  LL_TIM_EnableCounter(TIM11);
  /* USER CODE END TIM11_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM11 GPIO Configuration
  PB9   ------> TIM11_CH1
  */
  GPIO_InitStruct.Pin = TIM11_CH1_AUDIO_BCK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(TIM11_CH1_AUDIO_BCK_GPIO_Port, &GPIO_InitStruct);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(SD_CS_GPIO_Port, SD_CS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, RGB_LSB_R_Pin|RGB_MSB_R_Pin|RGB_LSB_G_Pin|RGB_MSB_G_Pin
                          |RGB_LSB_B_Pin|RGB_MSB_B_Pin|GPIO_PA10_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Sound_GPIO_Port, Sound_Pin);

  /**/
  LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

  /**/
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = RGB_LSB_R_Pin|RGB_MSB_R_Pin|RGB_LSB_G_Pin|RGB_MSB_G_Pin
                          |RGB_LSB_B_Pin|RGB_MSB_B_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Sound_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(Sound_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = GPIO_PA10_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIO_PA10_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


static inline void update_sign_led(void)
{
    if (sign_positive)
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);   // LED ON
    else
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);     // LED OFF
}
#if 0
void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	if(USBH_HID_GetDeviceType(phost) == HID_MOUSE)  // if the HID is Mouse
	{
		HID_MOUSE_Info_TypeDef *Mouse_Info;
		Mouse_Info = USBH_HID_GetMouseInfo(phost);  // Get the info
		int X_Val = Mouse_Info->x;  // get the x value
		int Y_Val = Mouse_Info->y;  // get the y value
		if (X_Val > 127) X_Val -= 255;
		if (Y_Val > 127) Y_Val -= 255;
		int len = sprintf (Uart_Buf, "X=%d, Y=%d, Button1=%d, Button2=%d, Button3=%d\n", X_Val, Y_Val, \
				                                Mouse_Info->buttons[0],Mouse_Info->buttons[1], Mouse_Info->buttons[2]);
		HAL_UART_Transmit(&huart2, (uint8_t *) Uart_Buf, len, 100);
	}

	if(USBH_HID_GetDeviceType(phost) == HID_KEYBOARD)  // if the HID is Mouse
	{
		uint8_t key;
		HID_KEYBD_Info_TypeDef *Keyboard_Info;
		Keyboard_Info = USBH_HID_GetKeybdInfo(phost);  // get the info
		key = USBH_HID_GetASCIICode(Keyboard_Info);  // get the key pressed

		//int len = sprintf (Uart_Buf, "Key Pressed = %c\n", key);
		//HAL_UART_Transmit(&huart2, (uint8_t *) Uart_Buf, len, 100);
	}
}

void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
    HID_KEYBD_Info_TypeDef *k = USBH_HID_GetKeybdInfo(phost);

    if (k == NULL)
        return;

    uint8_t key = USBH_HID_GetASCIICode(k);

    if (key != 0) {
        Keyboard_HandleKey(key);
    }
}
#endif
void Keyboard_HandleKey(uint8_t key)
{
    switch (key)
    {
        // movement keys
        case 'x':
            TIM3->CCR4 += multiplier;
            video_recompute_vertical_window();
            break;

        case 'z':
            TIM3->CCR4 -= multiplier;
            video_recompute_vertical_window();
            break;
        case 'v':
            TIM3->ARR += multiplier;
            video_recompute_vertical_window();
            break;

        case 'c':
            TIM3->ARR -= multiplier;
            video_recompute_vertical_window();
            break;
        case 'k':
            TIM5->CCR1 += multiplier;
            video_recompute_vertical_window();
            break;

        case 'o':
            TIM5->CCR1 -= multiplier;
            video_recompute_vertical_window();
            break;
        case 'l':
            TIM5->ARR += multiplier;
            video_recompute_vertical_window();
            break;

        case 'p':
            TIM5->ARR -= multiplier;
            video_recompute_vertical_window();
            break;

        // sign keys
        case 'y':
            sign_positive = 1;
            update_sign_led();
            break;

        case 'g':
            sign_positive = 0;
            update_sign_led();
            break;

        // multiplier keys (Shift+1..4)
        case '!':   // Shift+1
            multiplier = 1;
            break;

        case '@':   // Shift+2
            multiplier = 10;
            break;

        case '#':   // Shift+3
            multiplier = 100;
            break;

        case '$':   // Shift+4
            multiplier = 1000;
            break;
        case '%':   // shift+5
			multiplier = 10000;
        	break;
        case 'm':   // Shift+4
        	line_wait ++;
            break;
        case 'n':   // shift+5
        	line_wait --;
        	break;
        case 'b':   // shift+5
        	render_line ^= 1;
        	break;
        case 'h':   // shift+5
        	swap_line ^= 1;
        	break;
        case 'r':   // shift+5
        	margin_lines ++;
        	getTimes();
        	break;
        case 'd':   // shift+5
        	margin_lines --;
        	getTimes();
        	break;
        case '\t':   // TAB key
            //osd_enabled ^= 1;   // toggle
            break;


        default:
            break;
    }
}

void getTimes(void)
{
	t5_cnt   = TIM5->CNT;
	activeFstart  = TIM5->CCR1;     // start of active frame (in TIM5 ticks)
	activeFend    = TIM5->ARR;      // end of active frame (in TIM5 ticks)
	one_line = (TIM4->ARR + 1);     // ticks per line
	margin   = margin_lines * one_line;        // 2 lines before active
}

#if 0
static inline uint8_t Buttons_ReadRaw(void)
{

	uint8_t data = 0;
    // 1. Pulse S/L low → high to latch parallel inputs
    LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);
    // tiny delay to satisfy t_w (not strictly required on STM32)
    __NOP(); __NOP();
    LL_GPIO_SetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin);

    // 2. Shift out 8 bits via SPI1
    while(!LL_SPI_IsActiveFlag_TXE(SPI1));
    // Write dummy byte to trigger 8 clocks
    LL_SPI_TransmitData8(SPI1, 0xFF);

    // Wait for RXNE
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));

    // Read received byte
    data = LL_SPI_ReceiveData8(SPI1);
    return data;
}



uint8_t Buttons_Get(void)
{
    uint8_t raw = Buttons_ReadRaw();

    if (raw == btn_last_raw) {
        if (btn_counter < BTN_STABLE_COUNT)
            btn_counter++;

        if (btn_counter == BTN_STABLE_COUNT)
            btn_stable = raw;
    } else {
        btn_counter = 0;
        btn_last_raw = raw;
    }

    // Return active‑high (1 = pressed)
    return (uint8_t)(~btn_stable);
}




static inline int get_multiplier(uint8_t b)
{
    if (button_repeat(b, BIT_B)) {
        step_index++;
        if (step_index >= (sizeof(step_table)/sizeof(step_table[0])))
            step_index = 0;
    }
    return step_table[step_index];
}

static inline uint8_t button_repeat(uint8_t b, int bit)
{
    if (!(b & (1 << bit))) {
        // not pressed
        last_press_time[bit] = 0;
        return 0;
    }

    uint32_t now = HAL_GetTick();

    if (last_press_time[bit] == 0) {
        last_press_time[bit] = now;
        return 1; // initial press
    }

    if ((now - last_press_time[bit]) >= REPEAT_DELAY_MS) {
        last_press_time[bit] = now;
        return 1; // repeated press
    }

    return 0;
}
static inline void update_sign_led(void)
{
    if (sign_positive)
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);   // LED ON
    else
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);     // LED OFF
}
void Tuning_Update(void)
{
    uint8_t b = Buttons_Get();   // active‑high
     // 1, 10, or 100

    // Toggle +/− with centre button (G)
    if (button_repeat(b, BIT_G)) {
        sign_positive ^= 1;
        update_sign_led();
    }

    int mul = get_multiplier(b);
    int delta = sign_positive ? mul : -mul;

    // T3 CCR (E)
    if (button_repeat(b, BIT_E)) {
        TIM3->CCR1 += delta;
    }

    // T3 ARR (H)
    if (button_repeat(b, BIT_H)) {
        TIM3->ARR += delta;
    }

    // T5 CCR (F)
    if (button_repeat(b, BIT_F)) {
        TIM5->CCR1 += delta;
    }

    // T5 ARR (C)
    if (button_repeat(b, BIT_C)) {
        TIM5->ARR += delta;
    }
}
#endif

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
