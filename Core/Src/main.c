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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
extern DCMI_HandleTypeDef hDcmiHandler;
DMA2D_HandleTypeDef Dma2dHandle;
uint16_t pBuffer[ST7789H2_LCD_PIXEL_WIDTH * ST7789H2_LCD_PIXEL_WIDTH];
HAL_StatusTypeDef hal_status = HAL_OK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

static void IO_Init(void);
static void LCD_ImagePreparation(uint16_t x0, uint16_t y0, uint16_t xSize, uint16_t ySize);
static HAL_StatusTypeDef LCD_Write(uint32_t OrigAddress, uint32_t DestAddress, uint32_t TransferSize);
static void OnError_Handler(uint32_t condition);
static void DMA2D_Config(void);
static void TransferError(DMA2D_HandleTypeDef* Dma2dHandle);
static void TransferComplete(DMA2D_HandleTypeDef* Dma2dHandle);

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

 /*##-2- Initialize the LCD #################################################*/
  /*##-1- LEDs initialization  #################################################*/
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED1);
  
  /*##-2- LCD configuration  #################################################*/
  /* I/O initialization, required before LCD initialization */
  IO_Init();
  /* LCD initialization */
  BSP_LCD_Init(); 

  /* DMA2D initialization */   
  DMA2D_Config(); 

  /*##-3- Camera Initialization ############################*/
  /* Initialize the Camera in QVGA mode */
  //
#define VGA
#ifdef VGA
  BSP_CAMERA_Init(CAMERA_R640x480);
#else
  BSP_CAMERA_Init(CAMERA_R320x240);
#endif

  /* Wait 1s to let auto-loops in the camera module converge and lead to correct exposure */
  HAL_Delay(1000);
  
  /*##-4- Camera Continuous capture start in QVGA resolution ############################*/
  /* Disable unwanted HSYNC (IT_LINE)/VSYNC interrupts */
  __HAL_DCMI_DISABLE_IT(&hDcmiHandler, DCMI_IT_LINE | DCMI_IT_VSYNC);
 
  /* LCD size is 240 x 240 and format is RGB565 i.e. 16 bpp or 2 bytes/pixel. 
     The LCD frame size is therefore 240 * 240 half-words of (240*240)/2 32-bit long words . 
     Since the DMA associated to DCMI IP is configured in  BSP_CAMERA_MspInit() of stm32l496g_discovery_camera.c file
     with words alignment, the last parameter of HAL_DCMI_Start_DMA is set to:
      (ST7789H2_LCD_PIXEL_WIDTH*ST7789H2_LCD_PIXEL_HEIGHT)/2, that is 240 * 240 / 2
   */

#define SNAP
#ifdef SNAP
  hal_status = HAL_DCMI_Start_DMA(&hDcmiHandler, DCMI_MODE_SNAPSHOT,  (uint32_t)pBuffer , (ST7789H2_LCD_PIXEL_WIDTH*ST7789H2_LCD_PIXEL_HEIGHT)/2 );
  OnError_Handler(hal_status != HAL_OK);
  HAL_DCMI_Stop(&hDcmiHandler);
#else
  hal_status = HAL_DCMI_Start_DMA(&hDcmiHandler, DCMI_MODE_CONTINUOUS,  (uint32_t)pBuffer , (ST7789H2_LCD_PIXEL_WIDTH*ST7789H2_LCD_PIXEL_HEIGHT)/2 );
  OnError_Handler(hal_status != HAL_OK);

#endif


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef SNAP
	  HAL_Delay(1000);
	  hal_status = HAL_DCMI_Start_DMA(&hDcmiHandler, DCMI_MODE_SNAPSHOT,  (uint32_t)pBuffer , (ST7789H2_LCD_PIXEL_WIDTH*ST7789H2_LCD_PIXEL_HEIGHT)/2 );
	  OnError_Handler(hal_status != HAL_OK);
	  HAL_DCMI_Stop(&hDcmiHandler);
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 20;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, MFX_WAKEUP_Pin|LCD_PWR_ON_Pin|MIC_VDD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : JOY_DOWN_Pin JOY_LEFT_Pin JOY_UP_Pin */
  GPIO_InitStruct.Pin = JOY_DOWN_Pin|JOY_LEFT_Pin|JOY_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : STMOD_INT_Pin */
  GPIO_InitStruct.Pin = STMOD_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STMOD_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D3_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_TE_Pin */
  GPIO_InitStruct.Pin = LCD_TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MFX_WAKEUP_Pin LCD_PWR_ON_Pin MIC_VDD_Pin */
  GPIO_InitStruct.Pin = MFX_WAKEUP_Pin|LCD_PWR_ON_Pin|MIC_VDD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_RST_Pin */
  GPIO_InitStruct.Pin = Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Audio_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTP_INT_Pin */
  GPIO_InitStruct.Pin = CTP_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTP_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DCMI_CLK_Pin */
  GPIO_InitStruct.Pin = DCMI_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LPTIM2;
  HAL_GPIO_Init(DCMI_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MFX_IRQ_OUT_Pin */
  GPIO_InitStruct.Pin = MFX_IRQ_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MFX_IRQ_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_RIGHT_Pin */
  GPIO_InitStruct.Pin = JOY_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(JOY_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STMOD_RESET_Pin */
  GPIO_InitStruct.Pin = STMOD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STMOD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  Camera Frame Event callback.
  */
void BSP_CAMERA_FrameEventCallback(void)
{
  /* Fully fill the LCD screen */
  LCD_ImagePreparation(0, 0, ST7789H2_LCD_PIXEL_WIDTH, ST7789H2_LCD_PIXEL_HEIGHT);
  
  /* Write data (through DMA2D) */
  hal_status = LCD_Write((uint32_t) (&pBuffer), (uint32_t)&(LCD_ADDR->REG), ST7789H2_LCD_PIXEL_WIDTH * ST7789H2_LCD_PIXEL_HEIGHT);
  OnError_Handler(hal_status != HAL_OK); 
}


/**
  * @brief IO initialization.
  * @note  GPIO PH.00 setting to activate STM32L496 Discovery I/Os
  *        and I/O initialization.  
  * @retval None
  */
static void IO_Init(void)
{   
  GPIO_InitTypeDef GPIO_InitStruct;
  

  __HAL_RCC_GPIOH_CLK_ENABLE();
 
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull =   GPIO_NOPULL;
  GPIO_InitStruct.Alternate = 0;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    
  HAL_GPIO_Init( GPIOH, &GPIO_InitStruct ); 
 
  HAL_GPIO_WritePin( GPIOH, GPIO_PIN_0, GPIO_PIN_RESET); 
  
  /* Initialize the IO functionalities */
  BSP_IO_Init();  
}

/**
  * @brief LCD image preparation.
  * @note  Configure image position and size for Discovery STM32L496 LCD
  *        and set LCD in pixel writing mode.
  *        This API must be followed by LCD_Write() call. 
  * @param  x0: first pixel x position
  * @param  y0: first pixel y position 
  * @param  xSize: image width (in pixels)
  * @param  ySize: image height (in pixels)           
  * @retval None
  */
static void LCD_ImagePreparation(uint16_t x0, uint16_t y0, uint16_t xSize, uint16_t ySize)
{
  /* CASET: Column Address Set */ 
  LCD_IO_WriteReg(ST7789H2_CASET);
  /* Send commands */
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(x0);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(x0 + xSize -1);
  /* RASET: Row Address Set */  
  LCD_IO_WriteReg(ST7789H2_RASET);  
  /* Send commands */
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(y0);
  LCD_IO_WriteData(0x00);
  LCD_IO_WriteData(y0 + ySize -1); 
    
  /* Prepare to write to LCD RAM */
  LCD_IO_WriteReg(ST7789H2_WRITE_RAM);  
}            
  
/**
  * @brief LCD image write-up.
  * @note  Resort to DMA to feed image pixels to STM32L496 Discovery LCD.   
  *        LCD is configured in RGB565: one pixel is coded over 16 bits.   
  * @param  OrigAddress: Image address
  * @param  DestAddress: LCD RAM address
  * @param  TransferSize: image size (in pixels)      
  * @retval HAL status
  */          
static HAL_StatusTypeDef LCD_Write(uint32_t OrigAddress, uint32_t DestAddress, uint32_t TransferSize)
{ 
  /* Force 1 pixel per line and width in pixels x height in pixels   */
  /* as number of lines to align DMA2D transfer to LCD configuration */            
  if (HAL_OK != HAL_DMA2D_Start_IT(&Dma2dHandle, OrigAddress, DestAddress, 1, TransferSize))
  {
    return HAL_ERROR;
  }
    
  return HAL_OK;
}   

static void DMA2D_Config(void)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  
  /* Enable DMA2D clock */
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /* NVIC configuration for DMA2D transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);   

  /* Configure the DMA2D Mode, Color Mode and output offset */  
  Dma2dHandle.Init.Mode         = DMA2D_M2M;              /* DMA2D Mode memory to memory */
  Dma2dHandle.Init.ColorMode    = DMA2D_OUTPUT_RGB565;    /* Output color mode is RGB565 : 16 bpp */
  Dma2dHandle.Init.OutputOffset = 0x0;                    /* No offset in output */  
  Dma2dHandle.Init.RedBlueSwap   = DMA2D_RB_REGULAR;      /* No R&B swap for the output image */
  Dma2dHandle.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;   /* No alpha inversion for the output image */

  /* DMA2D Callbacks Configuration */
  Dma2dHandle.XferCpltCallback  = TransferComplete;
  Dma2dHandle.XferErrorCallback = TransferError;
  
  /* Foreground Configuration : Layer 1 */
  Dma2dHandle.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  Dma2dHandle.LayerCfg[1].InputAlpha = 0xFF;                    /* Fully opaque */
  Dma2dHandle.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;  /* Foreground layer format is RGB565 : 16 bpp */
  Dma2dHandle.LayerCfg[1].InputOffset = 0x0;                    /* No offset in input */
  Dma2dHandle.LayerCfg[1].RedBlueSwap   = DMA2D_RB_REGULAR;     /* No R&B swap for the input foreground image */
  Dma2dHandle.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;  /* No alpha inversion for the input foreground image */

  Dma2dHandle.Instance = DMA2D; 
  
  /* DMA2D Initialization */
  hal_status = HAL_DMA2D_Init(&Dma2dHandle);
  OnError_Handler(hal_status != HAL_OK);
  
  hal_status = HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);
  OnError_Handler(hal_status != HAL_OK);
}

/**
  * @brief  On Error Handler on condition TRUE.
  * @param  condition : Can be TRUE or FALSE
  * @retval None
  */
static void OnError_Handler(uint32_t condition)
{
  if(condition)
  {
    BSP_LED_On(LED1);
    while(1) { ; } /* Blocking on error */
  }
}

/**
  * @brief  DMA2D Transfer complete callback
  * @param  hdma2d: DMA2D handle. 
  * @note   This example shows a simple way to report end of DMA2D transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
static void TransferComplete(DMA2D_HandleTypeDef *hdma2d)
{
  /* Set LED2 On */
  BSP_LED_On(LED2);   
}

/**
  * @brief  DMA2D error callback
  * @param  hdma2d: DMA2D handle
  * @note   This example shows a simple way to report DMA2D transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
static void TransferError(DMA2D_HandleTypeDef *hdma2d)
{
  /* Set LED1 On */
  BSP_LED_On(LED1);
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
