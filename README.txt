# STM32L496G-DISCO with B-CAMS-OMV (ov5640)

@ DCMI_CaptureMode DCMI Capture Mode example

How to use the DCMI to interface with a camera module to continuously capture
RGB565 images, crop them from size 320x240 to 240x240 then display the video
stream on the LCD.

The Digital camera interface (DCMI) is configured to receive the captured frame from 
the OV5640 based camera module connected to the Discovery board CN1 connector.

The DCMI IP is configured to crop the camera module frame set to QVGA 320x240 resolution 
to the FRIDA LCD format 240x240.

Each time the DCMI End of Frame event callback is raised, the picture is transferred to the LCD frame buffer through DMA2D.   


At the beginning of the main program the HAL_Init() function is called to reset 
all the peripherals, initialize the Flash interface and the systick.
Then the SystemClock_Config() function is used to configure the system
clock (SYSCLK) to run at 80 MHz.

@par Directory contents 

  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Inc/stm32l4xx_hal_conf.h    HAL configuration file
  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Inc/stm32l4xx_it.h          Interrupt handlers header file
  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Inc/main.h                  Header for main.c module  
  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Src/stm32l4xx_it.c          Interrupt handlers
  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Src/main.c                  Main program
  - Disco_L496_DCMI_OV5640_CaptureMode_v1/Core/Src/system_stm32l4xx.c      STM32L4xx system source file

  - Disco_L496_DCMI_OV5640_CaptureMode_v1/BSP/Components/ov5640            ov5640 low level driver

  - Disco_L496_DCMI_OV5640_CaptureMode_v1\Drivers\BSP\STM32L496G-Discovery/stm32l496g_discovery.h 
	- line 341 : #define CAMERA_I2C_ADDRESS ((uint16_t) 0x78)            ov5640 I2C Address

  - Disco_L496_DCMI_OV5640_CaptureMode_v1\Drivers\BSP\STM32L496G-Discovery/stm32l496g_discovery.c
      - lines 197 to 199   change the input interface of CAMERA_IO_Write and CAMERA_IO_Read to handle 16 bit registers
	- lines 1593 to 1609 implement CAMERA_IO_Write and CAMERA_IO_Read 


@par Hardware and Software environment

  - This example runs on STM32L496xx devices.
    
  - This example has been tested on a STM32L496G-Discovery board with a B-CAMS-OMV
    camera module connected to the Discovery board CN1 connector. 
    https://www.st.com/en/evaluation-tools/b-cams-omv.html

@ IMPORTANT clock configuration

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
