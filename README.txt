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
