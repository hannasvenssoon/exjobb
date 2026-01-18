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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32u5xx_hal.h"
//#include "b_u585i_iot02a_motion_sensors.h"
#include "ism330dhcx.h"
#include "ism330dhcx_reg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SENSOR_ADDR (0x6B << 1) // I2C-adress
#define WHO_AM_I_REG 0x0F
#define ISM330DHCX_ADDR  (0x6B << 1)

#define FIFO_THRESHOLD_BLOCK_SIZE 320
#define MAX_SAMPLES 1024
#define ISM330DHCX_FIFO_XL_TAG  0x01
#define FIFO_FRAME_SIZE     7
#define FIFO_BLOCK_SAMPLES  64      // 32 samples per block read
#define FIFO_BLOCK_BYTES    (FIFO_FRAME_SIZE * FIFO_BLOCK_SAMPLES)
#define BIN_START_1  0xAA
#define BIN_START_2  0x55
#define BIN_END_1    0x55
#define BIN_END_2    0xAA

//I2C_HandleTypeDef hi2c2;
//UART_HandleTypeDef huart1; // För serial output
uint64_t unix_start_ms = 0;
//BSP_MOTION_SENSOR_Axes_t accel;
#define RX_BUF_SIZE 64

static uint8_t uart_rx_char;
static char uart_rx_buf[RX_BUF_SIZE];
static uint8_t uart_rx_idx = 0;
static volatile uint8_t logging_enabled = 0;
static volatile uint8_t buffer_dump = 0;
static volatile uint8_t buffer_full_flag = 0;
volatile uint8_t chunk_flag = 0;
volatile uint8_t stop_requested = 0;
static ISM330DHCX_Object_t acc_obj;
static uint32_t last_sample_tick = 0;
static uint8_t current_label = 0;   // 0 = ingen label vald
static uint32_t sample_count = 0;
static uint32_t t_start = 0;
static uint16_t fifo_count = 0;
static uint32_t fs_total_samples = 0;
static uint32_t fs_t0_ms = 0;

static uint16_t sample_index = 0;

static stmdev_ctx_t dev_ctx;

static uint8_t fifo_block[FIFO_BLOCK_BYTES];

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
MDF_HandleTypeDef AdfHandle0;
MDF_FilterConfigTypeDef AdfFilterConfig0;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;
OSPI_HandleTypeDef hospi2;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define RX_BUF_SIZE 64

//static uint8_t rx_byte;
//static char rx_buf[RX_BUF_SIZE];
//static uint8_t rx_idx = 0;

#define ACC_RINGBUFFER_SIZE 768
#define CHUNK_SIZE 256

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    int16_t ax;
    int16_t ay;
    int16_t az;
} fifo_sample_t;

/*typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	uint32_t timestamp;
} fifo_sample_t;*/

/*
typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    uint32_t timestamp;
} AccSample_t;

static AccSample_t acc_rb[ACC_RINGBUFFER_SIZE];
static volatile uint32_t rb_head = 0;
static volatile uint32_t rb_count = 0;
*/
typedef struct {
  int32_t ax;
  int32_t ay;
  int32_t az;
  uint8_t label;
  uint32_t timestamp;
} AccSample_t;

typedef struct {
  AccSample_t buf[ACC_RINGBUFFER_SIZE];
  uint16_t head;
  uint16_t tail;
  uint16_t count;
  //uint8_t  logging_enabled;
} AccRingBuffer_t;

//static volatile uint32_t rb_head = 0;
//static volatile uint32_t rb_count = 0;

static AccRingBuffer_t acc_rb = {0};
static fifo_sample_t fifo_buf[FIFO_THRESHOLD_BLOCK_SIZE+1];

/*static int32_t I2C_ReadReg(void *handle, uint8_t Reg,
                           uint8_t *pData, uint16_t Length)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;

    return (HAL_I2C_Mem_Read(
                hi2c,
                ISM330DHCX_ADDR,
                Reg,
                I2C_MEMADD_SIZE_8BIT,
                pData,
                Length,
                HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

static int32_t I2C_WriteReg(void *handle, uint8_t Reg,
                            uint8_t *pData, uint16_t Length)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;

    return (HAL_I2C_Mem_Write(
                hi2c,
                ISM330DHCX_ADDR,
                Reg,
                I2C_MEMADD_SIZE_8BIT,
                pData,
                Length,
                HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}*/
static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)handle,
        ISM330DHCX_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)handle,
        ISM330DHCX_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        HAL_MAX_DELAY) == HAL_OK) ? 0 : -1;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADF1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ICACHE_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_OCTOSPI2_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

static void ISM330DHCX_StartSampling(void);
static void ISM330DHCX_StopSampling(void);
void HandleUartCommand(const char *cmd);
static int AccRB_Push(int32_t ax, int32_t ay, int32_t az, uint8_t label, uint32_t ts);
static void AccRB_Clear();
void AccSamplingThread();
void Acc_Init(void);
//static void SendBuffer(void);
static int AccRb_Pop(AccSample_t *s);
static void FifoBuff_Push(int16_t ax, int16_t ay, int16_t az, uint32_t ts);
static void SendFIFOBuff_UART(void);
static void ReadFIFOBlock(void);
static void ISM330DHCX_FIFO_Config(void);
static void ISM330DHCX_SensorInit(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void test_sensor()
{
    uint8_t who_am_i;
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c2, SENSOR_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 1000);
    if (ret == HAL_OK)
    {
        if (who_am_i == 0x6B)
        {
            printf("Sensor OK! WHO_AM_I = 0x%X\r\n", who_am_i);
        }
        else
        {
            printf("Fel värde! WHO_AM_I = 0x%X\r\n", who_am_i);
        }
    }
    else
    {
        printf("Kommunikationsfel! HAL return code: %d\r\n", ret);
    }
}

void ReceiveUnixStartTime(void){
	char buf[32] = {0};
	HAL_UART_Receive(&huart1, (uint8_t*)buf, sizeof(buf), HAL_MAX_DELAY);
	unix_start_ms = atoll(buf);
	//printf("Start unix time: %llu\r\n", unix_start_ms);
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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADF1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_OCTOSPI1_Init();
  MX_OCTOSPI2_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_UCPD1_Init();
  MX_USB_OTG_FS_PCD_Init();

  printf("BOOT OK\r\n");
  uint32_t t0 = HAL_GetTick();
  HAL_Delay(1000);
  uint32_t t1 = HAL_GetTick();

  printf("Tick delta = %lu\r\n", t1 - t0);
  ISM330DHCX_SensorInit();

  HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
  //ISM330DHCX_FIFO_Config();
  //int loop = 0;
  /* USER CODE BEGIN 2 */
  while (1)
  {
	  if (logging_enabled)
	  {
		  //printf("Collecting...\r\n");
		  ReadFIFOBlock();
	  }

	  /* Dumpa bufferten när den är full */
	  if (fifo_count >= FIFO_THRESHOLD_BLOCK_SIZE)
	  {
		  SendFIFOBuff_UART();
		  fifo_count = 0;
	  }

	  /*if (fifo_count > (FIFO_THRESHOLD_BLOCK_SIZE-1))
	  {
		  SendFIFOBuff_UART();
		  fifo_count = 0;
	  }*/
	  //HAL_Delay(250);
  }
/*  	 BSP_MOTION_SENSOR_Init(0, MOTION_ACCELERO);
     BSP_MOTION_SENSOR_Enable(0, MOTION_ACCELERO);

     //ReceiveUnixStartTime();

     BSP_MOTION_SENSOR_SetFullScale(0, MOTION_ACCELERO, 4);


     int32_t fs;
     BSP_MOTION_SENSOR_GetFullScale(0, MOTION_ACCELERO, &fs);
     printf("Full-scale: %ld g\r\n", fs);

     float odr;
     BSP_MOTION_SENSOR_GetOutputDataRate(0, MOTION_ACCELERO, &odr);
     printf("ODR before: %.2f Hz \r\n", odr);

     BSP_MOTION_SENSOR_SetOutputDataRate(0, MOTION_ACCELERO, 3330.0f);

     BSP_MOTION_SENSOR_GetOutputDataRate(0, MOTION_ACCELERO, &odr);
     printf("New ODR: %.2f\r\n", odr);*/

     //Acc_Init();
     //HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
	 //printf("STM32 alive\r\n");



     /*while (1)
     {
         if (logging_enabled)
         {
             //uint32_t now = HAL_GetTick();

             //if (now - last_sample_tick >= 10)   // 100 Hz
             //{
                 //last_sample_tick = now;

                 BSP_MOTION_SENSOR_Axes_t axes;
                 BSP_MOTION_SENSOR_GetAxes(0, MOTION_ACCELERO, &axes);

                 sample_count++;

                 if (HAL_GetTick() - sample_t0 >= 1000)
                 {
                	 last_sample_tick = sample_count;
                	 sample_count = 0;
                	 sample_t0 = HAL_GetTick();
                 }
                 if (!AccRB_Push(axes.xval, axes.yval, axes.zval, current_label, HAL_GetTick()))
                 {
                     logging_enabled = 0;
                     buffer_full_flag = 1;
                     printf("Buffer full - sending data\r\n");
                 }
                 else if (acc_rb.count >= CHUNK_SIZE)
                 {
                     chunk_flag = 1;
                 }
             //}
         }

         if (buffer_full_flag || stop_requested || chunk_flag)
         {
             if (acc_rb.count >= CHUNK_SIZE)
             {
                 AccSample_t s;
                 uint16_t sent = 0;
                 uint16_t to_send;

                 if (chunk_flag)
                     to_send = CHUNK_SIZE;
                 else
                     to_send = acc_rb.count;

                 printf("DATA BEGIN\r\n");

                 while (sent < to_send && AccRb_Pop(&s))
                 {
                     printf("%lu,%ld,%ld,%ld,%d\r\n",
                            s.timestamp,
                            s.ax,
                            s.ay,
                            s.az,
                            s.label);
                     sent++;
                 }

                 printf("DATA END\r\n");
                 printf("Sampling rate: %lu Hz\r\n", last_sample_tick);
             }

             buffer_full_flag = 0;
             stop_requested = 0;
             chunk_flag = 0;
         }

         //HAL_Delay(1);
     }*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while (1)
  //{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ADF1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADF1_Init(void)
{

  /* USER CODE BEGIN ADF1_Init 0 */

  /* USER CODE END ADF1_Init 0 */

  /* USER CODE BEGIN ADF1_Init 1 */

  /* USER CODE END ADF1_Init 1 */

  /**
    AdfHandle0 structure initialization and HAL_MDF_Init function call
  */
  AdfHandle0.Instance = ADF1_Filter0;
  AdfHandle0.Init.CommonParam.ProcClockDivider = 1;
  AdfHandle0.Init.CommonParam.OutputClock.Activation = DISABLE;
  AdfHandle0.Init.SerialInterface.Activation = ENABLE;
  AdfHandle0.Init.SerialInterface.Mode = MDF_SITF_LF_MASTER_SPI_MODE;
  AdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
  AdfHandle0.Init.SerialInterface.Threshold = 4;
  AdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_FALLING;
  if (HAL_MDF_Init(&AdfHandle0) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    AdfFilterConfig0 structure initialization

    WARNING : only structure is filled, no specific init function call for filter
  */
  AdfFilterConfig0.DataSource = MDF_DATA_SOURCE_BSMX;
  AdfFilterConfig0.Delay = 0;
  AdfFilterConfig0.CicMode = MDF_ONE_FILTER_SINC4;
  AdfFilterConfig0.DecimationRatio = 2;
  AdfFilterConfig0.Gain = 0;
  AdfFilterConfig0.ReshapeFilter.Activation = DISABLE;
  AdfFilterConfig0.HighPassFilter.Activation = DISABLE;
  AdfFilterConfig0.SoundActivity.Activation = DISABLE;
  AdfFilterConfig0.AcquisitionMode = MDF_MODE_ASYNC_CONT;
  AdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  AdfFilterConfig0.DiscardSamples = 0;
  /* USER CODE BEGIN ADF1_Init 2 */

  /* USER CODE END ADF1_Init 2 */

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
  hi2c1.Init.Timing = 0x00F07BFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F07BFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 10;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 100;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */

  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 4;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi2.Init.DeviceSize = 26;
  hospi2.Init.ChipSelectHighTime = 2;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi2.Init.ClockPrescaler = 4;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi2.Init.ChipSelectBoundary = 0;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 2;
  sOspiManagerCfg.DQSPort = 2;
  sOspiManagerCfg.NCSPort = 2;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_2_HIGH;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_OSPI_DLYB_Cfg_Struct.Units = 0;
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel = 0;
  if (HAL_OSPI_DLYB_SetConfig(&hospi2, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */

  /* USER CODE END OCTOSPI2_Init 2 */

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

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief UCPD1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UCPD1_Init(void)
{

  /* USER CODE BEGIN UCPD1_Init 0 */

  /* USER CODE END UCPD1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**UCPD1 GPIO Configuration
  PA15 (JTDI)   ------> UCPD1_CC1
  PB15   ------> UCPD1_CC2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN UCPD1_Init 1 */

  /* USER CODE END UCPD1_Init 1 */
  /* USER CODE BEGIN UCPD1_Init 2 */

  /* USER CODE END UCPD1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UCPD_PWR_GPIO_Port, UCPD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WRLS_WKUP_B_GPIO_Port, WRLS_WKUP_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : WRLS_FLOW_Pin Mems_VLX_GPIO_Pin Mems_INT_LPS22HH_Pin */
  GPIO_InitStruct.Pin = WRLS_FLOW_Pin|Mems_VLX_GPIO_Pin|Mems_INT_LPS22HH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3_BOOT0_Pin */
  GPIO_InitStruct.Pin = PH3_BOOT0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PH3_BOOT0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UCPD_PWR_Pin */
  GPIO_InitStruct.Pin = UCPD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UCPD_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Button_Pin */
  GPIO_InitStruct.Pin = USER_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin Mems_VL53_xshut_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_GREEN_Pin|Mems_VL53_xshut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_CCK1_Pin */
  GPIO_InitStruct.Pin = MIC_CCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_CCK1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : WRLS_WKUP_B_Pin */
  GPIO_InitStruct.Pin = WRLS_WKUP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WRLS_WKUP_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WRLS_NOTIFY_Pin Mems_INT_IIS2MDC_Pin USB_IANA_Pin */
  GPIO_InitStruct.Pin = WRLS_NOTIFY_Pin|Mems_INT_IIS2MDC_Pin|USB_IANA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_UCPD_FLT_Pin Mems_ISM330DLC_INT1_Pin */
  GPIO_InitStruct.Pin = USB_UCPD_FLT_Pin|Mems_ISM330DLC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_SENSE_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Mems_STSAFE_RESET_Pin WRLS_WKUP_W_Pin */
  GPIO_InitStruct.Pin = Mems_STSAFE_RESET_Pin|WRLS_WKUP_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : MIC_SDIN0_Pin */
  GPIO_InitStruct.Pin = MIC_SDIN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_MDF1;
  HAL_GPIO_Init(MIC_SDIN0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance != USART1)
	  {
		  //UART1_RestartRx();
	    return;
	  }

	  char c = uart_rx_char;

	  // Ignore CR
	  if (c == '\r')
	  {

	    HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
		//UART1_RestartRx();
	    return;
	  }

	  // End of command
	  if (c == '\n')
	  {
	    uart_rx_buf[uart_rx_idx] = '\0';

	    /* SKICKA VIDARE – INGEN LOGG HÄR */
	    //printf("RX buf: %s\r\n", uart_rx_buf);
	    HandleUartCommand(uart_rx_buf);
	    // RESET BUFFER
		uart_rx_idx = 0;
		//uart_rx_buf[0] = '\0';
		memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
	  }
	  else
	  {
	    if (uart_rx_idx < sizeof(uart_rx_buf) - 1)
	    {
	      uart_rx_buf[uart_rx_idx++] = c;
	      //UART1_RestartRx();
	    }
	    else
	    {
	      // overflow protection
	      memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
	      uart_rx_idx = 0;
	      //UART1_RestartRx();
	    }
	  }

	  // ALWAYS restart RX
	  HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
}

void HandleUartCommand(const char *cmd)
{
	//printf("Receive command\r\n");
  if (strcmp(cmd, "START") == 0)
  {
	//AccRB_Clear();
    logging_enabled = 1;
    ISM330DHCX_StartSampling();
    //printf("OK START\r\n");
    //AccSamplingThread();
  }
  else if (strcmp(cmd, "STOP") == 0)
  {
	  logging_enabled = 0;
	  stop_requested = 1;
	  ISM330DHCX_StopSampling();
    //printf("OK STOP\r\n");
  }
  /*else if (strcmp(cmd, "GETDATA") == 0)
  {
      printf("OK GETDATA\r\n");
      SendBuffer();
  }*/
  /*else if (strncmp(cmd, "LABEL:", 6) == 0)
  {
      uint8_t label = (uint8_t)atoi(&cmd[6]);
      current_label = label;

      //printf("OK LABEL %d\r\n", current_label);
  }*/
  /*else
  {
    printf("ERR Unknown command\r\n");
  }*/
}

static void AccRB_Clear(void)
{
  acc_rb.head = 0;
  acc_rb.tail = 0;
  acc_rb.count = 0;
}

/*static void AccRB_Push(int32_t ax, int32_t ay, int32_t az, uint8_t label, uint32_t ts)
{
  if (!logging_enabled)
    return;

  acc_rb.buf[acc_rb.head].ax = ax;
  acc_rb.buf[acc_rb.head].ay = ay;
  acc_rb.buf[acc_rb.head].az = az;
  acc_rb.buf[acc_rb.head].label = label;
  acc_rb.buf[acc_rb.head].timestamp = ts;

  acc_rb.head = (acc_rb.head + 1) % ACC_RINGBUFFER_SIZE;

  if (acc_rb.count < ACC_RINGBUFFER_SIZE)
  {
    acc_rb.count++;
  }
  else
  {
    acc_rb.tail = (acc_rb.tail + 1) % ACC_RINGBUFFER_SIZE;
  }
}*/

static void FifoBuff_Push(int16_t ax, int16_t ay, int16_t az, uint32_t ts)
{
	if (fifo_count >= FIFO_THRESHOLD_BLOCK_SIZE)
	        return; // skydd mot overflow
    fifo_buf[fifo_count].ax = ax;
	fifo_buf[fifo_count].ay = ay;
	fifo_buf[fifo_count].az = az;
	fifo_buf[fifo_count].timestamp = ts;
}

/*static int AccRB_Push_Org(int32_t ax, int32_t ay, int32_t az, uint8_t label, uint32_t ts)
{
	if (acc_rb.count >= ACC_RINGBUFFER_SIZE)
	{
		return 0;
	}

	  acc_rb.buf[acc_rb.head].ax = ax;
	  acc_rb.buf[acc_rb.head].ay = ay;
	  acc_rb.buf[acc_rb.head].az = az;
	  acc_rb.buf[acc_rb.head].label = label;
	  acc_rb.buf[acc_rb.head].timestamp = ts;

	  acc_rb.head = (acc_rb.head +1) % ACC_RINGBUFFER_SIZE;
	  acc_rb.count++;

	  return 1;
}

static int AccRb_Pop_Org(AccSample_t *s)
{
	if (acc_rb.count == 0)
		return 0;


	*s = acc_rb.buf[acc_rb.tail];
	acc_rb.tail = (acc_rb.tail +1) % ACC_RINGBUFFER_SIZE;
	acc_rb.count--;

	return 1;
}*/


void AccSamplingThread()
{
	//ISM330DHCX_AxesRaw_t acc_raw;

 while (1)
  {
    if (logging_enabled)
    {
    	//SYS_DEBUGF(SYS_DBG_LEVEL_SL, ("[DATALOG] Data accisition started\r\n"));
    	//if (p_acc)
    		//{
    		//SYS_DEBUGF(SYS_DBG_LEVEL_SL, ("[DATALOG] Getting data from sensor\r\n"));
    	printf("Sampling...\r\n");
    		/* Läs rådata från ISM330DHCX */
    	//ISM330DHCX_AxesRaw_t raw;
    	//ISM330DHCX_ACC_GetAxesRaw(&acc_obj, &raw);
    	  /* Lägg in i ringbuffer */
    	//AccRB_Push(raw.x, raw.y, raw.z, HAL_GetTick());;




    		//}
    }
    else if(!logging_enabled)
    {
    	return;
    }

    /* 200 Hz sampling */
    //tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 200);
    HAL_Delay(1000);
  }
}

/*void Acc_Init(void)
{
	ISM330DHCX_IO_t io_ctx = {
	        .BusType  = ISM330DHCX_I2C_BUS,
	        .Address  = ISM330DHCX_I2C_ADD_L,
	        .Init     = NULL,
	        .DeInit   = NULL,
	        .ReadReg  = I2C_ReadReg,
	        .WriteReg = I2C_WriteReg,
	        .GetTick  = (int32_t (*)(void))HAL_GetTick
	    };

	    ISM330DHCX_RegisterBusIO(&acc_obj, &io_ctx);


	    ISM330DHCX_Init(&acc_obj);
	    ISM330DHCX_ACC_Enable(&acc_obj);


	    ISM330DHCX_ACC_SetOutputDataRate(&acc_obj, 200.0f);
	    ISM330DHCX_ACC_SetFullScale(&acc_obj, 4);
}*/

/*static void SendBuffer(void)
{
    printf("DATA BEGIN\r\n");

    uint16_t idx;
    uint16_t start;

    if (acc_rb.count < ACC_RINGBUFFER_SIZE)
        start = 0;
    else
        start = acc_rb.head;

    for (uint16_t i = 0; i < acc_rb.count; i++)
    {
        idx = (start + i) % ACC_RINGBUFFER_SIZE;

        AccSample_t *s = &acc_rb.buf[idx];

        printf("%lu,%ld,%ld,%ld,%d\r\n",
               s->timestamp,
               s->ax,
               s->ay,
               s->az,
			   s->label);
    }

    printf("DATA END\r\n");

    AccRB_Clear();
}*/

/*static void ISM330DHCX_SensorInit(void)
{
	printf("Sensor init\r\n");
	ISM330DHCX_IO_t io = {
	    .BusType  = ISM330DHCX_I2C_BUS,
	    .Address  = ISM330DHCX_I2C_ADD_L,
	    .Init     = NULL,
	    .DeInit   = NULL,
	    .ReadReg  = I2C_ReadReg,
	    .WriteReg = I2C_WriteReg,
	    .GetTick  = (int32_t (*)(void))HAL_GetTick,
	    .Handle   = &hi2c2
	};
	printf("Sensor init 2\r\n");
    ISM330DHCX_RegisterBusIO(&acc_obj, &io);

	printf("Sensor init 3\r\n");
     ST init (this disables XL internally!)
    ISM330DHCX_Init(&acc_obj);

	printf("Sensor init 4\r\n");
     ---------- FORCE ENABLE XL ----------
    ism330dhcx_xl_full_scale_set(
        &acc_obj.Ctx,
        ISM330DHCX_2g
    );
	printf("Sensor init 5\r\n");
    ism330dhcx_xl_data_rate_set(
        &acc_obj.Ctx,
        ISM330DHCX_XL_ODR_3332Hz
    );

	printf("Sensor init 6\r\n");
     Optional sanity check
    uint8_t ctrl1;
    ism330dhcx_read_reg(&acc_obj.Ctx, ISM330DHCX_CTRL1_XL, &ctrl1, 1);
    printf("CTRL1_XL after init = 0x%02X\r\n", ctrl1);
}*/
/*static void ISM330DHCX_SensorInit(void)
{
    uint8_t whoami;

    dev_ctx.read_reg  = platform_read;
    dev_ctx.write_reg = platform_write;
    dev_ctx.handle    = &hi2c2;

    ism330dhcx_device_id_get(&dev_ctx, &whoami);
    printf("WHO_AM_I = 0x%02X\r\n", whoami);

     Reset FIFO
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);
    HAL_Delay(10);

     Disable gyroscope
    ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_OFF);

     Disable gyro batching
    ism330dhcx_fifo_gy_batch_set(&dev_ctx, ISM330DHCX_GY_NOT_BATCHED);

     Disable temperature batching
    ism330dhcx_fifo_temp_batch_set(&dev_ctx, ISM330DHCX_TEMP_NOT_BATCHED);


     Enable accelerometer
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_3332Hz);
    ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_4g);

     Batch ONLY accelerometer into FIFO
    ism330dhcx_fifo_xl_batch_set(&dev_ctx, ISM330DHCX_XL_BATCHED_AT_3333Hz);

     Set FIFO to stream mode
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_FIFO_MODE);

    printf("FIFO configured\r\n");
}*/
static void ISM330DHCX_SensorInit(void)
{
    uint8_t whoami;
    uint8_t rst;

    dev_ctx.read_reg  = platform_read;
    dev_ctx.write_reg = platform_write;
    dev_ctx.handle    = &hi2c2;

    ism330dhcx_device_id_get(&dev_ctx, &whoami);
    printf("WHO_AM_I = 0x%02X\r\n", whoami);

    /* Reset device */
    ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        ism330dhcx_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Block Data Update */
    ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Disable sensors initially */
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_OFF);
    ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_OFF);

    /* Configure accelerometer (but DO NOT enable) */
    ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_4g);

    /* FIFO configuration (static) */
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);
    ism330dhcx_fifo_xl_batch_set(&dev_ctx, ISM330DHCX_XL_BATCHED_AT_3333Hz);
    ism330dhcx_fifo_gy_batch_set(&dev_ctx, ISM330DHCX_GY_NOT_BATCHED);

}

static void ISM330DHCX_StartSampling(void)
{
    fifo_count = 0;
    fs_total_samples = 0;
    fs_t0_ms = 0;

    /* Enable accelerometer */
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_3332Hz);

    /* Enable FIFO stream */
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_STREAM_MODE);

    //printf("Sampling started\r\n");
}

static void ISM330DHCX_StopSampling(void)
{
    /* Stop accelerometer */
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_OFF);

    /* Stop FIFO */
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);

    //printf("Sampling stopped\r\n");
}

/*static void ISM330DHCX_SensorInit@3200(void)
{
    uint8_t whoami;
    uint8_t rst;

    dev_ctx.read_reg  = platform_read;
    dev_ctx.write_reg = platform_write;
    dev_ctx.handle    = &hi2c2;

    ism330dhcx_device_id_get(&dev_ctx, &whoami);

     Reset once
    ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
            ism330dhcx_reset_get(&dev_ctx, &rst);
        } while (rst);

    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);

     Disable everything first
    ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_OFF);
    ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_OFF);

     Configure accelerometer
    ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_4g);
    //ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_3332Hz);

     FIFO
    //ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_BYPASS_MODE);
    ism330dhcx_fifo_xl_batch_set(&dev_ctx, ISM330DHCX_XL_BATCHED_AT_3333Hz);
    ism330dhcx_fifo_gy_batch_set(&dev_ctx, ISM330DHCX_GY_NOT_BATCHED);
    //ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_FIFO_MODE);
    ism330dhcx_fifo_mode_set(&dev_ctx, ISM330DHCX_STREAM_MODE);
}*/


static void ISM330DHCX_FIFO_Config(void)
{
    stmdev_ctx_t *ctx = &acc_obj.Ctx;

    ism330dhcx_fifo_mode_set(ctx, ISM330DHCX_BYPASS_MODE);
    /* Block data update */
    ism330dhcx_block_data_update_set(ctx, PROPERTY_ENABLE);

    /* Enable FIFO */
    //ism330dhcx_fifo_enable_set(ctx, PROPERTY_ENABLE);

    /* Batch accelerometer into FIFO */
    ism330dhcx_fifo_xl_batch_set(
        ctx,
        ISM330DHCX_XL_BATCHED_AT_3333Hz
    );
    /* Do NOT batch gyroscope */
        ism330dhcx_fifo_gy_batch_set(
            ctx,
            ISM330DHCX_GY_NOT_BATCHED
        );

    /* FIFO watermark = 511 samples */
    uint8_t wm_l = 0xFF;
    uint8_t wm_h = 0x01;
    ism330dhcx_write_reg(ctx, ISM330DHCX_FIFO_CTRL1, &wm_l, 1);
    ism330dhcx_write_reg(ctx, ISM330DHCX_FIFO_CTRL2, &wm_h, 1);

    /* Stream mode */
    ism330dhcx_fifo_mode_set(
        ctx,
        ISM330DHCX_STREAM_MODE
    );

    printf("FIFO configured\r\n");
}
static void ReadFIFOBlock(void)
{
    uint16_t fifo_level;
    ism330dhcx_fifo_data_level_get(&dev_ctx, &fifo_level);

    // Vänta tills sensorns FIFO har tillräckligt med data
    if (fifo_level < FIFO_BLOCK_SAMPLES)
        return;

    while (fifo_level > 0 && fifo_count < FIFO_THRESHOLD_BLOCK_SIZE)
    {
        uint16_t samples_to_read =
            (fifo_level > FIFO_BLOCK_SAMPLES) ?
            FIFO_BLOCK_SAMPLES : fifo_level;

        uint16_t bytes = samples_to_read * FIFO_FRAME_SIZE;

        // BLOCK READ – EN I2C-TRANSAKTION
        platform_read(
            dev_ctx.handle,
            ISM330DHCX_FIFO_DATA_OUT_TAG,
            fifo_block,
            bytes
        );

        for (uint16_t i = 0; i < samples_to_read; i++)
        {
            uint8_t *s = &fifo_block[i * FIFO_FRAME_SIZE];

            uint8_t tag = (s[0] & 0xF8) >> 3;
            if (tag != ISM330DHCX_XL_NC_TAG)
                continue;

            int16_t ax = (int16_t)(s[2] << 8 | s[1]);
            int16_t ay = (int16_t)(s[4] << 8 | s[3]);
            int16_t az = (int16_t)(s[6] << 8 | s[5]);

            FifoBuff_Push(ax, ay, az, HAL_GetTick());

            fifo_count++;
            fs_total_samples++;
        }

        fifo_level -= samples_to_read;
    }

    // Enkel frekvensmätning (valfri, kan tas bort senare)
    if (fs_t0_ms == 0)
        fs_t0_ms = HAL_GetTick();

    uint32_t now = HAL_GetTick();
    if (now - fs_t0_ms >= 5000)
    {
        float fs = fs_total_samples / 5.0f;
        printf("Measured sampling frequency: %.1f Hz\r\n", fs);
        fs_total_samples = 0;
        fs_t0_ms = now;
    }
}
/*static void ReadFIFOBlock@2200(void)
{
    uint16_t fifo_level;
    uint16_t fifo_buff_level = 0;
    ism330dhcx_fifo_data_level_get(&dev_ctx, &fifo_level);
    printf("Fifo level %d\r\n", fifo_level);
    if (fifo_level < FIFO_THRESHOLD_BLOCK_SIZE) //Wait until sensor FIFO queue reached threshold, before emtying buffer
           return;

    //while (fifo_level > 0)
    //{
    //for (uint16_t l = 0; l < FIFO_THRESHOLD_BLOCK_SIZE; l++)
    //{
    while (fifo_count < FIFO_THRESHOLD_BLOCK_SIZE)
    {
        uint16_t samples_to_read =
            (fifo_level > FIFO_BLOCK_SAMPLES) ?
            FIFO_BLOCK_SAMPLES : fifo_level;

        uint16_t bytes = samples_to_read * FIFO_FRAME_SIZE;
        //printf("Samples to read %d\r\n", samples_to_read);
        //BLOCK READ: EN I2C-TRANSAKTION
        platform_read(
            dev_ctx.handle,
            ISM330DHCX_FIFO_DATA_OUT_TAG,
            fifo_block,
            bytes
        );

        //Process block
        for (uint16_t i = 0; i < samples_to_read; i++)
        {
            uint8_t *sample = &fifo_block[i * FIFO_FRAME_SIZE];

            uint8_t tag = (sample[0] & 0xF8) >> 3;
            if (tag != ISM330DHCX_XL_NC_TAG)
                continue;

            int16_t ax = (int16_t)(sample[2] << 8 | sample[1]);
            int16_t ay = (int16_t)(sample[4] << 8 | sample[3]);
            int16_t az = (int16_t)(sample[6] << 8 | sample[5]);

            FifoBuff_Push(ax, ay, az, HAL_GetTick());

            fifo_count++;
            fs_total_samples++;
        }

        fifo_level -= samples_to_read;
        fifo_buff_level += samples_to_read;
        //printf("Fifobuff level %d\r\n", fifo_buff_level);
        //printf("Fifo count %d\r\n", fifo_count);

        if (fs_t0_ms == 0)
		{
			fs_t0_ms = HAL_GetTick();
		}

		//fs_total_samples += fifo_count;

		uint32_t now = HAL_GetTick();
		if (now - fs_t0_ms >= 5000)   // 5 sek fönster
		{
			float fs = fs_total_samples / 5.0f;
			printf("Measured sampling frequency: %.1f Hz\r\n", fs);

			fs_total_samples = 0;
			fs_t0_ms = now;
		}
   }
}*/
/*static void ReadFIFOBlockOrg(void)
{
	//printf("Read fifo block\r\n");
    uint16_t level;
    ism330dhcx_fifo_data_level_get(&dev_ctx, &level);
    //printf("Fifo level %d\r\n", level);

    if (level < FIFO_THRESHOLD_BLOCK_SIZE)
       return;

    //fifo_count = 0;
    //printf("Before for...\r\n");
    for (uint16_t i = 0; i < level; i++)
    {
    	//printf("Fifo count %d\r\n", fifo_count);
        uint8_t raw[7];   // [0]=TAG, [1..6]=XYZ

        ism330dhcx_fifo_out_raw_get(&dev_ctx, raw);

        uint8_t tag = (raw[0] & 0xF8) >> 3;
        //printf("TAG = 0x%02X\r\n", tag);

        //if (tag != ISM330DHCX_XL_3XC_TAG)
        if (tag != ISM330DHCX_XL_NC_TAG)
        	continue;
        FifoBuff_Push((int16_t)(raw[2] << 8 | raw[1]), (int16_t)(raw[4] << 8 | raw[3]), (int16_t)(raw[6] << 8 | raw[5]), HAL_GetTick());
        //{
        	//printf("Buffering...\r\n");
		fifo_buf[fifo_count].ax =
			(int16_t)(raw[2] << 8 | raw[1]);
		fifo_buf[fifo_count].ay =
			(int16_t)(raw[4] << 8 | raw[3]);
		fifo_buf[fifo_count].az =
			(int16_t)(raw[6] << 8 | raw[5]);

		fifo_buf[fifo_count].timestamp = HAL_GetTick();
		fifo_count++;
        //}

	    if (fs_t0_ms == 0)
	    {
	        fs_t0_ms = HAL_GetTick();
	    }

	    fs_total_samples += fifo_count;

	    uint32_t now = HAL_GetTick();
	    if (now - fs_t0_ms >= 5000)   // 5 sek fönster
	    {
	        float fs = fs_total_samples / 5.0f;
	        printf("Measured sampling frequency: %.1f Hz\r\n", fs);

	        fs_total_samples = 0;
	        fs_t0_ms = now;
	    }

    }
	//printf("exit fifo block\r\n");
}*/

static void SendFIFOBuff_UART(void)
{
    uint8_t start[2] = { BIN_START_1, BIN_START_2 };
    uint8_t end[2]   = { BIN_END_1, BIN_END_2 };

    /* Start marker */
    HAL_UART_Transmit(&huart1, start, 2, HAL_MAX_DELAY);

    /* Binary payload */
    HAL_UART_Transmit(
        &huart1,
        (uint8_t *)fifo_buf,
        fifo_count * sizeof(fifo_sample_t),
        HAL_MAX_DELAY
    );

    /* End marker */
    HAL_UART_Transmit(&huart1, end, 2, HAL_MAX_DELAY);
}

/*static void SendFIFOBuff_UART@2200(void)
{
    //printf("DATA BEGIN\r\n");

    for (uint16_t i = 0; i < fifo_count; i++)
    {
        printf("%lu,%d,%d,%d\r\n",
               fifo_buf[i].timestamp,
               fifo_buf[i].ax,
               fifo_buf[i].ay,
               fifo_buf[i].az);
    }

    //printf("DATA END\r\n");
}*/


/* USER CODE END 4 */

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
  if (htim->Instance == TIM6)
  {
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
