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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "lcd_init.h"
#include "pic.h"
#include "delay.h"
#include <string.h>

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
#define ADC_BUF_SIZE      720U
#define WAVE_WINDOW       480U
#define ADC_DMA_SIZE      (ADC_BUF_SIZE * 2)
#define ADC_MAX_VALUE     4095U
#define ADC_REFRESH_MS    500U

static uint16_t adc_dma_buf[ADC_DMA_SIZE];
static uint16_t adc_frame[2][ADC_BUF_SIZE];
static uint16_t adc_draw_buf[ADC_BUF_SIZE];
static uint16_t wave_new_y[LCD_W];
static uint16_t wave_prev_y[LCD_W];
static uint8_t  wave_prev_valid = 0;
static volatile uint8_t adc_frame_idx = 0;
static volatile uint8_t adc_ready = 0;
static uint32_t last_draw_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void LCD_ShowWaveform(const uint16_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  把DMA刚填满的一半搬运到显示快照，避免绘制时被DMA覆盖
  */
static void ADC_SaveFrame(const uint16_t *src)
{
  uint8_t idx = adc_frame_idx ^ 1;

  memcpy(adc_frame[idx], src, ADC_BUF_SIZE * sizeof(uint16_t));
  adc_frame_idx = idx;
  adc_ready = 1;
}

/**
  * @brief  DMA半满回调：前半缓冲区已填满，复制到快照
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    ADC_SaveFrame(&adc_dma_buf[0]);
  }
}

/**
  * @brief  DMA全满回调：后半缓冲区已填满，复制到快照
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    ADC_SaveFrame(&adc_dma_buf[ADC_BUF_SIZE]);
  }
}

/**
  * @brief  在固定时间窗内绘制波形：直接从缓冲区起点取固定 WAVE_WINDOW
  *         个采样点，线性插值后映射到屏幕宽度。横轴代表固定时间，
  *         频率变化时屏幕上显示的周期数会跟着变化，且不会出现拼接断点。
  * @param  buf 采样缓冲区
  * @param  len 采样点数
  */
static void LCD_ShowWaveform(const uint16_t *buf, uint16_t len)
{
  uint16_t x;
  uint16_t idx;
  float pos, frac, v;
  const uint16_t dw = LCD_W;
  const uint16_t start = 0;

  if (len < WAVE_WINDOW + 1) return;

  for (x = 0; x < dw; x++)
  {
    pos = (float)start + (float)x * (float)(WAVE_WINDOW - 1) / (float)(dw - 1);
    idx = (uint16_t)pos;
    if (idx >= (uint16_t)(len - 1))
    {
      idx = (uint16_t)(len - 2);
    }
    frac = pos - (float)idx;
    v = (float)buf[idx] + ((float)buf[idx + 1] - (float)buf[idx]) * frac;
    if (v < 0.0f) v = 0.0f;
    if (v > (float)ADC_MAX_VALUE) v = (float)ADC_MAX_VALUE;

    wave_new_y[x] = (uint16_t)(LCD_H - 1 - ((uint32_t)(v + 0.5f) * (LCD_H - 1)) / ADC_MAX_VALUE);
  }

  for (x = 0; x < dw; x++)
  {
    uint16_t nt, nb, ot, ob, y;

    if (x == 0)
    {
      nt = nb = wave_new_y[0];
      ot = ob = wave_prev_y[0];
    }
    else
    {
      nt = (wave_new_y[x] < wave_new_y[x - 1]) ? wave_new_y[x] : wave_new_y[x - 1];
      nb = (wave_new_y[x] > wave_new_y[x - 1]) ? wave_new_y[x] : wave_new_y[x - 1];
      ot = (wave_prev_y[x] < wave_prev_y[x - 1]) ? wave_prev_y[x] : wave_prev_y[x - 1];
      ob = (wave_prev_y[x] > wave_prev_y[x - 1]) ? wave_prev_y[x] : wave_prev_y[x - 1];
    }

    if (!wave_prev_valid)
    {
      for (y = nt; y <= nb; y++)
      {
        LCD_DrawPoint(x, y, CYAN);
      }
    }
    else
    {
      for (y = nt; y <= nb; y++)
      {
        if ((y < ot) || (y > ob))
        {
          LCD_DrawPoint(x, y, CYAN);
        }
      }
      for (y = ot; y <= ob; y++)
      {
        if ((y < nt) || (y > nb))
        {
          LCD_DrawPoint(x, y, BLACK);
        }
      }
    }
  }

  memcpy(wave_prev_y, wave_new_y, dw * sizeof(uint16_t));
  wave_prev_valid = 1;
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
  MX_ADC1_Init();
  MX_SPI5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  delay_init(180);
  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_DMA_SIZE);
  HAL_TIM_Base_Start(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (adc_ready && ((HAL_GetTick() - last_draw_tick) >= ADC_REFRESH_MS))
    {
      adc_ready = 0;
      last_draw_tick = HAL_GetTick();

      __disable_irq();
      memcpy(adc_draw_buf, adc_frame[adc_frame_idx], sizeof(adc_draw_buf));
      __enable_irq();

      LCD_ShowWaveform(adc_draw_buf, ADC_BUF_SIZE);
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
