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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CENTER_FREQ 50
#define SOGI_K 1
#define SAMPLE_TIME 0.000075
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct SOGI_Structure
{
    float32_t Ug, Ua, Ub, k, W0, dt;
}SOGI_s;

SOGI_s sogi_s;
float32_t t;
float32_t wave_input;

float32_t d,q;
float32_t sinVal, cosVal;
float32_t w = CENTER_FREQ * 2 * PI;
float32_t phase_W = 0.0;

uint16_t ADC1_Buffer[2];
uint16_t ADC2_Buffer[2];

arm_pid_instance_f32 PID_S;
float32_t out_cosine;
float32_t out_amplitude = 0.7;
float32_t out_duty;
uint16_t out_pwm;

SOGI_s sogi_s_I;
float32_t Id, Iq;

float32_t out_U;
float32_t out_U_set = 36;

uint16_t soft_start = 30000;

uint8_t PinB9_Pressed,PinC11_Pressed,PinC10_Pressed;
uint16_t PinB9_Pressed_Counter,PinC11_Pressed_Counter,PinC10_Pressed_Counter;

float32_t pf;
float32_t pf_buffer[10];
uint16_t pf_counter;
uint32_t pf_index;
float32_t Iq_buffer[20], Id_buffer[20];
float32_t Iq_mean, Id_mean;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SOGI_init(SOGI_s * s);
void SOGI_calculate(SOGI_s * s);
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
  MX_DMA_Init();
  MX_HRTIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	SOGI_init(&sogi_s);
	SOGI_init(&sogi_s_I);
	sogi_s_I.dt = 4*SAMPLE_TIME;
	
	PID_S.Kp = 0.002;
	PID_S.Ki = 0;
	PID_S.Kd = 0.002;
	arm_pid_init_f32(&PID_S, 1);
	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_Delay(200);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC1_Buffer, 2);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t *)ADC2_Buffer, 2);
	
	OLED_Init();
	
	HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	
	OLED_ShowChar(0, 0, 'A', OLED_6X8);
	OLED_Update();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		static uint16_t OLED_counter = 200;
		
		if(1)
		{
			if(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && !PinB9_Pressed)
			{
				if(PinB9_Pressed_Counter==200)
				{
					phase_W+=0.05;
					LIMIT(phase_W,-0.2,0.4);
					
					PinB9_Pressed = 1;
				}
				else
					PinB9_Pressed_Counter++;
			}
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
			{
				if(PinB9_Pressed && PinB9_Pressed_Counter)
					PinB9_Pressed_Counter--;
				if(PinB9_Pressed && !PinB9_Pressed_Counter)
					PinB9_Pressed = 0;
			}
			
			if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) && !PinC11_Pressed)
			{
				if(PinC11_Pressed_Counter==200)
				{
					phase_W-=0.05;
					LIMIT(phase_W,-0.2,0.4);
					
					PinC11_Pressed = 1;
				}
				else
					PinC11_Pressed_Counter++;
			}
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11))
			{
				if(PinC11_Pressed && PinC11_Pressed_Counter)
					PinC11_Pressed_Counter--;
				if(PinC11_Pressed && !PinC11_Pressed_Counter)
					PinC11_Pressed = 0;
			}
			
			if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) && !PinC10_Pressed)
			{
				if(PinC10_Pressed_Counter==200)
				{
					
					
					PinC10_Pressed = 1;
				}
				else
					PinC10_Pressed_Counter++;
			}
			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10))
			{
				if(PinC10_Pressed && PinC10_Pressed_Counter)
					PinC10_Pressed_Counter--;
				if(PinC10_Pressed && !PinC10_Pressed_Counter)
					PinC10_Pressed = 0;
			}
		}
		if(!OLED_counter)
		{
			OLED_Printf(0, 0, OLED_6X8, "PF:%.3f", pf);
			OLED_Printf(0, 8, OLED_6X8, "W:%.3f", phase_W);
			OLED_Update();
			OLED_counter = 400;
		}
		else
		{
			OLED_counter--;
		}
//		OLED_Clear();
		
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		static uint16_t SOGI_I_counter = 1;
		
		if(soft_start > 1)
			soft_start--;
		if(soft_start == 1 && (out_pwm < 6000))
		{
			out_amplitude = 0.8;
			HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
			soft_start = 0;
		}
		
		
		t += SAMPLE_TIME;
		if(t > (2 * PI / w))
			t -=	2 * PI / w;
		sogi_s.Ug = ADC1_Buffer[0] / 2048.0 - 1;
		arm_sin_cos_f32(180 / PI * w * t, &sinVal, &cosVal);
		SOGI_calculate(&sogi_s);
		arm_park_f32(cosVal, sinVal, &d, &q, sogi_s.Ub, sogi_s.Ua);
		w += phase_W - q * PI;
		out_duty = arm_cos_f32(w * t);
		out_duty = out_duty > 0 ? out_duty : -out_duty;
		out_duty *= out_amplitude;
		out_pwm = 54400 * out_duty;
		LIMIT(out_pwm,4200,48960);
		__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, out_pwm);
		
		
		if(!SOGI_I_counter)
		{
			sogi_s_I.Ug = ADC2_Buffer[0] / 2048.0 - 1;
			SOGI_calculate(&sogi_s_I);
			arm_park_f32(sogi_s_I.Ua, sogi_s_I.Ub, &Id, &Iq, sogi_s.Ub, sogi_s.Ua);
			SOGI_I_counter = 3;
			
			out_U = 0.02362 * ADC1_Buffer[1] - 48.24810356169533;
			out_amplitude -= arm_pid_f32(&PID_S, out_U_set - out_U) / 100;
			LIMIT(out_amplitude,0.5,0.8);
		}
		else
		{
			SOGI_I_counter--;
		}
	}
	
	if(htim->Instance == TIM4)
	{
		static uint16_t i;
		i++;
		Id_buffer[i] = Id;
		Iq_buffer[i] = Iq;
		if(i>=19)
		{
			i = 0;
			arm_mean_f32(Id_buffer, 20, &Id_mean);
			arm_max_f32(Iq_buffer, 20, &Iq_mean, &pf_index);
			arm_sqrt_f32(Id_mean*Id_mean + Iq_mean*Iq_mean, &pf);
			pf = Id_mean / pf -0.02;
		}
	}
}
void SOGI_init(SOGI_s * s)
{
    s -> Ug = 0;
    s -> Ua = 0;
    s -> Ub = 0;
    s -> k = SOGI_K;
    s -> W0 = CENTER_FREQ * 2 * PI;
    s -> dt = SAMPLE_TIME;
}
void SOGI_calculate(SOGI_s * s)
{
    s -> Ua += ((s -> Ug - s -> Ua) * s -> k - s -> Ub) * s -> W0 * s -> dt;
    LIMIT(s -> Ua, -1.0, 1.0);
    s -> Ub += s -> Ua * s -> W0 * s -> dt;
    LIMIT(s -> Ub, -1.0, 1.0);
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
