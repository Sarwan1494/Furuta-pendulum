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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "as5600.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PI 3.1415926535897932384626433832795
#define TRUE 1
#define FALSE 0
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 16

#define CLAMP(x, min, max) \
	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile uint16_t raw_angle;
volatile float angle;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static volatile uint16_t gLastError;

void MyFlagInterruptHandler(void);
float getPendulumAngle();
void calibrate_upright();
float wrapAngle(float angle);
void move_stepper_to(float deg);
void move_stepper_by(float deg);
void calculate_base_states();

// LQR gains
// float K[] = { 20.15, 3.5, -3.16, -1.3 };
float K[] = {29.15, 3.5, -3.16, -1.3};

// Buffer for uart string
uint8_t buf[50];

float u = 0.0;
float sensor_offset = 0.0;
float dt = 0.02; // 50Hz loop
float scale_factor = 400;

unsigned long loop_timer;

float base_angle;
float base_vel;
float theta_dot;
uint32_t time_ms;

/*12000
 *12000
 *4000
 *1500
 *
 */

L6474_Init_t init = {
	6500,																																													   /// Acceleration rate in step/s2. Range: (0..+inf).
	6500,																																													   /// Deceleration rate in step/s2. Range: (0..+inf).
	4000,																																													   /// Maximum speed in step/s. Range: (30..10000].
	900,																																													   /// Minimum speed in step/s. Range: [30..10000).
	250,																																													   /// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
	450,																																													   /// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
	L6474_CONFIG_OC_SD_ENABLE,																																								   /// Overcurrent shutwdown (OC_SD field of CONFIG register).
	L6474_CONFIG_EN_TQREG_TVAL_USED,																																						   /// Torque regulation method (EN_TQREG field of CONFIG register).
	L6474_STEP_SEL_1_8,																																										   /// Step selection (STEP_SEL field of STEP_MODE register).
	L6474_SYNC_SEL_1,																																										   /// Sync selection (SYNC_SEL field of STEP_MODE register).
	L6474_FAST_STEP_12us,																																									   /// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
	L6474_TOFF_FAST_8us,																																									   /// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
	3,																																														   /// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
	21,																																														   /// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
	L6474_CONFIG_TOFF_044us,																																								   /// Target Swicthing Period (field TOFF of CONFIG register).
	L6474_CONFIG_SR_320V_us,																																								   /// Slew rate (POW_SR field of CONFIG register).
	L6474_CONFIG_INT_16MHZ,																																									   /// Clock setting (OSC_CLK_SEL field of CONFIG register).
	(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN | L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE | L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	int32_t pos;
	uint16_t mySpeed;
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
	// MX_GPIO_Init();
	MX_USART2_UART_Init();
	// MX_SPI1_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	printf("Furuta pendulum, LQR controler\r\n");
	HAL_Delay(1000);

	printf("Initializing the motor driver\r\n");

	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);

	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, &init);

	uint16_t status = BSP_MotorControl_CmdGetStatus(0);
	printf("Status register value: %u\r\n", status);

	BSP_MotorControl_CmdEnable(0);

	BSP_MotorControl_SoftStop(0);
	HAL_Delay(10);

	BSP_MotorControl_HardStop(0);
	HAL_Delay(10);

	BSP_MotorControl_ResetAllDevices();
	HAL_Delay(50);

	status = BSP_MotorControl_CmdGetStatus(0);
	printf("Status register value: %u\r\n", status);

	// BSP_MotorControl_Move(0, FORWARD, 16000);

	/* Wait for the motor of device 0 ends moving */
	// BSP_MotorControl_WaitWhileActive(0);
	pos = BSP_MotorControl_GetPosition(0);
	printf("Position: %u\r\n", pos);

	/* Set the current position of device 0 to be the Home position */
	BSP_MotorControl_SetHome(0, pos);

	HAL_Delay(10);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(3000);

	// BSP_MotorControl_Move(0,FORWARD,3000);
	// BSP_MotorControl_WaitWhileActive(0);

	pos = BSP_MotorControl_GetPosition(0);
	printf("Position: %u\r\n", pos);

	// HAL_Delay(1000);

	// BSP_MotorControl_GoTo(0,360.0);
	// move_stepper_by(360.0);
	// move_stepper_to(360.0);
	pos = BSP_MotorControl_GetPosition(0);
	printf("Position: %u\r\n", pos);

	calibrate_upright();

	HAL_Delay(500);

	time_ms = HAL_GetTick();
	loop_timer = time_ms;
	printf("Current tick: %lu\r\n", time_ms);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		static bool start = FALSE;
		static float prev_angle = 0;
		static float vel_filtered = 0;
		static float velocity_cmd = 0;

		angle = getPendulumAngle();
		printf("Angle: %.2f, Base angle: %.2f, Base vel: %.2f, LQR: %.2f\r\n",
			   angle, base_angle, base_vel, u);
		// printf("Base angle: %.2f\r\n", base_angle);
		// printf("Base vel: %.2f\r\n", base_vel);

		float hold_zone = 0.5;
		float control_zone = 5.0;

		if (angle <= control_zone && angle >= -control_zone && start == FALSE)
		{
			start = TRUE;
			BSP_MotorControl_CmdEnable(0);
			BSP_MotorControl_HardStop(0);
			HAL_Delay(1);
		}
		else
		{
			BSP_MotorControl_SoftStop(0); // Stop powering the motor if the pendulum fell.
			BSP_MotorControl_CmdDisable(0);
			// BSP_MotorControl_ResetAllDevices();
			// HAL_Delay(50);
			HAL_Delay(2);
		}

		if (start)
		{
			if (fabs(angle) < hold_zone)
			{
				// Do nothing to avoid jitter
			}
			else if (fabs(angle > 10.0))
			{ // Pendulum fell
				start = FALSE;
			}
			else
			{
				float raw_vel = (angle - prev_angle) / dt;
				float a = 0.68;
				vel_filtered += a * (raw_vel - vel_filtered);
				theta_dot = vel_filtered;
				prev_angle = angle;
				if (fabs(theta_dot) < 2)
					theta_dot = 0;

				// Base state
				calculate_base_states();

				u = -(K[0] * angle + K[1] * theta_dot + K[2] * base_vel + K[3] * base_angle);

				u = CLAMP(u, -4000, 4000);
				// printf("LQR output: %.2f\r\n", u);
				velocity_cmd = u * dt;
				move_stepper_by(velocity_cmd);
				// move_stepper_to(velocity_cmd);
			}
		}
		time_ms = HAL_GetTick();
		while ((time_ms - loop_timer) < (dt * 1000))
		{
			time_ms = HAL_GetTick();
		}
		loop_timer = HAL_GetTick();

		// raw_angle = AS5600_ReadRawAngle(&hi2c1);

		// sprintf((char*)buf,"Raw angle: %u\r\n",raw_angle);

		// HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

		// angle = getPendulumAngle();//AS5600_ReadAngleDeg(&hi2c1);

		// printf("%f\r\n", angle);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */
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

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 5000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Flag_pin_Pin */
	GPIO_InitStruct.Pin = Flag_pin_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Flag_pin_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MyFlagInterruptHandler(void)
{
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);

	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
	{
		// HIZ state
		// Action to be customized
		strcpy((char *)buf, "1");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
	{
		// Forward direction is set
		// Action to be customized
		strcpy((char *)buf, "2");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}
	else
	{
		// Backward direction is set
		// Action to be customized
	}

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
	{
		// Command received by SPI can't be performed
		// Action to be customized
		strcpy((char *)buf, "3");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
	{
		// command received by SPI does not exist
		//  Action to be customized
		strcpy((char *)buf, "4");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0)
	{
		// undervoltage lock-out
		//  Action to be customized
		strcpy((char *)buf, "5");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
	{
		// thermal warning threshold is reached
		//  Action to be customized
		strcpy((char *)buf, "6");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0)
	{
		// thermal shut down threshold is reached
		//  Action to be customized
		strcpy((char *)buf, "7");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0)
	{
		// overcurrent detection
		//  Action to be customized
		strcpy((char *)buf, "8");
		HAL_UART_Transmit(&huart2, buf, strlen((char *)buf), HAL_MAX_DELAY);
	}
}

volatile int count = 0;

// Flag pin interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
	{
		uint16_t status;
		count++;

		status = BSP_MotorControl_CmdGetStatus(0);

		char msg[64];

		sprintf(msg, "FLAG interrupt! Status = %u\r\n", status);

		HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg),
						  HAL_MAX_DELAY);
	}
}

// Wrap angle -PI..PI
float wrapAngle(float angle)
{
	angle = fmod(angle + 180.0, 360.0);
	if (angle < 0)
		angle += 360.0;
	return angle - 180.0;
}

void calibrate_upright()
{
	float sum = 0;

	for (int i = 0; i < 200; i++)
	{
		// sensor.update();
		sum += AS5600_ReadAngleDeg(&hi2c1);
		// sum += sensor.getAngle();
		HAL_Delay(2);
	}

	sensor_offset = sum / 200.0;

	printf("Upright offset: ");
	printf("%f\r\n", sensor_offset);
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

float getPendulumAngle()
{
	float Angle = AS5600_ReadAngleDeg(&hi2c1) - sensor_offset;
	// printf("Angle: ");
	// printf("%f\r\n",angle);
	Angle = wrapAngle(Angle);
	return Angle;
}

// Tell the stepper motor to move to a particular angle in degrees
void move_stepper_to(float deg)
{
	int32_t steps =
		(int32_t)(STEPS_PER_REVOLUTION * MICROSTEPS * deg / 360.0f);
	uint8_t dir = 0;
	if (steps < 0)
	{
		steps = -1 * steps;
		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		dir = FORWARD;
	}
	else
	{

		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		dir = BACKWARD;
	}
	BSP_MotorControl_GoTo(0, steps);
	// BSP_MotorControl_WaitWhileActive(0);
}

void move_stepper_by(float deg)
{
	// int32_t steps = (int32_t)( STEPS_PER_REVOLUTION * MICROSTEPS * deg / 360.0);
	uint8_t dir = 0;

	float temp =
		STEPS_PER_REVOLUTION * MICROSTEPS * deg / 360.0f;

	// printf("deg=%f temp=%f\r\n", deg, temp);

	int32_t steps = (int32_t)temp;

	if (steps < 0)
	{
		steps = -1 * steps;
		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		// dir = FORWARD;
		dir = BACKWARD;
	}
	else
	{

		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		// dir = BACKWARD;
		dir = FORWARD;
	}
	BSP_MotorControl_Move(0, dir, steps);
	// BSP_MotorControl_GoTo(0, steps);

	// BSP_MotorControl_Run(0,dir,steps);
	// BSP_MotorControl_WaitWhileActive(0);
}

void calculate_base_states()
{

	static float prev_base_angle = 0;
	static float base_vel_filtered = 0;
	int32_t step_position;

	step_position = BSP_MotorControl_GetPosition(0);			  // Gives the absolute step count
	base_angle = (step_position * 360.0f) / (200.0 * MICROSTEPS); // Converting steps to degrees

	float alpha = 0.5; // stronger filtering
	float raw_vel = (base_angle - prev_base_angle) / dt;
	base_vel_filtered += alpha * (raw_vel - base_vel_filtered);

	base_vel = base_vel_filtered;
	// Update the angle
	prev_base_angle = base_angle;

	// printf("Base angle: %.2f\r\n", base_angle);
	// printf("Base vel: %.2f\r\n", base_vel);
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
	// gLastError = error;
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
