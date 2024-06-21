#include "main.h"
#include "i2c-lcd.h"
#include "String.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Define the maximum speed of the stepper motor in steps per second
#define MAX_SPEED 1000
// Define the acceleration of the stepper motor in steps per second squared
#define ACCELERATION 1000
// Define the number of steps per revolution of the stepper motor
#define STEPS_PER_REV 200
// Define the number of microsteps per step
#define MICROSTEPS 1
// Define the number of steps to move for each microstep
#define STEPS_PER_MICROSTEP (STEPS_PER_REV * MICROSTEPS)
// Define the maximum delay between steps in microseconds
#define MAX_STEP_DELAY (1000000 / MAX_SPEED)
// Define the minimum delay between steps in microseconds
#define MIN_STEP_DELAY (1000000 / (MAX_SPEED * ACCELERATION))
// Define the current step delay in microseconds
uint32_t step_delay = MAX_STEP_DELAY;

/// Define pin for A4988_X
#define DIR_PIN_X GPIO_PIN_4
#define DIR_PORT_X GPIOB
#define STEP_PIN_X GPIO_PIN_8
#define STEP_PORT_X GPIOB
#define EN_PIN_X GPIO_PIN_9
#define EN_PORT_X GPIOB

/// Define pin for A4988_Y
#define DIR_PIN_Y GPIO_PIN_9
#define DIR_PORT_Y GPIOA
#define STEP_PIN_Y GPIO_PIN_10
#define STEP_PORT_Y GPIOA
#define EN_PIN_Y GPIO_PIN_11
#define EN_PORT_Y GPIOA

/// Define pin for A4988_Z
#define DIR_PIN_Z GPIO_PIN_12
#define DIR_PORT_Z GPIOA
#define STEP_PIN_Z GPIO_PIN_15
#define STEP_PORT_Z GPIOA
#define EN_PIN_Z GPIO_PIN_3
#define EN_PORT_Z GPIOB

/// Define pin for switch controlling
#define SW_PIN_X GPIO_PIN_13
#define SW_PORT_X GPIOC
#define SW_PIN_Y GPIO_PIN_14
#define SW_PORT_Y GPIOC
#define SW_PIN_Z GPIO_PIN_15
#define SW_PORT_Z GPIOC

/// Define pin for servo
#define Servo_PIN GPIO_PIN_10
#define Servo_PORT GPIOB

/// Define pin for color sensor 
#define S0_PIN GPIO_PIN_0
#define S0_PORT GPIOB
#define S1_PIN GPIO_PIN_1
#define S1_PORT GPIOB
#define S2_PIN GPIO_PIN_15
#define S2_PORT GPIOB
#define S3_PIN GPIO_PIN_14
#define S3_PORT GPIOB

/// Define pin for servo 
#define Servo_PORT GPIOB
#define Servo_PIN GPIO_PIN_10

/// Define pin for color sensor
#define S0_PORT GPIOB
#define S0_PIN GPIO_PIN_0
#define S1_PORT GPIOB
#define S1_PIN GPIO_PIN_1
#define S2_PORT GPIOB
#define S2_PIN GPIO_PIN_15
#define S3_PORT GPIOB
#define S3_PIN GPIO_PIN_14
#define OUT_PORT GPIOA
#define OUT_PIN GPIO_PIN_8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

void Delay_us (uint16_t delay)
{
  for(int i=0;i<99;i++)
	{
		for(int j=0;j<(delay/10);j++)
		{
			
		}
	}
}
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void servo_write(int angle)
{
	TIM2->CCR3 = map(angle,0,180,250,1250);
}
void stepX(int steps, uint8_t direction)
{	
	HAL_GPIO_WritePin(EN_PORT_X,EN_PIN_X,GPIO_PIN_RESET);
  int x;
  if (direction == 1) HAL_GPIO_WritePin(DIR_PORT_X, DIR_PIN_X, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(DIR_PORT_X, DIR_PIN_X, GPIO_PIN_RESET);
  for(x=0; x<steps; x++)
  {
    HAL_GPIO_WritePin(STEP_PORT_X, STEP_PIN_X, GPIO_PIN_SET);
		Delay_us(MAX_STEP_DELAY);
		//HAL_Delay(1);
    HAL_GPIO_WritePin(STEP_PORT_X, STEP_PIN_X, GPIO_PIN_RESET);
		Delay_us(MAX_STEP_DELAY);
		//HAL_Delay(1);
  }
}
void stepY(int steps, uint8_t direction)
{	
	HAL_GPIO_WritePin(EN_PORT_Y,EN_PIN_Y,GPIO_PIN_RESET);
  int x;
  if (direction == 1)	HAL_GPIO_WritePin(DIR_PORT_Y, DIR_PIN_Y, GPIO_PIN_SET);
  else	HAL_GPIO_WritePin(DIR_PORT_Y, DIR_PIN_Y, GPIO_PIN_RESET);
  for(x=0; x<steps; x++)
  {
    HAL_GPIO_WritePin(STEP_PORT_Y, STEP_PIN_Y, GPIO_PIN_SET);
		//HAL_Delay(1);
    Delay_us(MAX_STEP_DELAY);
    HAL_GPIO_WritePin(STEP_PORT_Y, STEP_PIN_Y, GPIO_PIN_RESET);
    //HAL_Delay(1);
		Delay_us(MAX_STEP_DELAY);
  }
}
void stepZ(int steps, uint8_t direction)
{	
	HAL_GPIO_WritePin(EN_PORT_Z,EN_PIN_Z,GPIO_PIN_RESET);
  int x;
  if (direction == 1)
    HAL_GPIO_WritePin(DIR_PORT_Z, DIR_PIN_Z, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(DIR_PORT_Z, DIR_PIN_Z, GPIO_PIN_RESET);
  for(x=0; x<steps; x++)
  {
    HAL_GPIO_WritePin(STEP_PORT_Z, STEP_PIN_Z, GPIO_PIN_SET);
		//HAL_Delay(1);
		Delay_us(MAX_STEP_DELAY);
    HAL_GPIO_WritePin(STEP_PORT_Z, STEP_PIN_Z, GPIO_PIN_RESET);
		Delay_us(MAX_STEP_DELAY);
		//HAL_Delay(1);
  }
}
void init_stepper()
{
	while(HAL_GPIO_ReadPin(SW_PORT_Z,SW_PIN_Z) == GPIO_PIN_RESET)
	{
		stepY(1,0);
	} 
	while(HAL_GPIO_ReadPin(SW_PORT_X,SW_PIN_X) == GPIO_PIN_RESET)
	{
		stepX(1,0);
	}
	while(HAL_GPIO_ReadPin(SW_PORT_Y,SW_PIN_Y) == GPIO_PIN_RESET)
	{
		stepZ(1,0);
	}
	
}
void Move_to_grab_position()
{
	stepX(1650,1);
	stepZ(1500,1);
	
	servo_write(130); // mo tay gap

	stepY(350,1);
	servo_write(180); // dong tay gap
}
void Grab()
{
	HAL_Delay(200);
	servo_write(130); // mo tay gap
	stepY(360,1);
	servo_write(180); // dong tay gap
	stepY(360,0);
	
}
void Move_to_red()
{	
	HAL_Delay(100);
	while(HAL_GPIO_ReadPin(SW_PORT_Z,SW_PIN_Z) == GPIO_PIN_RESET)
	{
		stepY(1,0);
	}
	stepX(700,0);
	while(HAL_GPIO_ReadPin(SW_PORT_Y,SW_PIN_Y) == GPIO_PIN_RESET)
	{
		stepZ(1,0);
	}
	stepY(350,1);
	servo_write(130);
	HAL_Delay(100);

	// Move Back To Grab 
	stepY(350,0);
	stepZ(1500,1);
	stepX(700,1);
}
void Move_to_yellow()
{
	HAL_Delay(100);
	while(HAL_GPIO_ReadPin(SW_PORT_Z,SW_PIN_Z) == GPIO_PIN_RESET)
	{
		stepY(1,0);
	}
	stepX(1150,0);
	while(HAL_GPIO_ReadPin(SW_PORT_Y,SW_PIN_Y) == GPIO_PIN_RESET)
	{
		stepZ(1,0);
	}
	stepY(350,1);
	servo_write(130);
	HAL_Delay(100);
	
	// Move Back To Grab
	stepY(350,0);
	stepX(1150,1);
	stepZ(1500,1);
}
void Move_to_green()
{
	HAL_Delay(100);
	while(HAL_GPIO_ReadPin(SW_PORT_Z,SW_PIN_Z) == GPIO_PIN_RESET)
	{
		stepY(1,0);
	}
	while(HAL_GPIO_ReadPin(SW_PORT_X,SW_PIN_X) == GPIO_PIN_RESET)
	{
		stepX(1,0);
	}
	while(HAL_GPIO_ReadPin(SW_PORT_Y,SW_PIN_Y) == GPIO_PIN_RESET)
	{
		stepZ(1,0);
	}
	stepY(350,1);
	servo_write(130);
	HAL_Delay(100);
	
	// Move Back To Grab
	stepY(350,0);
	stepX(1650,1);
	stepZ(1500,1);
}


// handle color sensor
uint32_t red, blue, green;
uint32_t icValue =0;
uint32_t preIcValue =0;
uint32_t T =0;
uint32_t isCaptured = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(isCaptured == 0)
		{
			preIcValue = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			isCaptured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING);
		}
		else if(isCaptured == 1)
		{
			icValue = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim,0);
			if(icValue > preIcValue)
			{
				T = icValue - preIcValue;
			}
			else if(preIcValue > icValue)
			{
				T = 0xFFFF + preIcValue - icValue;
			}
			isCaptured = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING);
		}
	}
}
uint32_t redFreq = 0;
uint16_t Learn[100];
uint32_t red_fre = 0, green_fre = 0, yellow_fre = 0;
uint32_t readColor()
{	
		HAL_GPIO_WritePin(S2_PORT, S2_PIN, 0); 
	  HAL_GPIO_WritePin(S3_PORT, S3_PIN, 0);
    redFreq = T;
		return redFreq;
		HAL_Delay(1);
}

void learn_color()
{	
	HAL_GPIO_WritePin(EN_PORT_X,EN_PIN_X,GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_PORT_Y,EN_PIN_Y,GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_PORT_Z,EN_PIN_Z,GPIO_PIN_SET);
	uint32_t total = 0 ;
	//string time;
	// learn red
	lcd_put_cur(0,0);
	lcd_send_string("Hoc Mau Do");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("3");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("2");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("1");
	HAL_Delay(1000);
	lcd_clear();
	for (int i = 0; i < 100; i++) {
    readColor();
    Learn[i] = redFreq;
		total += Learn[i];
  }
	red_fre = total/100;
	total = 0;
	lcd_put_cur(0,0);
	lcd_send_string("Done");
	HAL_Delay(1000);
	lcd_clear();
	// learn green
	
	lcd_put_cur(0,0);
	lcd_send_string("Hoc Mau Xanh La");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("3");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("2");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("1");
	HAL_Delay(1000);
	lcd_clear();
	for (int i = 0; i < 100; i++) {
    readColor();
    Learn[i] = redFreq;
		total += Learn[i];
  }
	green_fre = total/100;
	total = 0;
	lcd_put_cur(0,0);
	lcd_send_string("Done");
	HAL_Delay(1000);
	lcd_clear();
	
	// learn yellow
	
	lcd_put_cur(0,0);
	lcd_send_string("Hoc Mau Vang");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("3");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("2");
	HAL_Delay(1000);
	lcd_put_cur(1,0);
	lcd_send_string("1");
	HAL_Delay(1000);
	lcd_clear();
	for (int i = 0; i < 100; i++) {
    readColor();
    Learn[i] = redFreq;
		total += Learn[i];
  }
	yellow_fre = total/100;
	total = 0;
	lcd_put_cur(0,0);
	lcd_send_string("Done");
	HAL_Delay(1000);
	lcd_clear();
}
uint32_t redBuffer[150], min_number ;
uint32_t Min()
{
	min_number = 10000;
	uint16_t i;
	for(i = 0;i < 150;i ++)
	{
			if(redBuffer[i] < min_number) min_number = redBuffer[i];
	}
	return min_number;
}
uint16_t isGreen = 0, isYellow = 0, isRed = 0, isNoObject = 0;
void colorDetect()
{	
	HAL_Delay(2000);
	readColor();
  if (redFreq < (green_fre + 10)) 
	{ 
    for (int i = 0; i < 150; i++) 
		{
      readColor();
      redBuffer[i] = redFreq;
    }
		Min();
    if ( (red_fre + yellow_fre) / 2 < min_number &&  min_number <  (green_fre+red_fre) / 2 ) 
		{
      isRed = 1 ;
			
    }
    else if (((red_fre + green_fre) / 2) < min_number) 
		{
      isGreen = 1;
			
    }
    else if (min_number < ((red_fre + yellow_fre) / 2)) 
		{
      isYellow = 1;
			
    }
  }
  else 
	{
    isNoObject = 1;
  }
}
uint32_t degree = 0;
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

	
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_GPIO_WritePin(S0_PORT, S0_PIN, 1); 
	HAL_GPIO_WritePin(S1_PORT, S1_PIN, 0);
	
	//servo_write(130);
	lcd_init();
	//init_stepper();
	
	learn_color();
	uint32_t isFirstTime = 1,countRed = 0, countYellow = 0, countGreen = 0;
	char sred[100], syellow[100], sgreen[100];
  while (1)
  {
		colorDetect();
		if(isRed == 1)
		{	
			countRed++;
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string("Do");
			lcd_put_cur(0,5);
			lcd_send_string("Vang");
			lcd_put_cur(0,10);
			lcd_send_string("Xanh");
			lcd_put_cur(1,0);
			sprintf(sred,"%d",countRed);
			lcd_send_string(sred);
			lcd_put_cur(1,5);
			sprintf(syellow,"%d",countYellow);
			lcd_send_string(syellow);
			lcd_put_cur(1,10);
			sprintf(sgreen,"%d",countGreen);
			lcd_send_string(sgreen);
			
			if(isFirstTime == 1)
			{
				Move_to_grab_position();
				Move_to_red();
				isFirstTime = 0;
			}
			else
			{
				Grab();
				Move_to_red();
			}
		}
		if(isGreen == 1)
		{	
			countGreen++;
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string("Do");
			lcd_put_cur(0,5);
			lcd_send_string("Vang");
			lcd_put_cur(0,10);
			lcd_send_string("Xanh");
			lcd_put_cur(1,0);
			sprintf(sred,"%d",countRed);
			lcd_send_string(sred);
			lcd_put_cur(1,5);
			sprintf(syellow,"%d",countYellow);
			lcd_send_string(syellow);
			lcd_put_cur(1,10);
			sprintf(sgreen,"%d",countGreen);
			lcd_send_string(sgreen);
			if(isFirstTime == 1)
			{
				Move_to_grab_position();
				Move_to_green();
				isFirstTime = 0;
			}
			else
			{
				Grab();
				Move_to_green();
			}
		}
		if(isYellow == 1)
		{	
			countYellow++;
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string("Do");
			lcd_put_cur(0,5);
			lcd_send_string("Vang");
			lcd_put_cur(0,10);
			lcd_send_string("Xanh");
			lcd_put_cur(1,0);
			sprintf(sred,"%d",countRed);
			lcd_send_string(sred);
			lcd_put_cur(1,5);
			sprintf(syellow,"%d",countYellow);
			lcd_send_string(syellow);
			lcd_put_cur(1,10);
			sprintf(sgreen,"%d",countGreen);
			lcd_send_string(sgreen);
			if(isFirstTime == 1)
			{
				Move_to_grab_position();
				Move_to_yellow();
				isFirstTime = 0;
			}
			else
			{
				Grab();
				Move_to_yellow();
			}
		}
		else 
		{
			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string("Do");
			lcd_put_cur(0,5);
			lcd_send_string("Vang");
			lcd_put_cur(0,10);
			lcd_send_string("Xanh");
			lcd_put_cur(1,0);
			sprintf(sred,"%d",countRed);
			lcd_send_string(sred);
			lcd_put_cur(1,5);
			sprintf(syellow,"%d",countYellow);
			lcd_send_string(syellow);
			lcd_put_cur(1,10);
			sprintf(sgreen,"%d",countGreen);
			lcd_send_string(sgreen);
		}
		HAL_Delay(1000);
		isGreen = 0, isYellow = 0, isRed = 0, isNoObject = 0;
  }
	
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 144-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB14 PB15
                           PB3 PB4 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
