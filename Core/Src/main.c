/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "I2C_LCD.h"
#include <stdio.h>
#include "VL53L0X.h"
#include "STEPPER.h"
#include <math.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MyI2C_LCD I2C_LCD_1
#define STEPPER_MOTOR1   0

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t HeartChar[] = {0x00, 0x00, 0x0a, 0x15, 0x11, 0x0a, 0x04, 0x00};
uint8_t SmileyFaceChar[] = {0x00, 0x00, 0x0a, 0x00, 0x1f, 0x11, 0x0e, 0x00};
statInfo_t_VL53L0X distanceStr;
uint16_t distance=0;
uint16_t angle=0;
char str_distance[12];
char result_str[12];
char str_angle[12];
uint16_t distance_1=0;
uint16_t distance_2=0;
uint16_t angle_1=0;
uint16_t angle_2=0;
uint16_t meas1_Ok=0;
uint16_t meas2_Ok=0;
uint16_t distable=0;
uint16_t distance_n_1=0;
uint16_t distance_n_2=0;
uint16_t distance_n_3=0;
uint16_t result=0;
uint16_t angle_meas=0;
uint16_t NBStep=0;
double angle_meas_rad=0;
uint16_t AD_RES = 0;
uint8_t Stepper1_Dir = DIR_CW;
uint8_t Stepper2_Dir = DIR_CM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void Scan(int sens,int stepNB, int delay);
double toRadians(double degrees);
void Start_measure(void);
void LCD_menu(void);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //Initialise le driver du motor.
  STEPPERS_Init_TMR(&htim3);
  // Initialise le LCD
  I2C_LCD_NoDisplay(MyI2C_LCD);
  I2C_LCD_Init(MyI2C_LCD);
  I2C_LCD_Display(MyI2C_LCD);
  I2C_LCD_Backlight(MyI2C_LCD);
  // Initialise the VL53L0X capteur TOF
  initVL53L0X(1, &hi2c1);
  // Configure the sensor for high accuracy and speed in 20 cm.
  setSignalRateLimit(200);
  setVcselPulsePeriod(VcselPeriodPreRange, 10);
  setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  setMeasurementTimingBudget(300 * 1000UL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) // Boucle infinie pour gérer le comportement principal
	{
	    // Configuration initiale de l'écran LCD pour afficher les options utilisateur
		LCD_menu();

	    // Vérification si BTN1 est pressé (démarrer la mesure sur 90°)
	    if(HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET)
	    {
	    	Start_measure();
	    }

	    // Vérifie si BTN3 est pressé (déplacement manuel vers la gauche)
	    if(HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == GPIO_PIN_RESET)
	    {
	        Scan(1, 20, 100); // 20 steps à droite delay 100ms
	    }

	    // Vérifie si BTN4 est pressé (déplacement manuel vers la droite)
	    if(HAL_GPIO_ReadPin(BTN4_GPIO_Port, BTN4_Pin) == GPIO_PIN_RESET)
	    {
	        Scan(0, 20, 100); // 20 steps à gauche delay 100ms
	    }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, IN0_Pin|IN1_Pin|IN2_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Start_Pin */
  GPIO_InitStruct.Pin = B1_Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN0_Pin IN1_Pin IN2_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN0_Pin|IN1_Pin|IN2_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN4_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN4_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) // redirige le printf sur UART
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
	return ch;
}

void Scan(int sens, int stepNB, int delay)// fonction moteur pas à pas plus delay
{
	STEPPER_Step_NonBlocking(STEPPER_MOTOR1, stepNB, sens); //Dernier param : 0 == gauche, 1 == droite
	HAL_Delay(delay);
}
double toRadians(double degrees) // conversion degrees en radian
{
    return degrees * M_PI / 180.0; // M_PI est une constante pour π dans math.h
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) //syncronise le moteur au timer
{
	STEPPER_TMR_OVF_ISR(htim);
}
void Start_measure(void)
{
    // Boucle pour scanner l'angle jusqu'à 90° (103 itérations)
    for (int i = 0; i <= 103; i++) // 205 pour 180° ; ici 103 = 90°
    {
        // Effectue un déplacement du moteur vers la droite de 5 pas
        Scan(1, 5, 5); // sens 0 = gauche, 1 = droite; stepNB = nombre d'étapes; delay = délai entre étapes (en ms) (5 pas = 1°)

        // Lecture de la distance via le capteur
        distance = readRangeSingleMillimeters(&distanceStr);

        // Conversion de la distance en chaîne pour affichage
        itoa(distance, str_distance, 10);

        // Affichage sur le terminal (via printf) des données mesurées
        printf("Distance: %d\n\r", distance);

        // Calcul de l'angle basé sur le nombre de pas effectués
        angle = i * 5 * 180 / 1024; // Calcul basé sur les pas moteur (2048 pas par tour)
        itoa(angle, str_angle, 10);
        printf("Angle: %d °\n\r", angle);

        // Détection de la première distance
        if ((distance < 200) && meas1_Ok != 1) // Condition : distance < 200 mm et non encore mesurée
        {
            if(abs(distance - distance_n_1) <= 5) // Vérifie si la distance est stable (±5 mm)
                distable++;

            if(distable == 4) // Si la distance est stable pendant 4 itérations
            {
                distance_1 = distance; // Enregistrement de la distance
                printf("Détection Distance1: %d\n\r", distance);
                angle_1 = angle; // Enregistrement de l'angle correspondant
                meas1_Ok = 1; // Première mesure validée
            }
        }

        // Détection de la deuxième distance
        if ((distance > distance_n_2 + 5) && meas2_Ok != 1 && meas1_Ok == 1) // Vérifie si une deuxième mesure est valide
        {
            distance_2 = distance_n_3; // Enregistrement de la deuxième distance
            printf("Détection Distance 2: %d\n\r", distance_n_3);
            angle_2 = angle - 3; // Enregistrement de l'angle correspondant
            meas2_Ok = 1; // Deuxième mesure validée
            NBStep = i; // Sauvegarde du nombre de boucles atteintes pour retour à 0 du moteur
            i = 104; // Stoppe la boucle for
        }

        // Mise à jour de l'affichage sur l'écran LCD
        I2C_LCD_Init(MyI2C_LCD);
        I2C_LCD_Display(MyI2C_LCD);
        I2C_LCD_Backlight(MyI2C_LCD);
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
        I2C_LCD_WriteString(MyI2C_LCD, "Distance= ");
        I2C_LCD_WriteString(MyI2C_LCD, str_distance); // Affichage de la distance
        I2C_LCD_WriteString(MyI2C_LCD, "mm");
        I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
        I2C_LCD_WriteString(MyI2C_LCD, "Angle= ");
        I2C_LCD_WriteString(MyI2C_LCD, str_angle); // Affichage de l'angle
        I2C_LCD_WriteString(MyI2C_LCD, "deg");

        // Mise à jour des variables de distance pour vérifier la stabilité
        distance_n_3 = distance_n_2; // sauvegarde de la mesure n-3
        distance_n_2 = distance_n_1; // sauvegarde de la mesure n-2
        distance_n_1 = distance;	 // sauvegarde de la mesure n-1
    }

    // Retourne le moteur à la position initiale
    Scan(0, NBStep * 5, 4000); // sens 0 = gauche, 1 = droite; stepNB = nombre de pas, delay

    // Réinitialise les variables pour la prochaine mesure
    meas1_Ok = 0;
    meas2_Ok = 0;
    distable = 0;
    distance_n_1 = 0;
    distance_n_2 = 0;
    distance_n_3 = 0;
    NBStep = 0;

    // Calcul des résultats à partir des distances mesurées
    printf("Distance1: %d\n\r", distance_1);
    printf("Distance2: %d\n\r", distance_2);
    printf("Angle1: %d\n\r", angle_1);
    printf("Angle2: %d\n\r", angle_2);

    // Calcul de l'angle entre les deux distances
    angle_meas = angle_2 - angle_1;
    printf("Anglemeas deg: %d\n\r", angle_meas);
    angle_meas_rad = toRadians((double)angle_meas); // Conversion en radians
    printf("Anglemeas rad: %f\n\r", angle_meas_rad);

    // Calcul de la mesure finale (loi des cosinus) a²=b²+c²-2bc x cos(A)
    result = sqrt(pow((double)distance_1, 2) + pow((double)distance_2, 2) - 2 * distance_1 * distance_2 * cos(angle_meas_rad));
    printf("Measure: %d\n\r", result);

    // Affichage du résultat final sur le LCD pendant 10 secondes
    itoa(result, result_str, 10);
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
    I2C_LCD_WriteString(MyI2C_LCD, "                "); // Efface la ligne
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "Measure = ");
    I2C_LCD_WriteString(MyI2C_LCD, result_str); // Affichage de la mesure finale
    I2C_LCD_WriteString(MyI2C_LCD, "mm");
    HAL_Delay(10000); // Pause avant de redémarrer
}
void LCD_menu(void)
{
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
    I2C_LCD_WriteString(MyI2C_LCD, "BT1 = START     ");
    I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
    I2C_LCD_WriteString(MyI2C_LCD, "BT3 = L BT4 = R ");
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
