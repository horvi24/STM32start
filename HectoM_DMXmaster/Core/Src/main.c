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
/*
 ******************************************************************************
 * Hemera Hecto Master
 *
 * HHM_DMXmaster(demo)
 *
 * hw: 20029097 HhectoM r.B
 ******************************************************************************
 * v0.3
 * - read key, dipsw        ...ok
 * - dmx master             ...ok
 * - hartbeat led           ...ok
 * - demo (police lights)   ...ok
 *
 * - control tiol
 * - bootloader thru tiol
 * - bluetooth
 ******************************************************************************
 * TIM16	DMX	Break		        IRQ_UPDATE
 * TIM17	DMR MAB		            IRQ_UPDATE, IRQ_CC1
 * PB14	    DMX USART Tx Break      IRQ from TIM17
 * USART2	DMX 250kbps 8B-2SB-0P	IRQ from TIM16
 * USART4	DBG 250kbps 9B-1SB-0P
 ******************************************************************************
 * source: https://github.com/aleksandrgilfanov/stm32f4-dmx-transmitter
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
#define PROG_NAME       "HH-M DMX512 master demo\r\n"
#define PROG_VERSION    "(31/05/24) v0.3\r\n"


#define LED_HB_PERIOD   42//200
#define LED_HB_BEAT     1//3 //10
#define LED_HB_PAUSE    12//20//40

#define OPT_SWITCH_1    1
#define OPT_SWITCH_2    2
#define OPT_SWITCH_3    3
#define OPT_SWITCH_4    4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t test_packet[512];
uint8_t i = 0, up = 1;
uint8_t HeartBeat = 0;
uint8_t OptSwitch = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART4 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart4, (uint8_t*) &ch, 1, 0xFFFF);

    return ch;
}

void read_Hardware(void){

    if (OPT_SW1())
        OptSwitch |= 1 << 0;
    else
        OptSwitch &= ~(1 << 0);
    if (OPT_SW2())
        OptSwitch |= 1 << 1;
    else
        OptSwitch &= ~(1 << 1);
    if (OPT_SW3())
        OptSwitch |= 1 << 2;
    else
        OptSwitch &= ~(1 << 2);
    if (OPT_SW4())
        OptSwitch |= 1 << 3;
    else
        OptSwitch &= ~(1 << 3);

    switch (HeartBeat++) {
        case 0:
        case LED_HB_PAUSE:
            LED_HB_ON();
            break;
        case LED_HB_BEAT:
        case (LED_HB_PAUSE + LED_HB_BEAT):
            LED_HB_OFF();
            break;
        case LED_HB_PERIOD:
            HeartBeat = 0;
            printf("%010lu: [", HAL_GetTick());
            print_half_byte(OptSwitch);
            printf("]\r\n");
            break;
    }
    if (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){
        printf("HH-M DMX512 master demo\r\n");
        printf(PROG_VERSION);
        while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){};
    };

}



void DMX_breath(uint8_t channel) {
    test_packet[channel] = i & 0xFF;
    test_packet[channel + 4] = i & 0xFF;
    test_packet[channel + 10] = (0xFF - i) & 0xFF;
    test_packet[channel + 14] = (0xFF - i) & 0xFF;

    if (up) {
        i = i + 10;
        if (i > 244)
            up = 0;
    } else {
        i = i - 10;
        if (i < 10)
            up = 1;
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
    MX_TIM16_Init();
    MX_TIM17_Init();
    MX_USART2_UART_Init();
    MX_USART4_UART_Init();
    /* USER CODE BEGIN 2 */

    for (int i = 0; i < sizeof(test_packet); i++)
        test_packet[i] = 0; //i & 0xFF;

    //dbg_dumppacket(test_packet,513);
    //while (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){};
    //while (!HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)){};

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        DBG_OUT1();
        DBG_OUT2();
        DBG_OUT3();
        dmx_send(test_packet, sizeof(test_packet));
        DMX_breath(0);

        read_Hardware();

        //HAL_Delay(1); //10ms

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

    /* USER CODE BEGIN TIM16_Init 0 */

    /* USER CODE END TIM16_Init 0 */

    /* USER CODE BEGIN TIM16_Init 1 */

    /* USER CODE END TIM16_Init 1 */
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = TIM_CLK_MHZ - 1;
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = DMX_SLOT + DMX_MBS;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM16_Init 2 */

    /*
     * Update flag should be cleaned, to prevent unwanted interrupt. Inerrupt
     * flag is generated earlier, by following call:
     * HAL_TIM_Base_Init -> TIM_Base_SetConfig -> TIMx->EGR = TIM_EGR_UG
     */
    __HAL_TIM_CLEAR_IT(&htim16, TIM_FLAG_UPDATE);
    /* Enable interrupts, but don't enable timer counter, to prevent interrupts */
    __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE);

    /* USER CODE END TIM16_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

    /* USER CODE BEGIN TIM17_Init 0 */

    /* USER CODE END TIM17_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = { 0 };
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

    /* USER CODE BEGIN TIM17_Init 1 */

    /* USER CODE END TIM17_Init 1 */
    htim17.Instance = TIM17;
    htim17.Init.Prescaler = TIM_CLK_MHZ - 1;
    htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim17.Init.Period = DMX_BREAK - 4 + DMX_MAB;
    htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim17.Init.RepetitionCounter = 0;
    htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim17) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = DMX_BREAK - 4;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1)
            != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig)
            != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM17_Init 2 */

    /*
     * Update flag should be cleaned, to prevent unwanted interrupt. Inerrupt
     * flag is generated earlier, by following call:
     * HAL_TIM_Base_Init -> TIM_Base_SetConfig -> TIMx->EGR = TIM_EGR_UG
     */
    //__HAL_TIM_CLEAR_IT(&htim17, TIM_FLAG_UPDATE);
    //__HAL_TIM_CLEAR_IT(&htim17, TIM_IT_CC1);//h24
    /* Enable interrupts, but don't enable timer counter, to prevent interrupts */
    __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim17, TIM_IT_CC1);
    //__HAL_TIM_ENABLE_IT(&htim17, TIM_IT_CC2);

    /* USER CODE END TIM17_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 250000;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_2;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
            != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
            != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART4_UART_Init(void) {

    /* USER CODE BEGIN USART4_Init 0 */

    /* USER CODE END USART4_Init 0 */

    /* USER CODE BEGIN USART4_Init 1 */

    /* USER CODE END USART4_Init 1 */
    huart4.Instance = USART4;
    huart4.Init.BaudRate = 250000;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART4_Init 2 */

    /* USER CODE END USART4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB,
    DBG_OUT3_Pin | DBG_OUT2_Pin | LED_HB_Pin | DBG_OUT1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DMX_TX_BREAK_GPIO_Port, DMX_TX_BREAK_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DMX_DE_GPIO_Port, DMX_DE_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : DBG_OUT3_Pin DBG_OUT2_Pin LED_HB_Pin DBG_OUT1_Pin */
    GPIO_InitStruct.Pin = DBG_OUT3_Pin | DBG_OUT2_Pin | LED_HB_Pin
            | DBG_OUT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : DMX_TX_BREAK_Pin */
    GPIO_InitStruct.Pin = DMX_TX_BREAK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DMX_TX_BREAK_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DMX_DE_Pin */
    GPIO_InitStruct.Pin = DMX_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DMX_DE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SW1_Pin SW2_Pin SW3_Pin SW4_Pin */
    GPIO_InitStruct.Pin = SW1_Pin | SW2_Pin | SW3_Pin | SW4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : KEY_Pin */
    GPIO_InitStruct.Pin = KEY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
