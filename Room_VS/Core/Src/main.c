/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "led_driver.h"
#include "ring_buffer.h"
#include "keypad_driver.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
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

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
led_handle_t led1 = { .port = GPIOA, .pin = GPIO_PIN_5 }; // LD2 en NUCLEO-L476RG

#define UART2_RX_LEN 16
uint8_t uart2_rx_buffer[UART2_RX_LEN];
ring_buffer_t uart2_rx_rb;
uint8_t uart2_rx_data; // Variable to hold received data

keypad_handle_t keypad = {
    .row_ports = {KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port},
    .row_pins  = {KEYPAD_R1_Pin, KEYPAD_R2_Pin, KEYPAD_R3_Pin, KEYPAD_R4_Pin},
    .col_ports = {KEYPAD_C1_GPIO_Port, KEYPAD_C2_GPIO_Port, KEYPAD_C3_GPIO_Port, KEYPAD_C4_GPIO_Port},
    .col_pins  = {KEYPAD_C1_Pin, KEYPAD_C2_Pin, KEYPAD_C3_Pin, KEYPAD_C4_Pin}
};

#define KEYPAD_BUFFER_LEN 16
uint8_t keypad_buffer[KEYPAD_BUFFER_LEN];
ring_buffer_t keypad_rb;

/* --- Variables de Control de Acceso --- */
#define ACCESS_CODE_LENGTH 4
const char access_code[ACCESS_CODE_LENGTH + 1] = "D514"; // Codigo de acceso
char entered_code[ACCESS_CODE_LENGTH + 1] = {0};
uint8_t entered_count = 0;
uint8_t access_granted = 0;
uint8_t access_denied = 0;
/* USER CODE END PV */

/* Redirección de printf a UART2 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    (void)file;
    return len;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t last_interrupt_time = 0;  // Variable global para almacenar el tiempo de la última interrupción
#define DEBOUNCE_DELAY 50           // Tiempo de debounce en milisegundos

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
      HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);
      ring_buffer_write(&uart2_rx_rb, uart2_rx_data);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_interrupt_time > DEBOUNCE_DELAY) {
        char key = keypad_scan(&keypad, GPIO_Pin);
        if (key != '\0') {
            printf("Tecla presionada: %c\r\n", key); // Imprime la tecla presionada
            if (!access_granted && !access_denied) {
                ring_buffer_write(&keypad_rb, (uint8_t)key);
            }
        }
        last_interrupt_time = current_time;  // Actualiza el tiempo de la última interrupción
    }
}

/* Ayuda */ // Función para encender un LED n veces con un retardo
void blink_led(led_handle_t *led, uint8_t times, uint32_t delay_ms) {
    for (uint8_t i = 0; i < times; i++) {
        led_on(led);
        HAL_Delay(delay_ms);
        led_off(led);
        HAL_Delay(delay_ms);
    }
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  ring_buffer_init(&uart2_rx_rb, uart2_rx_buffer, UART2_RX_LEN);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);

  led_init(&led1);
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN);
  keypad_init(&keypad);

  // Inicializa la pantalla OLED
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("Hola Jhojan", Font_7x10, White);
  ssd1306_SetCursor(10, 30);
  ssd1306_WriteString("Ingresa tu clave:", Font_7x10, White);
  ssd1306_UpdateScreen();

  printf("Sistema listo. Esperando pulsaciones del teclado...\r\n"); // Mensaje inicial por UART
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* --- Logica de Control de Acceso --- */
    uint8_t key_from_buffer;

    if (ring_buffer_read(&keypad_rb, &key_from_buffer)) {
        char key = (char)key_from_buffer;
        

        // Mostrar tecla en OLED
        ssd1306_SetCursor(10 + entered_count * 15, 50);
        char key_str[2] = {key, 0};
        ssd1306_WriteString(key_str, Font_7x10, White);
        ssd1306_UpdateScreen();

        // Almacenar tecla
        if (entered_count < ACCESS_CODE_LENGTH) {
            entered_code[entered_count++] = key;
        }

        // Verificar si se ha ingresado el código completo
        if (entered_count == ACCESS_CODE_LENGTH) {
            entered_code[ACCESS_CODE_LENGTH] = '\0';

            // Mostrar código ingresado por UART
            printf("Codigo ingresado: %s\r\n", entered_code);

            // Comparar
            if (strcmp(entered_code, access_code) == 0) {
                // Acceso permitido
                access_granted = 1;
                access_denied = 0;
                led_on(&led1); // Enciende el LED

                // Mostrar en OLED
                ssd1306_Fill(Black);
                ssd1306_SetCursor(10, 20);
                ssd1306_WriteString("Bienvenido", Font_7x10, White);
                ssd1306_UpdateScreen();

                printf("ACCESO PERMITIDO\r\n");
            } else {
                // Acceso denegado
                access_granted = 0;
                access_denied = 1;
                blink_led(&led1, 3, 200);

                // Mostrar en OLED
                ssd1306_Fill(Black);
                ssd1306_SetCursor(10, 20);
                ssd1306_WriteString("ACCESO DENEGADO", Font_7x10, White);
                ssd1306_UpdateScreen();

                printf("ACCESO DENEGADO\r\n"); // Mensaje por UART
            }

            // Esperar un tiempo antes de reiniciar
            HAL_Delay(1200);
            led_off(&led1);
            entered_count = 0;
            memset(entered_code, 0, sizeof(entered_code));
            access_granted = 0; // Reinicia la bandera
            access_denied = 0;    // Reinicia la bandera

            // Limpiar pantalla OLED
            ssd1306_Fill(Black);
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("Hola Jhojan", Font_7x10, White);
            ssd1306_SetCursor(10, 30);
            ssd1306_WriteString("Ingresa tu clave:", Font_7x10, White);
            ssd1306_UpdateScreen();
        }
    }
    else
    {
         HAL_Delay(10);
    }

    // Mostrar el estado del LED en la consola
    if (ring_buffer_count(&uart2_rx_rb) >= 5) {
        for (int i = 0; i < 5; i++) {
            if (ring_buffer_read(&uart2_rx_rb, &uart2_rx_data)) {
                HAL_UART_Transmit(&huart2, &uart2_rx_data, 1, HAL_MAX_DELAY);
            }
        }
    }

    HAL_Delay(10); // Pequeño retardo para evitar saturar el bucle
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  hi2c1.Init.Timing = 0x10D19CE4;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|KEYPAD_R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin KEYPAD_R1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|KEYPAD_R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C1_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEYPAD_C4_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_C2_Pin KEYPAD_C3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_C2_Pin|KEYPAD_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYPAD_R2_Pin KEYPAD_R4_Pin KEYPAD_R3_Pin */
  GPIO_InitStruct.Pin = KEYPAD_R2_Pin|KEYPAD_R4_Pin|KEYPAD_R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
#endif /* USE_FULL_ASSERT */