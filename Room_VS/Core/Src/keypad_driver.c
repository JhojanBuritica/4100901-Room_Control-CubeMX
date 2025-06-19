#include "keypad_driver.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
// Define el mapeo de teclas del keypad 4x4
static const char key_mapping[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// Inicializa el keypad
void keypad_init(keypad_handle_t *keypad) {
    // Configura los pines de las filas como salidas
    for (int row_index = 0; row_index < KEYPAD_ROWS; row_index++) {
        HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_RESET); // Inicializa en bajo
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->row_pins[row_index];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->row_ports[row_index], &GPIO_InitStruct);
    }

    // Configura los pines de las columnas como entradas con pull-up
    for (int col_index = 0; col_index < KEYPAD_COLS; col_index++) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->col_pins[col_index];
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Interrupción por flanco de bajada
        GPIO_InitStruct.Pull = GPIO_PULLUP;          // Pull-up para evitar lecturas flotantes
        HAL_GPIO_Init(keypad->col_ports[col_index], &GPIO_InitStruct);
    }
}

// Escanea el keypad para detectar la tecla presionada
char keypad_scan(keypad_handle_t *keypad, uint16_t col_pin) {
    char pressed_key = '\0';

    static uint32_t last_key_time = 0;
    if (HAL_GetTick() - last_key_time < 100) {
        // Debounce: ignora si la última pulsación fue hace menos de 100 ms
        return pressed_key;
    }
    last_key_time = HAL_GetTick();

    // Determina qué columna generó la interrupción
    switch (col_pin) {
        case KEYPAD_C1_Pin:
            // Escanea las filas para la columna 1
            for (int row_index = 0; row_index < KEYPAD_ROWS; row_index++) {
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_RESET); // Activa la fila
                if (HAL_GPIO_ReadPin(keypad->col_ports[0], keypad->col_pins[0]) == GPIO_PIN_RESET) {
                    pressed_key = key_mapping[row_index][0];
                }
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_SET); // Desactiva la fila
            }
            break;
        case KEYPAD_C2_Pin:
            // Escanea las filas para la columna 2
            for (int row_index = 0; row_index < KEYPAD_ROWS; row_index++) {
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_RESET); // Activa la fila
                if (HAL_GPIO_ReadPin(keypad->col_ports[1], keypad->col_pins[1]) == GPIO_PIN_RESET) {
                    pressed_key = key_mapping[row_index][1];
                }
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_SET); // Desactiva la fila
            }
            break;
        case KEYPAD_C3_Pin:
            // Escanea las filas para la columna 3
            for (int row_index = 0; row_index < KEYPAD_ROWS; row_index++) {
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_RESET); // Activa la fila
                if (HAL_GPIO_ReadPin(keypad->col_ports[2], keypad->col_pins[2]) == GPIO_PIN_RESET) {
                    pressed_key = key_mapping[row_index][2];
                }
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_SET); // Desactiva la fila
            }
            break;
        case KEYPAD_C4_Pin:
            // Escanea las filas para la columna 4
            for (int row_index = 0; row_index < KEYPAD_ROWS; row_index++) {
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_RESET); // Activa la fila
                if (HAL_GPIO_ReadPin(keypad->col_ports[3], keypad->col_pins[3]) == GPIO_PIN_RESET) {
                    pressed_key = key_mapping[row_index][3];
                }
                HAL_GPIO_WritePin(keypad->row_ports[row_index], keypad->row_pins[row_index], GPIO_PIN_SET); // Desactiva la fila
            }
            break;
    }
    keypad_init(keypad); // Reinicializa el keypad después de escanear

    return pressed_key;
}