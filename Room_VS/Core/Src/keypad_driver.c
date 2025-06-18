#include "keypad_driver.h"

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void keypad_init(keypad_handle_t* keypad) {
    // Poner todas las filas en ALTO
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }
}

char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    char key_pressed = '\0';

    // Debounce
    HAL_Delay(5);

    // Identificar columna
    int col_index = -1;
    for (int i = 0; i < KEYPAD_COLS; i++) {
        if (keypad->col_pins[i] == col_pin) {
            col_index = i;
            break;
        }
    }
    if (col_index == -1) {
        return '\0'; // No se encontró la columna
    }

    // Poner todas las filas en ALTO
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }

    // Identificar fila
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        // Poner la fila actual en BAJO
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET);

        // Leer el pin de columna
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
            // Tecla detectada
            key_pressed = keypad_map[row][col_index];

            // Esperar hasta que se suelte la tecla (vuelva a ser ALTA)
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
                HAL_Delay(1);
            }
            break;
        }
        // Restaurar la fila a ALTO
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
    }

    // Restaurar: inicializar el keypad
    keypad_init(keypad);

    return key_pressed;
}