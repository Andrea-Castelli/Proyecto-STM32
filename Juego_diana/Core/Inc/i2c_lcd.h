#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "stm32f4xx_hal.h"

void lcd_init (void);   // Inicializa el LCD
void lcd_send_cmd (char cmd);  // Envía comandos
void lcd_send_data (char data);  // Envía datos
void lcd_send_string (char *str);  // Envía frases
void lcd_put_cur(int row, int col);  // Mueve el cursor
void lcd_clear (void);  // Limpia la pantalla



#endif