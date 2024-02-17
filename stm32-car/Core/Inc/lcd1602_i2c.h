
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD1602_I2C_H
#define __LCD1602_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#define 	PCF8574_ADDRESS			(0x27 << 1)

typedef enum
{
	CLEAR_DISPLAY 					= 0x01,
	RETURN_HOME 					= 0x02,
	AUTO_INCREASE_CURSOR 			= 0x06,
	DISPLAY_OFF 					= 0x08,
	DISPLAY_ON_CURSOR_OFF 			= 0x0C,
	DISPLAY_ON_CURSOR_ON 			= 0x0E,
	DISPLAY_ON_CURSOR_BLINKING 		= 0x0F,
	SHIFT_SURSOR_TO_LEFT 			= 0x10,
	SHIFT_SURSOR_TO_RIGHT 			= 0x14,
	SCROLL_DISPLAY_TO_LEFT 			= 0x18,
	SCROLL_DISPLAY_TO_RIGHT 		= 0x1C,
	BACKLIGHT_ON					= 0xFE,
	BACKLIGHT_OFF					= 0xFF
} LCD1602_Command_t;

/*============== Functions prototypes ==============*/
void LCD1602_Init(I2C_HandleTypeDef* hi2c);
void LCD1602_SetFunction(LCD1602_Command_t cmd);
void LCD1602_SetCursor(uint8_t col, uint8_t row);
HAL_StatusTypeDef LCD1602_SendChar(char ch);
HAL_StatusTypeDef LCD1602_SendString(char *str);
/*==================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __LCD1602_I2C_H */
