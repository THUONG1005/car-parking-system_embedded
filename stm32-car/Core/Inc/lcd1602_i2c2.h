
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD1602_I2C2_H
#define __LCD1602_I2C2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#define 	PCF8574_ADDRESS2			(0x27 << 1)

typedef enum
{
	CLEAR_DISPLAY2 					= 0x01,
	RETURN_HOME2 					= 0x02,
	AUTO_INCREASE_CURSOR2 			= 0x06,
	DISPLAY_OFF2 					= 0x08,
	DISPLAY_ON_CURSOR_OFF2 			= 0x0C,
	DISPLAY_ON_CURSOR_ON2 			= 0x0E,
	DISPLAY_ON_CURSOR_BLINKING2 	= 0x0F,
	SHIFT_SURSOR_TO_LEFT2 			= 0x10,
	SHIFT_SURSOR_TO_RIGHT2 			= 0x14,
	SCROLL_DISPLAY_TO_LEFT2 		= 0x18,
	SCROLL_DISPLAY_TO_RIGHT2 		= 0x1C
} LCD1602_Command_t2;

/*============== Functions prototypes ==============*/
void LCD1602_Init2(I2C_HandleTypeDef *hi2c, uint8_t bl);
void LCD1602_SetFunction2(LCD1602_Command_t2 cmd);
void LCD1602_SetCursor2(uint8_t col, uint8_t row);
void LCD1602_SetBackLight2(uint8_t bl);
void LCD1602_SendString2(char *str);
/*==================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __LCD1602_I2C2_H */
