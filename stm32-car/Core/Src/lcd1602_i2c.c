
#include "lcd1602_i2c.h"

/******************************Privated functions*******************************/

#define LCD_BL  	3
#define LCD_EN  	2
#define LCD_RW  	1
#define LCD_RS  	0

#define COMMAND		0
#define DATA		1

static uint8_t 				_bl_status;
static I2C_HandleTypeDef*	_i2c_port;

static HAL_StatusTypeDef LCD1602_SendByte(uint8_t mode, uint8_t data)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	uint8_t data_arr[4] = 
	{
		(data & 0xF0) | (_bl_status << LCD_BL) | (1 << LCD_EN) | (0 << LCD_RW) | (mode << LCD_RS),
		(data & 0xF0) | (_bl_status << LCD_BL) | (0 << LCD_EN) | (0 << LCD_RW) | (mode << LCD_RS),
		(data << 4)   | (_bl_status << LCD_BL) | (1 << LCD_EN) | (0 << LCD_RW) | (mode << LCD_RS),
		(data << 4)   | (_bl_status << LCD_BL) | (0 << LCD_EN) | (0 << LCD_RW) | (mode << LCD_RS)
	};

	status = HAL_I2C_Master_Transmit(_i2c_port, PCF8574_ADDRESS, data_arr, 4, 1000);

	/* Wait to send */
	HAL_Delay(1);
	
	return status;
}

/********************************************************************************/

void LCD1602_Init(I2C_HandleTypeDef *hi2c)
{
	/* Set I2C port */
	_i2c_port = hi2c;
	
	/* Wait power on */
	HAL_Delay(20);
	
	/* Initialize LCD 4 bits mode */
	LCD1602_SendByte(COMMAND, 0x33);
	LCD1602_SendByte(COMMAND, 0x32);
	LCD1602_SendByte(COMMAND, 0x28);
	/* Set mode */
	LCD1602_SendByte(COMMAND, AUTO_INCREASE_CURSOR);
	LCD1602_SendByte(COMMAND, DISPLAY_ON_CURSOR_OFF);
	LCD1602_SendByte(COMMAND, CLEAR_DISPLAY);
}

void LCD1602_SetFunction(LCD1602_Command_t cmd)
{	
	if (cmd == BACKLIGHT_ON)
	{
		_bl_status = 1;
		LCD1602_SendByte(COMMAND, 0x06);	/* Dummy byte */
	}
	else if (cmd == BACKLIGHT_OFF)
	{
		_bl_status = 0;
		LCD1602_SendByte(COMMAND, 0x06);	/* Dummy byte */
	}
	else
	{
		LCD1602_SendByte(COMMAND, cmd);
	}
}

void LCD1602_SetCursor(uint8_t col, uint8_t row)
{
//	uint8_t cmd = ( (row == 0) ? 0x80 : 0xC0 ) | col;
//	LCD1602_SendByte(COMMAND, cmd);
	{
	uint8_t cmd = 0x80;
	
	switch (row)
	{
		case 0:
			cmd = 0x80;
			break;
		case 1:
			cmd = 0xC0;
			break;
		case 2:
			cmd = 0x94;
			break;
		case 3:
			cmd = 0xD4;
			break;
	}
	
	LCD1602_SendByte(0, cmd | col);
}

}

HAL_StatusTypeDef LCD1602_SendChar(char ch)
{
	return LCD1602_SendByte(DATA, ch);
}

HAL_StatusTypeDef LCD1602_SendString(char *str)
{
	while (*str != '\0')
	{
		if ( LCD1602_SendByte(DATA, *str++) != HAL_OK )
		{
			return HAL_ERROR;
		}
	}
	
	return HAL_OK;
}

