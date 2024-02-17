
#include "lcd1602_i2c2.h"

/******************************Privated functions*******************************/

#define LCD_BL2  	3
#define LCD_EN2	  2
#define LCD_RW2 	1
#define LCD_RS2 	0

#define COMMAND2		0
#define DATA2		1

static uint8_t 				backlight;
static I2C_HandleTypeDef 	*i2c_port2;

static void LCD1602_SendByte2(uint8_t mode, uint8_t data)
{
	uint8_t data_arr[4] = 
	{
		(data & 0xF0) | (backlight << LCD_BL2) | (1 << LCD_EN2) | (0 << LCD_RW2) | (mode << LCD_RS2),
		(data & 0xF0) | (backlight << LCD_BL2) | (0 << LCD_EN2) | (0 << LCD_RW2) | (mode << LCD_RS2),
		(data << 4)   | (backlight << LCD_BL2) | (1 << LCD_EN2) | (0 << LCD_RW2) | (mode << LCD_RS2),
		(data << 4)   | (backlight << LCD_BL2) | (0 << LCD_EN2) | (0 << LCD_RW2) | (mode << LCD_RS2)
	};

	HAL_I2C_Master_Transmit(i2c_port2, PCF8574_ADDRESS2, data_arr, 4, 1000);

	/* Wait to send */
	HAL_Delay(1);
}

/********************************************************************************/

void LCD1602_Init2(I2C_HandleTypeDef *hi2c2, uint8_t bl2)
{
	/* Set I2C port */
	i2c_port2 = hi2c2;
	
	/* Set backlight */
	if (bl2) { backlight = 1; }
	else { backlight = 0; }
	
	/* Wait power on */
	HAL_Delay(20);
	
	/* Initialize LCD 4 bits mode */
	LCD1602_SendByte2(COMMAND2, 0x33);
	LCD1602_SendByte2(COMMAND2, 0x32);
	LCD1602_SendByte2(COMMAND2, 0x28);
	/* Set mode */
	LCD1602_SendByte2(COMMAND2, AUTO_INCREASE_CURSOR2);
	LCD1602_SendByte2(COMMAND2, DISPLAY_ON_CURSOR_OFF2);
	LCD1602_SendByte2(COMMAND2, CLEAR_DISPLAY2);
}

void LCD1602_SetBackLight2(uint8_t bl)
{
	if (bl) { backlight = 1; }
	else { backlight = 0; }
	
	LCD1602_SendByte2(COMMAND2, 0x06);	/* Dummy byte */
}

void LCD1602_SetFunction2(LCD1602_Command_t2 cmd)
{	
	LCD1602_SendByte2(COMMAND2, cmd);			
}

void LCD1602_SetCursor2(uint8_t col, uint8_t row)
{
	uint8_t cmd = ( (row == 0) ? 0x80 : 0xC0 ) | col;
	LCD1602_SendByte2(COMMAND2, cmd);
}

void LCD1602_SendString2(char *str)
{
	while (*str != '\0')
	{
		LCD1602_SendByte2(DATA2, *str++);
	}
}

