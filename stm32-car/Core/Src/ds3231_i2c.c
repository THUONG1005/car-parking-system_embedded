
#include "ds3231_i2c.h"

#define	_DS3231_ADDRESS			(0x68 << 1)
#define _DS3231_TIMEOUT			1000

static I2C_HandleTypeDef*	_i2c_port = NULL;

static uint8_t BCDToDec(uint8_t bcd)
{
	return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t DecToBCD(uint8_t dec)
{
	return ((dec / 10) << 4) | (dec % 10);
}

void DS3231_Init(I2C_HandleTypeDef* hi2c)
{
	_i2c_port = hi2c;
}

void DS3231_DeInit(void)
{
	_i2c_port = NULL;
}

HAL_StatusTypeDef DS3231_GetTime(DS3231_TimeTypeDef* DS3231_TimeStruct)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t buffer[7];
	
	/* Second Register */
	status = HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 7, _DS3231_TIMEOUT);
	if (status != HAL_OK)
	{
		return status;
	}
	
	DS3231_TimeStruct->Sec 		= BCDToDec(buffer[0] & 0x7F);
	DS3231_TimeStruct->Min 		= BCDToDec(buffer[1] & 0x7F);
	DS3231_TimeStruct->Hour 	= BCDToDec(buffer[2] & 0x3F);	//24h format
	DS3231_TimeStruct->DoW 		= BCDToDec(buffer[3] & 0x07);
	DS3231_TimeStruct->Date 	= BCDToDec(buffer[4] & 0x3F);
	DS3231_TimeStruct->Month 	= BCDToDec(buffer[5] & 0x1F);
	DS3231_TimeStruct->Year 	= BCDToDec(buffer[6]);

	return status;
}

HAL_StatusTypeDef DS3231_SetTime(DS3231_TimeTypeDef* DS3231_TimeStruct)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t buffer[7] = 
	{	
		DecToBCD(DS3231_TimeStruct -> Sec),
		DecToBCD(DS3231_TimeStruct -> Min),
		DecToBCD(DS3231_TimeStruct -> Hour),
		DecToBCD(DS3231_TimeStruct -> DoW),
		DecToBCD(DS3231_TimeStruct -> Date),
		DecToBCD(DS3231_TimeStruct -> Month),
		DecToBCD(DS3231_TimeStruct -> Year)
	};
	
	/* Second Register */
	status = HAL_I2C_Mem_Write(_i2c_port, _DS3231_ADDRESS, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 7, _DS3231_TIMEOUT);

	return status;
}

HAL_StatusTypeDef DS3231_SetAlarm(DS3231_AlarmTypeDef* DS3231_AlarmStruct)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t buffer[4] = 
	{
		DecToBCD(DS3231_AlarmStruct->Sec),
		DecToBCD(DS3231_AlarmStruct->Min),
		DecToBCD(DS3231_AlarmStruct->Hour),
		DecToBCD(DS3231_AlarmStruct->Day_Date)
	};

	if (DS3231_AlarmStruct->Mode == ONCE_PER_SEC_MATCH)
	{
		buffer[0] |= (1 << 7);
		buffer[1] |= (1 << 7);
		buffer[2] |= (1 << 7);
		buffer[3] |= (1 << 7);
	}
	else if (DS3231_AlarmStruct->Mode == SEC_MATCH)
	{
		buffer[1] |= (1 << 7);
		buffer[2] |= (1 << 7);
		buffer[3] |= (1 << 7);
	}
	else if (DS3231_AlarmStruct->Mode == MIN_SEC_MATCH)
	{
		buffer[2] |= (1 << 7);
		buffer[3] |= (1 << 7);
	}
	else if (DS3231_AlarmStruct->Mode == HOU_MIN_SEC_MATCH)
	{
		buffer[3] |= (1 << 7);
	}
	else if (DS3231_AlarmStruct->Mode == DAT_HOU_MIN_SEC_MATCH)
	{
		//Do nothing
	}
	else if (DS3231_AlarmStruct->Mode == DAY_HOU_MIN_SEC_MATCH)
	{
		buffer[3] |= (1 << 6);
	}
	else
	{
		return status;	//Do nothing
	}

	/* Second Alarm 1 Register */
	status = HAL_I2C_Mem_Write(_i2c_port, _DS3231_ADDRESS, 0x07, I2C_MEMADD_SIZE_8BIT, buffer, 4, _DS3231_TIMEOUT);

	return status;
}

HAL_StatusTypeDef DS3231_GetAlarm(DS3231_AlarmTypeDef* DS3231_AlarmStruct)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t buffer[4];

	/* Second Alarm 1 Register */
	status = HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x07, I2C_MEMADD_SIZE_8BIT, buffer, 4, _DS3231_TIMEOUT);
	if (status != HAL_OK)
	{
		return status;
	}
	
	if ( (buffer[3] & 0x80) == 0x80 && (buffer[2] & 0x80) == 0x80 && (buffer[1] & 0x80) == 0x80 && (buffer[0] & 0x80) == 0x80)
	{
		DS3231_AlarmStruct->Mode = ONCE_PER_SEC_MATCH;
	}
	else if ( (buffer[3] & 0x80) == 0x80 && (buffer[2] & 0x80) == 0x80 && (buffer[1] & 0x80) == 0x80 && (buffer[0] & 0x80) == 0x00)
	{
		DS3231_AlarmStruct->Mode = SEC_MATCH;
	}
	else if ( (buffer[3] & 0x80) == 0x80 && (buffer[2] & 0x80) == 0x80 && (buffer[1] & 0x80) == 0x00 && (buffer[0] & 0x80) == 0x00)
	{
		DS3231_AlarmStruct->Mode = MIN_SEC_MATCH;
	}
	else if ( (buffer[3] & 0x80) == 0x80 && (buffer[2] & 0x80) == 0x00 && (buffer[1] & 0x80) == 0x00 && (buffer[0] & 0x80) == 0x00)
	{
		DS3231_AlarmStruct->Mode = HOU_MIN_SEC_MATCH;
	}
	else if ( (buffer[3] & 0xC0) == 0x00 && (buffer[2] & 0x80) == 0x00 && (buffer[1] & 0x80) == 0x00 && (buffer[0] & 0x80) == 0x00)
	{
		DS3231_AlarmStruct->Mode = DAT_HOU_MIN_SEC_MATCH;
	}
	else if ( (buffer[3] & 0xC0) == 0x40 && (buffer[2] & 0x80) == 0x00 && (buffer[1] & 0x80) == 0x00 && (buffer[0] & 0x80) == 0x00)
	{
		DS3231_AlarmStruct->Mode = DAY_HOU_MIN_SEC_MATCH;
	}	
	DS3231_AlarmStruct->Sec 		= BCDToDec(buffer[0] & 0x7F);
	DS3231_AlarmStruct->Min 		= BCDToDec(buffer[1] & 0x7F);
	DS3231_AlarmStruct->Hour 		= BCDToDec(buffer[2] & 0x3F);
	DS3231_AlarmStruct->Day_Date 	= BCDToDec(buffer[3] & 0x07);

	return status;
}

void DS3231_EnableAlarm(void)
{
	uint8_t buffer[2];
	
	/* Read Control and Status Register */
	HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x0E, I2C_MEMADD_SIZE_8BIT, buffer, 2, _DS3231_TIMEOUT);
	
	buffer[0] |= (1 << 2);	//Set bit 2: INTCN & Clear bit 0: A1IE
	buffer[1] &= ~(1 << 0);	//Clear bit 0: A1IF
	
	/* Write Control and Status Register */
	HAL_I2C_Mem_Write(_i2c_port, _DS3231_ADDRESS, 0x0E, I2C_MEMADD_SIZE_8BIT, buffer, 2, _DS3231_TIMEOUT);
}

void DS3231_DisableAlarm(void)
{
	uint8_t buffer[2];
	
	/* Read Control and Status Register */
	HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x0E, I2C_MEMADD_SIZE_8BIT, buffer, 2, _DS3231_TIMEOUT);

	buffer[0] &= ~(1 << 0);	//Clear bit 0: A1IE
	buffer[1] &= ~(1 << 0);	//Clear bit 0: A1IF
	
	/* Write Control and Status Register */
	HAL_I2C_Mem_Write(_i2c_port, _DS3231_ADDRESS, 0x0E, I2C_MEMADD_SIZE_8BIT, buffer, 2, _DS3231_TIMEOUT);
}

void DS3231_ClearAlarmFlag(void)
{
	uint8_t buffer;
	
	/* Read Status Register */
	HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x0F, I2C_MEMADD_SIZE_8BIT, &buffer, 1, _DS3231_TIMEOUT);

	buffer &= ~(1 << 0);	//Clear bit 0: A1IF
	
	/* Write Status Register */
	HAL_I2C_Mem_Write(_i2c_port, _DS3231_ADDRESS, 0x0F, I2C_MEMADD_SIZE_8BIT, &buffer, 1, _DS3231_TIMEOUT);
}

uint8_t DS3231_GetAlarmFlag(void)
{
	uint8_t buffer;
	
	/* Read Status Register */
	HAL_I2C_Mem_Read(_i2c_port, _DS3231_ADDRESS, 0x0F, I2C_MEMADD_SIZE_8BIT, &buffer, 1, _DS3231_TIMEOUT);

	if (buffer & 0x01)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
