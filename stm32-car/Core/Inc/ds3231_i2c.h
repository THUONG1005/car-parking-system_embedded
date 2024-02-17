
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DS3231_I2C_H
#define __DS3231_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"


/* Alarm 1 modes */
typedef enum
{
	DAY_HOU_MIN_SEC_MATCH = 0,
	DAT_HOU_MIN_SEC_MATCH,
	HOU_MIN_SEC_MATCH,
	MIN_SEC_MATCH,
	SEC_MATCH,
	ONCE_PER_SEC_MATCH
} DS3231_AlarmModeTypeDef;

typedef struct
{
	uint8_t Year;	/* Year (0 - 99) */
	uint8_t Month;	/* Month (1 - 12) */
	uint8_t Date;	/* Date (1 - 31) */
	uint8_t DoW;	/* Days of week (1 - 7) */
	uint8_t Hour;	/* Hour (0 - 23) */
	uint8_t Min;	/* Minute (0 - 59) */
	uint8_t Sec;	/* Second (0 - 59) */
} DS3231_TimeTypeDef;

typedef struct
{
	DS3231_AlarmModeTypeDef 	Mode;		/* DS3231_AlarmModeTypeDef */
	uint8_t 					Day_Date;	/* Day (1 - 7) or Date (1 - 31) */
	uint8_t 					Hour;		/* Hour (0 - 23) */
	uint8_t 					Min;		/* Minute (0 - 59) */
	uint8_t 					Sec;		/* Second (0 - 59) */
} DS3231_AlarmTypeDef;

/*============== Functions prototypes ==============*/
void DS3231_Init(I2C_HandleTypeDef*	hi2c);
void DS3231_DeInit(void);

HAL_StatusTypeDef DS3231_GetTime(DS3231_TimeTypeDef* DS3231_TimeStruct);
HAL_StatusTypeDef DS3231_SetTime(DS3231_TimeTypeDef* DS3231_TimeStruct);

HAL_StatusTypeDef DS3231_SetAlarm(DS3231_AlarmTypeDef* DS3231_AlarmStruct);
HAL_StatusTypeDef DS3231_GetAlarm(DS3231_AlarmTypeDef* DS3231_AlarmStruct);
void DS3231_EnableAlarm(void);
void DS3231_DisableAlarm(void);
uint8_t DS3231_GetAlarmFlag(void);
void DS3231_ClearAlarmFlag(void);
/*==================================================*/

#ifdef __cplusplus
}
#endif

#endif /* __DS3231_I2C_H */
