/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Private defines for STM  -----------------------------------------------------------*/


/* Private defines for MLX90395  -----------------------------------------------------------*/
#define REG_GONF1 0x00
#define REG_CONF2 0x01
#define REG_CONF3 0x02
#define REG_CONF4 0x03
#define REG_CONF_SensTCH 0x04
#define REG_CONF_SensTCL 0x05
#define REG_CONF_OffsetX 0x06
#define REG_CONF_OffsetY 0x07
#define REG_CONF_OffsetZ 0x08
#define REG_CONF_OffsetDriftX 0x09
#define REG_CONF_OffsetDriftY 0x0A
#define REG_CONF_OffsetDriftZ 0x0B
#define REG_CONF_SensXY 0x0C
#define REG_CONF_SensZ 0x0D
#define REG_CONF_WocXYThreshold 0x0E
#define REG_CONF_WocZThreshold 0x0F
#define REG_CONF_WocTTThreshold 0x10
#define REG_CONF_WocVThreshold 0x11

#define REG_I2C_ComandStatus 0x80
#define REG_I2C_CRC8CCITT 0x81
#define REG_I2CX1 0x83
#define REG_I2CX2 0x82
#define REG_I2CY1 0x85
#define REG_I2CY2 0x84
#define REG_I2CZ1 0x87
#define REG_I2CZ2 0x86
#define REG_I2CT1 0x89
#define REG_I2CT2 0x88
#define REG_I2CV1 0x8B
#define REG_I2CV2 0x8A
#define REG_I2C_ID1 0x26
#define REG_I2C_ID2 0x27
#define REG_I2C_ID3 0x28


#define SENSOR_ADDRESS_WR 0b00011010     //usunięty ostatni bit
#define SENSOR_ADDRESS_RD 0b00011010

#define SINGLE_MEASURE_MAGNETIC 0b00111110
#define BURST_MEASURE_MAGNETIC 0b00011111

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
