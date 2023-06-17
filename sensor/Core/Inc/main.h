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
#define REG_CONF1 0x00
#define REG_CONF2 0x02
#define REG_CONF3 0x04
#define REG_CONF4 0x06
#define REG_CONF_SensTCH 0x08
#define REG_CONF_SensTCL 0x0A
#define REG_CONF_OffsetX 0x0C
#define REG_CONF_OffsetY 0x0E
#define REG_CONF_OffsetZ 0x10
#define REG_CONF_OffsetDriftX 0x12
#define REG_CONF_OffsetDriftY 0x14
#define REG_CONF_OffsetDriftZ 0x16
#define REG_CONF_SensXY 0x18
#define REG_CONF_SensZ 0x1A
#define REG_CONF_WocXYThreshold 0x1C
#define REG_CONF_WocZThreshold 0x1E
#define REG_CONF_WocTTThreshold 0x20
#define REG_CONF_WocVThreshold 0x22

#define REG_I2C_ComandStatus 0x80
#define REG_I2C_CRC8CCITT 0x81
#define REG_I2CX2 0x82
#define REG_I2CX1 0x83
#define REG_I2CY2 0x84
#define REG_I2CY1 0x85
#define REG_I2CZ2 0x86
#define REG_I2CZ1 0x87
#define REG_I2CT2 0x88
#define REG_I2CT1 0x89
#define REG_I2CV2 0x8A
#define REG_I2CV1 0x8B

#define REG_I2C_ID1 0x26
#define REG_I2C_ID2 0x27
#define REG_I2C_ID3 0x28

#define MEDIUM_SENSOR 0b00011010
#define HIGH_SENSOR 0b00011000

#define SINGLE_MEASURE_MAGNETIC 0b00111110
#define SINGLE_MEASURE_TEMPERATURE 0b00110001
#define SINGLE_MEASURE_MT 0b00110000
#define SINGLE_MEASURE_VOLTAGE 0b11000000

#define BURST_MEASURE_MAGNETIC 0b00011110
#define BURST_MEASURE_TEMPERATURE 0b00010001
#define BURST_MEASURE_MT 0b00010000

#define EXIT_MODE 0b10000000
#define RESET_SENSOR 0b11110000

#define TWO_BITS 0b00000011

#define medium_sensor_constant 400
#define high_sensor_constant 140

//#define MLX_I2C_M ((i2c_memory *) 0x80)
//#define MLX_I2C_H ((i2c_memory *) 0x80)

/* Exported types ------------------------------------------------------------*/
typedef volatile uint8_t reg_8;

typedef struct _memory_i2c
{
  reg_8 x1;
  reg_8 x0;
  reg_8 y1;
  reg_8 y0;
  reg_8 z1;
  reg_8 z0;
} i2c_measurement_memory;



/* Exported constants --------------------------------------------------------*/
static uint16_t gain_multiplier[16] = {200, 250, 330, 400, 500, 600, 750, 1000,
					 100, 125, 167, 200, 250, 300, 375, 500};

static uint16_t resolution_correction[4] = {1, 2, 4, 8};
/* Exported functions  -------------------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
