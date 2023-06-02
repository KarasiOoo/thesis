/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);

int16_t xm_cal, ym_cal, zm_cal, xh_cal, yh_cal, zh_cal;
int64_t offset_xm, offset_ym, offset_zm, offset_xh, offset_yh, offset_zh;
uint16_t samples = 100; 

int __io_putchar(int ch)
{
  if(ch == '\n')
  {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
return 1;
}

/* Private user code ---------------------------------------------------------*/
void WriteRegMid(uint8_t reg_address,  uint8_t command)
{
  HAL_I2C_Mem_Write(&hi2c1, MEDIUM_SENSOR, reg_address, 1, &command, 1, HAL_MAX_DELAY);
  return;
}

void ReadRegMid(uint8_t reg_address, uint8_t* aquired_data, uint8_t lenght)
{
  HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, reg_address, 1, aquired_data, lenght, HAL_MAX_DELAY);
  return;
}

void WriteRegHigh(uint8_t reg_address,  uint8_t command)
{
  HAL_I2C_Mem_Write(&hi2c2, HIGH_SENSOR, reg_address, 1, &command, 1, HAL_MAX_DELAY);
  return;
}

void ReadRegHigh(uint8_t reg_address, uint8_t* aquired_data, uint8_t lenght)
{
  HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, reg_address, 1, aquired_data, lenght, HAL_MAX_DELAY);
  return;
}

int16_t BinaryToDecimal (int16_t binary)
{
  int16_t decimal, last_digit;
  uint8_t base = 1;

  while(binary)
  {
    last_digit = binary % 10;
    binary = binary / 10;
    decimal += last_digit * base;
    base = base * 2;
  }
  return decimal;
}

void ReadMagnetic()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor;
  
  uint8_t memory[12];
  int16_t measured_value_x, measured_value_y, measured_value_z, sensitivity;
  //int16_t measured_decimal_value_x, measured_decimal_value_y, measured_decimal_value_z;
  int32_t measured_value_xf, measured_value_yf, measured_value_zf;
  int16_t calibration_x, calibration_y, calibration_z;

  printf("Select which sensor you want to perform measurement: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY); 

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    calibration_x = xm_cal;
    calibration_y = ym_cal;
    calibration_z = zm_cal;
    sensitivity = 400;
    printf("Medium range sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    calibration_x = xh_cal;
    calibration_y = yh_cal;
    calibration_z = zh_cal;
    sensitivity = 140;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }
  
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, memory, 12, HAL_MAX_DELAY);

  measured_value_x = memory[2] << 8 | memory[3];
  measured_value_y = memory[4] << 8 | memory[5];
  measured_value_z = memory[6] << 8 | memory[7];

  measured_value_xf = measured_value_x * 1000 / sensitivity;
  measured_value_yf = measured_value_y * 1000 / sensitivity;
  measured_value_zf = measured_value_z * 1000 / sensitivity;

  //measured_value_xf = measured_value_xf - calibration_x;
  //measured_value_yf = measured_value_yf - calibration_y;
  //measured_value_zf = measured_value_zf - calibration_z;

  printf("X: %05ld,\t Y: %05ld,\t Z: %05ld (uT)\n", measured_value_xf, measured_value_yf, measured_value_zf);
  return;
}

void ReadTemperature()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor;

  uint8_t memory[2];
  int16_t t_val;

  printf("Select which sensor you want to perform measurement: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY);

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium range sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2CT2, 1, memory, 2, HAL_MAX_DELAY);

  t_val = memory[0] << 8 | memory[1];
  t_val = (t_val) / 50;

  printf("Temperature: %d\n", t_val);

  return;

}

void ReadMagneticTemperature()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address;
  
  uint8_t memory[12];
  uint8_t status, sensor;
  int16_t measured_value_x, measured_value_y, measured_value_z;//, t_val;
  float t_val;

  printf("Select which sensor you want to perform measurement: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY); 

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium range sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }
  
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, memory, 12, HAL_MAX_DELAY);

  status = memory[0];
  measured_value_x = memory[2] << 8 | memory[3];
  measured_value_y = memory[4] << 8 | memory[5];
  measured_value_z = memory[6] << 8 | memory[7];
  t_val = ((memory[8] << 8) | memory[9]);
  t_val = (t_val / 50) - 4;

  printf("Status: %02x,\t X:%05i,\t Y:%05i,\t Z:%05i,\t T:%3.2f,\n", status, measured_value_x, measured_value_y, measured_value_z, t_val);
  return;
}


void ReadVoltage(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, v_val;
  uint8_t command = SINGLE_MEASURE_VOLTAGE;
  uint8_t memory[12];

  if(sensor == 1)
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Voltage supplying Medium range sensor: \n");
  }
  else if(sensor == 2)
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("Voltage supplying High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, &command, 1, HAL_MAX_DELAY);
  HAL_Delay(1);
  //HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, measured_voltage, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, memory, 2, HAL_MAX_DELAY);
  v_val = memory[10] << 8 | memory[11];
  printf("%01i.%01i\n", v_val/10, v_val%10);
  return;
}


void ReadMagneticBurstAveraged(uint8_t samples)
{
  uint8_t entry_meas, exit_meas, state, measure, print_m_ready, print_h_ready, hal_error;
  uint8_t gain_m, gain_h, temp_mem_m[6], temp_mem_h[6], data_ready_m, data_ready_h, hal_error_m, hal_error_h;
  i2c_measurement_memory memory_m[samples], memory_h[samples];
  int16_t ext_xm, ext_ym, ext_zm, ext_xh, ext_yh, ext_zh;
  int32_t value_xm[samples], value_ym[samples], value_zm[samples], value_xh[samples], value_yh[samples], value_zh[samples];
  int64_t sum_value_xm, sum_value_ym, sum_value_zm, avg_value_xm, avg_value_ym, avg_value_zm;
  int64_t sum_value_xh, sum_value_yh, sum_value_zh, avg_value_xh, avg_value_yh, avg_value_zh;
  int32_t i_value_xm, f_value_xm, i_value_ym, f_value_ym, i_value_zm, f_value_zm;
  int32_t i_value_xh, f_value_xh, i_value_yh, f_value_yh, i_value_zh, f_value_zh;
  uint32_t p_value_xm, p_value_ym, p_value_zm, p_value_xh, p_value_yh, p_value_zh;

  entry_meas = 0;
  exit_meas = 0;
  state = 1;

  HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_CONF1, 1, &gain_m, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_CONF1, 1, &gain_h, 1, HAL_MAX_DELAY);

  gain_m = gain_m >> 4;
  gain_h = gain_h >> 4;

  WriteRegMid(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  WriteRegHigh(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  
  printf("   Mid:, \t High:\n");

  while(state)
  {
    sum_value_xm = 0, sum_value_ym = 0, sum_value_zm = 0, avg_value_xm = 0, avg_value_ym = 0, avg_value_zm = 0;
    sum_value_xh = 0, sum_value_yh = 0, sum_value_zh = 0, avg_value_xh = 0, avg_value_yh = 0, avg_value_zh = 0;

    for(measure = 0; measure < samples; measure++)
    {
      memset(&memory_m, 0, sizeof(memory_m));
      memset(&memory_h, 0, sizeof(memory_h));

      do
      {
        hal_error_m = HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_I2C_ComandStatus, 1, &data_ready_m, 1, HAL_MAX_DELAY);
        //printf("Med status: %x\n", hal_error);
        hal_error_h = HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_I2C_ComandStatus, 1, &data_ready_h, 1, HAL_MAX_DELAY);
        //printf("High status: %x\n", hal_error);

        data_ready_m = data_ready_m & 0x1;        
        data_ready_h = data_ready_h & 0x1;
        HAL_Delay(1);
      } while (data_ready_m != 1 || data_ready_h != 1);
      
      if(data_ready_m == 1)
      {
        hal_error_m = HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_I2CX2, 1, temp_mem_m, 6, HAL_MAX_DELAY);
        if(hal_error_m != 0x00)
        {
          printf("Error occur during performing measurement on Medium range sensor!\n");
          state = 0;
          break;
        }

        memory_m[measure].x1 = *(reg_8 *)&temp_mem_m[0];
        memory_m[measure].x0 = *(reg_8 *)&temp_mem_m[1];
        memory_m[measure].y1 = *(reg_8 *)&temp_mem_m[2];
        memory_m[measure].y0 = *(reg_8 *)&temp_mem_m[3];
        memory_m[measure].z1 = *(reg_8 *)&temp_mem_m[4];
        memory_m[measure].z0 = *(reg_8 *)&temp_mem_m[5];

        ext_xm = (int16_t)((int8_t)memory_m[measure].x1);
        ext_ym = (int16_t)((int8_t)memory_m[measure].y1);
        ext_zm = (int16_t)((int8_t)memory_m[measure].z1);

        value_xm[measure] = (int32_t)ext_xm << 8 | memory_m[measure].x0;
        value_ym[measure] = (int32_t)ext_ym << 8 | memory_m[measure].y0;
        value_zm[measure] = (int32_t)ext_zm << 8 | memory_m[measure].z0;
   
        value_xm[measure] = (((value_xm[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;
        value_ym[measure] = (((value_ym[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;
        value_zm[measure] = (((value_zm[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;

        sum_value_xm = sum_value_xm + value_xm[measure];
        sum_value_ym = sum_value_ym + value_ym[measure];
        sum_value_zm = sum_value_zm + value_zm[measure]; 
      }
      
      if(data_ready_h == 1)
      {
        hal_error_h = HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_I2CX2, 1, temp_mem_h, 6, HAL_MAX_DELAY);

        if(hal_error_h != 0x00)
        {
          printf("Error occur during performing measurement on High range sensor!\n");
          state = 0;
          break;
        }

        memory_h[measure].x1 = *(reg_8 *)&temp_mem_h[0];
        memory_h[measure].x0 = *(reg_8 *)&temp_mem_h[1];
        memory_h[measure].y1 = *(reg_8 *)&temp_mem_h[2];
        memory_h[measure].y0 = *(reg_8 *)&temp_mem_h[3];
        memory_h[measure].z1 = *(reg_8 *)&temp_mem_h[4];
        memory_h[measure].z0 = *(reg_8 *)&temp_mem_h[5];

        ext_xh = (int16_t)((int8_t)memory_h[measure].x1);
        ext_yh = (int16_t)((int8_t)memory_h[measure].y1);
        ext_zh = (int16_t)((int8_t)memory_h[measure].z1);

        value_xh[measure] = (int32_t)ext_xh << 8 | memory_h[measure].x0;
        value_yh[measure] = (int32_t)ext_yh << 8 | memory_h[measure].y0;
        value_zh[measure] = (int32_t)ext_zh << 8 | memory_h[measure].z0;

        value_xh[measure] = (((value_xh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;
        value_yh[measure] = (((value_yh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;
        value_zh[measure] = (((value_zh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;

        sum_value_xh = sum_value_xh + value_xh[measure];
        sum_value_yh = sum_value_yh + value_yh[measure];
        sum_value_zh = sum_value_zh + value_zh[measure];
      }  
    }

    HAL_UART_Receive(&huart2, &exit_meas, 1, 0); 
    if(entry_meas != exit_meas)
    {
      state = 0;
    }

    avg_value_xm = sum_value_xm / samples;
    avg_value_ym = sum_value_ym / samples;
    avg_value_zm = sum_value_zm / samples;

    // avg_value_xm = avg_value_xm - offset_xm;
    // avg_value_ym = avg_value_ym - offset_ym;
    // avg_value_zm = avg_value_zm - offset_zm;
    // avg_value_xh = avg_value_xh - offset_xh;
    // avg_value_xh = avg_value_xh - offset_xh;
    // avg_value_xh = avg_value_xh - offset_xh;

    i_value_xm = avg_value_xm / 1000;
    f_value_xm = avg_value_xm % 1000;
    i_value_ym = avg_value_ym / 1000;
    f_value_ym = avg_value_ym % 1000;
    i_value_zm = avg_value_zm / 1000;
    f_value_zm = avg_value_zm % 1000;


    avg_value_xh = sum_value_xh / samples;
    avg_value_yh = sum_value_yh / samples;
    avg_value_zh = sum_value_zh / samples;

    i_value_xh = avg_value_xh / 1000;
    f_value_xh = avg_value_xh % 1000;
    i_value_yh = avg_value_yh / 1000;
    f_value_yh = avg_value_yh % 1000;
    i_value_zh = avg_value_zh / 1000;
    f_value_zh = avg_value_zh % 1000;

    printf("X: ");
    if (f_value_xm < 0)
    {
      p_value_xm = ~f_value_xm + 1;
      if (i_value_xm >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_xm = f_value_xm;
    }
    printf("%02ld.%03ld,\t ", i_value_xm, p_value_xm);

    if (f_value_xh < 0)
    {
      p_value_xh = ~f_value_xh + 1;
      if (i_value_xh >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_xh = f_value_xh;
    }
    printf("%02ld.%03ld\n", i_value_xh, p_value_xh);

    printf("Y: ");
    if (f_value_ym < 0)
    {
      p_value_ym = ~f_value_ym + 1;
      if (i_value_ym >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_ym = f_value_ym;
    }
    printf("%02ld.%03ld,\t ", (int32_t)i_value_ym, p_value_ym);

    if (f_value_yh < 0)
    {
      p_value_yh = ~f_value_yh + 1;
      if (i_value_yh >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_yh = f_value_yh;
    }
    printf("%02ld.%03ld\n", (int32_t)i_value_yh, p_value_yh);

    printf("Z: ");
    if (f_value_zm < 0)
    {
      p_value_zm = ~f_value_zm + 1;
      if (i_value_zm >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_zm = f_value_zm;
    }
    printf("%02ld.%03ld,\t ", (int32_t)i_value_zm, p_value_zm);

    if (f_value_zh < 0)
    {
      p_value_zh = ~f_value_zh + 1;
      if (i_value_zh >= 0)
      {
        printf("-");
      }
    }
    else
    {
      p_value_zh = f_value_zh;
    }
    printf("%02ld.%03ld\n\n\n", (int32_t)i_value_zh, p_value_zh);
  }
  printf("Measurement finished.\n");
  WriteRegMid(REG_I2C_ComandStatus, EXIT_MODE);
  WriteRegHigh(REG_I2C_ComandStatus, EXIT_MODE);
  return;
}

void SetOffsetManually(void)
{
  volatile uint8_t gain_m, gain_h, data_ready_m, data_ready_h, hal_error_m, hal_error_h, measure, num_of_samples;
  uint8_t temp_mem_m[6], temp_mem_h[6];
  i2c_measurement_memory memory_m[samples], memory_h[samples];
  int16_t extended_xm, extended_ym, extended_zm, extended_xh, extended_yh, extended_zh;
  int32_t value_xm[samples], value_ym[samples], value_zm[samples], value_xh[samples], value_yh[samples], value_zh[samples];
  int64_t sum_value_xm, sum_value_ym, sum_value_zm, sum_value_xh, sum_value_yh, sum_value_zh;
  HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_CONF1, 1, &gain_m, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_CONF1, 1, &gain_h, 2, HAL_MAX_DELAY);

  gain_m = gain_m >> 4;
  gain_h = gain_h >> 4;

  WriteRegMid(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  WriteRegHigh(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);

  for(measure = 0; measure < num_of_samples; measure++)
  {
    do
    {
      hal_error_m = HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_I2C_ComandStatus, 1, &data_ready_m, 1, HAL_MAX_DELAY);
      hal_error_h = HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_I2C_ComandStatus, 1, &data_ready_h, 1, HAL_MAX_DELAY);

      data_ready_m = data_ready_m & 0x1;        
      data_ready_h = data_ready_h & 0x1;
      HAL_Delay(1);
    } while (data_ready_m != 1 || data_ready_h != 1);

    if(data_ready_m == 1)
    {
      hal_error_m = HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_I2CX2, 1, temp_mem_m, 6, HAL_MAX_DELAY);
      
      if(hal_error_m != 0x00)
      {
        printf("Error occur on Medium range sensor!\n");
        break;
      }

      memory_m[measure].x1 = *(reg_8 *)&temp_mem_m[0];
      memory_m[measure].x0 = *(reg_8 *)&temp_mem_m[1];
      memory_m[measure].y1 = *(reg_8 *)&temp_mem_m[2];
      memory_m[measure].y0 = *(reg_8 *)&temp_mem_m[3];
      memory_m[measure].z1 = *(reg_8 *)&temp_mem_m[4];
      memory_m[measure].z0 = *(reg_8 *)&temp_mem_m[5];

      extended_xm = (int16_t)((int8_t)memory_m[measure].x1);
      extended_ym = (int16_t)((int8_t)memory_m[measure].y1);
      extended_zm = (int16_t)((int8_t)memory_m[measure].z1);

      value_xm[measure] = (int32_t)extended_xm << 8 | memory_m[measure].x0;
      value_ym[measure] = (int32_t)extended_ym << 8 | memory_m[measure].y0;
      value_zm[measure] = (int32_t)extended_zm << 8 | memory_m[measure].z0;

      value_xm[measure] = (((value_xm[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;
      value_ym[measure] = (((value_ym[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;
      value_zm[measure] = (((value_zm[measure] * 1000) / 400) * gain_multiplier[gain_m]) / 1000;

      sum_value_xm = sum_value_xm + value_xm[measure];
      sum_value_ym = sum_value_ym + value_ym[measure];
      sum_value_zm = sum_value_zm + value_zm[measure]; 
    }

    if(data_ready_h == 1)
    {
      hal_error_h = HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_I2CX2, 1, temp_mem_h, 6, HAL_MAX_DELAY);
      
      if(hal_error_h != 0x00)
      {
        printf("Error occur on High range sensor!\n");
        break;
      }

      memory_h[measure].x1 = *(reg_8 *)&temp_mem_h[0];
      memory_h[measure].x0 = *(reg_8 *)&temp_mem_h[1];
      memory_h[measure].y1 = *(reg_8 *)&temp_mem_h[2];
      memory_h[measure].y0 = *(reg_8 *)&temp_mem_h[3];
      memory_h[measure].z1 = *(reg_8 *)&temp_mem_h[4];
      memory_h[measure].z0 = *(reg_8 *)&temp_mem_h[5];

      extended_xh = (int16_t)((int8_t)memory_h[measure].x1);
      extended_yh = (int16_t)((int8_t)memory_h[measure].y1);
      extended_zh = (int16_t)((int8_t)memory_h[measure].z1);

      value_xh[measure] = (int32_t)extended_xh << 8 | memory_h[measure].x0;
      value_yh[measure] = (int32_t)extended_yh << 8 | memory_h[measure].z0;
      value_zh[measure] = (int32_t)extended_zh << 8 | memory_h[measure].y0;

      value_xh[measure] = (((value_xh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;
      value_yh[measure] = (((value_yh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;
      value_zh[measure] = (((value_zh[measure] * 1000) / 140) * gain_multiplier[gain_m]) / 1000;

      sum_value_xh = sum_value_xh + value_xh[measure];
      sum_value_yh = sum_value_yh + value_yh[measure];
      sum_value_zh = sum_value_zh + value_zh[measure];
    }  
  }
  offset_xm = sum_value_xm / samples;
  offset_ym = sum_value_ym / samples;
  offset_zm = sum_value_zm / samples;

  offset_xh = sum_value_xh / samples;
  offset_yh = sum_value_yh / samples;
  offset_zh = sum_value_zh / samples;

  printf("Offset calibration finished.\n");
  WriteRegMid(REG_I2C_ComandStatus, EXIT_MODE);
  WriteRegHigh(REG_I2C_ComandStatus, EXIT_MODE);
}

uint8_t SetSamples(void)
{
  uint8_t get_samples[3];
  printf("Give hundert part to rounding:\n");
  HAL_UART_Receive(&huart2, &get_samples[2], 1, HAL_MAX_DELAY);
  printf("Give ten part to rounding:\n");
  HAL_UART_Receive(&huart2, &get_samples[1], 1, HAL_MAX_DELAY);
  printf("Give unit part to rounding:\n");
  HAL_UART_Receive(&huart2, &get_samples[0], 1, HAL_MAX_DELAY);

  get_samples[2] = get_samples[2] - 0x30;
  get_samples[1] = get_samples[1] - 0x30;
  get_samples[0] = get_samples[0] - 0x30;

  samples = (get_samples[2] * 100) + (get_samples[1] * 10) + get_samples[0];
  printf("Selected amount of samples: %i.\n", samples);
  return samples;
}

void ReadStatus(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address;

  uint8_t status;

  if(sensor == 1)
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium range sensor: \n");
  }
  else if (sensor == 2)
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, &status, 1, HAL_MAX_DELAY);

  printf("Status: %02x\n", status);
  return;
}

void ReadConfig()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor;

  uint8_t reg_zero[2], reg_one[2], reg_two[2], reg_six[2], reg_seven[2], reg_eight[2], reg_c[2], reg_d[2];
  uint16_t reg_zero_complex, reg_one_complex, reg_two_complex, offset_x, offset_y, offset_z, sensitivity_xy, sensitivity_z;
  uint8_t gain, burst_data_rate, burst_sel, digital_filter, resolution_x, resolution_y, resolution_z;

  printf("Select which sensor you want to perform measurement: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY); 

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium range sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF1, 1, reg_zero, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF2, 1, reg_one, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, reg_two, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetX, 1, reg_six, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetY, 1, reg_seven, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetZ, 1, reg_eight, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_SensXY, 2, reg_c, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_SensZ, 2, reg_d, 2, HAL_MAX_DELAY);

  reg_zero_complex = (reg_zero[0] << 8) | reg_zero[1];
  gain = (reg_zero_complex >> 4) & 0x0f;

  reg_one_complex = (reg_one[0] << 8) | reg_one[1];
  burst_data_rate = reg_one_complex & 0x1f;
  burst_sel = (reg_one_complex >> 6) & 0x0f;

  reg_two_complex = (reg_two[0] << 8) | reg_two[1];
  digital_filter = (reg_two_complex >> 2) & 0x07;
  resolution_x = (reg_two_complex >> 5) & 0x03;
  resolution_y = (reg_two_complex >> 7) & 0x03;
  resolution_z = (reg_two_complex >> 9) & 0x03;

  offset_x = (reg_six[0] << 8) | reg_six[1];
  offset_y = (reg_seven[0] << 8) | reg_seven[1];
  offset_z = (reg_eight[0] << 8) | reg_eight[1];

  sensitivity_xy = ((reg_c[0] << 8) | reg_c[1]) & 0x01ff;
  sensitivity_z = ((reg_d[0] << 8) | reg_d[1]) & 0x01ff;

  printf("Gain:\t 0x%02x\n", gain);
  printf("Burst data rate: 0x%04x\n", burst_data_rate);
  printf("Digital Filter: 0x%02x\n", digital_filter);
  printf("Resolution X: 0x%02x\n", resolution_x);
  printf("Resolution Y: 0x%02x\n", resolution_y);
  printf("Resolution Z: 0x%02x\n", resolution_z);
  printf("Offset X:\t 0x%04x\n", offset_x);
  printf("Offset Y:\t 0x%04x\n", offset_y);
  printf("Offset Z:\t 0x%04x\n", offset_z);
  printf("Sensitivity XY: 0x%04x\n", sensitivity_xy);
  printf("Sensitivity Z: 0x%04x\n", sensitivity_z);

  return;
}

void SetGain()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor, reg[2], gain_sel_all_h, gain_sel_all_l;
  uint16_t gain_sel_all, reg16;
  uint8_t gain_sel[4];
  uint8_t hal_error;

  printf("Select which sensor you want to configure: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY); 

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Set the highest bit for bit selection: (0 or 1)\n");
  printf("'0' stands for 'x1' multiplier, '1' gives 'x0.5' for Gain A1\n");
  HAL_UART_Receive(&huart2, &gain_sel[3], 1, HAL_MAX_DELAY);
  printf("Gain A2 is set by 3 following bits: \n\t'0' = x0.2\t '1' = x0.25\t '2' = x0.33\t '3' = x0.4\n\t'4' = x0.5\t '5' = x0.6\t '6' = x0.75\t '7' = x1\n");
  printf("Set second highest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[2], 1, HAL_MAX_DELAY);
  printf("Set third highest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[1], 1, HAL_MAX_DELAY);
  printf("Set the lowest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[0], 1, HAL_MAX_DELAY);

  gain_sel[3] = gain_sel[3] - 0x30;
  gain_sel[2] = gain_sel[2] - 0x30;
  gain_sel[1] = gain_sel[1] - 0x30;
  gain_sel[0] = gain_sel[0] - 0x30;

  gain_sel_all = 0;
  gain_sel_all = (((gain_sel[3] << 3) | gain_sel[2] << 2) | gain_sel[1] << 1) | gain_sel[0];
  printf("Value which will be injected: 0x%02x\n", gain_sel_all);

  hal_error = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF1, 1, reg, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  reg16 = reg[0] << 8 | reg[1];
  printf("Value read from device: 0x%04x\n", reg16);

  reg16 = reg16 & 0xff0f;
  printf("Register prepared to inject new value: 0x%04x\n", reg16);

  gain_sel_all = reg16 | (gain_sel_all << 4);
  printf("Value which will be sent to the reg: 0x%04x \n", gain_sel_all);

  gain_sel_all_h = gain_sel_all >> 8;
  printf("MSB part: 0x%02x: \n", gain_sel_all_h);

  gain_sel_all_l = gain_sel_all & 0x00ff;
  printf("LSB part: 0x%02x: \n", gain_sel_all_l);

  gain_sel_all = (gain_sel_all_l << 8) | gain_sel_all_h;
  printf("Corrected order for HAL method: 0x%04x: \n", gain_sel_all);
  hal_error = HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF1, 1, &gain_sel_all, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  printf("Set done.\n");
  return;
}

void SetBurstDataRate()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor;

  uint8_t burst_data_rate[6], reg[2];
  uint8_t burst_data_rate_all, hal_error;
  uint16_t reg16;

  printf("Select which sensor you want to configure: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY); 

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Set fifth (the highest bit) for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[5], 1, HAL_MAX_DELAY);
  printf("Set forth for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[4], 1, HAL_MAX_DELAY);
  printf("Set third for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[3], 1, HAL_MAX_DELAY);
  printf("Set second for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[2], 1, HAL_MAX_DELAY);
  printf("Set first for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[1], 1, HAL_MAX_DELAY);
  printf("Set zeroth (the lowest) for BurstDataRate: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &burst_data_rate[0], 1, HAL_MAX_DELAY);

  burst_data_rate[5] = burst_data_rate[5] - 0x30;
  burst_data_rate[4] = burst_data_rate[4] - 0x30;
  burst_data_rate[3] = burst_data_rate[3] - 0x30;
  burst_data_rate[2] = burst_data_rate[2] - 0x30;
  burst_data_rate[1] = burst_data_rate[1] - 0x30;
  burst_data_rate[0] = burst_data_rate[0] - 0x30;

  burst_data_rate_all = 0;
  burst_data_rate_all = (((((burst_data_rate[5] << 5) | burst_data_rate[4] << 4) | burst_data_rate[3] << 3) | burst_data_rate[2] << 2) 
                        | burst_data_rate[1] << 1) | burst_data_rate[0];
  hal_error = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF2, 1, reg, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  reg16 = reg[1] << 8 | reg[0];
  reg16 = reg16 & 0xFFC0;
  reg16 = reg16 | burst_data_rate_all;
  printf("Value which will be sent to the reg: %02x \n", reg16);
  hal_error = HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF2, 1, &reg16, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  printf("Set done.\n");
  return;
}

void SetDigFilter()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor, hal_error;

  uint8_t dig_filter[3];
  uint8_t reg[2];
  uint16_t dig_filter_all, reg16;

  printf("Select which sensor you want to configure: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY);

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Set the highest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &dig_filter[2], 1, HAL_MAX_DELAY);
  printf("Set middle bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &dig_filter[1], 1, HAL_MAX_DELAY);
  printf("Set the lowest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &dig_filter[0], 1, HAL_MAX_DELAY);

  dig_filter[2] = dig_filter[2] - 0x30;
  dig_filter[1] = dig_filter[1] - 0x30;
  dig_filter[0] = dig_filter[0] - 0x30;

  dig_filter_all = 0;
  dig_filter_all = ((dig_filter[2] << 2) | dig_filter[1] << 1) | dig_filter[0];
  hal_error = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, reg, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  reg16 = (reg[1] << 8) | reg[0];
  reg16 = reg16 & 0b1111111111100011;
  dig_filter_all = reg16 | (dig_filter_all << 2);
  printf("Value which will be sent to the reg: %02x \n", dig_filter_all);
  hal_error = HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF3, 1, &dig_filter_all, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  printf("Set done.\n");
  return;
}

void SetResolution()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor, resolution;
  uint8_t reg_r[2];
  uint8_t x_res[2], y_res[2], z_res[2];
  uint16_t reg_w;

  printf("Select which sensor you want to configure: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY);

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Set higher X bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &x_res[1], 1, HAL_MAX_DELAY);
  printf("Set lower X bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &x_res[0], 1, HAL_MAX_DELAY);
  printf("Set higher Y bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &y_res[1], 1, HAL_MAX_DELAY);
  printf("Set lower Y bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &y_res[0], 1, HAL_MAX_DELAY);
  printf("Set higher Z bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &z_res[1], 1, HAL_MAX_DELAY);
  printf("Set lower Z bit: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &z_res[0], 1, HAL_MAX_DELAY);

  x_res[1] = x_res[1] - 0x30;
  x_res[0] = x_res[0] - 0x30;
  y_res[1] = y_res[1] - 0x30;
  y_res[0] = y_res[0] - 0x30;
  z_res[1] = z_res[1] - 0x30;
  z_res[0] = z_res[0] - 0x30;
  resolution = ((((z_res[1] << 5 | z_res[0] << 4) | y_res[1] << 3) | y_res[0] << 2) | x_res[1] << 1) | x_res[0];

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, reg_r, 2, HAL_MAX_DELAY);
  reg_w = (reg_r[1] << 8) | reg_r[0];
  reg_w = reg_w & (0b11000000 << 5);
  reg_w = reg_w | (resolution << 5);
  printf("Value which will be sent to the reg: %04x \n", reg_w);
  HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF3, 1, &reg_w, 2, HAL_MAX_DELAY);
  printf("Set done.\n");
  return;
}


void SetSensitivity()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, sensor, dimention;

  uint8_t sensitivity_xy[10], sensitivity_z[10], reg[2];
  uint16_t sensitivity_xy_all, sensitivity_z_all;
  uint8_t i = 0; 

  printf("Select which sensor you want to configure: 1 - mid, 2 - high.\n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY);

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Which sensitivity you want to set: 1 - XY, 2 - Z\n");
  HAL_UART_Receive(&huart2, &dimention, 1, HAL_MAX_DELAY);
  switch(dimention)
  {
    case '1':
      for (i = 0; i <= 9; i++)
      {
        printf("Write %i bit of sensitivity XY:\n", i);
        HAL_UART_Receive(&huart2, &sensitivity_xy[i], 1, HAL_MAX_DELAY);
        sensitivity_xy[i] = sensitivity_xy[i] - 0x30;
      }
      sensitivity_xy_all = (((((((((sensitivity_xy[9] << 9) | sensitivity_xy[8] << 8) 
                          | sensitivity_xy[7] << 7) | sensitivity_xy[6] << 6) | sensitivity_xy[5] << 5) 
                          | sensitivity_xy[4] << 4) | sensitivity_xy[3] << 3) | sensitivity_xy[2] << 2) 
                          | sensitivity_xy[1] << 1) | sensitivity_xy[0];
      HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_SensXY, 1, reg, 2, HAL_MAX_DELAY);

      HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF_SensXY, 1, &sensitivity_xy_all, 2, HAL_MAX_DELAY);
      break;
    case '2':
      for (i = 0; i <= 9; i++)
      {
        printf("Write %i bit of sensitivity Z:\n", i);
        HAL_UART_Receive(&huart2, &sensitivity_z[i], 1, HAL_MAX_DELAY);
        sensitivity_z[i] = sensitivity_z[i] - 0x30;
      }
      sensitivity_z_all = ((((((((((sensitivity_z[9] << 9) | sensitivity_z[8] << 8) 
                          | sensitivity_z[7] << 7) | sensitivity_z[6] << 6) | sensitivity_z[5] << 5) 
                          | sensitivity_z[4] << 4) | sensitivity_z[3] << 3) | sensitivity_z[2] << 2) 
                          | sensitivity_z[1] << 1) | sensitivity_z[0]);
      HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF_SensZ, 1, &sensitivity_z_all, 2, HAL_MAX_DELAY);
      break;
    default:
      printf("Unknown command.\n");
      break;
  }
  return;
}

void ReadReg(void)
{
  uint8_t sensor, reg_conf, reg_address, dev_address;
  uint8_t value[2];
  //uint16_t value;
  uint16_t value_displayed;
  I2C_HandleTypeDef i2c_address;

  printf("Choose sensor: \n");
  HAL_UART_Receive(&huart2, &sensor, 1, HAL_MAX_DELAY);

  if(sensor == '1')
  {
    i2c_address = hi2c1;
    dev_address = MEDIUM_SENSOR;
    printf("Medium range sensor: \n");
  }
  else if (sensor == '2')
  {
    i2c_address = hi2c2;
    dev_address = HIGH_SENSOR;
    printf("High range sensor: \n");
  }
  else
  {
    printf("Wrong number given!\n");
    return;
  }

  printf("Choose register: \n");
  printf("z - REG_CONF1\n");
  printf("x - REG_CONF2\n");
  printf("c - REG_CONF3\n");
  printf("v - REG_CONF_OffsetX\n");
  printf("b - REG_CONF_OffsetY\n");
  printf("n - REG_CONF_OffsetZ\n");
  HAL_UART_Receive(&huart2, &reg_conf, 1, HAL_MAX_DELAY); 

  switch(reg_conf)
  {
    case 'z':
      reg_address = REG_CONF1;
      break;
    case 'x':
      reg_address = REG_CONF2;
      break;
    case 'c':
      reg_address = REG_CONF3;
      break;
    case 'v':
      reg_address = REG_CONF_OffsetX;
      break;
    case 'b':
      reg_address = REG_CONF_OffsetY;
      break;
    case 'n':
      reg_address = REG_CONF_OffsetZ;
      break;
  }

  HAL_I2C_Mem_Read(&i2c_address, dev_address, reg_address, 1, &value, 2, HAL_MAX_DELAY);
  //HAL_I2C_Mem_Read(&i2c_address, dev_address, (reg_address + 0b1), 1, &value[1], 1, HAL_MAX_DELAY);
  value_displayed = (value[0] << 8) | value[1];
  printf("Register value: 0x%04x\n", value_displayed);
  return;
}

int main(void)
{
  uint8_t v_en = 1;                                           

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  HAL_Delay(300);

  WriteRegMid(REG_I2C_ComandStatus, RESET_SENSOR);
  WriteRegHigh(REG_I2C_ComandStatus, RESET_SENSOR);
  HAL_Delay(300);


  while (1)
  {
    uint8_t action, new_task;
    printf("Choose action:\n");
    printf("Measure:\n");
    printf("\t m - Once magnetic field.\n");
    printf("\t t - Once temperature.\n");
    printf("\t v - Once voltage.\n");
    printf("\t b - Continues magnetic field and temperature.\n");
    printf("\t n - Set amount of samples to take average from it in Continues measurement.\n");
    printf("\t z - Reset.\n");
    printf("\t q - Check RAW config register value.\n");
    printf("Calibration/settings:\n");
    printf("\t c - Show status reg.\n");
    printf("\t o - Calibrate offset for both sensors.\n");
    printf("\t a - Show all config parameters.\n");
    printf("\t s - Set sensitivity.\n");
    printf("\t g - Set gain.\n");
    printf("\t f - Set Digital Filter, average from x measurement,\n");
    printf("\t r - Set resolution for magnetic measurements.\n");
    printf("\t u - Set frequency of measurement for Burst Mode (BurstDataRate)(NY).\n");
    printf("What do you want to do?\t\n");
    printf("\n");

    HAL_UART_Receive(&huart2, &action, 1, HAL_MAX_DELAY);

    switch(action)
    {
      case 'm':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_MAGNETIC);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_MAGNETIC);
        printf("Measured values of magnetic field:\n");
        ReadMagnetic();
        break;
      case 't':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_TEMPERATURE);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_TEMPERATURE);
        printf("Measured value of temperature:\n");
        ReadTemperature();
        break;
      case 'v':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_VOLTAGE);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_VOLTAGE);
        HAL_Delay(1);
        printf("Measured value of voltage:\n");
        ReadVoltage(1);
        ReadVoltage(2);
        break;
      case 'b':
        printf("Continues measurement:\n");
        ReadMagneticBurstAveraged(samples);
        break;
      case 'n':
        printf("Set amount of samples.\n");
        samples = SetSamples();
        break;
      case 'z':
        printf("Device reset in progress...\n");
        WriteRegMid(REG_I2C_ComandStatus, RESET_SENSOR);
        WriteRegHigh(REG_I2C_ComandStatus, RESET_SENSOR);
        HAL_Delay(300);
        printf("Device reset done\n");
        break;
      case 'q':
        ReadReg();
        break;
      case 'c':
        ReadStatus(1);
        ReadStatus(2);
        break;
      case 'o':
        printf("Offset calibration.\n");
        SetOffsetManually();
        break;
      case 'a':
        printf("All configuration values:\n");
        ReadConfig();
        break;
      case 's':
        printf("Sensitivity settings.\n");
        SetSensitivity();
        break;
      case 'g':
        printf("Gain settings.\n");
        SetGain();
        break;
      case 'f':
        printf("DigFilter settings.\n");
        SetDigFilter();
        break;
      case 'r':
        printf("Resolution settings.\n");
        SetResolution();
        break;
      case 'u':
        printf("Burst mode settings.\n");
        SetBurstDataRate();
        break;
      case 'w':
        if(v_en == 0)
          {
            v_en = 1;
            printf("Voltage measurement enabled.\n");
          }
        else
          {
            v_en = 0;
            printf("Voltage measurement disabled.\n");
          }
          break;
      default:
        printf("Unknown command.\n");
        break;
    }
    printf("Press any button to continue.\n");
    HAL_UART_Receive(&huart2, &new_task, 1, HAL_MAX_DELAY);
    printf("\n\n\n");
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
