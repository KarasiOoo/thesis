/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"

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
  int16_t measured_value_x, measured_value_y, measured_value_z;
  int16_t measured_decimal_value_x, measured_decimal_value_y, measured_decimal_value_z;
  int32_t measured_value_xf, measured_value_yf, measured_value_zf;
  int16_t calibration_x, calibration_y, calibration_z;

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

  measured_value_x = memory[2] << 8 | memory[3];
  measured_value_y = memory[4] << 8 | memory[5];
  measured_value_z = memory[6] << 8 | memory[7];

  measured_decimal_value_x = BinaryToDecimal(measured_value_x);
  measured_decimal_value_y = BinaryToDecimal(measured_value_y);
  measured_decimal_value_z = BinaryToDecimal(measured_value_z);

  measured_value_xf = measured_value_x * 1000 / 400;
  measured_value_yf = measured_value_y * 1000 / 400;
  measured_value_zf = measured_value_z * 1000 / 400;

  measured_value_xf = measured_value_xf - calibration_x;
  measured_value_yf = measured_value_yf - calibration_y;
  measured_value_zf = measured_value_zf - calibration_z;

  printf("X: %05i,\t Y: %05i,\t Z: %05i (raw-binary)\n", measured_value_x, measured_value_y, measured_value_z);
  printf("X: %05i,\t Y: %05i,\t Z: %05i (decimal)\n", measured_decimal_value_x, measured_decimal_value_y, measured_decimal_value_z);
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
  t_val = t_val / 50;

  printf("Status: %02x,\t X:%05i,\t Y:%05i,\t Z:%05i,\t T:%3.2f,\n", status, measured_value_x, measured_value_y, measured_value_z, t_val);
  return;
}


void ReadVoltage(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, v_val;
  uint8_t command = SINGLE_MEASURE_VOLTAGE;
  //int16_t* measured_voltage;
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
  printf("%02i\n", v_val);
  return;
}

void ReadMagneticBurst()
{
  uint8_t burst, burst_break, state;
  uint8_t memory_m[12], memory_h[12];
  int16_t measured_value_xm, measured_value_ym, measured_value_zm, measured_value_xh, measured_value_yh, measured_value_zh;
  int32_t measured_converted_value_xm, measured_converted_value_ym, measured_converted_value_zm;
  int32_t measured_converted_value_xh, measured_converted_value_yh, measured_converted_value_zh;

  burst = 0;
  burst_break = 0;
  state = 1;

  WriteRegMid(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  WriteRegHigh(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  HAL_UART_Receive(&huart2, &burst, 1, 200); 

  printf("   Mid:, \t High:\n");

  while(state == 1)
  {
    HAL_I2C_Mem_Read(&hi2c1, MEDIUM_SENSOR, REG_I2C_ComandStatus, 1, memory_m, 12, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c2, HIGH_SENSOR, REG_I2C_ComandStatus, 1, memory_h, 12, HAL_MAX_DELAY);

    measured_value_xm = memory_m[2] << 8 | memory_m[3];
    measured_value_ym = memory_m[4] << 8 | memory_m[5];
    measured_value_zm = memory_m[6] << 8 | memory_m[7];
    measured_value_xh = memory_h[2] << 8 | memory_h[3];
    measured_value_yh = memory_h[4] << 8 | memory_h[5];
    measured_value_zh = memory_h[6] << 8 | memory_h[7];

    measured_converted_value_xm = measured_value_xm * 1000 / 400;
    measured_converted_value_ym = measured_value_ym * 1000 / 400;
    measured_converted_value_zm = measured_value_zm * 1000 / 400;
    measured_converted_value_xh = measured_value_xh * 1000 / 400;
    measured_converted_value_yh = measured_value_yh * 1000 / 400;
    measured_converted_value_zh = measured_value_zh * 1000 / 400;


    printf("X: %06ld,\t %06ld\n", measured_converted_value_xm, measured_converted_value_xh);
    printf("Y: %06ld,\t %06ld\n", measured_converted_value_ym, measured_converted_value_yh);
    printf("Z: %06ld,\t %06ld\n\n", measured_converted_value_zm, measured_converted_value_zh);

    HAL_UART_Receive(&huart2, &burst_break, 1, 150); 
    if(burst != burst_break)
    {
      state = 0;
    }
  }
  printf("'%i,\t%i'", burst, burst_break);
  printf("Measurement finished.\n");
  WriteRegMid(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  WriteRegHigh(REG_I2C_ComandStatus, BURST_MEASURE_MAGNETIC);
  return;
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

  uint8_t read_gain_sel, burst_data_rate, digital_filter_dislpayed, resolution_x, resolution_y, resolution_z;
  uint8_t digital_filter[2], resolution[2], offset_x[2], offset_y[2], offset_z[2], sensitivity_xy[2], sensitivity_z[2];
  uint16_t offset_x16, offset_y16, offset_z16, sensitivity_xy16, sensitivity_z16;
  uint8_t hal_error_sens_xy, hal_error_sens_z;

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

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF1, 1, &read_gain_sel, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF2, 1, &burst_data_rate, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, &digital_filter, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, resolution, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetX, 1, offset_x, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetY, 1, offset_y, 2, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_OffsetZ, 1, offset_z, 2, HAL_MAX_DELAY);
  hal_error_sens_xy = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_SensXY, 1, sensitivity_xy, 2, HAL_MAX_DELAY);
  printf("Status sensXY: %x\n", hal_error_sens_xy);
  hal_error_sens_xy = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF_SensZ, 1, sensitivity_z, 2, HAL_MAX_DELAY);
  printf("Status sensZ: %x\n", hal_error_sens_z);

  read_gain_sel = read_gain_sel >> 4;
  burst_data_rate = burst_data_rate & 0b00111111;
  digital_filter_dislpayed = (digital_filter[0] & 0b00011100) >> 2;
  resolution_x = (resolution[0] >> 5) & TWO_BITS;
  resolution_y = ((resolution[1] << 1) | (resolution[0] >> 7)) & TWO_BITS;
  //resolution_y = ((resolution[1] << 8 | resolution[0]) >> 7) & TWO_BITS;
  resolution_z = (resolution[1] >> 1) & TWO_BITS;
  offset_x16 = (offset_x[1] << 8) | offset_x[0];
  //offset_x16 = (offset_x[0] << 8) | offset_x[1];
  offset_y16 = (offset_y[1] << 8) | offset_y[0];
  offset_z16 = (offset_z[1] << 8) | offset_z[0];
  sensitivity_xy16 = (sensitivity_xy[1] << 8) | sensitivity_xy[0];
  sensitivity_z16  = (sensitivity_z[1]  << 8 ) | sensitivity_z[0];
  //sensitivity_z16 = (sensitivity_z[0] << 8 ) | sensitivity_z[1];

  printf("Gain:\t %02x\n", read_gain_sel);
  printf("Burst data rate: %02x\n", burst_data_rate);
  printf("Digital Filter: %02x\n", digital_filter_dislpayed);
  printf("Resolution X: %02x\n", resolution_x);
  printf("Resolution Y: %02x\n", resolution_y);
  printf("Resolution Z: %02x\n", resolution_z);
  printf("Offset X:\t %04x\n", offset_x16);
  printf("Offset Y:\t %04x\n", offset_y16);
  printf("Offset Z:\t %04x\n", offset_z16);
  printf("Sensitivity XY: %04x\n", sensitivity_xy16);
  printf("Sensitivity Z: %04x\n", sensitivity_z16);

  return;
}

void SetGain()
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address, gain_sel_all, sensor, reg;

  uint8_t gain_sel[4];

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
  HAL_UART_Receive(&huart2, &gain_sel[3], 1, HAL_MAX_DELAY);
  printf("Set second highest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[2], 1, HAL_MAX_DELAY);
  printf("Set third highest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[1], 1, HAL_MAX_DELAY);
  printf("Set the lowest bit for bit selection: (0 or 1)\n");
  HAL_UART_Receive(&huart2, &gain_sel[0], 1, HAL_MAX_DELAY);


  gain_sel_all = 0;
  gain_sel_all = (((gain_sel[3] << 3) | gain_sel[2] << 2) | gain_sel[1] << 1) | gain_sel[0];      //0b0000 xxxx
  hal_error = HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF1, 1, &reg, 2, HAL_MAX_DELAY);
  reg16 = reg[1] << 8 | reg[0];
  reg16 = reg16 & 0b1111111100001111;
  gain_sel_all = reg16 | (gain_sel_all << 4);
  printf("Value which will be sent to the reg: %02x \n", gain_sel_all);
  printf("Hal status = %x\n", hal_error);
  hal_error = HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF1, 1, &gain_sel_all, 2, HAL_MAX_DELAY);
  printf("Hal status = %x\n", hal_error);
  printf("Set done.\n");
  return;
}
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

  resolution = ((((x_res[1] << 5 | x_res[0] << 4) | y_res[1] << 3) | y_res[0] << 2) | z_res[1] << 1) | z_res[0];

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_CONF3, 1, reg_r, 2, HAL_MAX_DELAY);
  reg_w = (reg_r[1] << 8) | reg_r[0];
  reg_w = reg_w & (0b11000000 << 5);
  reg_w = reg_w | (resolution << 5);
  printf("Value which will be sent to the reg: %04x \n", reg_w);
  HAL_I2C_Mem_Write(&i2c_address, dev_address, REG_CONF3, 1, &reg_w, 2, HAL_MAX_DELAY);
  printf("Set done.\n");
  return;

}

int main(void)
{
  uint8_t v_en = 1;                                           //flaga czy mierzenie napięcia jest włączone czy nie
  uint8_t calibration_mid[6], calibration_high[6];
  int16_t xm_cal, ym_cal, zm_cal, xh_cal, yh_cal, zh_cal;     //zmienne przechowujące wartości zerujące sensor

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
    printf("\t z - Reset.\n");
    printf("Calibration/settings:\n");
    printf("\t c - Show status reg.\n");
    printf("\t k - Calibrate magnetic sensor.\n");
    printf("\t a - Show all config parameters.\n");
    printf("\t s - Set sensitivity(NY).\n");
    printf("\t o - Set offset(NY).\n");
    printf("\t g - Set gain.\n");
    printf("\t r - Set resolution for magnetic measurements.\n");
    printf("\t f - Set frequency of measurement for Burst Mode (BurstDataRate)(NY).\n");
    printf("\t w - Enable/disable voltage measurement(NY).\n");
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
        ReadMagneticBurst();
        break;
      case 'z':
        printf("Reset in progress...\n");
        WriteRegMid(REG_I2C_ComandStatus, RESET_SENSOR);
        WriteRegHigh(REG_I2C_ComandStatus, RESET_SENSOR);
        HAL_Delay(300);
        printf("Reset done\n");
        break;
      case 'c':
        ReadStatus(1);
        ReadStatus(2);
        break;
      case 'k':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_MAGNETIC);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_MAGNETIC);
        HAL_Delay(10);
        ReadRegMid(REG_I2CX2, calibration_mid, 6);
        ReadRegHigh(REG_I2CX2, calibration_high, 6);
        xm_cal = (calibration_mid[0] << 8) | calibration_mid[1];
        ym_cal = (calibration_mid[2] << 8) | calibration_mid[3];
        zm_cal = (calibration_mid[4] << 8) | calibration_mid[5];
        xh_cal = (calibration_high[0] << 8) | calibration_high[1];
        yh_cal = (calibration_high[2] << 8) | calibration_high[3];
        zh_cal = (calibration_high[4] << 8) | calibration_high[5];
        
        xm_cal = BinaryToDecimal(xm_cal);
        ym_cal = BinaryToDecimal(ym_cal);
        zm_cal = BinaryToDecimal(zm_cal);
        xh_cal = BinaryToDecimal(xh_cal);
        yh_cal = BinaryToDecimal(yh_cal);
        zh_cal = BinaryToDecimal(zh_cal);

        printf("Sensor calibrated, calibration values:\n");
        printf("Medium:\n");
        printf("X: %05i, Y: %05i, Z: %05i\n", xm_cal, ym_cal, zm_cal);
        printf("High:\n");
        printf("X: %05i, Y: %05i, Z: %05i\n", xh_cal, yh_cal, zh_cal);
        break;
      case 'a':
        printf("All configuration values:\n");
        ReadConfig();
        break;
      case 's':
        printf("Sensitivity settings.\n");
        break;
      case 'g':
        printf("Gain settings.\n");
        SetGain();
        break;
      case 'r':
        printf("Resolution settings.\n");
        SetResolution();
        break;
      case 'f':
        printf("Burst mode settings.\n");
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
