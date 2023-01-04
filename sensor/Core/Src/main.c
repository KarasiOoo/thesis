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

void ReadMagnetic(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address;
  
  uint8_t memory[12];
  int16_t x_val, y_val, z_val;
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
  
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, memory, 12, HAL_MAX_DELAY);

  x_val = memory[2] << 8 | memory[3];
  y_val = memory[4] << 8 | memory[5];
  z_val = memory[6] << 8 | memory[7];

  printf("X: %05i,\t Y: %05i,\t Z: %05i\n", x_val, y_val, z_val);
  return;
}

void ReadTemperature(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address;

  uint8_t memory[2];
  float t_valf;

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

  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2CT2, 1, memory, 2, HAL_MAX_DELAY);

  t_valf = memory[0] << 8 | memory[1];
  t_valf = t_valf / 50;

  printf("Temperature: %3.2f\n", t_valf);
  return;

}

void ReadMagneticTemperature(uint8_t sensor)
{
  I2C_HandleTypeDef i2c_address;
  uint8_t dev_address;
  
  uint8_t memory[12];
  uint8_t status;
  int16_t x_val, y_val, z_val;//, t_val;
  //float x_valf, y_valf, z_valf, t_valf;
  float t_val;

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
  
  HAL_I2C_Mem_Read(&i2c_address, dev_address, REG_I2C_ComandStatus, 1, memory, 12, HAL_MAX_DELAY);

  status = memory[0];
  x_val = memory[2] << 8 | memory[3];
  y_val = memory[4] << 8 | memory[5];
  z_val = memory[6] << 8 | memory[7];
  t_val = ((memory[8] << 8) | memory[9]);
  t_val = t_val / 50;

  printf("Status: %02x,\t X:%05i,\t Y:%05i,\t Z:%05i,\t T:%3.2f,\n", status, x_val, y_val, z_val, t_val);
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
  printf("%05i\n", v_val);
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

  //WriteRegMid(REG_I2C_ComandStatus, BURST_MEASURE_MT);
  //WriteRegHigh(REG_I2C_ComandStatus, BURST_MEASURE_MT);
  //HAL_Delay(300);

  while (1)
  {
    uint8_t action, new_task;
    printf("Choose action:\n");
    printf("Measure:\n");
    printf("\t m - Once magnetic field\n");
    printf("\t t - Once temperature\n");
    printf("\t v - Once voltage\n");
    printf("\t c - Continues magnetic field and temperature\n");
    printf("Calibration/settings:\n");
    printf("\t a - Show all config parameters\n");
    printf("\t s - Set sensitivity\n");
    printf("\t o - Set offset\n");
    printf("\t g - Set gain\n");
    printf("\t i - Increase gain\n");
    printf("\t d - Decrease gain\n");
    printf("\t r - Set resolution for magnetic measurements\n");
    printf("\t f - Set frequency of measurement for Burst Mode (BurstDataRate)\n");
    printf("\t w - Enable/disable voltage measurement\n");
    printf("What do you want to do?\t\n");
    printf("\n");

    HAL_UART_Receive(&huart2, &action, 1, HAL_MAX_DELAY);

    switch(action)
    {
      case 'm':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_MT);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_MT);
        printf("Measured values of magnetic field:\n");
        ReadMagnetic(1);
        ReadMagnetic(2);
        break;
      case 't':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_TEMPERATURE);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_TEMPERATURE);
        printf("Measured value of temperature:\n");
        ReadTemperature(1);
        ReadTemperature(2);
        break;
      case 'v':
        WriteRegMid(REG_I2C_ComandStatus, SINGLE_MEASURE_VOLTAGE);
        WriteRegHigh(REG_I2C_ComandStatus, SINGLE_MEASURE_VOLTAGE);
        HAL_Delay(1);
        printf("Measured value of voltage:\n");
        ReadVoltage(1);
        ReadVoltage(2);
        break;
      case 'c':
        printf("Continues measurement:\n");
        break;
      case 'a':
        printf("All configuration values:\n");
        break;
      case 's':
        printf("Sensetivity settings.\n");
        break;
      case 'g':
        printf("Gain settings.\n");
        break;
      case 'i':
        printf("Gain increased by P.\n");
        break;
      case 'd':
        printf("Gain decreased by P.\n");
        break;
      case 'r':
        printf("Resolution settings.\n");
        break;
      case 'f':
        printf("Burst mode settings.\n");
        break;
      case 'w':
        if(v_en == 1)
          {
            printf("Voltage measurement enabled.\n");
          }
        else
          {
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

    // ReadMagneticTemperature(1);
    // ReadMagneticTemperature(2);
    // printf("\n");
    // HAL_Delay(1000);
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
