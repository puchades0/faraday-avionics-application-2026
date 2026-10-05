/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ism330dhcx_reg.h"
#include "h3lis331dl_reg.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_SPI_READ_BIT (1U << 7)
#define IMU_RESET_TIMEOUT_MS 100U
#define IMU_BOOT_TIME_MS 10U

#define HIGHG_SPI_READ_BIT  (1U << 7)
#define HIGHG_SPI_MULTI_BIT (1U << 6)
#define HIGHG_BOOT_TIME_MS 5U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static stmdev_ctx_t imu_ctx = {0};

static int imu_identified = 0;
static int imu_ready = 0;

static float imu_acceleration_g[3] = {0};
static float imu_angular_rate_dps[3] = {0};

static int32_t imu_acceleration_status = 0;
static int32_t imu_angular_rate_status = 0;

static uint32_t imu_acceleration_time_ms = 0;
static uint32_t imu_angular_rate_time_ms = 0;

static stmdev_ctx_t highg_ctx = {0};

static int highg_identified = 0;
static int highg_ready = 0;

static float highg_acceleration_g[3] = {0};

static int32_t highg_acceleration_status = 0;
static uint32_t highg_acceleration_time_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Devuelve 1 si se identifica la IMU esperada; 0 si falla la lectura o no coincide.
static int IMU_CheckIdentity(void)
{
    uint8_t id = 0;
    int32_t status;

    // Leer WHO_AM_I mediante la biblioteca de ST, que utiliza nuestra IMU_Read.
    status = ism330dhcx_device_id_get(&imu_ctx, &id);

    // La biblioteca devuelve 0 cuando la lectura termina correctamente.
    // Además, el identificador recibido debe coincidir con el del ISM330DHCX.
    if (status == 0 && id == ISM330DHCX_ID)
    {
      return 1;
    }
    return 0;
}

static int32_t IMU_Read(void *handle, uint8_t reg,
                        uint8_t *bufp, uint16_t len)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle;
    uint8_t command = reg | IMU_SPI_READ_BIT;
    HAL_StatusTypeDef status;

    memset(bufp, 0, len);

    // 1. Seleccionar la IMU.
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // 2. Enviar command y guardar el resultado en status.
    status = HAL_SPI_Transmit(spi, &command, 1, 5);
    // 3. Solo si el envío ha ido bien, recibir len bytes
    //    en bufp y actualizar status.
    if (status == HAL_OK)
    {
      status = HAL_SPI_Receive(spi, bufp, len, 5);
    }

    // 4. Deseleccionar la IMU, aunque haya fallado algo.
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    // 5. Devolver 0 si todo ha ido bien; -1 si ha fallado.
    if (status == HAL_OK)
    {
      return 0;
    }
    return -1;
}

static int32_t IMU_Write(void *handle, uint8_t reg,
                         const uint8_t *bufp, uint16_t len)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle;
    uint8_t command = reg & ~IMU_SPI_READ_BIT;
    HAL_StatusTypeDef status;

    // 1. Seleccionar la IMU.
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    // 2. Enviar command y guardar el resultado en status.
    status = HAL_SPI_Transmit(spi, &command, 1, 5);
    // 3. Solo si el envío ha ido bien, transmitir len bytes
    //    en bufp y actualizar status.
    if (status == HAL_OK)
    {
      status = HAL_SPI_Transmit(spi, bufp, len, 5);
    }

    // 4. Deseleccionar la IMU, aunque haya fallado algo.
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    // 5. Devolver 0 si todo ha ido bien; -1 si ha fallado.
    if (status == HAL_OK)
    {
      return 0;
    }
    return -1;
}

static int32_t IMU_Reset(void)
{
    uint8_t reset_pending = 1;
    uint32_t start;
    int32_t status;

    // 1. Solicitar el reinicio. Si falla la comunicación, devolver -1.
    status = ism330dhcx_reset_set(&imu_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
      return -1;
    }
    // 2. Guardar el instante en que empieza la espera.
    start = HAL_GetTick();
    // 3. Consultar el estado del reinicio hasta que termine,
    //    falle la comunicación o se agote el tiempo.
    while (reset_pending != 0)
    {
      status = ism330dhcx_reset_get(&imu_ctx, &reset_pending);
      if (status != 0)
        {
          return -1;
        }
      if (reset_pending == 0)
      {
        return 0;
      }
      if (HAL_GetTick() - start >= IMU_RESET_TIMEOUT_MS)
      {
        return -1;
      }
      HAL_Delay(1);
    }
    // 4. Devolver 0 si el reinicio ha terminado correctamente.
    return 0;
}

static int32_t IMU_Configure(void)
{
    int32_t status;

    // Desactivar la interfaz I2C de la IMU, porque utilizamos SPI.
    status = ism330dhcx_i2c_interface_set(&imu_ctx, ISM330DHCX_I2C_DISABLE);
    if (status != 0)
    {
        return -1;
    }
    // Activar la configuración del dispositivo indicada por ST.
    status = ism330dhcx_device_conf_set(&imu_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Evitar mezclar los dos bytes de una medida durante su lectura.
    status = ism330dhcx_block_data_update_set(&imu_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Recibir los bytes de las medidas sin enviar una dirección nueva para cada uno.
    status = ism330dhcx_auto_increment_set(&imu_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Configurar el rango del acelerómetro a ±16g.
    status = ism330dhcx_xl_full_scale_set(&imu_ctx, ISM330DHCX_16g);
    if (status != 0)
    {
        return -1;
    }
    // Configurar el rango del giróscopo a ±4000 °/s.
    status = ism330dhcx_gy_full_scale_set(&imu_ctx, ISM330DHCX_4000dps);
    if (status != 0)
    {
        return -1;
    }
    // Configurar frecuencia del acelerómetro a 208 Hz.
    status = ism330dhcx_xl_data_rate_set(&imu_ctx, ISM330DHCX_XL_ODR_208Hz);
    if (status != 0)
    {
        return -1;
    }
    // Configurar frecuencia del giróscopo a 208 Hz.
    status = ism330dhcx_gy_data_rate_set(&imu_ctx, ISM330DHCX_GY_ODR_208Hz);
    if (status != 0)
    {
        return -1;
    }
    // Todas las configuraciones han resultado exitosas, devuelve 0.
    return 0;
}

static int32_t IMU_Init(void)
{
  // Esperar al arranque del sensor antes de iniciar la comunicación.
  HAL_Delay(IMU_BOOT_TIME_MS);
  // Se comprueba la identificación del sensor.
  imu_identified = IMU_CheckIdentity();
  if (imu_identified == 0)
  {
    return -1;
  }
  // Se lleva a cabo el Reset del sensor.
  if (IMU_Reset() != 0)
  {
    return -1;
  }
  // Se lleva a cabo la configuración del sensor.
  if (IMU_Configure() != 0)
  {
    return -1;
  }
  // Si todos los pasos han sido correctos devuelve 0.
  return 0;
}

static int32_t IMU_ReadAcceleration(float acceleration_g[3])
{
    uint8_t data_ready = 0;
    int16_t raw[3] = {0};
    int32_t status;

    // 1. Consultar si hay una medida nueva.
    //    Si falla la consulta, devolver -1.
    status = ism330dhcx_xl_flag_data_ready_get(&imu_ctx, &data_ready);
    if (status != 0)
    {
        return -1;
    }
    // 2. Si no hay una medida nueva, devolver 0.
    if (data_ready == 0)
    {
      return 0;
    }
    // 3. Leer los tres ejes en raw.
    //    Si falla la lectura, devolver -1.
    status = ism330dhcx_acceleration_raw_get(&imu_ctx, raw);
    if (status != 0)
    {
        return -1;
    }
    // 4. Convertir cada eje a g y guardarlo en acceleration_g.
    for (int i = 0; i < 3; i++)
    {
      acceleration_g[i] = ism330dhcx_from_fs16g_to_mg(raw[i]) / 1000.0f;
    }
    // 5. Devolver 1 para indicar que se ha obtenido una medida nueva.
    return 1;
}

static int32_t IMU_ReadAngularRate(float angular_rate_dps[3])
{
    uint8_t data_ready = 0;
    int16_t raw[3] = {0};
    int32_t status;

    // 1. Consultar si hay una medida nueva.
    //    Si falla la consulta, devolver -1.
    status = ism330dhcx_gy_flag_data_ready_get(&imu_ctx, &data_ready);
    if (status != 0)
    {
        return -1;
    }
    // 2. Si no hay una medida nueva, devolver 0.
    if (data_ready == 0)
    {
      return 0;
    }
    // 3. Leer los tres ejes en raw.
    //    Si falla la lectura, devolver -1.
    status = ism330dhcx_angular_rate_raw_get(&imu_ctx, raw);
    if (status != 0)
    {
        return -1;
    }
    // 4. Convertir cada eje a dps y guardarlo en angular_rate_dps.
    for (int i = 0; i < 3; i++)
    {
      angular_rate_dps[i] = ism330dhcx_from_fs4000dps_to_mdps(raw[i]) / 1000.0f;
    }
    // 5. Devolver 1 para indicar que se ha obtenido una medida nueva.
    return 1;
}

static int32_t HIGHG_Read(void *handle, uint8_t reg,
                          uint8_t *bufp, uint16_t len)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle;
    uint8_t command = reg & ~(HIGHG_SPI_READ_BIT | HIGHG_SPI_MULTI_BIT);
    command = command | HIGHG_SPI_READ_BIT;
    if (len > 1)
    {
      command = command | HIGHG_SPI_MULTI_BIT;
    }
    HAL_StatusTypeDef status;

    memset(bufp, 0, len);

    // 1. Seleccionar el acelerómetro de alto rango.
    HAL_GPIO_WritePin(HIGHG_CS_GPIO_Port, HIGHG_CS_Pin, GPIO_PIN_RESET);
    // 2. Enviar command y guardar el resultado en status.
    status = HAL_SPI_Transmit(spi, &command, 1, 5);
    // 3. Solo si el envío ha ido bien, recibir len bytes
    //    en bufp y actualizar status.
    if (status == HAL_OK)
    {
      status = HAL_SPI_Receive(spi, bufp, len, 5);
    }

    // 4. Deseleccionar el acelerómetro de alto rango, aunque haya fallado algo.
    HAL_GPIO_WritePin(HIGHG_CS_GPIO_Port, HIGHG_CS_Pin, GPIO_PIN_SET);
    // 5. Devolver 0 si todo ha ido bien; -1 si ha fallado.
    if (status == HAL_OK)
    {
      return 0;
    }
    return -1;
}

static int32_t HIGHG_Write(void *handle, uint8_t reg,
                           const uint8_t *bufp, uint16_t len)
{
    SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)handle;
    uint8_t command = reg & ~(HIGHG_SPI_READ_BIT | HIGHG_SPI_MULTI_BIT);
    if (len > 1)
    {
      command |= HIGHG_SPI_MULTI_BIT;
    }
    HAL_StatusTypeDef status;

    // 1. Seleccionar el acelerómetro de alto rango.
    HAL_GPIO_WritePin(HIGHG_CS_GPIO_Port, HIGHG_CS_Pin, GPIO_PIN_RESET);
    // 2. Enviar command y guardar el resultado en status.
    status = HAL_SPI_Transmit(spi, &command, 1, 5);
    // 3. Solo si el envío ha ido bien, transmitir len bytes
    //    en bufp y actualizar status.
    if (status == HAL_OK)
    {
      status = HAL_SPI_Transmit(spi, bufp, len, 5);
    }

    // 4. Deseleccionar el acelerómetro de alto rango, aunque haya fallado algo.
    HAL_GPIO_WritePin(HIGHG_CS_GPIO_Port, HIGHG_CS_Pin, GPIO_PIN_SET);
    // 5. Devolver 0 si todo ha ido bien; -1 si ha fallado.
    if (status == HAL_OK)
    {
      return 0;
    }
    return -1;
}

// Devuelve 1 si se identifica el acelerómetro de alto rango esperado; 0 si falla la lectura o no coincide.
static int HIGHG_CheckIdentity(void)
{
    uint8_t id = 0;
    int32_t status;

    // Leer WHO_AM_I mediante la biblioteca de ST, que utiliza nuestra HIGHG_Read.
    status = h3lis331dl_device_id_get(&highg_ctx, &id);

    // La biblioteca devuelve 0 cuando la lectura termina correctamente.
    // Además, el identificador recibido debe coincidir con H3LIS331DL_ID (0x32).
    if (status == 0 && id == H3LIS331DL_ID)
    {
      return 1;
    }
    return 0;
}

static int32_t HIGHG_Configure(void)
{
    int32_t status;

    // Detener las medidas mientras se configura el sensor.
    status = h3lis331dl_data_rate_set(&highg_ctx, H3LIS331DL_ODR_OFF);
    if (status != 0)
    {
        return -1;
    }
    // Activar BDU.
    status = h3lis331dl_block_data_update_set(&highg_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Establecer el orden de los bytes, coloca el byte menos significativo en la dirección más baja.
    status = h3lis331dl_data_format_set(&highg_ctx, H3LIS331DL_LSB_AT_LOW_ADD);
    if (status != 0)
    {
        return -1;
    }
    // Desactivar el filtro paso alto.
    status = h3lis331dl_hp_path_set(&highg_ctx, H3LIS331DL_HP_DISABLE);
    if (status != 0)
    {
        return -1;
    }
    // Seleccionar el rango de ±200 g.
    status = h3lis331dl_full_scale_set(&highg_ctx, H3LIS331DL_200g);
    if (status != 0)
    {
        return -1;
    }
    // Habilitar el eje X.
    status = h3lis331dl_axis_x_data_set(&highg_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Habilitar el eje Y.
    status = h3lis331dl_axis_y_data_set(&highg_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Habilitar el eje Z.
    status = h3lis331dl_axis_z_data_set(&highg_ctx, PROPERTY_ENABLE);
    if (status != 0)
    {
        return -1;
    }
    // Activar las medidas a la frecuencia escogida de 400 Hz.
    status = h3lis331dl_data_rate_set(&highg_ctx, H3LIS331DL_ODR_400Hz);
    if (status != 0)
    {
        return -1;
    }

    return 0;
}

static int32_t HIGHG_Init(void)
{
  // Esperar al arranque del sensor antes de iniciar la comunicación.
  HAL_Delay(HIGHG_BOOT_TIME_MS);
  // Se comprueba la identificación del sensor
  highg_identified = HIGHG_CheckIdentity();
  if (highg_identified == 0)
  {
    return -1;
  }
  // Se lleva a cabo la configuración del sensor.
  if (HIGHG_Configure() != 0)
  {
    return -1;
  }
  // Si todos los pasos han sido correctos devuelve 0.
  return 0;
}

static int32_t HIGHG_ReadAcceleration(float acceleration_g[3])
{
    uint8_t data_ready = 0;
    int16_t raw[3] = {0};
    int32_t status;

    // 1. Consultar si hay una medida nueva.
    //    Si falla la consulta, devolver -1.
    status = h3lis331dl_flag_data_ready_get(&highg_ctx, &data_ready);
    if (status != 0)
    {
        return -1;
    }
    // 2. Si no hay una medida nueva, devolver 0.
    if (data_ready == 0)
    {
      return 0;
    }
    // 3. Leer los tres ejes en raw.
    //    Si falla la lectura, devolver -1.
    status = h3lis331dl_acceleration_raw_get(&highg_ctx, raw);
    if (status != 0)
    {
        return -1;
    }
    // 4. Convertir cada eje a g y guardarlo en acceleration_g.
    for (int i = 0; i < 3; i++)
    {
      acceleration_g[i] = h3lis331dl_from_fs200_to_mg(raw[i]) / 1000.0f;
    }
    // 5. Devolver 1 para indicar que se ha obtenido una medida nueva.
    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  imu_ctx.read_reg = IMU_Read;
  imu_ctx.write_reg = IMU_Write;
  imu_ctx.mdelay = HAL_Delay;
  imu_ctx.handle = &hspi1;

  imu_ready = (IMU_Init() == 0);

  highg_ctx.read_reg = HIGHG_Read;
  highg_ctx.write_reg = HIGHG_Write;
  highg_ctx.mdelay = HAL_Delay;
  highg_ctx.handle = &hspi1;

  highg_ready = (HIGHG_Init() == 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Se comprueba que la IMU esté lista.
    if (imu_ready == 1)
    {
      // Se lleva a cabo la lectura de aceleración.
      imu_acceleration_status = IMU_ReadAcceleration(imu_acceleration_g);
      // En caso de obtener una muestra nueva se actualiza la marca de tiempo.
      if (imu_acceleration_status == 1)
      {
        imu_acceleration_time_ms = HAL_GetTick();
      }
      // Se lleva a cabo la lectura de velocidad angular.
      imu_angular_rate_status = IMU_ReadAngularRate(imu_angular_rate_dps);
      // En caso de obtener una muestra nueva se actualiza la marca de tiempo.
      if (imu_angular_rate_status == 1)
      {
        imu_angular_rate_time_ms = HAL_GetTick();
      }
    }

    // Se comprueba que el acelerómetro de alto rango esté listo.
    if (highg_ready == 1)
    {
      // Se lleva a cabo la lectura de aceleración.
      highg_acceleration_status = HIGHG_ReadAcceleration(highg_acceleration_g);
      // En caso de obtener una muestra nueva se actualiza la marca de tiempo.
      if (highg_acceleration_status == 1)
      {
        highg_acceleration_time_ms = HAL_GetTick();
      }
    }

    // Se espera 1 ms antes de la siguiente lectura
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HIGHG_CS_GPIO_Port, HIGHG_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGHG_CS_Pin */
  GPIO_InitStruct.Pin = HIGHG_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HIGHG_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
