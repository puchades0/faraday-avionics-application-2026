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
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_ll_adc.h"
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Datos NAV-PVT convertidos a unidades físicas; se publican juntos al decodificar el mensaje.
typedef struct
{
    double longitude_deg;
    double latitude_deg;

    // Altura respecto al nivel medio del mar, no respecto al punto de lanzamiento.
    float height_msl_m;

    float velocity_north_mps;
    float velocity_east_mps;
    float velocity_up_mps;

    // Estimaciones de error proporcionadas por el receptor.
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    float speed_accuracy_mps;

    // Época GNSS en ms de la semana e instante de procesamiento en el reloj del STM32.
    uint32_t time_of_week_ms;
    uint32_t reception_time_ms;

    uint8_t fix_type;
    uint8_t satellites;
    // Validez por indicadores y coordenadas; GNSS_Update también comprueba la antigüedad.
    uint8_t fix_valid;
} GNSS_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_SPI_READ_BIT (1U << 7)
#define IMU_RESET_TIMEOUT_MS 100U
#define IMU_BOOT_TIME_MS 10U

#define HIGHG_SPI_READ_BIT  (1U << 7)
#define HIGHG_SPI_MULTI_BIT (1U << 6)
#define HIGHG_BOOT_TIME_MS 5U

#define BARO_I2C_ADDRESS_7BIT 0x28U
#define BARO_I2C_ADDRESS_HAL (BARO_I2C_ADDRESS_7BIT << 1)
#define BARO_CMD_SINGLE_MEASUREMENT 0xAAU
#define BARO_I2C_TIMEOUT_MS 5U

#define BARO_STATUS_BUSY_BIT          (1U << 5)
#define BARO_STATUS_MEMORY_ERROR_BIT  (1U << 2)
#define BARO_STATUS_OVERFLOW_BIT      (1U << 0)

#define BARO_STATUS_FIXED_MASK \
    ((1U << 7) | (1U << 6) | (1U << 4) | (1U << 3))
#define BARO_STATUS_FIXED_VALUE       (1U << 6)

#define BARO_DIGITAL_SCALE     16777216.0f
#define BARO_PRESSURE_MAX_HPA   1500.0f

#define BARO_PERIOD_MS 20U
#define BARO_CONVERSION_WAIT_MS 4U
#define BARO_MEASUREMENT_TIMEOUT_MS 10U

// Capacidad del DMA circular y del mensaje UBX en reconstrucción, en bytes.
#define GNSS_RX_BUFFER_SIZE 512U
#define GNSS_MESSAGE_BUFFER_SIZE 128U

// Tipo de solución y máscaras de validez de NAV-PVT.
#define GNSS_FIX_TYPE_3D       3U
#define GNSS_FIX_OK_BIT        (1U << 0)
#define GNSS_INVALID_LLH_BIT   (1U << 0)

// Limitar el trabajo de cada pasada para poder atender los otros sensores.
#define GNSS_MAX_BYTES_PER_UPDATE 128U

// Intervalo entre reinicios de recepción y antigüedad máxima admitida, en milisegundos.
#define GNSS_RETRY_PERIOD_MS 1000U
#define GNSS_DATA_TIMEOUT_MS 500U

// Capacidad de la trama transmitida y espera máxima de transmisión.
#define GNSS_TX_BUFFER_SIZE 128U
#define GNSS_TX_TIMEOUT_MS 200U

// Tiempo máximo para recibir la aceptación o el rechazo de una orden.
#define GNSS_ACK_TIMEOUT_MS 1500U

// Claves CFG-VALSET del protocolo u-blox M10 y valor del modelo Airborne <4g.
#define GNSS_CFG_NAVSPG_DYNMODEL 0x20110021U
#define GNSS_DYNMODEL_AIRBORNE_4G 8U

// Selección de protocolos de salida y frecuencia del mensaje NAV-PVT en UART1.
#define GNSS_CFG_UART1OUTPROT_UBX       0x10740001U
#define GNSS_CFG_UART1OUTPROT_NMEA      0x10740002U
#define GNSS_CFG_MSGOUT_NAV_PVT_UART1   0x20910007U

// Claves para seleccionar las constelaciones y sus señales.
#define GNSS_CFG_SIGNAL_SBAS_ENA       0x10310020U
#define GNSS_CFG_SIGNAL_BDS_ENA        0x10310022U
#define GNSS_CFG_SIGNAL_QZSS_ENA        0x10310024U
#define GNSS_CFG_SIGNAL_GLO_ENA        0x10310025U
#define GNSS_CFG_SIGNAL_GPS_ENA        0x1031001FU
#define GNSS_CFG_SIGNAL_GPS_L1CA_ENA   0x10310001U
#define GNSS_CFG_SIGNAL_GAL_ENA        0x10310021U
#define GNSS_CFG_SIGNAL_GAL_E1_ENA     0x10310007U

// Claves del periodo de medida, soluciones de navegación y velocidad UART.
#define GNSS_CFG_RATE_MEAS             0x30210001U
#define GNSS_CFG_RATE_NAV              0x30210002U
#define GNSS_CFG_UART1_BAUDRATE        0x40520001U

// Espera adicional al ACK tras modificar la selección de señales GNSS.
#define GNSS_SIGNAL_SETTLE_MS          500U

// Velocidad final en bit/s y espera durante el cambio de velocidad.
#define GNSS_UART_BAUDRATE       115200U
#define GNSS_BAUD_SETTLE_MS      1500U

// Intentos por velocidad durante el arranque y pausa entre intentos fallidos.
#define GNSS_STARTUP_ATTEMPTS 3U
#define GNSS_STARTUP_RETRY_MS 200U

// ADC de 12 bits: espera de conversión y máximo código digital.
#define ADC_CONVERSION_TIMEOUT_MS 2U
#define ADC_MAX_VALUE 4095U

// Periodos solicitados: alimentación a 10 Hz y temperatura interna a 1 Hz.
#define POWER_READ_PERIOD_MS       100U
#define TEMPERATURE_READ_PERIOD_MS 1000U

// Factores de los divisores resistivos de ejemplo para el ejercicio
// En la realidad deben corresponder a los divisores de la placa.
#define BATTERY_DIVIDER_FACTOR     4.0f
#define RAIL1_DIVIDER_FACTOR       2.0f
#define RAIL2_DIVIDER_FACTOR       2.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

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

static float baro_pressure_hpa = 0.0f;
static float baro_temperature_c = 0.0f;
static int32_t baro_measurement_status = 0;
static uint32_t baro_measurement_time_ms = 0;

static int baro_measurement_pending = 0;
static uint32_t baro_last_request_ms = 0;
static uint32_t baro_measurement_start_ms = 0;

// Recepción GNSS: el DMA escribe el array y el bucle retira los bytes.
// volatile se usa en los indicadores y contadores compartidos con interrupciones.
static uint8_t gnss_rx_buffer[GNSS_RX_BUFFER_SIZE] = {0};
static volatile uint8_t gnss_rx_event = 0;
static uint16_t gnss_rx_read_position = 0;

// Vueltas completas realizadas por el DMA desde el inicio de la recepción.
static volatile uint32_t gnss_rx_completed_buffers = 0;

// Bytes retirados del búfer desde el inicio de la recepción.
static uint32_t gnss_rx_read_count = 0;

// Trama UBX en reconstrucción: bytes acumulados, tamaño esperado y tamaño de la última válida.
static uint8_t gnss_message_buffer[GNSS_MESSAGE_BUFFER_SIZE] = {0};
static uint16_t gnss_message_count = 0;
static uint16_t gnss_expected_length = 0;
static uint16_t gnss_message_length = 0;

// Último NAV-PVT decodificado; consultar fix_valid antes de utilizar sus datos.
static GNSS_Data_t gnss_data = {0};

// Mensajes rechazados por formato, longitud o checksum; se conserva tras reiniciar la recepción.
static uint32_t gnss_message_errors = 0;

// Solicitud de recuperación por error UART o llenado del búfer; último error HAL para diagnóstico.
static volatile uint8_t gnss_rx_error_pending = 0;
static volatile uint32_t gnss_rx_last_error = HAL_UART_ERROR_NONE;

// Estado de recepción, instante del último intento de recuperación y configuración completada.
static int gnss_rx_ready = 0;
static uint32_t gnss_last_restart_ms = 0;
static int gnss_configured = 0;

// Confirmación de la orden en curso: clase/ID esperados y resultado (0 pendiente, 1 ACK, -1 NAK).
static int gnss_ack_waiting = 0;
static uint8_t gnss_ack_expected_class = 0;
static uint8_t gnss_ack_expected_id = 0;
static int32_t gnss_ack_result = 0;

// Veces que la recepción ha alcanzado o superado la capacidad del búfer.
static uint32_t gnss_rx_overflow_count = 0;

// Últimas medidas del ADC: VDDA y líneas en voltios; temperatura interna en °C.
static float adc_vdda_v = 0.0f;
static float battery_voltage_v = 0.0f;
static float rail1_voltage_v = 0.0f;
static float rail2_voltage_v = 0.0f;
static float mcu_temperature_c = 0.0f;

// Estado de cada adquisición: 0 = todavía no realizada, 1 = correcta, -1 = fallida.
static int32_t power_measurement_status = 0;
static int32_t temperature_measurement_status = 0;

// Instantes de solicitud para programar los periodos de lectura.
static uint32_t power_last_request_ms = 0;
static uint32_t temperature_last_request_ms = 0;

// Instantes de las últimas adquisiciones correctas; no avanzan si una lectura falla.
static uint32_t power_measurement_time_ms = 0;
static uint32_t temperature_measurement_time_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
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

static int32_t BARO_StartMeasurement(void)
{
    uint8_t command = BARO_CMD_SINGLE_MEASUREMENT;
    HAL_StatusTypeDef status;

    // Enviar el comando por I2C1 y guardar el resultado en status.
    status = HAL_I2C_Master_Transmit(&hi2c1, BARO_I2C_ADDRESS_HAL, &command, 1, BARO_I2C_TIMEOUT_MS);
    // Devolver 0 si el envío funciona; -1 si falla.
    if (status != HAL_OK)
    {
      return -1;
    }
    return 0;
}

static int32_t BARO_ReadStatus(uint8_t *status_byte)
{
    HAL_StatusTypeDef status;

    // Leer un byte del barómetro y guardarlo donde apunta status_byte.
    status = HAL_I2C_Master_Receive(&hi2c1, BARO_I2C_ADDRESS_HAL, status_byte, 1, BARO_I2C_TIMEOUT_MS);
    // Devolver -1 si falla la comunicación; 0 si funciona.
    if (status != HAL_OK)
    {
      return -1;
    }
    return 0;
}

static int32_t BARO_CheckStatus(uint8_t status_byte)
{
    // 1. Comprobar los bits fijos. Si no coinciden, devolver -1.
    if ((status_byte & BARO_STATUS_FIXED_MASK) != BARO_STATUS_FIXED_VALUE)
    {
      return -1;
    }
    // 2. Si hay un error de memoria interna, devolver -1.
    if ((status_byte & BARO_STATUS_MEMORY_ERROR_BIT) != 0U)
    {
      return -1;
    }
    // 3. Si sigue midiendo, devolver 0.
    if ((status_byte & BARO_STATUS_BUSY_BIT) != 0U)
    {
      return 0;
    }
    // 4. Con la medición terminada, si hay desbordamiento, devolver -1.
    if ((status_byte & BARO_STATUS_OVERFLOW_BIT) != 0U)
    {
      return -1;
    }
    // 5. Devolver 1: medición terminada sin los errores comprobados.
    return 1;
}

static int32_t BARO_ReadRaw(uint32_t *pressure_raw,
                           uint32_t *temperature_raw)
{
    uint8_t data[7] = {0};
    HAL_StatusTypeDef status;
    int32_t measurement_status;

    // 1. Recibir los siete bytes mediante HAL_I2C_Master_Receive.
    status = HAL_I2C_Master_Receive(&hi2c1, BARO_I2C_ADDRESS_HAL, data, 7, BARO_I2C_TIMEOUT_MS);
    // 2. Si falla la comunicación, devolver -1.
    if (status != HAL_OK)
    {
      return -1;
    }
    // 3. Comprobar data[0] utilizando BARO_CheckStatus.
    //    Guardar su resultado en measurement_status.
    measurement_status = BARO_CheckStatus(data[0]);
    // 4. Si measurement_status no es 1, devolver ese resultado.
    if (measurement_status != 1)
    {
      return measurement_status;
    }
    // 5. Construir los valores de presión y temperatura
    //    y guardarlos mediante los punteros de salida.
    *pressure_raw = ((uint32_t)data[1] << 16)
             | ((uint32_t)data[2] << 8)
             | (uint32_t)data[3];
    *temperature_raw = ((uint32_t)data[4] << 16)
             | ((uint32_t)data[5] << 8)
             | (uint32_t)data[6];
    // 6. Devolver 1.
    return 1;
}

static int32_t BARO_ReadMeasurements(float *pressure_hpa,
                                    float *temperature_c)
{
    uint32_t pressure_raw = 0;
    uint32_t temperature_raw = 0;
    int32_t status;

    // 1. Llamar a BARO_ReadRaw pasando las direcciones
    //    de pressure_raw y temperature_raw.
    status = BARO_ReadRaw(&pressure_raw, &temperature_raw);
    // 2. Si su resultado no es 1, devolver ese resultado.
    if (status != 1)
    {
      return status;
    }
    // 3. Convertir pressure_raw a hPa y escribir
    //    el resultado donde apunta pressure_hpa.
    *pressure_hpa = ((float)pressure_raw - 0.1f * BARO_DIGITAL_SCALE) / (0.8f * BARO_DIGITAL_SCALE) * BARO_PRESSURE_MAX_HPA;
    // 4. Convertir temperature_raw a grados Celsius y escribir
    //    el resultado donde apunta temperature_c.
    *temperature_c = (float)temperature_raw / BARO_DIGITAL_SCALE * 165.0f - 40.0f;
    // 5. Devolver 1.
    return 1;
}

static void BARO_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (baro_measurement_pending == 0)
    {
        // 1. Si aún no han transcurrido BARO_PERIOD_MS
        //    desde baro_last_request_ms, salir con return.
        if ((uint32_t)(now - baro_last_request_ms) < BARO_PERIOD_MS)
        {
          return;
        }
        // 2. Guardar now en baro_last_request_ms.
        baro_last_request_ms = now;
        // 3. Llamar a BARO_StartMeasurement.
        //    Si falla, poner baro_measurement_status a -1 y salir.
        if (BARO_StartMeasurement() != 0)
        {
          baro_measurement_status = -1;
          return;
        }
        // 4. Si funciona:
        //    - Guardar HAL_GetTick() en baro_measurement_start_ms.
        //    - Poner baro_measurement_pending a 1.
        //    - Poner baro_measurement_status a 0.
        baro_measurement_start_ms = HAL_GetTick();
        baro_measurement_pending = 1;
        baro_measurement_status = 0;
    }
    else
    {
        uint8_t sensor_status = 0;

        // 1. Si aún no han pasado BARO_CONVERSION_WAIT_MS
        //    desde baro_measurement_start_ms, salir.
        if ((uint32_t)(now - baro_measurement_start_ms) < BARO_CONVERSION_WAIT_MS)
        {
          return;
        }
        // 2. Leer el byte de estado con BARO_ReadStatus.
        //    Si falla:
        //    - Poner baro_measurement_status a -1.
        //    - Poner baro_measurement_pending a 0.
        //    - Salir.
        if (BARO_ReadStatus(&sensor_status) != 0)
        {
          baro_measurement_status = -1;
          baro_measurement_pending = 0;
          return;

        }
        // 3. Interpretar sensor_status con BARO_CheckStatus
        //    y guardar el resultado en baro_measurement_status.
        baro_measurement_status = BARO_CheckStatus(sensor_status);
        // 4. Si ese resultado es 1, llamar a BARO_ReadMeasurements
        //    pasando las direcciones de baro_pressure_hpa
        //    y baro_temperature_c.
        //    Guardar su resultado en baro_measurement_status.
        if (baro_measurement_status == 1)
        {
          baro_measurement_status = BARO_ReadMeasurements(&baro_pressure_hpa, &baro_temperature_c);
        }
        // 5. Si baro_measurement_status es 1,
        //    actualizar baro_measurement_time_ms con HAL_GetTick().
        if (baro_measurement_status == 1)
        {
          baro_measurement_time_ms = HAL_GetTick();
        }
        // 6. Si baro_measurement_status es distinto de 0:
        //    - Poner baro_measurement_pending a 0.
        //    - Salir.
        if (baro_measurement_status != 0)
        {
          baro_measurement_pending = 0;
          return;
        }
        // 7. Si seguimos esperando y se ha alcanzado el timeout:
        //    - Poner baro_measurement_status a -1.
        //    - Poner baro_measurement_pending a 0.
        if ((uint32_t)(HAL_GetTick() - baro_measurement_start_ms) >= BARO_MEASUREMENT_TIMEOUT_MS)
        {
          baro_measurement_status = -1;
          baro_measurement_pending = 0;
        }
    }
}
// Inicia USART1 con DMA circular y avisos de recepción.
// Devuelve 0 si se inicia y -1 si falla; la UART y el DMA deben estar configurados.
static int32_t GNSS_StartReception(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gnss_rx_buffer, GNSS_RX_BUFFER_SIZE);
    if (status != HAL_OK)
    {
      return -1;
    }
    return 0;
}

// Invalida la solución, detiene la recepción y reinicia los contadores y el mensaje parcial.
// Conserva los contadores de diagnóstico. Devuelve 0 al reiniciar y -1 si falla.
static int32_t GNSS_RestartReception(void)
{
    gnss_data.fix_valid = 0;
    if (HAL_UART_AbortReceive(&huart1) != HAL_OK)
    {
      return -1;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    // Reiniciar los contadores solo después de detener el DMA.
    gnss_rx_read_position = 0;
    gnss_message_count = 0;
    gnss_expected_length = 0;
    gnss_message_length = 0;
    gnss_rx_event = 0;
    gnss_rx_error_pending = 0;
    gnss_rx_completed_buffers = 0;
    gnss_rx_read_count = 0;
    return GNSS_StartReception();
}

// Obtiene el total de bytes recibidos desde el último inicio, con aritmética uint32_t.
// Se llama desde el programa principal. Combina vueltas completas y posición del DMA.
// El contador requiere atender las interrupciones de DMA antes de que pase otra vuelta.
static uint32_t GNSS_GetReceivedCount(void)
{
  uint32_t interrupt_state;
  uint32_t completed_buffers;
  uint32_t remaining;

  // Leer una instantánea sin que el callback modifique el contador de vueltas.
  // El DMA continúa escribiendo durante este breve tramo.
  interrupt_state = __get_PRIMASK();
  __disable_irq();

  completed_buffers = gnss_rx_completed_buffers;
  remaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  if (__HAL_DMA_GET_FLAG(&hdma_usart1_rx,
                        __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_rx)) != RESET)
  {
      // Contar localmente la vuelta pendiente; el callback actualizará después el global.
      // No borrar el indicador que HAL necesita atender.
      completed_buffers++;
      remaining = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
      // Si el contador está en la transición, esta vuelta ya se ha contabilizado.
      if (remaining == 0)
      {
        remaining = GNSS_RX_BUFFER_SIZE;
      }
  }

  // Restaurar el estado previo, sin habilitar interrupciones que ya estuvieran bloqueadas.
  __set_PRIMASK(interrupt_state);

  return completed_buffers * GNSS_RX_BUFFER_SIZE
    + (GNSS_RX_BUFFER_SIZE - remaining);
}

// Retira un byte del búfer circular sin confundir una vuelta completa con un búfer vacío.
// Devuelve 1 si entrega un byte, 0 si no hay pendientes y -1 si se alcanza la capacidad.
// En caso de 0 o -1 no modifica la salida ni avanza el lector.
static int32_t GNSS_ReadByte(uint8_t *received_byte)
{
    uint32_t pending;
    uint8_t byte;

    // Restar totales permite distinguir una vuelta completa de un búfer vacío.
    pending = GNSS_GetReceivedCount() - gnss_rx_read_count;
    // Rechazar también el búfer justo lleno, antes de que otro byte sobrescriba datos.
    if (pending >= GNSS_RX_BUFFER_SIZE)
    {
      return -1;
    }
    else if (pending == 0)
    {
      return 0;
    }

    // Copiar sin avanzar aún el lector: el DMA sigue escribiendo.
    byte = gnss_rx_buffer[gnss_rx_read_position];

    pending = GNSS_GetReceivedCount() - gnss_rx_read_count;
    if (pending >= GNSS_RX_BUFFER_SIZE)
    {
      return -1;
    }

    // Entregar el byte solo tras comprobar de nuevo la capacidad después de la copia.
    *received_byte = byte;

    gnss_rx_read_count++;
    gnss_rx_read_position++;

    // La posición vuelve al principio del array; el recuento de bytes continúa.
    if (gnss_rx_read_position == GNSS_RX_BUFFER_SIZE)
    {
      gnss_rx_read_position = 0;
    }

    return 1;
}

// Atiende los avisos de USART1 en interrupción: cuenta vueltas y avisa al bucle.
// El procesamiento de mensajes se realiza fuera de la interrupción.
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,
                              uint16_t Size)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    // Solo TC cuenta una vuelta; HT e IDLE también pueden llamar a este callback.
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC)
    {
        gnss_rx_completed_buffers++;
    }

    gnss_rx_event = 1;
    // La posición actual se obtiene del contador DMA, no del tamaño notificado.
    (void)Size;
}

// Registra el error de USART1 y solicita que el bucle recupere la recepción.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
      return;
    }
    gnss_rx_last_error = HAL_UART_GetError(huart);
    gnss_rx_error_pending = 1;
}

// Calcula CK_A y CK_B sobre length bytes, con sumas de ocho bits.
// Para UBX se incluyen clase, identificador, longitud y datos, sin sincronismo ni checksum.
static void GNSS_ComputeChecksum(const uint8_t *data,
                                 uint16_t length,
                                 uint8_t *checksum_a,
                                 uint8_t *checksum_b)
{
    uint8_t a = 0;
    uint8_t b = 0;

    for (int i = 0; i < length; i++)
    {
      a = (uint8_t)(a + data[i]);
      b = (uint8_t)(b + a);
    }

    *checksum_a = a;
    *checksum_b = b;
}

// Construye y transmite una trama UBX con su checksum. No espera confirmación del receptor.
// Devuelve 0 si se transmite y -1 por fallo, argumentos inválidos o exceso de tamaño.
static int32_t GNSS_SendUbx(uint8_t message_class,
                           uint8_t message_id,
                           const uint8_t *payload,
                           uint16_t payload_length)
{
    uint8_t packet[GNSS_TX_BUFFER_SIZE];
    uint16_t packet_length;
    HAL_StatusTypeDef status;

    if (payload_length > GNSS_TX_BUFFER_SIZE - 8U)
    {
      return -1;
    }
    if (payload_length > 0 && payload == NULL)
    {
      return -1;
    }
    packet[0] = 0xB5;
    packet[1] = 0x62;
    packet[2] = message_class;
    packet[3] = message_id;
    packet[4] = (uint8_t)(payload_length & 0xFFU);
    packet[5] = (uint8_t)(payload_length >> 8);
    if (payload_length > 0)
    {
      memcpy(packet + 6, payload, payload_length);
    }
    // El checksum excluye los dos bytes de sincronismo.
    GNSS_ComputeChecksum(packet + 2, payload_length + 4U, &packet[6U + payload_length], &packet[7U + payload_length]);
    packet_length = payload_length + 8U;
    status = HAL_UART_Transmit(&huart1, packet, packet_length, GNSS_TX_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      return -1;
    }

    return 0;
}

// Examina una trama ya validada y resuelve la confirmación pendiente si coincide.
// Distingue aceptación (ACK-ACK) y rechazo (ACK-NAK); ignora otras respuestas.
static void GNSS_HandleAck(const uint8_t *message, uint16_t length)
{
    if (length != 10)
    {
      return;
    }
    if (message[2] != 0x05)
    {
      return;
    }
    if (message[3] != 0x00 && message[3] != 0x01)
    {
      return;
    }
    if (gnss_ack_waiting != 1)
    {
      return;
    }
    if (message[6] != gnss_ack_expected_class || message[7] != gnss_ack_expected_id)
    {
      return;
    }
    if (message[3] == 0x01)
    {
      gnss_ack_result = 1;
    }
    else
    {
      gnss_ack_result = -1;
    }
    gnss_ack_waiting = 0;
}

// Comprueba sincronismo, longitud y checksum de una trama UBX completa.
// Devuelve 1 si cumple esas comprobaciones y 0 si falla alguna.
static int GNSS_CheckMessage(const uint8_t *message,
                             uint16_t length)
{
    uint16_t payload_length;
    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;

    if (length < 8)
    {
      return 0;
    }
    if (message[0] != 0xB5 || message[1] != 0x62)
    {
      return 0;
    }
    payload_length = (uint16_t)message[4] | ((uint16_t)message[5] << 8);
    if ((uint32_t)payload_length + 8U != length)
    {
      return 0;
    }
    GNSS_ComputeChecksum(message + 2, length - 4U, &checksum_a, &checksum_b);
    if (checksum_a != message[length - 2U] || checksum_b != message[length - 1U])
    {
      return 0;
    }
    return 1;
}

// Reconstruye un entero sin signo a partir de cuatro bytes, del menos al más significativo.
static uint32_t GNSS_ReadU32LE(const uint8_t *data)
{
    uint32_t result = (uint32_t)data[0]
                    | ((uint32_t)data[1] << 8)
                    | ((uint32_t)data[2] << 16)
                    | ((uint32_t)data[3] << 24);

    return result;
}

// Reconstruye cuatro bytes como entero con signo de 32 bits.
// Conserva el patrón de bits para interpretar también coordenadas y velocidades negativas.
static int32_t GNSS_ReadI32LE(const uint8_t *data)
{
    uint32_t raw;
    int32_t value;

    raw = GNSS_ReadU32LE(data);
    // Copiar los bits evita depender de una conversión numérica fuera del rango con signo.
    memcpy(&value, &raw, sizeof(value));
    return value;
}

// Escribe un entero de 32 bits en cuatro bytes, del menos al más significativo.
static void GNSS_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

// Decodifica una trama UBX ya validada y publica los campos de NAV-PVT.
// Devuelve 1 si la decodifica, 0 si es otro mensaje y -1 si la longitud es incorrecta.
// Un retorno de 1 no garantiza una posición válida: hay que consultar fix_valid.
static int32_t GNSS_DecodeNavPvt(const uint8_t *message,
                                uint16_t length)
{
    GNSS_Data_t result = {0};

    if (length < 8)
    {
      return -1;
    }
    if (message[2] != 0x01 || message[3] != 0x07)
    {
      return 0;
    }
    if (length != 100)
    {
      return -1;
    }

    const uint8_t *payload = message + 6;

    // Campos de NAV-PVT según el protocolo M10; convertir a grados, metros y m/s.
    result.time_of_week_ms = GNSS_ReadU32LE(payload);
    result.fix_type = payload[20];
    result.satellites = payload[23];
    result.longitude_deg = (double)GNSS_ReadI32LE(payload + 24) * 1e-7;
    result.latitude_deg = (double)GNSS_ReadI32LE(payload + 28) * 1e-7;
    result.height_msl_m = (float)GNSS_ReadI32LE(payload + 36) / 1000.0f;
    result.horizontal_accuracy_m = (float)GNSS_ReadU32LE(payload + 40) / 1000.0f;
    result.vertical_accuracy_m = (float)GNSS_ReadU32LE(payload + 44) / 1000.0f;
    result.velocity_north_mps = (float)GNSS_ReadI32LE(payload + 48) / 1000.0f;
    result.velocity_east_mps = (float)GNSS_ReadI32LE(payload + 52) / 1000.0f;
    // NAV-PVT expresa la componente vertical hacia abajo; aquí se toma positiva hacia arriba.
    result.velocity_up_mps = -(float)GNSS_ReadI32LE(payload + 56) / 1000.0f;
    result.speed_accuracy_mps = (float)GNSS_ReadU32LE(payload + 68) / 1000.0f;
    // Marca de procesamiento en el STM32, distinta de la época GNSS de la solución.
    result.reception_time_ms = HAL_GetTick();
    // Comprobar solución 3D, indicadores de validez y rangos de coordenadas.
    // Las estimaciones de error se conservan; aquí no se aplican umbrales de precisión.
    if (result.fix_type == GNSS_FIX_TYPE_3D &&
        (payload[21] & GNSS_FIX_OK_BIT) != 0U &&
        (payload[78] & GNSS_INVALID_LLH_BIT) == 0U &&
        result.longitude_deg >= -180.0 &&
        result.longitude_deg <= 180.0 &&
        result.latitude_deg >= -90.0 &&
        result.latitude_deg <= 90.0)
    {
      result.fix_valid = 1;
    }
    gnss_data = result;
    return 1;
}

// Reconstruye una trama UBX byte a byte y comprueba su longitud y checksum.
// Devuelve 1 con una trama válida en gnss_message_buffer, 0 si continúa buscando
// o reuniendo datos y -1 si rechaza una trama. Tras completarla o rechazarla, busca otra.
static int32_t GNSS_ProcessByte(uint8_t byte)
{
    // Buscar los dos bytes de sincronismo antes de interpretar la cabecera.
    if (gnss_message_count == 0)
    {
        if (byte == 0xB5)
        {
          gnss_message_buffer[0] = byte;
          gnss_message_count = 1;
        }
        return 0;
    }

    if (gnss_message_count == 1)
    {
        if (byte == 0x62)
        {
          gnss_message_buffer[1] = byte;
          gnss_message_count = 2;
        }
        // Otro 0xB5 puede ser el comienzo de una nueva cabecera.
        else if (byte != 0xB5)
        {
          gnss_message_count = 0;
        }
        return 0;
    }

    if (gnss_message_count >= GNSS_MESSAGE_BUFFER_SIZE)
    {
      gnss_message_count = 0;
      gnss_expected_length = 0;
      return -1;
    }
    gnss_message_buffer[gnss_message_count] = byte;
    gnss_message_count ++;
    // La cabecera ya permite rechazar longitudes que no caben en el búfer.
    if (gnss_message_count == 6)
    {
      uint16_t payload_length = (uint16_t)gnss_message_buffer[4] | ((uint16_t)gnss_message_buffer[5] << 8);
      if (payload_length > (GNSS_MESSAGE_BUFFER_SIZE - 8U))
      {
        gnss_message_count = 0;
        gnss_expected_length = 0;
        return -1;
      }
      gnss_expected_length = payload_length + 8U;
    }
    // Validar la trama completa antes de entregarla a los manejadores.
    if (gnss_expected_length != 0U && gnss_message_count == gnss_expected_length)
    {
      int message_valid = GNSS_CheckMessage(gnss_message_buffer, gnss_message_count);

      if (message_valid == 1)
      {
        gnss_message_length = gnss_message_count;
      }

      gnss_message_count = 0;
      gnss_expected_length = 0;

      if ( message_valid == 0)
      {
        return -1;
      }

      return 1;
    }
    return 0;
}

// Procesa un número limitado de bytes por llamada para compartir tiempo con otros sensores.
// Entrega las tramas a los manejadores de confirmación y navegación.
// Si se llena el búfer, invalida la solución y solicita reiniciar la recepción.
static void GNSS_ProcessReceived(void)
{
    uint8_t byte;
    int32_t status;

    if  (gnss_rx_event == 0 || gnss_rx_error_pending != 0)
    {
      return;
    }
    // Borrar el aviso antes de procesar para conservar los nuevos avisos de interrupción.
    gnss_rx_event = 0;
    for (uint16_t i = 0; i < GNSS_MAX_BYTES_PER_UPDATE; i++)
    {
        status = GNSS_ReadByte(&byte);
        // Una pérdida del búfer requiere reiniciar la recepción, no continuar con el mensaje parcial.
        if (status == -1)
        {
            gnss_rx_overflow_count++;
            gnss_rx_error_pending = 1;
            gnss_data.fix_valid = 0;
            return;
        }
        else if (status == 0)
        {
          break;
        }
        status = GNSS_ProcessByte(byte);
        if (status == 1)
        {
          GNSS_HandleAck(gnss_message_buffer, gnss_message_length);
          status = GNSS_DecodeNavPvt(gnss_message_buffer, gnss_message_length);
        }
        if (status == -1)
        {
          gnss_message_errors++;
        }
    }

    // Solicitar otra pasada si se agotó el cupo o llegaron más bytes.
    if (GNSS_GetReceivedCount() != gnss_rx_read_count)
    {
        gnss_rx_event = 1;
    }
}

// Envía una orden UBX y procesa la recepción mientras espera su confirmación.
// Devuelve 0 si el receptor la acepta; -1 por rechazo, error o tiempo agotado.
// Se usa durante la configuración, antes de entrar en el bucle de adquisición.
static int32_t GNSS_SendAndWaitAck(uint8_t message_class,
                                 uint8_t message_id,
                                 const uint8_t *payload,
                                 uint16_t payload_length)
{
    uint32_t start_ms;
    int32_t status;

    if (gnss_rx_ready == 0 || gnss_rx_error_pending == 1)
    {
      return -1;
    }
    gnss_ack_expected_class = message_class;
    gnss_ack_expected_id = message_id;
    gnss_ack_result = 0;
    gnss_ack_waiting = 1;
    status = GNSS_SendUbx(message_class, message_id, payload, payload_length);
    if (status != 0)
    {
      gnss_ack_waiting = 0;
      return -1;
    }
    start_ms = HAL_GetTick();
    while (gnss_ack_waiting == 1 && HAL_GetTick() - start_ms < GNSS_ACK_TIMEOUT_MS)
    {
      if (gnss_rx_error_pending == 1)
      {
        break;
      }
      GNSS_ProcessReceived();
      if (gnss_ack_waiting == 1)
      {
        HAL_Delay(1);
      }
    }
    // Cerrar la espera también si terminó por error o por tiempo agotado.
    gnss_ack_waiting = 0;
    if (gnss_rx_error_pending == 1 || gnss_ack_result != 1)
    {
      return -1;
    }
    return 0;
}

// Aplica una clave CFG-VALSET de un byte en la RAM del receptor.
// Devuelve 0 si recibe aceptación y -1 si falla; no guarda el ajuste de forma permanente.
static int32_t GNSS_SetConfigU8(uint32_t key, uint8_t value)
{
    uint8_t payload[9] = {0};

    // Seleccionar solo la capa RAM; la cabecera empieza con versión y campos reservados a cero.
    payload[1] = 1;
    GNSS_WriteU32LE(payload + 4, key);
    payload[8] = value;
    return GNSS_SendAndWaitAck(0x06, 0x8A, payload, sizeof(payload));
}

// Aplica una clave CFG-VALSET de dos bytes en la RAM del receptor.
// Devuelve 0 si recibe aceptación y -1 si falla; no guarda el ajuste de forma permanente.
static int32_t GNSS_SetConfigU16(uint32_t key, uint16_t value)
{
    uint8_t payload[10] = {0};

    // Seleccionar solo la capa RAM; los dos bytes del valor se envían del menor al mayor.
    payload[1] = 1;
    GNSS_WriteU32LE(payload + 4, key);
    payload[8] = (uint8_t)value;
    payload[9] = (uint8_t)(value >> 8);
    return GNSS_SendAndWaitAck(0x06, 0x8A, payload, sizeof(payload));
}

// Cambia la velocidad de USART1 del STM32 y reinicia su recepción DMA.
// No modifica el receptor GNSS. Devuelve 0 si funciona y -1 si falla.
static int32_t GNSS_SetLocalBaudrate(uint32_t baudrate)
{
    int32_t status;

    gnss_rx_ready = 0;
    gnss_ack_waiting = 0;
    gnss_data.fix_valid = 0;
    status = HAL_UART_AbortReceive(&huart1);
    if (status != HAL_OK)
    {
      return -1;
    }
    huart1.Init.BaudRate = baudrate;
    status = HAL_UART_Init(&huart1);
    if (status != HAL_OK)
    {
      return -1;
    }
    status = GNSS_RestartReception();
    gnss_rx_ready = (status == 0);
    return status;
}

// Cambia la velocidad del receptor y del STM32 y verifica la comunicación resultante.
// Devuelve 0 si la nueva comunicación responde y -1 si falla algún paso.
static int32_t GNSS_ChangeBaudrate(uint32_t baudrate)
{
    uint8_t payload[12] = {0};
    int32_t status;

    payload[1] = 1;
    GNSS_WriteU32LE(payload + 4, GNSS_CFG_UART1_BAUDRATE );
    GNSS_WriteU32LE(payload + 8, baudrate);
    // No esperar el ACK del cambio de velocidad: la comunicación atraviesa una transición.
    gnss_rx_ready = 0;
    gnss_ack_waiting = 0;
    gnss_data.fix_valid = 0;
    status = HAL_UART_AbortReceive(&huart1);
    if (status != HAL_OK)
    {
      return -1;
    }
    status = GNSS_SendUbx(0x06, 0x8A, payload, sizeof(payload));
    if (status != 0)
    {
      return -1;
    }
    // Dar tiempo al receptor antes de adaptar la UART local.
    HAL_Delay(GNSS_BAUD_SETTLE_MS);
    status = GNSS_SetLocalBaudrate(baudrate);
    if (status != 0)
    {
      return -1;
    }
    // Una orden confirmada a la nueva velocidad comprueba ambos sentidos de comunicación.
    status = GNSS_SetConfigU8(GNSS_CFG_NAVSPG_DYNMODEL, GNSS_DYNMODEL_AIRBORNE_4G);
    return status;
}

// Busca el receptor a 9600 o 115200 bit/s y aplica GPS + Galileo a 10 Hz,
// Airborne <4g y salida NAV-PVT por UART a 115200 bit/s.
// Devuelve 0 si completa la configuración y -1 si falla; incluye esperas de arranque.
static int32_t GNSS_Configure(void)
{

    // Probar el valor de fábrica y el que puede conservar el receptor si solo se reinicia el STM32.
    const uint32_t baudrates[] = {9600U, GNSS_UART_BAUDRATE};
    int32_t status = -1;

    for (uint32_t baud_index = 0;
        baud_index < sizeof(baudrates) / sizeof(baudrates[0]);
        baud_index++)
    {
        // Reintentar para dar margen al arranque del receptor.
        for (uint32_t attempt = 0; attempt < GNSS_STARTUP_ATTEMPTS; attempt++)
        {
            gnss_rx_ready = (GNSS_SetLocalBaudrate(baudrates[baud_index]) == 0);
            if (gnss_rx_ready == 0)
            {
              return -1;
            }
            status = GNSS_SetConfigU8(GNSS_CFG_NAVSPG_DYNMODEL, GNSS_DYNMODEL_AIRBORNE_4G);
            if (status == 0)
            {
              break;
            }
            if (attempt + 1U < GNSS_STARTUP_ATTEMPTS)
            {
              HAL_Delay(GNSS_STARTUP_RETRY_MS);
            }
        }
        if (status == 0)
        {
          break;
        }
    }

    if (status != 0)
    {
      return -1;
    }

    // Habilitar UBX y desactivar NMEA; detener NAV-PVT mientras se aplican los ajustes.
    status = GNSS_SetConfigU8(GNSS_CFG_UART1OUTPROT_UBX, 1U);
    if (status != 0)
    {
      return -1;
    }

    status = GNSS_SetConfigU8(GNSS_CFG_UART1OUTPROT_NMEA, 0U);
    if (status != 0)
    {
      return -1;
    }

    status = GNSS_SetConfigU8(GNSS_CFG_MSGOUT_NAV_PVT_UART1, 0U);
    if (status != 0)
    {
      return -1;
    }

    // Seleccionar GPS + Galileo. Tras cada ajuste de señales, esperar 500 ms,
    // además del ACK, para que se reinicie el subsistema GNSS.
    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_SBAS_ENA, 0U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_BDS_ENA, 0U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_QZSS_ENA, 0U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_GLO_ENA, 0U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_GPS_L1CA_ENA, 1U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_GPS_ENA, 1U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_GAL_E1_ENA, 1U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    status = GNSS_SetConfigU8(GNSS_CFG_SIGNAL_GAL_ENA, 1U);
    if (status != 0)
    {
      return -1;
    }
    HAL_Delay(GNSS_SIGNAL_SETTLE_MS);

    // Aplicar la velocidad final aunque la búsqueda inicial funcionara a 9600 bit/s.
    status = GNSS_ChangeBaudrate(GNSS_UART_BAUDRATE);
    if (status != 0)
    {
      return -1;
    }

    // Una solución cada 100 ms y un NAV-PVT por solución: salida nominal de 10 Hz.
    status = GNSS_SetConfigU16(GNSS_CFG_RATE_NAV, 1U);
    if (status != 0)
    {
      return -1;
    }

    status = GNSS_SetConfigU16(GNSS_CFG_RATE_MEAS, 100U);
    if (status != 0)
    {
      return -1;
    }

    status = GNSS_SetConfigU8(GNSS_CFG_MSGOUT_NAV_PVT_UART1, 1U);
    if (status != 0)
    {
      return -1;
    }

    return 0;
}

// Procesa navegación, invalida datos antiguos y reintenta la recepción tras un error.
// Solo utiliza el receptor si su configuración inicial terminó correctamente.
static void GNSS_Update(void)
{
    uint32_t now = HAL_GetTick();

    // No aceptar datos de un receptor cuya configuración quedó incompleta.
    if (gnss_configured == 0)
    {
        gnss_data.fix_valid = 0;
        return;
    }

    // Espaciar los intentos de recuperación para no ocupar continuamente el bucle.
    if (gnss_rx_error_pending == 1 || gnss_rx_ready == 0)
    {
      gnss_data.fix_valid = 0;
      if (now - gnss_last_restart_ms >= GNSS_RETRY_PERIOD_MS)
      {
        gnss_last_restart_ms = now;
        gnss_rx_ready = (GNSS_RestartReception() == 0);
      }
      return;
    }
    GNSS_ProcessReceived();
    // Un error o llenado del búfer durante el procesamiento invalida la solución.
    if (gnss_rx_error_pending == 1)
    {
      gnss_rx_ready = 0;
      gnss_data.fix_valid = 0;
      return;
    }
    // Conservar los campos para diagnóstico, pero retirar su validez si dejan de actualizarse.
    if (HAL_GetTick() - gnss_data.reception_time_ms >= GNSS_DATA_TIMEOUT_MS)
    {
      gnss_data.fix_valid = 0;
    }
}

// Configura un canal del ADC y realiza una conversión de 12 bits con espera limitada.
// Devuelve 0 si escribe la lectura en raw y -1 si falla; conserva la salida si hay error.
static int32_t ADC_ReadRaw(uint32_t channel, uint16_t *raw)
{
    ADC_ChannelConfTypeDef config = {0};
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef stop_status;
    uint32_t value = 0;

    if (raw == NULL)
    {
      return -1;
    }
    config.Channel = channel;
    config.Rank = 1;
    // A 25 MHz, 480 ciclos dan 19,2 us de muestreo para las entradas internas.
    config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    status = HAL_ADC_ConfigChannel(&hadc1, &config);
    if (status != HAL_OK)
    {
      return -1;
    }
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK)
    {
      return -1;
    }
    status = HAL_ADC_PollForConversion(&hadc1, ADC_CONVERSION_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      value = HAL_ADC_GetValue(&hadc1);
    }
    // Detener el ADC también si la espera terminó con error.
    stop_status = HAL_ADC_Stop(&hadc1);
    if (status != HAL_OK || stop_status != HAL_OK)
    {
      return -1;
    }
    *raw = (uint16_t)value;
    return 0;
}

// Estima VDDA en voltios con VREFINT y su calibración de fábrica.
// Devuelve 0 si publica el resultado y -1 si falla la lectura o los valores comprobados.
static int32_t ADC_ReadSupplyVoltage(float *vdda_v)
{
    uint16_t raw;
    uint16_t calibration;

    if (vdda_v == NULL)
    {
      return -1;
    }
    if (ADC_ReadRaw(ADC_CHANNEL_VREFINT, &raw) != 0)
    {
      return -1;
    }
    calibration = *VREFINT_CAL_ADDR;
    // Evitar división por cero y rechazar valores nulos o en el máximo digital.
    if (raw == 0 || calibration == 0 ||
        raw >= ADC_MAX_VALUE || calibration >= ADC_MAX_VALUE)
    {
      return -1;
    }
    // La calibración corresponde a 3,3 V; la lectura de VREFINT varía inversamente con VDDA.
    *vdda_v = ((float)VREFINT_CAL_VREF / 1000.0f) * ((float)calibration / raw);
    return 0;
}

// Convierte la lectura de un canal a voltios de la línea, usando VDDA y su divisor.
// Devuelve 0 si publica el resultado y -1 por error, argumento inválido o fondo de escala.
static int32_t ADC_ReadVoltage(uint32_t channel,
                              float vdda_v,
                              float divider_factor,
                              float *voltage_v)
{
    uint16_t raw;

    if (voltage_v == NULL)
    {
      return -1;
    }
    if (vdda_v <= 0 || divider_factor < 1)
    {
      return -1;
    }
    if (ADC_ReadRaw(channel, &raw) != 0)
    {
      return -1;
    }
    // En el máximo digital no se puede distinguir una tensión en el límite de una saturación.
    if (raw >= ADC_MAX_VALUE)
    {
      return -1;
    }
    *voltage_v = (float)raw / ADC_MAX_VALUE * vdda_v * divider_factor;
    return 0;
}

// Calcula la temperatura interna del STM32 en °C con sus dos calibraciones de fábrica.
// Corrige la lectura según VDDA. Devuelve 0 si publica el resultado y -1 si falla.
static int32_t ADC_ReadTemperature(float vdda_v, float *temperature_c)
{
    uint16_t raw;
    uint16_t calibration1;
    uint16_t calibration2;
    float corrected_raw;

    if (temperature_c == NULL || vdda_v <= 0)
    {
      return -1;
    }
    if (ADC_ReadRaw(ADC_CHANNEL_TEMPSENSOR, &raw) != 0)
    {
      return -1;
    }
    calibration1 = *TEMPSENSOR_CAL1_ADDR;
    calibration2 = *TEMPSENSOR_CAL2_ADDR;
    if (raw == 0 || calibration1 == 0 || calibration2 == 0 ||
        raw >= ADC_MAX_VALUE || calibration1 >= ADC_MAX_VALUE || calibration2 >= ADC_MAX_VALUE ||
        calibration1 == calibration2)
    {
      return -1;
    }
    // Llevar la lectura a la tensión de calibración antes de interpolar entre 30 y 110 °C.
    corrected_raw = (float)raw * vdda_v / ((float)TEMPSENSOR_CAL_VREFANALOG / 1000.0f);
    *temperature_c = TEMPSENSOR_CAL1_TEMP
                   + (corrected_raw - calibration1)
                   / ((float)calibration2 - calibration1)
                   * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);
    return 0;
}

// Solicita alimentación cada 100 ms y temperatura cada 1000 ms, según las constantes.
// Publica las tensiones juntas solo si todas se leen; la temperatura se evalúa aparte.
// Conserva valores y marcas de tiempo anteriores si falla una adquisición.
static void ADC_Update(void)
{
    uint32_t now = HAL_GetTick();
    int32_t vdda_status;

    float vdda_v;
    float battery_v;
    float rail1_v;
    float rail2_v;
    float temperature_c;

    if (now - power_last_request_ms < POWER_READ_PERIOD_MS)
    {
      return;
    }
    power_last_request_ms = now;
    vdda_status = ADC_ReadSupplyVoltage(&vdda_v);
    // Publicar el conjunto solo cuando VDDA y las tres líneas se han leído correctamente.
    power_measurement_status = -1;
    if (vdda_status == 0 &&
        ADC_ReadVoltage(ADC_CHANNEL_10, vdda_v, BATTERY_DIVIDER_FACTOR, &battery_v) == 0 &&
        ADC_ReadVoltage(ADC_CHANNEL_11, vdda_v, RAIL1_DIVIDER_FACTOR, &rail1_v) == 0&&
        ADC_ReadVoltage(ADC_CHANNEL_12, vdda_v, RAIL2_DIVIDER_FACTOR, &rail2_v) == 0)
    {
      adc_vdda_v = vdda_v;
      battery_voltage_v = battery_v;
      rail1_voltage_v = rail1_v;
      rail2_voltage_v = rail2_v;
      power_measurement_time_ms = HAL_GetTick();
      power_measurement_status = 1;
    }
    // Reevaluar el tiempo después de las lecturas de tensión.
    // La temperatura puede leerse aunque falle una línea, siempre que VDDA sea válida.
    now = HAL_GetTick();
    if (now - temperature_last_request_ms >= TEMPERATURE_READ_PERIOD_MS)
    {
      temperature_last_request_ms = now;
      temperature_measurement_status = -1;
      if (vdda_status == 0)
      {
        if (ADC_ReadTemperature(vdda_v, &temperature_c) == 0)
        {
          mcu_temperature_c = temperature_c;
          temperature_measurement_time_ms = HAL_GetTick();
          temperature_measurement_status = 1;
        }
      }
    }
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Habilitar la referencia interna y el sensor de temperatura del ADC.
  SET_BIT(ADC->CCR, ADC_CCR_TSVREFE);
  // Esperar a que los circuitos internos se estabilicen.
  HAL_Delay(1);

  // Completar la configuración del GNSS antes de iniciar las medidas inerciales.
  gnss_last_restart_ms = HAL_GetTick();
  gnss_rx_ready = (GNSS_RestartReception() == 0);

  // Configurar el receptor únicamente si se ha iniciado la recepción.
  if (gnss_rx_ready == 1)
  {
      gnss_configured = (GNSS_Configure() == 0);
  }

  // Inicializar la IMU y habilitar su lectura solo si termina correctamente.
  imu_ctx.read_reg = IMU_Read;
  imu_ctx.write_reg = IMU_Write;
  imu_ctx.mdelay = HAL_Delay;
  imu_ctx.handle = &hspi1;

  imu_ready = (IMU_Init() == 0);

  // Inicializar el acelerómetro de alto rango con el mismo criterio.
  highg_ctx.read_reg = HIGHG_Read;
  highg_ctx.write_reg = HIGHG_Write;
  highg_ctx.mdelay = HAL_Delay;
  highg_ctx.handle = &hspi1;

  highg_ready = (HIGHG_Init() == 0);

  // Iniciar los periodos de adquisición al terminar la configuración.
  baro_last_request_ms = HAL_GetTick();
  power_last_request_ms = baro_last_request_ms;
  temperature_last_request_ms = baro_last_request_ms;

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
      // Marcar el instante de lectura solo si se ha obtenido una muestra nueva.
      if (imu_acceleration_status == 1)
      {
        imu_acceleration_time_ms = HAL_GetTick();
      }
      // Se lleva a cabo la lectura de velocidad angular.
      imu_angular_rate_status = IMU_ReadAngularRate(imu_angular_rate_dps);
      // Marcar el instante de lectura solo si se ha obtenido una muestra nueva.
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
      // Marcar el instante de lectura solo si se ha obtenido una muestra nueva.
      if (highg_acceleration_status == 1)
      {
        highg_acceleration_time_ms = HAL_GetTick();
      }
    }

    // Se gestiona la adquisición de medidas del barómetro.
    BARO_Update();

    // Se procesan los datos del GNSS y se supervisa la recepción.
    GNSS_Update();

    // Actualizar las medidas de alimentación y temperatura.
    ADC_Update();
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
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
