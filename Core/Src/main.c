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
#include <stdio.h>
#include "asm330lhhx.h"
#include "ms5611.h"
#include "mmc5983.h"
#include "adxl375.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Default: exclusive ADXL375 bring-up. No other sensor SPI traffic is allowed.
   Without SDI gating, restoring shared traffic requires a separate bus review. */
#ifndef ADXL375_EXCLUSIVE_TEST
#define ADXL375_EXCLUSIVE_TEST 1
#endif

/* Temporary bus isolation: set to 0 to restore ASM330/MS5611 traffic. */
#ifndef MMC5983_ONLY_TEST
#define MMC5983_ONLY_TEST 1
#endif

/* Temporary MMC5983 bring-up test: set to 0 to remove all diagnostic traffic.
   May also be overridden with -DMMC5983_STARTUP_DIAGNOSTIC=0. */
#ifndef MMC5983_STARTUP_DIAGNOSTIC
#define MMC5983_STARTUP_DIAGNOSTIC 1
#endif

// ADXL375 Defines
#define ADXL_SPI_READ	0x80
#define ADXL_REG_ID		0x00
#define ADXL_ACCEL_REG 0x32
#define ADXL_MULTI_BYTE_READ 0x40

// MS5611 Defines
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

XSPI_HandleTypeDef hxspi1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static ASM330LHHX_t asm330;
static uint8_t asm330_ready = 0U;
static MS5611_t ms5611;
/* User-selectable MS5611 settings. Copied to ms5611.config at startup. */
static MS5611_Config_t ms5611_config = {
    .pressure_osr = MS5611_OSR_4096,
    .temperature_osr = MS5611_OSR_4096,
    .spi_timeout_ms = 100U,
    .second_order_compensation = true
};
static MMC5983_t magnetometer;
static MMC5983_Config_t mmc5983_config = {
    .bandwidth = MMC5983_BW_100HZ,
    .automatic_set_reset = true,
    .spi_timeout_ms = 100U,
    .measurement_timeout_ms = 50U
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file , char *ptr, int len){
	for(int i=0; i < len; i++){
		ITM_SendChar(*ptr++);
	}
	return len;
}

// NOTE: could be half precision floats to save space
typedef struct{
	float x;
	float y;
	float z;

} accel_data;

typedef struct{
	float x;
	float y;
	float z;
} gyro_data;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM2) // Check if the interrupt is from TIM2
    {
        // Toggle the D1 LED
        HAL_GPIO_TogglePin(D1_LED_GPIO_Port, D1_LED_Pin);
    }
}

uint8_t adxl_read_id(void){
    // txData[0]: Read bit (0x80) | Register address (0x00)
    // txData[1]: Dummy byte to keep the clock running while receiving
    uint8_t txData[2] = {0x80, 0x00};
    uint8_t rxData[2] = {0, 0};

    // set cs low
    HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);

    // transmit address and receive data simultaneously over 2 bytes
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);

    // set cs high
    HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);

    // rxData[0] is garbage received during the address transmission
    // rxData[1] is the actual device ID received during the dummy byte transmission
    return rxData[1];
}

uint8_t asm330lhhx_read_id(void){
    // txData[0]: Read bit (0x80) | WHO_AM_I Register (0x0F)
    uint8_t txData[2] = {0x8F, 0x00};
    uint8_t rxData[2] = {0, 0};

    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, txData, rxData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_SET);

    return rxData[1];
}


/* ADXL375 acquisition now uses adxl375.c (nominal 49 mg/LSB). */

/* Fixed-point printing retains two decimal places, including -0.xx,
   without enabling floating-point printf in newlib-nano. */
static void print_hundredths(int32_t value)
{
    unsigned long magnitude = (unsigned long)(value < 0 ? -(int64_t)value : value);
    printf("%s%lu.%02lu", value < 0 ? "-" : "", magnitude / 100UL, magnitude % 100UL);
}

static void ms5611_calculation_self_test(void)
{
    /* Published datasheet example (page 8); this tests math, not the wiring. */
    const uint16_t example_prom[8] = { 0U, 40127U, 36924U, 23317U,
                                      23282U, 33464U, 28312U, 0U };
    MS5611_Data_t example;
    MS5611_Status_t status = MS5611_Compensate(example_prom, 9085466U, 8569150U,
                                              true, &example);
    printf("MS5611 datasheet calculation self-test: %s (expected 1000.09 mbar, 20.07 C)\r\n",
           status == MS5611_OK && example.pressure_centi_mbar == 100009 &&
           example.temperature_centi_c == 2007 ? "PASS" : "FAIL");
}


#if MMC5983_STARTUP_DIAGNOSTIC
/* BEGIN removable MMC5983 startup diagnostic */
static void mmc5983_print_id_probe(const char *phase, unsigned int attempt)
{
    uint8_t tx[2] = { 0xAFU, 0x00U };
    uint8_t rx[2] = { 0xA5U, 0xA5U }; // Sentinel distinguishes failed transfers.
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 100U);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    printf("MMC5983 diagnostic %s #%u: TX=AF 00 RX=%02X %02X, HAL=%u%s\r\n",
           phase, attempt, (unsigned int)rx[0], (unsigned int)rx[1],
           (unsigned int)status,
           status == HAL_OK && rx[1] == MMC5983_PRODUCT_ID ? " ID OK" : " ID NOT VERIFIED");
}

static void mmc5983_startup_diagnostic(void)
{
    /* All devices must be deselected before changing the shared SPI clock. */
    HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    SPI_InitTypeDef saved_init = hspi2.Init;
    __HAL_SPI_DISABLE(&hspi2);
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
      Error_Handler();
    }
    /* Establish idle-high SCK before asserting CS. No CSTART is issued here.
       KeepIOState retains pin control when HAL closes each transaction. */
    __HAL_SPI_ENABLE(&hspi2);
    printf("MMC5983 diagnostic: mode 3, divider /256, KeepIOState enabled, 8-bit frames\r\n");
    printf("MMC5983 diagnostic: all CS started HIGH; expected RX[1]=30\r\n");
    HAL_Delay(20U);
    for (unsigned int attempt = 1U; attempt <= 3U; ++attempt) {
      mmc5983_print_id_probe("PRE-reset", attempt);
      HAL_Delay(5U);
    }

    uint8_t reset_tx[2] = { 0x0AU, 0x80U }; // CTRL1.SW_RST
    uint8_t reset_rx[2] = { 0xA5U, 0xA5U };
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef reset_status = HAL_SPI_TransmitReceive(&hspi2, reset_tx, reset_rx, 2U, 100U);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    printf("MMC5983 diagnostic reset: TX=0A 80 RX=%02X %02X, HAL=%u\r\n",
           (unsigned int)reset_rx[0], (unsigned int)reset_rx[1], (unsigned int)reset_status);
    HAL_Delay(20U); // Datasheet reset startup is 10 ms; allow margin.
    for (unsigned int attempt = 1U; attempt <= 3U; ++attempt) {
      mmc5983_print_id_probe("POST-reset", attempt);
      HAL_Delay(5U);
    }

    /* Exercise the real library under the test settings as well. Init performs
       another reset and checks OTP, so label this separately from raw probes. */
    MMC5983_t test_device;
    MMC5983_AssignSPI(&test_device, &hspi2, MAG_CS_GPIO_Port, MAG_CS_Pin);
    test_device.config = mmc5983_config;
    MMC5983_Status_t init_status = MMC5983_Init(&test_device);
    printf("MMC5983 diagnostic library init: ID=%02X status=%u HAL=%u sensor_status=%02X\r\n",
           (unsigned int)test_device.product_id, (unsigned int)init_status,
           (unsigned int)test_device.last_hal_status, (unsigned int)test_device.last_sensor_status);

    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    __HAL_SPI_DISABLE(&hspi2);
    hspi2.Init = saved_init;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
      Error_Handler();
    }
    printf("MMC5983 diagnostic complete; original SPI mode, clock and IO settings restored\r\n");
}
/* END removable MMC5983 startup diagnostic */
#endif

#if ADXL375_EXCLUSIVE_TEST
/* Removable exclusive bring-up loop. No other SPI2 sensor is accessed. */
static void adxl375_exclusive_test(void)
{
    HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
    __HAL_SPI_DISABLE(&hspi2);
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
    __HAL_SPI_ENABLE(&hspi2); // Establish idle-high SCK before selecting ADXL.
    HAL_Delay(20U);
    printf("ADXL375 EXCLUSIVE test: mode 3, /32 (~156 kHz), all other sensor traffic disabled\r\n");
    printf("No SDI OR gate fitted: this test does not validate shared-bus safety.\r\n");
    ADXL375_t adxl;
    ADXL375_AssignSPI(&adxl, &hspi2, ADXL_CS_GPIO_Port, ADXL_CS_Pin);
    ADXL375_Status_t init = ADXL375_Init(&adxl);
    printf("ADXL375 init=%u HAL=%u (0=OK, 3=wrong ID, 4=config mismatch)\r\n",
           (unsigned int)init, (unsigned int)adxl.last_hal_status);
    for (;;) {
      ADXL375_Registers_t registers;
      ADXL375_Status_t status = ADXL375_ReadStatus(&adxl, &registers);
      if (status == ADXL375_OK) {
        printf("ADXL375 ID=%02X (expected E5), BW=%02X POWER=%02X FORMAT=%02X INT=%02X FIFO=%02X\r\n",
               (unsigned int)registers.device_id, (unsigned int)registers.bandwidth_rate,
               (unsigned int)registers.power_control, (unsigned int)registers.data_format,
               (unsigned int)registers.interrupt_source, (unsigned int)registers.fifo_status);
      } else {
        printf("ADXL375 status read failed: status=%u HAL=%u\r\n",
               (unsigned int)status, (unsigned int)adxl.last_hal_status);
      }
      if (adxl.initialized) {
        ADXL375_Data_t data;
        status = ADXL375_Read(&adxl, &data);
        if (status == ADXL375_OK) {
          printf("ADXL375 fresh sample [mg]: X=%ld Y=%ld Z=%ld (raw=%d,%d,%d)\r\n",
                 (long)data.x_mg, (long)data.y_mg, (long)data.z_mg,
                 (int)data.x, (int)data.y, (int)data.z);
        } else {
          printf("ADXL375 sample failed: status=%u HAL=%u\r\n",
                 (unsigned int)status, (unsigned int)adxl.last_hal_status);
        }
      }
      HAL_Delay(500U);
    }
}
#endif

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
  MX_OCTOSPI1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2); // Start the timer with interrupt

  /* Keep all other devices off the shared SPI bus. */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  ASM330LHHX_AssignSPI(&asm330, &hspi2, ASM330_CS_GPIO_Port, ASM330_CS_Pin);
  HAL_Delay(20U); // Allow sensor power-on reset to complete.
  setvbuf(stdout, NULL, _IONBF, 0); // Send printf output directly to ITM port 0.

#if ADXL375_EXCLUSIVE_TEST
  adxl375_exclusive_test(); // Does not return: no other sensor transactions.
#endif

#if MMC5983_STARTUP_DIAGNOSTIC
  /* Run before any sensor SPI traffic; power-cycle for a true cold-start test. */
  mmc5983_startup_diagnostic();
#endif

  if (!MMC5983_ONLY_TEST) {
  uint8_t asm_id = 0U;
  HAL_StatusTypeDef asm_status = ASM330LHHX_ReadDeviceID(&asm330, &asm_id);
  printf("ASM330LHHX WHO_AM_I: 0x%02X (expected 0x%02X), HAL status: %u\r\n",
         (unsigned int)asm_id, (unsigned int)ASM330LHHX_WHO_AM_I_VALUE,
         (unsigned int)asm_status);

  if ((asm_status == HAL_OK) && (asm_id == ASM330LHHX_WHO_AM_I_VALUE)) {
    asm_status = ASM330LHHX_Init(&asm330,
                                ASM330LHHX_ODR_104HZ, ASM330LHHX_XL_FS_2G,
                                ASM330LHHX_ODR_104HZ, ASM330LHHX_G_FS_250DPS);
    if (asm_status == HAL_OK) {
      asm330_ready = 1U;
      printf("ASM330LHHX initialized: 104 Hz, +/-2 g, +/-250 dps\r\n");
    } else {
      printf("ASM330LHHX initialization failed, HAL status: %u\r\n",
             (unsigned int)asm_status);
    }
  } else {
    printf("ASM330LHHX not detected; check wiring and SPI configuration.\r\n");
  }

  ms5611_calculation_self_test();
  MS5611_AssignSPI(&ms5611, &hspi2, MS5611_CS_GPIO_Port, MS5611_CS_Pin);
  ms5611.config = ms5611_config;
  MS5611_Status_t ms_status = MS5611_Init(&ms5611);
  if (ms_status == MS5611_OK) {
    printf("MS5611 initialized, PROM CRC OK; pressure OSR=%u, temperature OSR=%u\r\n",
           256U << ((unsigned int)ms5611.config.pressure_osr / 2U),
           256U << ((unsigned int)ms5611.config.temperature_osr / 2U));
    for (unsigned int i = 1U; i <= 6U; ++i) {
      printf("MS5611 C%u=%u\r\n", i, (unsigned int)ms5611.prom[i]);
    }
  } else {
    printf("MS5611 init failed: status=%u, HAL=%u (check PS low, CS and wiring)\r\n",
           (unsigned int)ms_status, (unsigned int)ms5611.last_hal_status);
  }

  } else {
    printf("MMC5983-only test: ASM330 and MS5611 initialization/read traffic disabled\r\n");
  }

  MMC5983_AssignSPI(&magnetometer, &hspi2, MAG_CS_GPIO_Port, MAG_CS_Pin);
  magnetometer.config = mmc5983_config;
  /* Use the existing SPI2 mode-0 configuration for MMC5983 too. */
  MMC5983_Status_t mag_init_status = MMC5983_Init(&magnetometer);
  printf("MMC5983 product ID: 0x%02X (expected 0x%02X), init status=%u, HAL=%u\r\n",
         (unsigned int)magnetometer.product_id, (unsigned int)MMC5983_PRODUCT_ID,
         (unsigned int)mag_init_status, (unsigned int)magnetometer.last_hal_status);
  if (mag_init_status == MMC5983_OK) {
    printf("MMC5983 initialized: single-shot, BW=%u, auto SET/RESET=%u, 18-bit, uT\r\n",
           (unsigned int)magnetometer.config.bandwidth,
           (unsigned int)magnetometer.config.automatic_set_reset);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(500U);
    if (!MMC5983_ONLY_TEST && asm330_ready) {
      ASM330LHHX_FloatData_t accel_mg;
      ASM330LHHX_FloatData_t gyro_mdps;
      HAL_StatusTypeDef accel_status = ASM330LHHX_ReadAccelMg(&asm330, &accel_mg);
      HAL_StatusTypeDef gyro_status = ASM330LHHX_ReadGyroMdps(&asm330, &gyro_mdps);

      /* Integer output avoids requiring newlib-nano float printf support.
         1000 mg = 1 g; 1000 mdps = 1 degree/second. */
      if (accel_status == HAL_OK) {
        printf("Accel [mg]: X=%ld Y=%ld Z=%ld\r\n",
               (long)accel_mg.x, (long)accel_mg.y, (long)accel_mg.z);
      } else {
        printf("Accel read failed, HAL status: %u\r\n", (unsigned int)accel_status);
      }
      if (gyro_status == HAL_OK) {
        printf("Gyro [mdps]: X=%ld Y=%ld Z=%ld\r\n",
               (long)gyro_mdps.x, (long)gyro_mdps.y, (long)gyro_mdps.z);
      } else {
        printf("Gyro read failed, HAL status: %u\r\n", (unsigned int)gyro_status);
      }
    }
    if (!MMC5983_ONLY_TEST && ms5611.initialized) {
      MS5611_Data_t barometer;
      MS5611_Status_t status = MS5611_Read(&ms5611, &barometer);
      if (status == MS5611_OK) {
        printf("MS5611 pressure: ");
        print_hundredths(barometer.pressure_centi_mbar);
        printf(" mbar, temperature: ");
        print_hundredths(barometer.temperature_centi_c);
        printf(" C (D1=%lu, D2=%lu)\r\n",
               (unsigned long)barometer.raw_pressure,
               (unsigned long)barometer.raw_temperature);
      } else {
        printf("MS5611 read failed: status=%u, HAL=%u\r\n",
               (unsigned int)status, (unsigned int)ms5611.last_hal_status);
      }
    }
    if (magnetometer.initialized) {
      MMC5983_MagneticData_t field;
      MMC5983_Temperature_t mag_temperature;
      MMC5983_Status_t field_status = MMC5983_ReadMagnetic(&magnetometer, &field);
      HAL_StatusTypeDef field_hal_status = magnetometer.last_hal_status;
      MMC5983_Status_t temp_status = MMC5983_ReadTemperature(&magnetometer, &mag_temperature);
      if (field_status == MMC5983_OK) {
        printf("MMC5983 field [uT]: X=");
        print_hundredths(field.x_centi_ut);
        printf(" Y=");
        print_hundredths(field.y_centi_ut);
        printf(" Z=");
        print_hundredths(field.z_centi_ut);
        printf(" (raw=%lu,%lu,%lu)%s\r\n",
               (unsigned long)field.raw_x, (unsigned long)field.raw_y,
               (unsigned long)field.raw_z, field.at_rail ? " WARNING: axis at rail" : "");
      } else {
        printf("MMC5983 field read failed: status=%u, HAL=%u\r\n",
               (unsigned int)field_status, (unsigned int)field_hal_status);
      }
      if (temp_status == MMC5983_OK) {
        printf("MMC5983 temperature: ");
        print_hundredths(mag_temperature.temperature_centi_c);
        printf(" C (raw=%u, approximate 0.8 C/LSB)\r\n", (unsigned int)mag_temperature.raw);
      } else {
        printf("MMC5983 temperature read failed: status=%u, HAL=%u\r\n",
               (unsigned int)temp_status, (unsigned int)magnetometer.last_hal_status);
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  *  Warning : Only applied when the LSE is disabled.
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 100;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hxspi1.Instance = OCTOSPI1;
  hxspi1.Init.FifoThresholdByte = 1;
  hxspi1.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi1.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_16B;
  hxspi1.Init.ChipSelectHighTimeCycle = 1;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi1.Init.ClockPrescaler = 0;
  hxspi1.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi1.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hxspi1.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi1.Init.DelayBlockBypass = HAL_XSPI_DELAY_BLOCK_BYPASS;
  hxspi1.Init.Refresh = 0;
  if (HAL_XSPI_Init(&hxspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_BYPASS;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* Use SPI mode 0. /32 keeps SCK below the 10 MHz limit even with
     a 250 MHz peripheral clock. */
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_LED_Pin|D2_LED_Pin, GPIO_PIN_RESET);

  /* Preload CS high before enabling output drivers; keep sensors deselected
     throughout peripheral initialization. Also configured in Melchior.ioc. */
  HAL_GPIO_WritePin(GPIOA, MAG_CS_Pin|ASM330_CS_Pin|MS5611_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : D1_LED_Pin D2_LED_Pin MAG_CS_Pin ASM330_CS_Pin
                           MS5611_CS_Pin */
  GPIO_InitStruct.Pin = D1_LED_Pin|D2_LED_Pin|MAG_CS_Pin|ASM330_CS_Pin
                          |MS5611_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MAG_INT_Pin ASM330_INT2_Pin ASM330_INT1_Pin */
  GPIO_InitStruct.Pin = MAG_INT_Pin|ASM330_INT2_Pin|ASM330_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADXL_INT2_Pin ADXL_INT1_Pin */
  GPIO_InitStruct.Pin = ADXL_INT2_Pin|ADXL_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADXL_CS_Pin */
  GPIO_InitStruct.Pin = ADXL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
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
