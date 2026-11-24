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
#include <math.h>
#include "asm330lhhx.h"
#include "ms5611.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Temporary mode selector: 0 = normal, 1 = flash capture, 2 = MMC5983 capture. */
static const unsigned int MODE = 0U;

#define ALPHA 0.05f /* Accelerometer correction weight per update. */
#define LOOP_PERIOD_MS 20U
#define BARO_PERIOD_MS 100U
#define MAX_ATTITUDE_DT_S 0.25f
#define TWO_PI_RAD 6.28318530718f
#define RAD_TO_DEG 57.2957795131f
#define MDPS_TO_RPS 0.0000174532925199f

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
static const ASM330LHHX_Config_t asm330_config = {
		.accel_odr = ASM330LHHX_ODR_104,
		.accel_fs = ASM330LHHX_XL_FS_2G,
		.gyro_odr = ASM330LHHX_ODR_104,
		.gyro_fs = ASM330LHHX_G_FS_1000DPS,
		.spi_timeout_ms = 100U
};

static uint8_t asm330_ready = 0U;
static MS5611_t ms5611;
/* User-selectable settings; Init copies each configuration. */
static const MS5611_Config_t ms5611_config = {
    .pressure_osr = MS5611_OSR_4096,
    .temperature_osr = MS5611_OSR_4096,
    .spi_timeout_ms = 100U,
    .second_order_compensation = true
};

static uint32_t past_tick = 0U; /* Last IMU attempt; rate-limits failed reads too. */
static uint32_t attitude_tick = 0U; /* Last valid attitude sample. */
static bool attitude_initialized = false;
static float thetaHat_rad = 0.0f;
static float phiHat_rad = 0.0f;

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
int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; ++i) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

/* Fixed-point printing retains two decimal places, including -0.xx,
   without enabling floating-point printf in newlib-nano. */
static void print_hundredths(int32_t value)
{
    unsigned long magnitude = (unsigned long)(value < 0 ? -(int64_t)value : value);
    printf("%s%lu.%02lu", value < 0 ? "-" : "", magnitude / 100UL, magnitude % 100UL);
}

/* Removable scope test: ONLY 9F + three response bytes, once per 500 ms.
   OCTOSPI controls CS. No reset, erase, writes, or sensor transactions. */
static void flash_id_capture_loop(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect = HAL_XSPI_SELECT_IO_3_0;
    cmd.Instruction = 0x9FU;
    cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength = 3U; // 32 clocks total: 8 command + 24 response.

    printf("Flash capture: repeating 9F every 500 ms; trigger on PB10/CS falling\r\n");
    for (;;) {
        HAL_Delay(500U);
        uint8_t id[3] = {0};
        HAL_StatusTypeDef status = HAL_XSPI_Command(&hxspi1, &cmd, 100U);
        if (status == HAL_OK) {
            status = HAL_XSPI_Receive(&hxspi1, id, 100U);
        }
        printf("Flash capture ID=%02X %02X %02X (expected EF 40 18), HAL=%u\r\n",
               (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2],
               (unsigned int)status);
    }
}

/* Startup-only communication check: no program/erase or status writes.
   Recovery reset can interrupt an existing program/erase: do not call while
   logging to flash. This does not test storage integrity or quad IO. */
static bool flash_check(void)
{
    XSPI_RegularCmdTypeDef cmd = {0};
    uint8_t id[3] = {0}, alternate_id[2] = {0}, sr1 = 0U;
    bool jedec_ok = false;
    const char *step = "initial JEDEC ID";
    const char *phases[] = { "initial", "after wake", "after reset" };

    cmd.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd.IOSelect = HAL_XSPI_SELECT_IO_3_0;
    cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    /* Zero initialization selects SDR, no dummy cycles, and no DQS. */

    for (unsigned int attempt = 0U; attempt < 3U; ++attempt) {
        if (attempt != 0U) {
            cmd.DataMode = HAL_XSPI_DATA_NONE; // Command only; CS released by HAL.
            cmd.DataLength = 1U; // Ignored when DataMode is NONE.
            if (attempt == 1U) {
                step = "release power-down (AB)";
                cmd.Instruction = 0xABU;
                if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK) goto transfer_failed;
                HAL_Delay(1U);
            } else {
                step = "enable reset (66)";
                cmd.Instruction = 0x66U;
                if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK) goto transfer_failed;
                HAL_Delay(1U); // Separate transactions, with CS high between them.
                step = "reset device (99)";
                cmd.Instruction = 0x99U;
                if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK) goto transfer_failed;
                HAL_Delay(1U);
            }
            printf("Flash recovery: %s sent\r\n", attempt == 1U ? "AB" : "66, 99");
        }

        step = phases[attempt];
        cmd.Instruction = 0x9FU;
        cmd.DataMode = HAL_XSPI_DATA_1_LINE;
        cmd.DataLength = sizeof(id);
        if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK ||
            HAL_XSPI_Receive(&hxspi1, id, 100U) != HAL_OK) goto transfer_failed;
        jedec_ok = id[0] == 0xEFU && id[1] == 0x40U && id[2] == 0x18U;
        printf("Flash JEDEC ID (%s): %02X %02X %02X (expected EF 40 18)\r\n",
               phases[attempt], (unsigned int)id[0], (unsigned int)id[1], (unsigned int)id[2]);
        if (jedec_ok) break; // Do not wake/reset a device already responding correctly.
    }

    step = "alternate ID (90)";
    cmd.Instruction = 0x90U;
    cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
    cmd.Address = 0U;
    cmd.DataLength = sizeof(alternate_id);
    if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK ||
        HAL_XSPI_Receive(&hxspi1, alternate_id, 100U) != HAL_OK) goto transfer_failed;
    printf("Flash alternate ID: %02X %02X (expected EF 17)\r\n",
           (unsigned int)alternate_id[0], (unsigned int)alternate_id[1]);

    step = "status register (05)";
    cmd.Instruction = 0x05U;
    cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd.DataLength = 1U;
    if (HAL_XSPI_Command(&hxspi1, &cmd, 100U) != HAL_OK ||
        HAL_XSPI_Receive(&hxspi1, &sr1, 100U) != HAL_OK) goto transfer_failed;
    bool ok = jedec_ok && alternate_id[0] == 0xEFU && alternate_id[1] == 0x17U && !(sr1 & 1U);
    printf("Flash SR1=%02X BUSY=%u; communication check %s\r\n",
           (unsigned int)sr1, (unsigned int)(sr1 & 1U), ok ? "PASS" : "FAIL");
    return ok;

transfer_failed:
    printf("Flash check FAIL: transfer failed at %s, HAL error=0x%08lX\r\n",
           step, (unsigned long)HAL_XSPI_GetError(&hxspi1));
    return false;
}

/* Temporary MMC5983 connection probe: remove this function and its call later.
   Uses existing SPI2 settings (mode 0); datasheet timing describes mode 3.
   No reset, configuration writes, or measurement commands are issued. */
static bool mmc5983_check(void)
{
    uint8_t tx[2] = { 0xAFU, 0x00U }; // Read bit | product-ID register 0x2F.
    uint8_t rx[2] = { 0U, 0U };

    HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ASM330_CS_GPIO_Port, ASM330_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 100U);
    HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

    bool ok = status == HAL_OK && rx[1] == 0x30U;
    printf("MMC5983 ID=%02X (expected 30), RX=%02X %02X, HAL=%u: %s\r\n",
           (unsigned int)rx[1], (unsigned int)rx[0], (unsigned int)rx[1],
           (unsigned int)status, ok ? "PASS" : "FAIL");
    return ok;
}

/* Removable scope test: ONLY product-ID reads, once per 500 ms.
   Uses the existing SPI2 settings; all other shared-bus CS pins stay high. */
static void mmc5983_id_capture_loop(void)
{
    printf("MMC5983 capture: repeating AF 00 every 500 ms; trigger on MAG_CS falling\r\n");
    for (;;) {
        HAL_Delay(500U);
        (void)mmc5983_check();
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
  MX_OCTOSPI1_Init();
  MX_SPI2_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Keep all other devices off the shared SPI bus. */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MS5611_CS_GPIO_Port, MS5611_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
  ASM330LHHX_AssignSPI(&asm330, &hspi2, ASM330_CS_GPIO_Port, ASM330_CS_Pin);
  HAL_Delay(20U); // Allow sensor power-on reset to complete.
  setvbuf(stdout, NULL, _IONBF, 0); // Send printf output directly to ITM port 0.
  if (MODE == 1U) {
    flash_id_capture_loop(); // Does not return: isolates flash ID traffic.
  } else if (MODE == 2U) {
    mmc5983_id_capture_loop(); // Does not return: isolates MMC5983 ID traffic.
  }
  (void)flash_check();
  (void)mmc5983_check(); // Temporary connection probe.

  uint8_t asm_id = 0U;
  HAL_StatusTypeDef asm_status = ASM330LHHX_ReadDeviceID(&asm330, &asm_id);
  if ((asm_status == HAL_OK) && (asm_id == ASM330LHHX_WHO_AM_I_VALUE)) {
    HAL_StatusTypeDef asm_init_status = ASM330LHHX_Init(&asm330, &asm330_config);
    if (asm_init_status == HAL_OK) {
      asm330_ready = 1U;
      printf("ASM330LHHX initialized\r\n");
    } else {
      printf("ASM330LHHX initialization failed, HAL status: %u\r\n",
             (unsigned int)asm_status);
    }
  } else {
    printf("ASM330LHHX not detected; check wiring and SPI configuration.\r\n");
  }

  MS5611_AssignSPI(&ms5611, &hspi2, MS5611_CS_GPIO_Port, MS5611_CS_Pin);
  MS5611_Status_t ms_status = MS5611_Init(&ms5611, &ms5611_config);
  if (ms_status == MS5611_OK) {
    printf("MS5611 initialized\r\n");
  } else {
    printf("MS5611 init failed: status=%u, HAL=%u (check PS low, CS and wiring)\r\n",
           (unsigned int)ms_status, (unsigned int)ms5611.last_hal_status);
  }

  printf("time_ms,roll_deg,pitch_deg\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (asm330_ready && ((HAL_GetTick() - past_tick) >= LOOP_PERIOD_MS)) {
      past_tick = HAL_GetTick();
      ASM330LHHX_FloatData_t accel, gyro;
      HAL_StatusTypeDef accel_status = ASM330LHHX_ReadAccelMg(&asm330, &accel);
      HAL_StatusTypeDef gyro_status = ASM330LHHX_ReadGyroMdps(&asm330, &gyro);
      if (accel_status == HAL_OK && gyro_status == HAL_OK) {
        /* Application axis remap: X <- sensor Y, Y <- sensor X; Z unchanged.
           This literal swap is a reflection, not a right-handed mounting rotation. */
        // float sensor_accel_x = accel.x;
        // accel.x = accel.y;
        // accel.y = sensor_accel_x;
        // float sensor_gyro_x = gyro.x;
        // gyro.x = gyro.y;
        // gyro.y = sensor_gyro_x;

        uint32_t now = HAL_GetTick();
        float dt = (now - attitude_tick) * 0.001f;
        float accel_norm_sq = accel.x * accel.x + accel.y * accel.y + accel.z * accel.z;

        /* Convention: level = +Z gravity reading; roll about X, pitch about Y.
           Remap BOTH accel and gyro together if the mounting differs.
           Ratios use mg directly; reject zero/non-finite samples. */
        if (isfinite(accel_norm_sq) && accel_norm_sq > 1.0f &&
            isfinite(gyro.x) && isfinite(gyro.y) && isfinite(gyro.z)) {
          float phiHat_acc_rad = atan2f(accel.y, accel.z);
          float thetaHat_acc_rad =
              atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));

          if (!attitude_initialized || dt > MAX_ATTITUDE_DT_S ||
              fabsf(cosf(thetaHat_rad)) < 0.05f) {
            /* Seed at startup/after a long gap. Near pitch +/-90 degrees,
               fall back to accelerometer tilt rather than singular gyro rates.
               This is not a valid full-attitude solution near the singularity. */
            phiHat_rad = phiHat_acc_rad;
            thetaHat_rad = thetaHat_acc_rad;
            attitude_initialized = true;
          } else {
            float p_rps = gyro.x * MDPS_TO_RPS;
            float q_rps = gyro.y * MDPS_TO_RPS;
            float r_rps = gyro.z * MDPS_TO_RPS;
            float sin_phi = sinf(phiHat_rad);
            float cos_phi = cosf(phiHat_rad);

            /* Both derivatives use the previous estimate. */
            float phi_dot = p_rps + tanf(thetaHat_rad) *
                (q_rps * sin_phi + r_rps * cos_phi);
            float theta_dot = q_rps * cos_phi - r_rps * sin_phi;
            float phi_pred = phiHat_rad + phi_dot * dt;
            float theta_pred = thetaHat_rad + theta_dot * dt;

            /* Shortest angular correction avoids averaging +pi and -pi to 0. */
            phiHat_rad = remainderf(phi_pred + ALPHA *
                remainderf(phiHat_acc_rad - phi_pred, TWO_PI_RAD), TWO_PI_RAD);
            thetaHat_rad = (1.0f - ALPHA) * theta_pred + ALPHA * thetaHat_acc_rad;
          }
          attitude_tick = now;

          /* CSV via the existing printf -> ITM/SWV output, without float printf. */
          printf("%lu,", (unsigned long)now);
          print_hundredths((int32_t)lroundf(phiHat_rad * RAD_TO_DEG * 100.0f));
          printf(",");
          print_hundredths((int32_t)lroundf(thetaHat_rad * RAD_TO_DEG * 100.0f));
          printf("\r\n");
        }
      } else {
        printf("IMU Read Fail: Accel HAL=%u, Gyro HAL=%u\r\n", (unsigned int)accel_status, (unsigned int)gyro_status);
      }
    }
    /*
    if (ms5611.initialized && ((HAL_GetTick() - baro_tick) >= BARO_PERIOD_MS)) {
      baro_tick = HAL_GetTick();
      MS5611_Data_t data;
      MS5611_Status_t status = MS5611_Read(&ms5611, &data);
      if (status == MS5611_OK) {
        printf("MS5611 pressure: ");
        print_hundredths(data.pressure_centi_mbar);
        printf(" mbar, temperature: ");
        print_hundredths(data.temperature_centi_c);
        printf(" C\r\n");
      } else {
        printf("MS5611 read failed: status=%u HAL=%u\r\n",
               (unsigned int)status, (unsigned int)ms5611.last_hal_status);
      }
    }
    */


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
  hxspi1.Init.MemorySize = HAL_XSPI_SIZE_128MB;
  hxspi1.Init.ChipSelectHighTimeCycle = 8;
  hxspi1.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi1.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi1.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi1.Init.ClockPrescaler = 16;
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
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MAG_CS_Pin|ASM330_CS_Pin|MS5611_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
