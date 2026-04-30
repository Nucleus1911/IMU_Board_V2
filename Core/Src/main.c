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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// RS-485 Protocol Constants
#define ADCS_BUS_START_BYTE        0xAA
#define IMUCUBE_MODULE_ID          0x04
#define ADCS_BUS_BROADCAST_ID      0xFF

// Commands (receive from ADCS master)
#define CMD_PING                   0x01
#define CMD_GET_STATUS             0x02
#define CMD_GET_TELEMETRY          0x03
#define CMD_RESET                  0x04

// Responses (send to ADCS master)
#define TLM_ACK                    0x80
#define TLM_NACK                   0x81
#define TLM_IMUCUBE_ALL_SENSORS    0x90
#define TLM_IMUCUBE_STATUS         0x91

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// I2C pin check
volatile uint8_t scl_state = 0;
volatile uint8_t sda_state = 0;

// I2C scan variables
volatile uint8_t device_count = 0;
volatile uint8_t found_addr[10];
volatile HAL_StatusTypeDef result;

// RM3100 BIST variables
#define RM3100_ADDR    (0x20 << 1)  // 7-bit addr 0x20, shifted for HAL
#define RM3100_BIST    0x33         // BIST register
#define RM3100_POLL    0x00         // POLL register
#define RM3100_MX      0x24         // Measurement X (3 bytes: 0x24, 0x25, 0x26)
volatile uint8_t rm3100_bist_result = 0;
volatile uint8_t rm3100_bist_pass   = 0;  // 1 = PASS, 0 = FAIL

// RM3100 measurement data
volatile uint8_t rm3100_raw[9];            // 9 bytes: X(3) + Y(3) + Z(3)
volatile int32_t rm3100_x = 0;
volatile int32_t rm3100_y = 0;
volatile int32_t rm3100_z = 0;

// IIS2MDC variables
#define IIS2MDC_ADDR     (0x1E << 1)
#define IIS2MDC_WHO_AM_I 0x4F
#define IIS2MDC_CFG_A    0x60
#define IIS2MDC_CFG_C    0x62
#define IIS2MDC_STATUS   0x67
#define IIS2MDC_OUT_X_L  0x68
volatile uint8_t iis2mdc_who = 0;
volatile uint8_t iis2mdc_status = 0;
volatile uint8_t iis2mdc_ok = 0;
volatile uint8_t iis2mdc_raw[6];
volatile int16_t iis2mdc_x = 0;
volatile int16_t iis2mdc_y = 0;
volatile int16_t iis2mdc_z = 0;

// ICM-45686 variables
#define ICM45686_ADDR       (0x68 << 1)
#define ICM45686_WHOAMI     0x72
#define ICM45686_PWR_MGMT0  0x1F
#define ICM45686_INT_STATUS 0x2D
#define ICM45686_STATUS     0x39
#define ICM45686_ACCEL_X1   0x00
#define ICM45686_GYRO_X1    0x06
volatile uint8_t icm_who_72 = 0;     // WHO_AM_I read from 0x72
volatile uint8_t icm_who_75 = 0;     // WHO_AM_I read from 0x75
volatile uint8_t icm_ok = 0;
volatile uint8_t icm_int_status = 0;
volatile uint8_t icm_status = 0;
volatile uint8_t icm_pwr_readback = 0;
volatile uint8_t icm_accel_raw[6];
volatile uint8_t icm_gyro_raw[6];
volatile int16_t icm_ax = 0, icm_ay = 0, icm_az = 0;
volatile int16_t icm_gx = 0, icm_gy = 0, icm_gz = 0;
volatile uint8_t icm_dump[128];  // dump registers 0x00–0x7F to find data

// I3G4250D variables
#define I3G4250D_ADDR     (0x69 << 1)  // SDO = VDD
#define I3G4250D_WHO_AM_I 0x0F
#define I3G4250D_CTRL1    0x20
#define I3G4250D_STATUS   0x27
#define I3G4250D_OUT_X_L  0x28
volatile uint8_t i3g_who_0F = 0;   // WHO_AM_I at 0x0F (ST sensors)
volatile uint8_t i3g_who_72 = 0;   // WHO_AM_I at 0x72 (InvenSense)
volatile uint8_t i3g_who_75 = 0;   // WHO_AM_I at 0x75 (InvenSense)
volatile uint8_t i3g_ok = 0;
volatile uint8_t i3g_status = 0;
volatile uint8_t i3g_raw[6];
volatile int16_t i3g_x = 0, i3g_y = 0, i3g_z = 0;

// MCP9808 variables
#define MCP9808_ADDR      (0x1F << 1)
#define MCP9808_MANUF_ID  0x06    // Manufacturer ID register (expect 0x0054)
#define MCP9808_DEVICE_ID 0x07    // Device ID register (expect 0x0400)
#define MCP9808_TEMP      0x05    // Ambient temperature register
volatile uint16_t mcp_manuf_id = 0;
volatile uint16_t mcp_device_id = 0;
volatile uint8_t mcp_ok = 0;
volatile uint8_t mcp_raw[2];
volatile float mcp_temp = 0.0f;    // temperature in °C

// SCH16T SPI variables
volatile uint16_t spi_tx[2];
volatile uint16_t spi_rx[2];
volatile uint8_t  sch16t_detected = 0;
volatile HAL_StatusTypeDef spi_result;
volatile uint16_t spi_test_rx0 = 0;
volatile uint16_t spi_test_rx1 = 0;
volatile uint32_t spi_resp_cmd = 0;
volatile uint32_t spi_resp_zeros = 0;
volatile uint32_t spi_resp_ones = 0;
volatile int32_t sch_gx = 0, sch_gy = 0, sch_gz = 0;
volatile int32_t sch_ax = 0, sch_ay = 0, sch_az = 0;
volatile uint32_t sch_frame_built = 0;   // what build_frame generates
volatile uint32_t sch_raw_resp = 0;      // raw response

// RS-485 transmit buffer
char tx_buf[256];

// RS-485 mode flag: 0 = protocol mode (rx/tx), 1 = continuous TX mode
volatile uint8_t rs485_mode = 0;  // change to 1 for continuous TX

// Protocol structs (packed, no padding)
typedef struct __attribute__((packed)) {
  float rm3100_x, rm3100_y, rm3100_z;           // 12 bytes
  float iis2mdc_x, iis2mdc_y, iis2mdc_z;        // 12 bytes
  float icm_accel_x, icm_accel_y, icm_accel_z;   // 12 bytes
  float icm_gyro_x, icm_gyro_y, icm_gyro_z;      // 12 bytes
  float sch16t_accel_x, sch16t_accel_y, sch16t_accel_z; // 12 bytes
  float sch16t_gyro_x, sch16t_gyro_y, sch16t_gyro_z;   // 12 bytes
  float i3g_gyro_x, i3g_gyro_y, i3g_gyro_z;     // 12 bytes
  float mcp_temp_val;                             // 4 bytes
} IMUCube_AllSensorsPayload_t;  // 88 bytes total

_Static_assert(sizeof(IMUCube_AllSensorsPayload_t) == 88, "payload must be 88 bytes");

typedef struct __attribute__((packed)) {
  uint8_t ack_type;
  uint8_t original_cmd;
  uint8_t status_code;
  uint8_t reserved;
} AckResponse_t;  // 4 bytes

// RS-485 RX/TX buffers
uint8_t rs485_rx_buf[140];   // max frame = 134 bytes
uint8_t rs485_tx_frame[140];
volatile uint8_t rs485_rx_byte = 0;
volatile uint8_t rs485_rx_idx = 0;
volatile uint8_t rs485_cmd_ready = 0;
volatile uint8_t rs485_msg_type = 0;
volatile uint8_t rs485_payload_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t calc_crc3(uint32_t frame)
{
  uint8_t crc = 0x05;
  for (int i = 31; i >= 3; i--)
  {
    uint8_t bit = (frame >> i) & 0x01;
    crc = (crc << 1) | bit;
    if (crc & 0x08) crc ^= 0x0B;
  }
  return crc & 0x07;
}

uint32_t sch16t_build_frame(uint16_t address, uint8_t rw, uint32_t data)
{
  uint32_t frame = 0;
  frame |= ((uint32_t)(address & 0x03FF)) << 22;
  frame |= ((uint32_t)(rw & 0x01)) << 21;
  frame |= ((data & 0x3FFFF)) << 3;
  frame |= calc_crc3(frame);
  return frame;
}

uint32_t sch16t_transfer(uint32_t frame)
{
  uint16_t tx[2], rx[2];
  tx[0] = (uint16_t)(frame >> 16);
  tx[1] = (uint16_t)(frame & 0xFFFF);

  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, (uint8_t *)rx, 2, 100);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  return ((uint32_t)rx[0] << 16) | rx[1];
}

// Send data via RS-485 (DE kept HIGH permanently)
volatile HAL_StatusTypeDef uart_result;

// CRC16-CCITT (matches ADCS master implementation exactly)
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
    {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc;
}

// Send raw bytes via RS-485 (low level)
void RS485_RawSend(uint8_t *data, uint16_t len)
{
  // DE HIGH, RE HIGH → transmit mode
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  HAL_Delay(1);  // let transceiver settle

  // Send data
  uart_result = HAL_UART_Transmit(&huart1, data, len, 1000);

  // Wait for last byte to fully leave the shift register
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
  HAL_Delay(1);

  // DE LOW, RE LOW → back to listen mode
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
}

// Send reply frame via RS-485.
// ADCS compatibility: emits a fixed 136-byte frame matching the ADCS_Packet_t
// struct layout (4 header + 128 payload region + 2 CRC at offset 132 +
// 2 reserved padding), with CRC stored little-endian. This deviates from the
// documented protocol spec but is required for the current ADCS firmware
// which receives with sizeof(ADCS_Packet_t). See IMUCube_side_implementation.md §13.
void RS485_SendFrame(uint8_t msg_type, uint8_t *payload, uint8_t payload_len)
{
  // Zero the full 136 bytes first so the payload padding region and the
  // reserved tail (bytes 134-135) don't leak stale data onto the bus.
  memset(rs485_tx_frame, 0, 136);

  // Build frame header
  rs485_tx_frame[0] = ADCS_BUS_START_BYTE;  // 0xAA
  rs485_tx_frame[1] = IMUCUBE_MODULE_ID;     // 0x04
  rs485_tx_frame[2] = msg_type;
  rs485_tx_frame[3] = payload_len;

  // Live payload goes into the fixed 128-byte payload region (offset 4..131).
  if (payload_len > 0)
    memcpy(&rs485_tx_frame[4], payload, payload_len);

  // Calculate CRC over header + live payload bytes only — the rest of the
  // 128-byte payload region is padding and is NOT covered (matches ADCS).
  uint16_t crc = crc16_ccitt(rs485_tx_frame, 4 + payload_len);

  // CRC16 at fixed offset 132, little-endian byte order (matches how ADCS
  // serializes its uint16_t crc16 struct field on little-endian ARM).
  rs485_tx_frame[132] = crc & 0xFF;          // LSB first
  rs485_tx_frame[133] = (crc >> 8) & 0xFF;   // MSB second
  // rs485_tx_frame[134..135] already zeroed by memset (reserved padding).

  // Always send the full 136-byte frame.
  RS485_RawSend(rs485_tx_frame, 136);
}

// Handle PING command → reply ACK
void Handle_Ping(void)
{
  AckResponse_t ack;
  ack.ack_type = TLM_ACK;
  ack.original_cmd = CMD_PING;
  ack.status_code = 0x00;
  ack.reserved = 0x00;

  RS485_SendFrame(TLM_ACK, (uint8_t *)&ack, sizeof(ack));
}

// Handle GET_TELEMETRY → reply with all sensor data
void Handle_GetTelemetry(void)
{
  IMUCube_AllSensorsPayload_t tlm;

  // Pack current sensor values into the struct
  tlm.rm3100_x = (float)rm3100_x;
  tlm.rm3100_y = (float)rm3100_y;
  tlm.rm3100_z = (float)rm3100_z;

  tlm.iis2mdc_x = (float)iis2mdc_x;
  tlm.iis2mdc_y = (float)iis2mdc_y;
  tlm.iis2mdc_z = (float)iis2mdc_z;

  tlm.icm_accel_x = (float)icm_ax;
  tlm.icm_accel_y = (float)icm_ay;
  tlm.icm_accel_z = (float)icm_az;
  tlm.icm_gyro_x = (float)icm_gx;
  tlm.icm_gyro_y = (float)icm_gy;
  tlm.icm_gyro_z = (float)icm_gz;

  tlm.sch16t_accel_x = (float)sch_ax;
  tlm.sch16t_accel_y = (float)sch_ay;
  tlm.sch16t_accel_z = (float)sch_az;
  tlm.sch16t_gyro_x = (float)sch_gx;
  tlm.sch16t_gyro_y = (float)sch_gy;
  tlm.sch16t_gyro_z = (float)sch_gz;

  tlm.i3g_gyro_x = (float)i3g_x;
  tlm.i3g_gyro_y = (float)i3g_y;
  tlm.i3g_gyro_z = (float)i3g_z;

  tlm.mcp_temp_val = mcp_temp;

  RS485_SendFrame(TLM_IMUCUBE_ALL_SENSORS, (uint8_t *)&tlm, sizeof(tlm));
}

// Parse received RS-485 frame and dispatch command
void RS485_ProcessFrame(void)
{
  // ADCS compatibility: fixed 136-byte wire frame. See §13 in
  // IMUCube_side_implementation.md for the rationale behind this shim.
  if (rs485_rx_idx < 136) return;

  // Extract CRC16 from fixed offset 132-133, little-endian byte order.
  // Reason: ADCS stores the CRC in a `uint16_t crc16` struct field at
  // offsetof(ADCS_Packet_t, crc16) == 132, then transmits the raw struct
  // via HAL_UART_Transmit — on little-endian ARM that serializes as
  // [LSB][MSB], NOT the big-endian order the spec calls for.
  uint16_t received_crc = (uint16_t)rs485_rx_buf[132]
                       | ((uint16_t)rs485_rx_buf[133] << 8);

  // CRC input remains just the header + live payload bytes (the rest of
  // the 128-byte payload region is padding that ADCS doesn't cover either).
  uint16_t calc_crc = crc16_ccitt(rs485_rx_buf, 4 + rs485_payload_len);

  if (received_crc != calc_crc) return;  // bad CRC, drop silently

  // Check module ID
  uint8_t mod_id = rs485_rx_buf[1];
  if (mod_id != IMUCUBE_MODULE_ID && mod_id != ADCS_BUS_BROADCAST_ID) return;

  // Dispatch command
  uint8_t cmd = rs485_rx_buf[2];
  uint8_t is_broadcast = (mod_id == ADCS_BUS_BROADCAST_ID);

  switch (cmd)
  {
    case CMD_PING:
      if (!is_broadcast) Handle_Ping();
      break;

    case CMD_GET_TELEMETRY:
      if (!is_broadcast) Handle_GetTelemetry();
      break;

    case CMD_RESET:
      if (!is_broadcast)
      {
        // Send ACK first
        AckResponse_t ack = { TLM_ACK, CMD_RESET, 0x00, 0x00 };
        RS485_SendFrame(TLM_ACK, (uint8_t *)&ack, sizeof(ack));
        HAL_Delay(10);
        NVIC_SystemReset();
      }
      break;

    default:
    {
      // Unknown command → NACK
      if (!is_broadcast)
      {
        AckResponse_t nack = { TLM_NACK, cmd, 0xFF, 0x00 };
        RS485_SendFrame(TLM_NACK, (uint8_t *)&nack, sizeof(nack));
      }
      break;
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t bist_cmd;
  uint8_t poll_cmd;

  // RS-485: set initial direction based on mode
  if (rs485_mode == 1)
  {
    // Mode 1: continuous TX — DE HIGH, RE HIGH (transmit)
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  }
  else
  {
    // Mode 0: protocol — DE LOW, RE LOW (receive/listen)
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
  }
  HAL_Delay(10);

  // Startup test: send 0xAA
  uint8_t startup_test = 0xAA;
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  uart_result = HAL_UART_Transmit(&huart1, &startup_test, 1, 1000);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
  // Back to receive mode if protocol mode
  if (rs485_mode == 0)
  {
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
  }

  /* ---- RM3100 BIST ---- */
  bist_cmd = 0x8F;
  poll_cmd = 0x70;
  HAL_I2C_Mem_Write(&hi2c1, RM3100_ADDR, RM3100_BIST, I2C_MEMADD_SIZE_8BIT, &bist_cmd, 1, 50);
  HAL_I2C_Mem_Write(&hi2c1, RM3100_ADDR, RM3100_POLL, I2C_MEMADD_SIZE_8BIT, &poll_cmd, 1, 50);
  HAL_Delay(200);
  HAL_I2C_Mem_Read(&hi2c1, RM3100_ADDR, RM3100_BIST, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&rm3100_bist_result, 1, 50);
  rm3100_bist_pass = ((rm3100_bist_result & 0x70) == 0x70) ? 1 : 0;
  bist_cmd = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, RM3100_ADDR, RM3100_BIST, I2C_MEMADD_SIZE_8BIT, &bist_cmd, 1, 50);
  HAL_Delay(10);

  /* ---- IIS2MDC init ---- */
  HAL_I2C_Mem_Read(&hi2c1, IIS2MDC_ADDR, IIS2MDC_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&iis2mdc_who, 1, 50);
  if (iis2mdc_who == 0x40)
  {
    iis2mdc_ok = 1;
    uint8_t cfg_c = 0x10;
    HAL_I2C_Mem_Write(&hi2c1, IIS2MDC_ADDR, IIS2MDC_CFG_C, I2C_MEMADD_SIZE_8BIT, &cfg_c, 1, 50);
  }

  /* ---- ICM-45686 init ---- */
  HAL_I2C_Mem_Read(&hi2c1, ICM45686_ADDR, 0x72, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&icm_who_72, 1, 50);
  if (icm_who_72 == 0xE9 || icm_who_72 == 0xE5)
  {
    icm_ok = 1;
    uint8_t accel_cfg = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, ICM45686_ADDR, 0x1B, I2C_MEMADD_SIZE_8BIT, &accel_cfg, 1, 50);
    uint8_t gyro_cfg = 0x09;
    HAL_I2C_Mem_Write(&hi2c1, ICM45686_ADDR, 0x1C, I2C_MEMADD_SIZE_8BIT, &gyro_cfg, 1, 50);
    uint8_t pwr = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, ICM45686_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &pwr, 1, 50);
    HAL_Delay(300);
  }

  /* ---- I3G4250D init ---- */
  HAL_I2C_Mem_Read(&hi2c1, I3G4250D_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&i3g_who_0F, 1, 50);
  if (i3g_who_0F == 0xD3)
  {
    i3g_ok = 1;
    uint8_t ctrl1 = 0x0F;
    HAL_I2C_Mem_Write(&hi2c1, I3G4250D_ADDR, I3G4250D_CTRL1, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 50);
    HAL_Delay(50);
  }

  /* ---- MCP9808 init ---- */
  HAL_I2C_Mem_Read(&hi2c1, MCP9808_ADDR, MCP9808_MANUF_ID, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mcp_raw, 2, 50);
  mcp_manuf_id = (uint16_t)(mcp_raw[0] << 8 | mcp_raw[1]);
  if (mcp_manuf_id == 0x0054) mcp_ok = 1;

  /* ---- SCH16T: Full Startup (ALL exact datasheet frames) ---- */

  // Step 1: CS HIGH
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  // Step 2-3: Wait for power + NVM read
  HAL_Delay(50);

  // Step 6: EN_SENSOR=1
  sch16t_transfer(0x0D60000A);

  // Step 7: Wait 215ms
  HAL_Delay(215);

  // Step 8: Read ALL status registers once (exact frames with correct CRC)
  sch16t_transfer(0x05000007);  // STAT_SUM
  sch16t_transfer(0x05400005);  // STAT_SUM_SAT
  sch16t_transfer(0x05800003);  // STAT_COM
  sch16t_transfer(0x05C00001);  // STAT_RATE_COM
  sch16t_transfer(0x06000002);  // STAT_RATE_X
  sch16t_transfer(0x06400000);  // STAT_RATE_Y
  sch16t_transfer(0x06800006);  // STAT_RATE_Z
  sch16t_transfer(0x06C00004);  // STAT_ACC_X
  sch16t_transfer(0x07000001);  // STAT_ACC_Y
  sch16t_transfer(0x07400003);  // STAT_ACC_Z
  sch16t_transfer(0x00000000);  // dummy flush

  // Step 9: EOI=1
  sch16t_transfer(0x0D60001C);

  // Step 10: Wait 3ms
  HAL_Delay(3);

  // Step 11: Read status registers twice
  for (int pass = 0; pass < 2; pass++)
  {
    sch16t_transfer(0x05000007);
    sch16t_transfer(0x05400005);
    sch16t_transfer(0x05800003);
    sch16t_transfer(0x05C00001);
    sch16t_transfer(0x06000002);
    sch16t_transfer(0x06400000);
    sch16t_transfer(0x06800006);
    sch16t_transfer(0x06C00004);
    sch16t_transfer(0x07000001);
    sch16t_transfer(0x07400003);
    sch16t_transfer(0x00000000);
  }

  HAL_Delay(10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (rs485_mode == 1)
    {
      /* MODE 1: Continuous TX — send 0xAA pattern back-to-back */
      static uint8_t aa_buf[32];
      static uint8_t aa_buf_init = 0;
      if (!aa_buf_init)
      {
        memset(aa_buf, 0xAA, sizeof(aa_buf));
        HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
        aa_buf_init = 1;
      }
      uart_result = HAL_UART_Transmit(&huart1, aa_buf, sizeof(aa_buf), 1000);
      /* No delay — loop immediately to keep the line saturated */
      continue;
    }

    /* ---- RS-485 Protocol Mode ---- */
    /* MODE 0: wait for command → read sensors → respond */

    // Clear any UART error flags (overrun, framing, noise)
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF);
    if (huart1.RxState != HAL_UART_STATE_READY)
      huart1.RxState = HAL_UART_STATE_READY;

    // Block here waiting for start byte (1s timeout — MCU is idle until master asks)
    uart_result = HAL_UART_Receive(&huart1, (uint8_t *)&rs485_rx_byte, 1, 1000);

    if (uart_result == HAL_OK && rs485_rx_byte == ADCS_BUS_START_BYTE)
    {
      // Got start byte — collect rest of frame
      rs485_rx_buf[0] = ADCS_BUS_START_BYTE;
      rs485_rx_idx = 1;

      // Read 3 more header bytes
      uart_result = HAL_UART_Receive(&huart1, &rs485_rx_buf[1], 3, 50);
      if (uart_result == HAL_OK)
      {
        rs485_rx_idx = 4;
        rs485_payload_len = rs485_rx_buf[3];

        if (rs485_payload_len <= 128)
        {
          // ADCS compatibility shim: ADCS transmits a fixed 136-byte ADCS_Packet_t
          // struct regardless of payload_length (4 header + 128 payload region +
          // 2 CRC at offset 132 + 2 reserved padding). Drain all 132 trailing
          // bytes so the UART RX FIFO stays aligned. See
          // IMUCube_side_implementation.md §13 for the full rationale.
          uint8_t remaining = 132;
          uart_result = HAL_UART_Receive(&huart1, &rs485_rx_buf[4], remaining, 100);
          if (uart_result == HAL_OK)
          {
            rs485_rx_idx = 4 + remaining;

            /* ---- Read ALL sensors NOW, before responding ---- */

            /* ---- SCH16T (SPI) ---- */
            sch16t_transfer(0x00400001);
            sch_raw_resp = sch16t_transfer(0x00800007);
            sch_gx = (int16_t)((sch_raw_resp >> 4) & 0xFFFF);
            spi_resp_zeros = sch16t_transfer(0x00C00005);
            sch_gy = (int16_t)((spi_resp_zeros >> 4) & 0xFFFF);
            spi_resp_ones = sch16t_transfer(0x01000000);
            sch_gz = (int16_t)((spi_resp_ones >> 4) & 0xFFFF);
            uint32_t r4 = sch16t_transfer(0x01400002);
            sch_ax = (int16_t)((r4 >> 4) & 0xFFFF);
            uint32_t r5 = sch16t_transfer(0x01800004);
            sch_ay = (int16_t)((r5 >> 4) & 0xFFFF);
            uint32_t r6 = sch16t_transfer(0x00000000);
            sch_az = (int16_t)((r6 >> 4) & 0xFFFF);

            /* ---- RM3100 (I2C) — bypass on error ---- */
            poll_cmd = 0x70;
            if (HAL_I2C_Mem_Write(&hi2c1, RM3100_ADDR, RM3100_POLL, I2C_MEMADD_SIZE_8BIT, &poll_cmd, 1, 50) == HAL_OK)
            {
              HAL_Delay(200);
              if (HAL_I2C_Mem_Read(&hi2c1, RM3100_ADDR, RM3100_MX, I2C_MEMADD_SIZE_8BIT, (uint8_t *)rm3100_raw, 9, 50) == HAL_OK)
              {
                rm3100_x = (int32_t)((rm3100_raw[0] << 16) | (rm3100_raw[1] << 8) | rm3100_raw[2]);
                if (rm3100_x & 0x800000) rm3100_x |= 0xFF000000;
                rm3100_y = (int32_t)((rm3100_raw[3] << 16) | (rm3100_raw[4] << 8) | rm3100_raw[5]);
                if (rm3100_y & 0x800000) rm3100_y |= 0xFF000000;
                rm3100_z = (int32_t)((rm3100_raw[6] << 16) | (rm3100_raw[7] << 8) | rm3100_raw[8]);
                if (rm3100_z & 0x800000) rm3100_z |= 0xFF000000;
              }
            }

            /* ---- IIS2MDC (I2C) — bypass on error ---- */
            if (iis2mdc_ok)
            {
              uint8_t cfg_a = 0x01;
              if (HAL_I2C_Mem_Write(&hi2c1, IIS2MDC_ADDR, IIS2MDC_CFG_A, I2C_MEMADD_SIZE_8BIT, &cfg_a, 1, 50) == HAL_OK)
              {
                HAL_Delay(20);
                if (HAL_I2C_Mem_Read(&hi2c1, IIS2MDC_ADDR, IIS2MDC_OUT_X_L, I2C_MEMADD_SIZE_8BIT, (uint8_t *)iis2mdc_raw, 6, 50) == HAL_OK)
                {
                  iis2mdc_x = (int16_t)(iis2mdc_raw[1] << 8 | iis2mdc_raw[0]);
                  iis2mdc_y = (int16_t)(iis2mdc_raw[3] << 8 | iis2mdc_raw[2]);
                  iis2mdc_z = (int16_t)(iis2mdc_raw[5] << 8 | iis2mdc_raw[4]);
                }
              }
            }

            /* ---- ICM-45686 (I2C) — bypass on error ---- */
            if (icm_ok)
            {
              if (HAL_I2C_Mem_Read(&hi2c1, ICM45686_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t *)icm_accel_raw, 6, 50) == HAL_OK)
              {
                icm_ax = (int16_t)(icm_accel_raw[0] << 8 | icm_accel_raw[1]);
                icm_ay = (int16_t)(icm_accel_raw[2] << 8 | icm_accel_raw[3]);
                icm_az = (int16_t)(icm_accel_raw[4] << 8 | icm_accel_raw[5]);
              }
              if (HAL_I2C_Mem_Read(&hi2c1, ICM45686_ADDR, 0x06, I2C_MEMADD_SIZE_8BIT, (uint8_t *)icm_gyro_raw, 6, 50) == HAL_OK)
              {
                icm_gx = (int16_t)(icm_gyro_raw[0] << 8 | icm_gyro_raw[1]);
                icm_gy = (int16_t)(icm_gyro_raw[2] << 8 | icm_gyro_raw[3]);
                icm_gz = (int16_t)(icm_gyro_raw[4] << 8 | icm_gyro_raw[5]);
              }
            }

            /* ---- I3G4250D (I2C) — bypass on error ---- */
            if (i3g_ok)
            {
              if (HAL_I2C_Mem_Read(&hi2c1, I3G4250D_ADDR, 0xA8, I2C_MEMADD_SIZE_8BIT, (uint8_t *)i3g_raw, 6, 50) == HAL_OK)
              {
                i3g_x = (int16_t)(i3g_raw[1] << 8 | i3g_raw[0]);
                i3g_y = (int16_t)(i3g_raw[3] << 8 | i3g_raw[2]);
                i3g_z = (int16_t)(i3g_raw[5] << 8 | i3g_raw[4]);
              }
            }

            /* ---- MCP9808 (I2C) — bypass on error ---- */
            if (mcp_ok)
            {
              if (HAL_I2C_Mem_Read(&hi2c1, MCP9808_ADDR, MCP9808_TEMP, I2C_MEMADD_SIZE_8BIT, (uint8_t *)mcp_raw, 2, 50) == HAL_OK)
              {
                mcp_raw[0] = mcp_raw[0] & 0x1F;
                if (mcp_raw[0] & 0x10)
                {
                  mcp_raw[0] = mcp_raw[0] & 0x0F;
                  mcp_temp = 256.0f - ((float)(mcp_raw[0] << 4) + (float)(mcp_raw[1] / 16.0f));
                }
                else
                {
                  mcp_temp = ((float)(mcp_raw[0] << 4) + (float)(mcp_raw[1] / 16.0f));
                }
              }
            }

            /* ---- Sensors done, now process command and respond ---- */
            RS485_ProcessFrame();
          }
        }
      }
      rs485_rx_idx = 0;
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
  hi2c1.Init.Timing = 0x00100D14;
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
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|RS485_RE_Pin|RS485_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SCH16T_DRY_SYNC_Pin */
  GPIO_InitStruct.Pin = SCH16T_DRY_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SCH16T_DRY_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin RS485_RE_Pin RS485_DE_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|RS485_RE_Pin|RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
