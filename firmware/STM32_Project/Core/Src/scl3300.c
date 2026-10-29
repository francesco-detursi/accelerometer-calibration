#include "scl3300.h"

#include <stddef.h>

#define SCL3300_FRAME_SIZE_BYTES 4U
#define SCL3300_SPI_TIMEOUT_MS 100U
#define SCL3300_INTERFRAME_DELAY_MS 1U

#define SCL3300_RESET_DELAY_MS 3U
#define SCL3300_MODE1_SETTLING_TIME_MS 25U
#define SCL3300_MODE2_SETTLING_TIME_MS 15U
#define SCL3300_MODE3_SETTLING_TIME_MS 100U
#define SCL3300_MODE4_SETTLING_TIME_MS 100U

#define SCL3300_STANDARD_GRAVITY_MS2 9.80665f

#define SCL3300_RS_STARTUP_OK 0x01U
#define SCL3300_WHOAMI_EXPECTED 0xC1U

#define SCL3300_CMD_READ_ACC_X 0x040000F7UL
#define SCL3300_CMD_READ_ACC_Y 0x080000FDUL
#define SCL3300_CMD_READ_ACC_Z 0x0C0000FBUL
#define SCL3300_CMD_READ_TEMPERATURE 0x140000EFUL

#define SCL3300_CMD_READ_STATUS 0x180000E5UL
#define SCL3300_CMD_READ_ERR_FLAG1 0x1C0000E3UL
#define SCL3300_CMD_READ_ERR_FLAG2 0x200000C1UL

#define SCL3300_CMD_ENABLE_ANGLES 0xB0001F6FUL

#define SCL3300_CMD_MODE_1 0xB400001FUL
#define SCL3300_CMD_MODE_2 0xB4000102UL
#define SCL3300_CMD_MODE_3 0xB4000225UL
#define SCL3300_CMD_MODE_4 0xB4000338UL

#define SCL3300_CMD_SW_RESET 0xB4002098UL
#define SCL3300_CMD_READ_WHOAMI 0x40000091UL

static uint8_t SCL3300_CRC8(uint8_t bit_value, uint8_t crc);

static SCL3300_Status_t SCL3300_SendCommand(
    SCL3300_Handle_t *sensor,
    uint32_t command_frame);

static SCL3300_Status_t SCL3300_SetMode(
    SCL3300_Handle_t *sensor,
    SCL3300_Mode_t mode);

static uint32_t SCL3300_GetModeCommand(SCL3300_Mode_t mode);

static uint32_t SCL3300_GetModeSettlingTime(SCL3300_Mode_t mode);

static float SCL3300_GetAccelerationSensitivity(SCL3300_Mode_t mode);

static float SCL3300_ConvertTemperatureToCelsius(
    int16_t temperature_raw);

static uint8_t SCL3300_CRC8(uint8_t bit_value, uint8_t crc)
{
  uint8_t temp;

  temp = (uint8_t)(crc & 0x80U);

  if (bit_value == 1U)
  {
    temp ^= 0x80U;
  }

  crc <<= 1U;

  if (temp != 0U)
  {
    crc ^= 0x1DU;
  }

  return crc;
}

uint8_t SCL3300_CalculateCRC(uint32_t frame)
{
  uint8_t crc;
  int32_t bit_index;

  crc = 0xFFU;

  for (bit_index = 31; bit_index > 7; bit_index--)
  {
    uint8_t bit_value;

    bit_value = (uint8_t)((frame >> bit_index) & 0x01U);
    crc = SCL3300_CRC8(bit_value, crc);
  }

  return (uint8_t)(~crc);
}

SCL3300_Status_t SCL3300_InitHandle(
    SCL3300_Handle_t *sensor,
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin)
{
  if ((sensor == NULL) ||
      (hspi == NULL) ||
      (cs_port == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  sensor->hspi = hspi;
  sensor->cs_port = cs_port;
  sensor->cs_pin = cs_pin;
  sensor->mode = SCL3300_MODE_1;
  sensor->acceleration_sensitivity_lsb_g = 6000.0f;

  HAL_GPIO_WritePin(
      sensor->cs_port,
      sensor->cs_pin,
      GPIO_PIN_SET);

  HAL_Delay(SCL3300_INTERFRAME_DELAY_MS);

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_TransferFrame(
    SCL3300_Handle_t *sensor,
    uint32_t tx_frame,
    SCL3300_Response_t *response,
    bool validate_response_crc)
{
  uint8_t tx[4];
  uint8_t rx[4];
  HAL_StatusTypeDef hal_status;

  if ((sensor == NULL) ||
      (sensor->hspi == NULL) ||
      (sensor->cs_port == NULL) ||
      (response == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  tx[0] = (uint8_t)((tx_frame >> 24) & 0xFFU);
  tx[1] = (uint8_t)((tx_frame >> 16) & 0xFFU);
  tx[2] = (uint8_t)((tx_frame >> 8) & 0xFFU);
  tx[3] = (uint8_t)(tx_frame & 0xFFU);

  rx[0] = 0U;
  rx[1] = 0U;
  rx[2] = 0U;
  rx[3] = 0U;

  HAL_GPIO_WritePin(
      sensor->cs_port,
      sensor->cs_pin,
      GPIO_PIN_RESET);

  hal_status = HAL_SPI_TransmitReceive(
      sensor->hspi,
      tx,
      rx,
      SCL3300_FRAME_SIZE_BYTES,
      SCL3300_SPI_TIMEOUT_MS);

  HAL_GPIO_WritePin(
      sensor->cs_port,
      sensor->cs_pin,
      GPIO_PIN_SET);

  HAL_Delay(SCL3300_INTERFRAME_DELAY_MS);

  if (hal_status != HAL_OK)
  {
    return SCL3300_HAL_ERROR;
  }

  response->raw_frame =
      ((uint32_t)rx[0] << 24) |
      ((uint32_t)rx[1] << 16) |
      ((uint32_t)rx[2] << 8) |
      (uint32_t)rx[3];

  response->data_unsigned =
      (uint16_t)(((uint16_t)rx[1] << 8) |
                 (uint16_t)rx[2]);

  response->data_signed = (int16_t)response->data_unsigned;
  response->return_status = (uint8_t)(rx[0] & 0x03U);
  response->received_crc = rx[3];
  response->calculated_crc = SCL3300_CalculateCRC(response->raw_frame);

  response->crc_valid =
      response->calculated_crc == response->received_crc;

  if (validate_response_crc && !response->crc_valid)
  {
    return SCL3300_CRC_ERROR;
  }

  return SCL3300_OK;
}

static SCL3300_Status_t SCL3300_SendCommand(
    SCL3300_Handle_t *sensor,
    uint32_t command_frame)
{
  SCL3300_Response_t ignored_response;

  return SCL3300_TransferFrame(
      sensor,
      command_frame,
      &ignored_response,
      false);
}

SCL3300_Status_t SCL3300_ReadRegister(
    SCL3300_Handle_t *sensor,
    uint32_t command_frame,
    SCL3300_Response_t *response)
{
  SCL3300_Response_t previous_response;
  SCL3300_Status_t status;

  if ((sensor == NULL) ||
      (response == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_TransferFrame(
      sensor,
      command_frame,
      &previous_response,
      false);

  if (status != SCL3300_OK)
  {
    return status;
  }

  status = SCL3300_TransferFrame(
      sensor,
      command_frame,
      response,
      true);

  return status;
}
static uint32_t SCL3300_GetModeCommand(SCL3300_Mode_t mode)
{
  switch (mode)
  {
    case SCL3300_MODE_1:
      return SCL3300_CMD_MODE_1;

    case SCL3300_MODE_2:
      return SCL3300_CMD_MODE_2;

    case SCL3300_MODE_3:
      return SCL3300_CMD_MODE_3;

    case SCL3300_MODE_4:
      return SCL3300_CMD_MODE_4;

    default:
      return 0UL;
  }
}

static uint32_t SCL3300_GetModeSettlingTime(SCL3300_Mode_t mode)
{
  switch (mode)
  {
    case SCL3300_MODE_1:
      return SCL3300_MODE1_SETTLING_TIME_MS;

    case SCL3300_MODE_2:
      return SCL3300_MODE2_SETTLING_TIME_MS;

    case SCL3300_MODE_3:
      return SCL3300_MODE3_SETTLING_TIME_MS;

    case SCL3300_MODE_4:
      return SCL3300_MODE4_SETTLING_TIME_MS;

    default:
      return SCL3300_MODE1_SETTLING_TIME_MS;
  }
}

static float SCL3300_GetAccelerationSensitivity(SCL3300_Mode_t mode)
{
  switch (mode)
  {
    case SCL3300_MODE_1:
      return 6000.0f;

    case SCL3300_MODE_2:
      return 3000.0f;

    case SCL3300_MODE_3:
    case SCL3300_MODE_4:
      return 12000.0f;

    default:
      return 6000.0f;
  }
}

static SCL3300_Status_t SCL3300_SetMode(
    SCL3300_Handle_t *sensor,
    SCL3300_Mode_t mode)
{
  uint32_t mode_command;
  SCL3300_Status_t status;

  if (sensor == NULL)
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  mode_command = SCL3300_GetModeCommand(mode);

  if (mode_command == 0UL)
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_SendCommand(sensor, mode_command);

  if (status != SCL3300_OK)
  {
    return status;
  }

  sensor->mode = mode;
  sensor->acceleration_sensitivity_lsb_g =
      SCL3300_GetAccelerationSensitivity(mode);

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_ReadWhoAmI(
    SCL3300_Handle_t *sensor,
    uint8_t *whoami)
{
  SCL3300_Response_t response;
  SCL3300_Status_t status;

  if ((sensor == NULL) ||
      (whoami == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_WHOAMI,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  *whoami = (uint8_t)(response.data_unsigned & 0x00FFU);

  if (*whoami != SCL3300_WHOAMI_EXPECTED)
  {
    return SCL3300_WHOAMI_ERROR;
  }

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_ReadStatus(
    SCL3300_Handle_t *sensor,
    uint16_t *status_register,
    uint8_t *return_status)
{
  SCL3300_Response_t response;
  SCL3300_Status_t status;

  if ((sensor == NULL) ||
      (status_register == NULL) ||
      (return_status == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_STATUS,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  *status_register = response.data_unsigned;
  *return_status = response.return_status;

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_ReadErrorFlag1(
    SCL3300_Handle_t *sensor,
    uint16_t *error_flags)
{
  SCL3300_Response_t response;
  SCL3300_Status_t status;

  if ((sensor == NULL) ||
      (error_flags == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_ERR_FLAG1,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  *error_flags = response.data_unsigned;

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_ReadErrorFlag2(
    SCL3300_Handle_t *sensor,
    uint16_t *error_flags)
{
  SCL3300_Response_t response;
  SCL3300_Status_t status;

  if ((sensor == NULL) ||
      (error_flags == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_ERR_FLAG2,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  *error_flags = response.data_unsigned;

  return SCL3300_OK;
}

SCL3300_Status_t SCL3300_Start(
    SCL3300_Handle_t *sensor,
    SCL3300_Mode_t mode,
    SCL3300_Diagnostics_t *diagnostics)
{
  SCL3300_Status_t status;
  SCL3300_Response_t status_response_1;
  SCL3300_Response_t status_response_2;
  SCL3300_Response_t status_response_3;

  if ((sensor == NULL) ||
      (diagnostics == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  diagnostics->whoami = 0U;
  diagnostics->status = 0U;
  diagnostics->err_flag1 = 0U;
  diagnostics->err_flag2 = 0U;
  diagnostics->return_status = 0U;
  diagnostics->whoami_valid = false;
  diagnostics->startup_valid = false;

  status = SCL3300_SendCommand(
      sensor,
      SCL3300_CMD_SW_RESET);

  if (status != SCL3300_OK)
  {
    return status;
  }

  HAL_Delay(SCL3300_RESET_DELAY_MS);

  status = SCL3300_SetMode(sensor, mode);

  if (status != SCL3300_OK)
  {
    return status;
  }

  status = SCL3300_SendCommand(
      sensor,
      SCL3300_CMD_ENABLE_ANGLES);

  if (status != SCL3300_OK)
  {
    return status;
  }

  HAL_Delay(SCL3300_GetModeSettlingTime(mode));

  status = SCL3300_TransferFrame(
      sensor,
      SCL3300_CMD_READ_STATUS,
      &status_response_1,
      false);

  if (status != SCL3300_OK)
  {
    return status;
  }

  status = SCL3300_TransferFrame(
      sensor,
      SCL3300_CMD_READ_STATUS,
      &status_response_2,
      true);

  if (status != SCL3300_OK)
  {
    return status;
  }

  status = SCL3300_TransferFrame(
      sensor,
      SCL3300_CMD_READ_STATUS,
      &status_response_3,
      true);

  if (status != SCL3300_OK)
  {
    return status;
  }

  diagnostics->status = status_response_3.data_unsigned;
  diagnostics->return_status = status_response_3.return_status;

  diagnostics->startup_valid =
      status_response_3.return_status == SCL3300_RS_STARTUP_OK;

  status = SCL3300_ReadWhoAmI(
      sensor,
      &diagnostics->whoami);

  diagnostics->whoami_valid =
      (status == SCL3300_OK) &&
      (diagnostics->whoami == SCL3300_WHOAMI_EXPECTED);

  if (!diagnostics->whoami_valid)
  {
    return SCL3300_WHOAMI_ERROR;
  }

  status = SCL3300_ReadErrorFlag1(
      sensor,
      &diagnostics->err_flag1);

  if (status != SCL3300_OK)
  {
    return status;
  }

  status = SCL3300_ReadErrorFlag2(
      sensor,
      &diagnostics->err_flag2);

  if (status != SCL3300_OK)
  {
    return status;
  }

  if (!diagnostics->startup_valid)
  {
    return SCL3300_STARTUP_ERROR;
  }

  return SCL3300_OK;
}
static float SCL3300_ConvertTemperatureToCelsius(
    int16_t temperature_raw)
{
  return -273.0f + ((float)temperature_raw / 18.9f);
}

SCL3300_Status_t SCL3300_ReadMeasurement(
    SCL3300_Handle_t *sensor,
    SCL3300_Measurement_t *measurement)
{
  SCL3300_Response_t response;
  SCL3300_Status_t status;
  float sensitivity;

  if ((sensor == NULL) ||
      (measurement == NULL))
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  sensitivity = sensor->acceleration_sensitivity_lsb_g;

  if (sensitivity <= 0.0f)
  {
    return SCL3300_INVALID_ARGUMENT;
  }

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_ACC_X,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  measurement->acc_x_raw = response.data_signed;

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_ACC_Y,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  measurement->acc_y_raw = response.data_signed;

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_ACC_Z,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  measurement->acc_z_raw = response.data_signed;

  status = SCL3300_ReadRegister(
      sensor,
      SCL3300_CMD_READ_TEMPERATURE,
      &response);

  if (status != SCL3300_OK)
  {
    return status;
  }

  measurement->temperature_raw = response.data_signed;

  measurement->acc_x_g =
      (float)measurement->acc_x_raw / sensitivity;

  measurement->acc_y_g =
      (float)measurement->acc_y_raw / sensitivity;

  measurement->acc_z_g =
      (float)measurement->acc_z_raw / sensitivity;

  measurement->acc_x_ms2 =
      measurement->acc_x_g * SCL3300_STANDARD_GRAVITY_MS2;

  measurement->acc_y_ms2 =
      measurement->acc_y_g * SCL3300_STANDARD_GRAVITY_MS2;

  measurement->acc_z_ms2 =
      measurement->acc_z_g * SCL3300_STANDARD_GRAVITY_MS2;

  measurement->temperature_c =
      SCL3300_ConvertTemperatureToCelsius(
          measurement->temperature_raw);

  return SCL3300_OK;
}
