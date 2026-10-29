#ifndef SCL3300_H
#define SCL3300_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  SCL3300_MODE_1 = 1,
  SCL3300_MODE_2 = 2,
  SCL3300_MODE_3 = 3,
  SCL3300_MODE_4 = 4
} SCL3300_Mode_t;

typedef enum
{
  SCL3300_OK = 0,
  SCL3300_INVALID_ARGUMENT,
  SCL3300_HAL_ERROR,
  SCL3300_CRC_ERROR,
  SCL3300_WHOAMI_ERROR,
  SCL3300_STARTUP_ERROR
} SCL3300_Status_t;

typedef struct
{
  SPI_HandleTypeDef *hspi;

  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;

  SCL3300_Mode_t mode;
  float acceleration_sensitivity_lsb_g;
} SCL3300_Handle_t;

typedef struct
{
  uint32_t raw_frame;

  uint16_t data_unsigned;
  int16_t data_signed;

  uint8_t return_status;

  uint8_t received_crc;
  uint8_t calculated_crc;

  bool crc_valid;
} SCL3300_Response_t;

typedef struct
{
  int16_t acc_x_raw;
  int16_t acc_y_raw;
  int16_t acc_z_raw;
  int16_t temperature_raw;

  float acc_x_g;
  float acc_y_g;
  float acc_z_g;

  float acc_x_ms2;
  float acc_y_ms2;
  float acc_z_ms2;

  float temperature_c;
} SCL3300_Measurement_t;

typedef struct
{
  uint8_t whoami;

  uint16_t status;
  uint16_t err_flag1;
  uint16_t err_flag2;

  uint8_t return_status;

  bool whoami_valid;
  bool startup_valid;
} SCL3300_Diagnostics_t;

SCL3300_Status_t SCL3300_InitHandle(
    SCL3300_Handle_t *sensor,
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin);

SCL3300_Status_t SCL3300_Start(
    SCL3300_Handle_t *sensor,
    SCL3300_Mode_t mode,
    SCL3300_Diagnostics_t *diagnostics);

SCL3300_Status_t SCL3300_TransferFrame(
    SCL3300_Handle_t *sensor,
    uint32_t tx_frame,
    SCL3300_Response_t *response,
    bool validate_response_crc);

SCL3300_Status_t SCL3300_ReadRegister(
    SCL3300_Handle_t *sensor,
    uint32_t command_frame,
    SCL3300_Response_t *response);

SCL3300_Status_t SCL3300_ReadWhoAmI(
    SCL3300_Handle_t *sensor,
    uint8_t *whoami);

SCL3300_Status_t SCL3300_ReadStatus(
    SCL3300_Handle_t *sensor,
    uint16_t *status_register,
    uint8_t *return_status);

SCL3300_Status_t SCL3300_ReadErrorFlag1(
    SCL3300_Handle_t *sensor,
    uint16_t *error_flags);

SCL3300_Status_t SCL3300_ReadErrorFlag2(
    SCL3300_Handle_t *sensor,
    uint16_t *error_flags);

SCL3300_Status_t SCL3300_ReadMeasurement(
    SCL3300_Handle_t *sensor,
    SCL3300_Measurement_t *measurement);

uint8_t SCL3300_CalculateCRC(uint32_t frame);

#endif
