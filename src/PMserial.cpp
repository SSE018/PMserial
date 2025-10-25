/* SerialPM
 Arduino library for PM sensors with serial interface
  PMS1003 aka G1
  PMS3003 aka G2
  PMS5003 aka G5
  PMS7003 aka G7
  PMSA003 aka G10
*/

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include <PMserial.h>
#include <cstddef>
#include <cstdint>

/* Sensor data format:
  https://github.com/avaldebe/AQmon/blob/master/lua_modules/pms3003.lua

PMS2003, PMS3003:
  24 byte long messages via UART 9600 8N1 (3.3V TTL).
DATA(MSB,LSB): Message header (4 bytes), 2 pairs of bytes (MSB,LSB)
  -1(  1,  2): Begin message       (hex:424D, ASCII 'BM')
   0(  3,  4): Message body length (hex:0014, decimal 20)
DATA(MSB,LSB): Message body (28 bytes), 10 pairs of bytes (MSB,LSB)
   1(  5,  6): PM 1.0 [ug/m3] (TSI standard)
   2(  7,  8): PM 2.5 [ug/m3] (TSI standard)
   3(  9, 10): PM 10. [ug/m3] (TSI standard)
   4( 11, 12): PM 1.0 [ug/m3] (std. atmosphere)
   5( 13, 14): PM 2.5 [ug/m3] (std. atmosphere)
   6( 15, 16): PM 10. [ug/m3] (std. atmosphere)
   7( 17, 18): no idea.
   8( 19, 19): no idea.
   9( 21, 22): no idea.
  10( 23, 24): cksum=byte01+..+byte22
PMS1003, PMS5003, PMS7003:
  32 byte long messages via UART 9600 8N1 (3.3V TTL).
DATA(MSB,LSB): Message header (4 bytes), 2 pairs of bytes (MSB,LSB)
  -1(  1,  2): Begin message       (hex:424D, ASCII 'BM')
   0(  3,  4): Message body length (hex:001C, decimal 28)
DATA(MSB,LSB): Message body (28 bytes), 14 pairs of bytes (MSB,LSB)
   1(  5,  6): PM 1.0 [ug/m3] (TSI standard)
   2(  7,  8): PM 2.5 [ug/m3] (TSI standard)
   3(  9, 10): PM 10. [ug/m3] (TSI standard)
   4( 11, 12): PM 1.0 [ug/m3] (std. atmosphere)
   5( 13, 14): PM 2.5 [ug/m3] (std. atmosphere)
   6( 15, 16): PM 10. [ug/m3] (std. atmosphere)
   7( 17, 18): num. particles with diameter > 0.3 um in 100 cm3 of air
   8( 19, 19): num. particles with diameter > 0.5 um in 100 cm3 of air
   9( 21, 22): num. particles with diameter > 1.0 um in 100 cm3 of air
  10( 23, 24): num. particles with diameter > 2.5 um in 100 cm3 of air
  11( 25, 26): num. particles with diameter > 5.0 um in 100 cm3 of air
  12( 27, 28): num. particles with diameter > 10. um in 100 cm3 of air
  13( 29, 30): Reserved
  14( 31, 32): cksum=byte01+..+byte30
*/

#define UART_BUF_SIZE (128)
#define RD_BUF_SIZE (40)

static const char TAG[] = "PMSerial";
static uint8_t *dataBuf = nullptr;

static bool tsi_mode = false;
static bool truncated_num = false;

static QueueHandle_t uart_queue;
static TaskHandle_t recv_task_handle = nullptr;

static bool is_uart_driver_installed = false;
static bool is_inited = false;
static bool is_buffered = false;

const uint8_t TSI_START = 4, // PM [ug/m3] (TSI standard)
    ATM_START = 10,          // PM [ug/m3] (std. atmosphere)
    NUM_START = 16;          // num. particles in 100 cm3 of air

const uint8_t msgLen = 7,                                          //
    cmd_actv[msgLen] = {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71}, // set actv
    cmd_slep[msgLen] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73}, // sleep
    cmd_wkup[msgLen] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74}, // wake
    cmd_pasv[msgLen] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70}, // set pasv
    cmd_read[msgLen] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71}; // pasv read

uint16_t SerialPM::buff2wd(uint8_t n, const uint8_t *raw_data) {
  // read two bytes starting at index n (n and n+1); caller must ensure bounds
  return (uint16_t(raw_data[n]) << 8) | uint16_t(raw_data[n + 1]);
}

esp_err_t SerialPM::init() {
  if (uart_is_driver_installed(uart_port)) {
    ESP_LOGE(
        TAG,
        "uart is already initialized, delete driver before call SerialPM.");
    return ESP_FAIL;
  }

  uart_config_t uart_config;
  uart_config.baud_rate = 9600;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  uart_config.rx_flow_ctrl_thresh = 122;

  ESP_ERROR_CHECK(uart_driver_install(uart_port, UART_BUF_SIZE * 2,
                                      UART_BUF_SIZE * 2, 20, &uart_queue, 0));
  uart_param_config(uart_port, &uart_config);
  uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);
  is_uart_driver_installed = true;

  dataBuf = (uint8_t *)malloc(RD_BUF_SIZE);
  if (!dataBuf) {
    ESP_LOGE(TAG, "Failed to allocate dataBuf");
    return ESP_ERR_NO_MEM;
  }

  BaseType_t created = xTaskCreate(SerialPM::uart_recv_handler_launcher,
                                   "uart_recv_handler_launcher", 3072, this, 12,
                                   &recv_task_handle);
  if (created != pdPASS) {
    ESP_LOGE(TAG, "Failed to create UART receive task");
    free(dataBuf);
    dataBuf = nullptr;
    return ESP_FAIL;
  }

  // set passive mode (non-blocking). We allow set_mode to run now; the rx task
  // will set is_buffered true once it receives valid frames.
  esp_err_t ret = set_mode(PMSX_SENSOR_MODE_PASSIVE);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "set_mode returned error during init");
    // continue initialization even on error, caller can handle
  }
  is_inited = true;
  return ESP_OK;
}

void SerialPM::uart_recv_handler() {
  uart_event_t event;
  while (true) {
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {

        if (event.size > RD_BUF_SIZE * 2) {
          ESP_LOGW(TAG,
                   "uart event size is bigger than RD_BUF_SIZE, flush fifo.");
          ESP_ERROR_CHECK(uart_flush_input(uart_port));
          continue;
        }

        memset(dataBuf, 0, RD_BUF_SIZE);
        is_buffered = false;
        int uart_data_size =
            uart_read_bytes(uart_port, dataBuf, event.size, portMAX_DELAY);

        if (uart_data_size <= 0 || RD_BUF_SIZE * 2 < uart_data_size) {
          ESP_LOGW(TAG, "uart_read_bytes returned %d. flush fifo.",
                   uart_data_size);
          ESP_ERROR_CHECK(uart_flush_input(uart_port));
          continue;
        }

        ESP_LOGD(TAG, "data recv: %d bytes", uart_data_size);

        if (!is_inited && uart_data_size == 1)
          is_buffered = true;
        pmsx_uart_status_t st;
        st = data_checker(dataBuf, uart_data_size);
        status = st;
        if (st == OK) {
          if (buff2wd(2, dataBuf) != 4) { // is not command response,
            data_parser(dataBuf, tsi_mode, truncated_num);
          }
          is_buffered = true;
        }
      }
    }
  }
  return;
}

esp_err_t SerialPM::uart_send_cmd(uint8_t *cmd) {

  if (!is_inited && !is_buffered) {
    int64_t time_wait_until = esp_timer_get_time() + 3000 * 1000;
    while (!is_buffered) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (time_wait_until < esp_timer_get_time()) {
        ESP_LOGE(TAG, "UART timed out.");
        return ESP_FAIL;
      }
    }
  }
  uart_write_bytes(uart_port, cmd, msgLen);

  esp_err_t ret;
  ret = uart_wait_tx_done(uart_port, portMAX_DELAY);
  return ret;
}

esp_err_t SerialPM::set_mode(pmsx_sensor_mode_t _mode) {
  esp_err_t ret;
  uint8_t *cmd;
  if (sensor_mode == _mode)
    return ESP_OK;

  if (_mode == PMSX_SENSOR_MODE_ACTIVE) {
    cmd = (uint8_t *)cmd_actv;
  } else {
    cmd = (uint8_t *)cmd_pasv;
  }
  ret = uart_send_cmd(cmd);
  if (ret != ESP_OK)
    return ret;

  int64_t time_wait_until = esp_timer_get_time() + max_wait_ms * 1000;
  while (sensor_mode != _mode) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (time_wait_until < esp_timer_get_time()) {
      ESP_LOGE(TAG, "mode cmd timed out.");
      return ESP_FAIL;
    }
  }
  ESP_LOGI(TAG, "sensor mode changed.");
  return ret;
}

esp_err_t SerialPM::set_sleep(pmsx_sensor_sleep_t _sleep) {
  esp_err_t ret;
  uint8_t *cmd;
  if (sleep_state == _sleep)
    return ESP_OK;

  if (_sleep == PMSX_SENSOR_SLEEP) {
    cmd = (uint8_t *)cmd_slep;
  } else {
    cmd = (uint8_t *)cmd_wkup;
  }
  ret = uart_send_cmd(cmd);
  if (ret != ESP_OK)
    return ret;

  int64_t time_wait_until = esp_timer_get_time() + max_wait_ms * 1000;
  while (sleep_state != _sleep) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (time_wait_until < esp_timer_get_time()) {
      ESP_LOGE(TAG, "sleep/awake cmd timed out.");
      return ESP_FAIL;
    }
  }
  return ret;
}

SerialPM::~SerialPM() {
  if (recv_task_handle) {
    vTaskDelete(recv_task_handle);
    recv_task_handle = nullptr;
  }
  if (is_uart_driver_installed) {
    ESP_ERROR_CHECK(uart_driver_delete(uart_port));
    is_uart_driver_installed = false;
  }
  if (dataBuf) {
    free(dataBuf);
    dataBuf = nullptr;
  }
}

pmsx_uart_status_t SerialPM::data_checker(uint8_t *raw_data,
                                          uint8_t raw_data_length) {
  if (buff2wd(0, raw_data) != 0x424D)
    return ERROR_MSG_HEADER;

  uint16_t Frame_Length = buff2wd(2, raw_data);

  if (Frame_Length > RD_BUF_SIZE - 4)
    return ERROR_MSG_LENGTH;

  uint8_t Msg_Length = Frame_Length + 4;
  if (Msg_Length > raw_data_length)
    return ERROR_MSG_LENGTH;

  if (!checkBuffer(Msg_Length, raw_data))
    return ERROR_MSG_CKSUM;

  if (Msg_Length != 8) { // flame_len 4 (datalen 8) is command return
    pmsx_sensor_type_t detected_sensor;
    switch (Msg_Length) {
    case 24:
      detected_sensor = PLANTOWER_24B;
      break;
    case 32:
      detected_sensor = PLANTOWER_32B;
      break;
    case 40:
      detected_sensor = PLANTOWER_40B;
      break;
    default:
      return ERROR_MSG_UNKNOWN;
    }

    // self discovery
    if (sensor_type == PLANTOWER_AUTO)
      sensor_type = detected_sensor;

    if (sensor_type != detected_sensor) {
      if (detected_sensor == PLANTOWER_32B && sensor_type != PLANTOWER_32B_S &&
          sensor_type != PLANTOWER_32B_T) {
        return ERROR_PMS_TYPE;
      }
    }
  } else {
    // Frame length=8, parse command resp
    uint16_t cmd = buff2wd(4, raw_data);
    switch (cmd) {
    case 0xe100:
      ESP_LOGI(TAG, "Mode set to PASV");
      sensor_mode = PMSX_SENSOR_MODE_PASSIVE;
      break;
    case 0xe101:
      ESP_LOGI(TAG, "Mode set to ACTV");
      sensor_mode = PMSX_SENSOR_MODE_ACTIVE;
      break;
    case 0xe400:
      ESP_LOGI(TAG, "Device set to SLEEP");
      sleep_state = PMSX_SENSOR_SLEEP;
      break;
    case 0xe401:
      ESP_LOGI(TAG, "Device set to WakeUP");
      sleep_state = PMSX_SENSOR_AWAKE;
      break;
    default:
      ESP_LOGW(TAG, "Unknown reply from PMSx device");
      break;
    }
  }
  return OK;
}

bool SerialPM::checkBuffer(uint8_t Len, uint8_t *raw_data) {
  if (Len < 2)
    return false;

  uint16_t cksum = buff2wd(Len - 2, raw_data);
  uint32_t sum = 0;
  for (size_t n = 0; n < Len - 2; n++)
    sum += raw_data[n];

  return (cksum == uint16_t(sum & 0xFFFF));
}

void SerialPM::data_parser(uint8_t *raw_data, bool tsi_mode,
                           bool truncated_num) {
  if (!has_particulate_matter())
    return;

  uint16_t Frame_Length = buff2wd(2, raw_data);
  uint8_t Msg_Length = Frame_Length + 4;

  for (uint8_t bin = 0, n = tsi_mode ? TSI_START : ATM_START; bin < 3;
       bin++, n += 2) {
    if (n + 1 >= Msg_Length) {
      pm[bin] = 0;
    } else {
      pm[bin] = buff2wd(n, raw_data);
    }
  }

  if (!has_number_concentration())
    return;
  for (uint8_t bin = 0, n = NUM_START; bin < 6; bin++, n += 2) {
    if (n + 1 >= Msg_Length) {
      nc[bin] = 0;
    } else {
      nc[bin] = buff2wd(n, raw_data); // number particles w/diameter > r_bin
    }
  }

  switch (sensor_type) {
  case PMS5003S:
    hcho = buff2wd(28, raw_data) * 1e-3;
    break;
  case PMS5003T:
    temp = int16_t(n5p0) * 1e-1 + temp_offset; // cast to signed integer 16bits
    rhum = n10p0 * 1e-1 + rhum_offset;
    n5p0 = 0;
    n10p0 = 0;
    break;
  case PMS5003ST:
    hcho = buff2wd(28, raw_data) * 1e-3;
    temp = int16_t(buff2wd(30, raw_data)) * 1e-1 +
           temp_offset; // cast to signed integer 16bits
    rhum = buff2wd(32, raw_data) * 1e-1 + rhum_offset;
    break;
  default:
    hcho = 0;
    rhum = 0;
    temp = 0;
    break;
  }

  if (!truncated_num)
    return;
  for (uint8_t bin = 0; bin < 5; bin++) {
    nc[bin] -= nc[bin + 1]; // de-accumulate number concentrations
  }
}

pmsx_uart_status_t SerialPM::read(bool tsi_mode, bool truncated_num) {
  if (sensor_mode == PMSX_SENSOR_MODE_PASSIVE)
    uart_send_cmd((uint8_t *)&cmd_read);
  return status;
}

uint16_t *SerialPM::get_data() {
  if (sensor_mode == PMSX_SENSOR_MODE_PASSIVE)
    uart_send_cmd((uint8_t *)&cmd_read);
  return data;
}
