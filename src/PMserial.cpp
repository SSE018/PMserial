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
#include "hal/uart_types.h"
#include "portmacro.h"
#include <PMserial.h>
#include <cstddef>
#include <cstdint>
#include <iostream>

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
static uint8_t *databuf;
static uint8_t dataLen;

static bool tsi_mode = false;
static bool truncated_num = false;

static QueueHandle_t uart_queue;
static TaskHandle_t recv_task_handle;

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

  databuf = (uint8_t *)malloc(RD_BUF_SIZE);
  assert(databuf);

  xTaskCreate(SerialPM::uart_recv_handler_launcher,
              "uart_recv_handler_launcher", 3072, this, 12, &recv_task_handle);

  ESP_ERROR_CHECK_WITHOUT_ABORT(set_mode(PMSX_SENSOR_MODE_PASSIVE));

  is_inited = true;
  return ESP_OK;
}

void SerialPM::uart_recv_handler() {
  uart_event_t event;
  while (true) {
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {
        memset(databuf, 0, RD_BUF_SIZE);
        is_buffered = false;

        uart_read_bytes(uart_port, databuf, event.size, portMAX_DELAY);
        ESP_LOGD(TAG, "data recv");

        dataLen = event.size;

        if (!is_inited && dataLen == 1)
          is_buffered = true;

        // printf("%d: %d: ", dataLen, buff2wd(2));
        // for (int i = 0; i < dataLen; i += 2)
        //   printf("%04x ", buff2wd(i));
        // std::cout << std::endl;

        status = data_checker();
        if (status == OK) {
          is_buffered = true;
          if (dataLen != 8)
            data_parser(tsi_mode, truncated_num);
        }
      }
    }
  }
  std::cout << "handler end\n";
  return;
}

esp_err_t SerialPM::uart_wait_with_timeout(uint16_t xTimeinMS) {
  is_buffered = false;
  int64_t time_wait_until = esp_timer_get_time() + xTimeinMS * 1000;
  while (!is_buffered) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (esp_timer_get_time() > time_wait_until) {
      ESP_LOGE(TAG, "UART timed out.");
      return ESP_FAIL;
    }
  }
  return ESP_OK;
}

esp_err_t SerialPM::uart_send_cmd(uint8_t *cmd) {
  esp_err_t ret;
  if (!is_inited) {
    ret = uart_wait_with_timeout(3000);
    if (ret != ESP_OK)
      return ret;
  }
  uart_write_bytes(uart_port, cmd, msgLen);
  ret = uart_wait_tx_done(uart_port, portMAX_DELAY);
  if (ret != ESP_OK)
    return ret;

  ret = uart_wait_with_timeout(max_wait_ms);
  if (ret != ESP_OK)
    return ret;

  if (cmd != cmd_read && dataLen == 8) {
    switch (buff2wd(4)) {
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
  return ret;
}

esp_err_t SerialPM::set_mode(pmsx_sensor_mode_t _mode) {
  esp_err_t ret;
  uint8_t *cmd;
  if (sensor_mode != _mode) {
    if (_mode == PMSX_SENSOR_MODE_ACTIVE) {
      cmd = (uint8_t *)cmd_actv;
    } else {
      cmd = (uint8_t *)cmd_pasv;
    }
    ret = uart_send_cmd(cmd);
    ESP_LOGI(TAG, "cmd sent");
    return ret;
  }
  return ESP_OK;
}

esp_err_t SerialPM::set_sleep(pmsx_sensor_sleep_t _sleep) {
  esp_err_t ret;
  uint8_t *cmd;
  if (sleep_state != _sleep) {
    if (_sleep == PMSX_SENSOR_SLEEP) {
      cmd = (uint8_t *)cmd_slep;
    } else {
      cmd = (uint8_t *)cmd_wkup;
    }
    ret = uart_send_cmd(cmd);
    return ret;
  }
  return ESP_OK;
}

SerialPM::~SerialPM() {
  vTaskDelete(recv_task_handle);
  if (is_uart_driver_installed) {
    ESP_ERROR_CHECK(uart_driver_delete(uart_port));
    is_uart_driver_installed = false;
  }
  free(databuf);
  databuf = NULL;
}

uint16_t SerialPM::buff2wd(uint8_t n) {
  return (databuf[n] << 8) | databuf[n + 1];
}

pmsx_uart_status_t SerialPM::data_checker() {
  if (buff2wd(0) != 0x424D)
    return ERROR_MSG_HEADER;

  if (buff2wd(2) + 4 != dataLen)
    return ERROR_MSG_LENGTH;

  if (dataLen != 8) { // dataLen 8 is command return
    pmsx_sensor_type_t sensor_type_detect;
    switch (dataLen) {
    case 24:
      sensor_type_detect = PLANTOWER_24B;
      break;
    case 32:
      sensor_type_detect = PLANTOWER_32B;
      break;
    case 40:
      sensor_type_detect = PLANTOWER_40B;
      break;
    default:
      return ERROR_MSG_UNKNOWN;
    }

    // self discovery
    if (sensor_type == PLANTOWER_AUTO)
      sensor_type = sensor_type_detect;

    if (sensor_type != sensor_type_detect) {
      if (sensor_type_detect == PLANTOWER_32B &&
          sensor_type != PLANTOWER_32B_S && sensor_type != PLANTOWER_32B_T) {
        return ERROR_PMS_TYPE;
      }
    }
  }
  if (!checkBuffer(dataLen))
    return ERROR_MSG_CKSUM;
  return OK;
}

bool SerialPM::checkBuffer(size_t Len) {
  uint16_t cksum = buff2wd(Len - 2);
  for (uint8_t n = 0; n < Len - 2; n++) {
    cksum -= databuf[n];
  }
  return (cksum == 0);
}

void SerialPM::data_parser(bool tsi_mode, bool truncated_num) {
  uint8_t bin, n;
  if (!has_particulate_matter())
    return;
  for (bin = 0, n = tsi_mode ? TSI_START : ATM_START; bin < 3; bin++, n += 2) {
    pm[bin] = buff2wd(n);
  }

  if (!has_number_concentration())
    return;
  for (bin = 0, n = NUM_START; bin < 6; bin++, n += 2) {
    nc[bin] = buff2wd(n); // number particles w/diameter > r_bin
  }

  switch (sensor_type) {
  case PMS5003S:
    hcho = buff2wd(28) * 1e-3;
    break;
  case PMS5003T:
    temp = int16_t(n5p0) * 1e-1 + temp_offset; // cast to signed integer 16bits
    rhum = n10p0 * 1e-1 + rhum_offset;
    n5p0 = 0;
    n10p0 = 0;
    break;
  case PMS5003ST:
    hcho = buff2wd(28) * 1e-3;
    temp = int16_t(buff2wd(30)) * 1e-1 +
           temp_offset; // cast to signed integer 16bits
    rhum = buff2wd(32) * 1e-1 + rhum_offset;
    break;
  default:
    break;
  }

  if (!truncated_num)
    return;
  for (bin = 0; bin < 5; bin++) {
    nc[bin] -= nc[bin + 1]; // de-accumulate number concentrations
  }
}

pmsx_uart_status_t SerialPM::read(bool tsi_mode, bool truncated_num) {
  if (sensor_mode == PMSX_SENSOR_MODE_PASSIVE)
    uart_send_cmd((uint8_t *)&cmd_read);
  return status;
}
