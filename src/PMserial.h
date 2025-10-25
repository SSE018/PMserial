/* SerialPM
 Arduino library for PM sensors with serial interface
  PMS1003 aka G1
  PMS3003 aka G2
  PMS5003 aka G5
  PMS7003 aka G7
  PMSA003 aka G10
*/
#ifndef _SERIALPM_H
#define _SERIALPM_H

#include "esp_err.h"
#include "hal/uart_types.h"
#include "soc/gpio_num.h"
#include <stdint.h>

typedef enum {
  PLANTOWER_AUTO,  // self discovery
  PLANTOWER_24B,   // 24 byte long message, no count info (LBC)
  PLANTOWER_32B,   // 32 byte long message, w/count info (LBC)
  PLANTOWER_32B_S, // 32 byte long message, w/count info and HCHO (LBC)
  PLANTOWER_32B_T, // 32 byte long message, w/partial count info, temp and
                   // rhum (LBC)
  PLANTOWER_40B,   // 40 byte long message, w/count info, temp, rhum and HCHO
                   // (LBC)
  PMSx003 = PLANTOWER_AUTO,   // self discovery
  PMS1003 = PLANTOWER_32B,    // G1
  PMS3003 = PLANTOWER_24B,    // G3
  PMS5003 = PLANTOWER_32B,    // G5
  PMS5003S = PLANTOWER_32B_S, // G5S
  PMS5003T = PLANTOWER_32B_T, // G5T
  PMS5003ST = PLANTOWER_40B,  // G5ST
  PMS7003 = PLANTOWER_32B,    // G7
  PMSA003 = PLANTOWER_32B     // G10
} pmsx_sensor_type_t;

typedef enum {
  PMSX_SENSOR_MODE_PASSIVE,
  PMSX_SENSOR_MODE_ACTIVE,
} pmsx_sensor_mode_t;

typedef enum {
  PMSX_SENSOR_SLEEP,
  PMSX_SENSOR_AWAKE,
} pmsx_sensor_sleep_t;

typedef enum {
  OK,
  ERROR_TIMEOUT,
  ERROR_PMS_TYPE,
  ERROR_MSG_UNKNOWN,
  ERROR_MSG_HEADER,
  ERROR_MSG_BODY,
  ERROR_MSG_START,
  ERROR_MSG_LENGTH,
  ERROR_MSG_CKSUM
} pmsx_uart_status_t;

#define PMS_ERROR_TIMEOUT "Sensor read timeout"
#define PMS_ERROR_PMS_TYPE "Wrong PMSx003 sensor type"
#define PMS_ERROR_MSG_UNKNOWN "Unknown message protocol"
#define PMS_ERROR_MSG_HEADER "Incomplete message header"
#define PMS_ERROR_MSG_BODY "Incomplete message body"
#define PMS_ERROR_MSG_START "Wrong message start"
#define PMS_ERROR_MSG_LENGTH "Message too long"
#define PMS_ERROR_MSG_CKSUM "Wrong message checksum"

class SerialPM {
public:
  union {
    uint16_t data[9]; // all PM/NC data
    struct {
      uint16_t pm[3]; // particulate matter [ug/m3]
      uint16_t nc[6]; // number concentration [#/100cc]
    };
    struct {
      // pmX [ug/m3]: PM1.0, PM2.5 & PM10
      uint16_t pm01, pm25, pm10;
      // nXpY [#/100cc]: number concentrations under X.Y um
      uint16_t n0p3, n0p5, n1p0, n2p5, n5p0, n10p0;
    };
  };
  union {
    float extra[3]; // T/RH/HCHO
    struct {
      // temperature [°C], relative humidity [%], formaldehyde
      // concentration [mg/m3]
      float temp, rhum, hcho;
    };
  };

  SerialPM(pmsx_sensor_type_t _sensor, uart_port_t _uart_port,
           gpio_num_t _rx_pin, gpio_num_t _tx_pin)
      : sensor_type(_sensor), uart_port(_uart_port), rx_pin(_rx_pin),
        tx_pin(_tx_pin), sensor_mode(PMSX_SENSOR_MODE_ACTIVE),
        sleep_state(PMSX_SENSOR_AWAKE) {
    SerialPM::init();
  };

  // usage SerialPM read();
  // if sensor is passive, xmit uart pasv read cmd.
  // if active ,do nothing.
  pmsx_uart_status_t read(bool tsi_mode = false, bool truncated_num = false);
  uint16_t *get_data();

  pmsx_uart_status_t get_status() { return status; }
  operator bool() { return status == OK; }

  esp_err_t set_mode(pmsx_sensor_mode_t _mode);
  esp_err_t set_sleep(pmsx_sensor_sleep_t _sleep);
  inline pmsx_sensor_mode_t get_mode() { return sensor_mode; }
  inline pmsx_sensor_sleep_t get_sleep() { return sleep_state; }

  // adding offsets works well in normal range
  // might introduce under- or overflow at the ends of the sensor range
  inline void set_rhum_offset(float _offset) { rhum_offset = _offset; };
  inline void set_temp_offset(float _offset) { temp_offset = _offset; };
  inline float get_rhum_offset() { return rhum_offset; };
  inline float get_temp_offset() { return temp_offset; };

  ~SerialPM();

protected:
  // variables
  pmsx_sensor_type_t sensor_type; // sensor type/message protocol
  uart_port_t uart_port;          // uart config
  gpio_num_t rx_pin, tx_pin;      // uart config
  pmsx_sensor_mode_t sensor_mode;
  pmsx_uart_status_t status;
  pmsx_sensor_sleep_t sleep_state;

  float temp_offset = 0.0; // Correct Temperature & Humidity
  float rhum_offset = 0.0;

  // Functions
  esp_err_t init();         // called by constructor
  void uart_recv_handler(); // uart event handler and wrapper
  static void uart_recv_handler_launcher(void *args) {
    static_cast<SerialPM *>(args)->uart_recv_handler();
    return;
  }

  // utility functions
  uint16_t buff2wd(uint8_t n, uint8_t *raw_data);
  bool checkBuffer(size_t Len, uint8_t *raw_data);
  esp_err_t uart_send_cmd(uint8_t *cmd);
  pmsx_uart_status_t data_checker(uint8_t *raw_data, uint8_t raw_data_length);
  void data_parser(uint8_t *raw_data, bool tsi_mode, bool truncated_num);

  // message timing
  static const uint16_t max_wait_ms = 1000;

  // inline functions
  inline bool has_particulate_matter() { return status == OK; }
  inline bool has_number_concentration() {
    return (status == OK) && (sensor_type != PMS3003);
  }
  inline bool has_temperature_humidity() {
    return (status == OK) &&
           ((sensor_type == PMS5003T) || (sensor_type == PMS5003ST));
  }
  inline bool has_formaldehyde() {
    return (status == OK) &&
           ((sensor_type == PMS5003S) || (sensor_type == PMS5003ST));
  }
};
#endif //_SERIALPM_H
