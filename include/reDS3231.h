/* 
   RU: Библиотека для работы с часами реального времени на микросхеме DS3231
   EN: Library for working with real-time clock on the DS3231 chip
   --------------------------------------------------------------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_DS3231_H__
#define __RE_DS3231_H__

#include <stdint.h>
#include <esp_err.h>
#include <sys/time.h> 
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS3231_ADDR 0x68

/**
 * Alarm 1
 */
typedef struct 
{
  bool    inerrupt_enabled;
  bool    mday_match;
  uint8_t mday_value;
  bool    wday_match;
  uint8_t wday_value;
  bool    hours_match;
  uint8_t hours_value;
  bool    minutes_match;
  uint8_t minutes_value;
  bool    seconds_match;
  uint8_t seconds_value;
} ds3231_alarm1_t;

/**
 * Alarm 2
 */
typedef struct 
{
  bool    inerrupt_enabled;
  bool    mday_match;
  uint8_t mday_value;
  bool    wday_match;
  uint8_t wday_value;
  bool    hours_match;
  uint8_t hours_value;
  bool    minutes_match;
  uint8_t minutes_value;
} ds3231_alarm2_t;

/**
 * Squarewave frequency
 */
typedef enum {
    DS3231_SQW_INTERRUPT = 0, // Alarm interrupt
    DS3231_SQW_1HZ,           // Squarewave 1 Hz
    DS3231_SQW_1024HZ,        // Squarewave 1024 Hz
    DS3231_SQW_4096HZ,        // Squarewave 4096 Hz
    DS3231_SQW_8192HZ         // Squarewave 8192 Hz
} ds3231_sqw_t;

/**
 * Real-time clock DS3231
 */
class reDS3231 {
  public:
    reDS3231(i2c_port_t numI2C, uint8_t addrI2C);
    ~reDS3231();

    bool start();
    bool is_running();

    bool set_config(bool bat_osc_enabled, bool bat_sqw_enabled);

    bool get_oscillator_stop_flag(bool * flag);
    bool reset_oscillator_stop_flag();

    bool get_time(struct tm *time);
    bool set_time(struct tm *time);

    bool get_alarm1_config(ds3231_alarm1_t *alarm);
    bool set_alarm1_config(ds3231_alarm1_t *alarm);
    bool get_alarm1_flag(bool *status);

    bool get_alarm2_config(ds3231_alarm2_t *alarm);
    bool set_alarm2_config(ds3231_alarm2_t *alarm);
    bool get_alarm2_flag(bool *status);

    bool get_squarewave(ds3231_sqw_t *freq);
    bool set_squarewave(ds3231_sqw_t freq);
  private:
    i2c_port_t _numI2C = I2C_NUM_0; 
    uint8_t _addrI2C = 0;
    bool _enabled = false;

    bool read_register(uint8_t reg, uint8_t *val, uint8_t size);
    bool write_register(uint8_t reg, uint8_t *val, uint8_t size);
};

#ifdef __cplusplus
}
#endif

#endif // __RE_DS3231_H__