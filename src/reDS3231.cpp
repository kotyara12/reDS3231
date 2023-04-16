#include "reDS3231.h"
#include "reI2C.h"
#include "rLog.h"
#include "reEvents.h"

static const char* logTAG = "DS3231";

#define I2C_TIMEOUT         1000

#define REG_SECONDS         0x00
#define REG_MINUTES         0x01
#define REG_HOURS           0x02
#define REG_WEEKDAY         0x03
#define REG_DAY             0x04
#define REG_MONTH           0x05
#define REG_YEAR            0x06
#define REG_DATETIME        REG_SECONDS

#define REG_ALARM1_SECONDS  0x07
#define REG_ALARM1_MINUTES  0x08
#define REG_ALARM1_HOURS    0x09
#define REG_ALARM1_DAY      0x0A
#define REG_ALARM1          REG_ALARM1_SECONDS

#define REG_ALARM2_MINUTES  0x0B
#define REG_ALARM2_HOURS    0x0C
#define REG_ALARM2_DAY      0x0D
#define REG_ALARM2          REG_ALARM2_MINUTES

#define REG_CONTROL         0x0E
#define REG_STATUS          0x0F

#define REG_AGING_OFFSET    0x10
#define REG_AGING_TEMP_LSB  0x11
#define REG_AGING_TEMP_MSB  0x12

#define BYTE_SECONDS        0x00
#define BYTE_MINUTES        0x01
#define BYTE_HOURS          0x02
#define BYTE_WDAY           0x03
#define BYTE_MDAY           0x04
#define BYTE_MONTH          0x05
#define BYTE_YEAR           0x06
#define BUFSIZE_DATETIME    0x07

#define BYTE_A1_SECONDS     0x00
#define BYTE_A1_MINUTES     0x01
#define BYTE_A1_HOURS       0x02
#define BYTE_A1_DAY         0x03
#define BUFSIZE_ALARM1      0x04

#define BYTE_A2_MINUTES     0x00
#define BYTE_A2_HOURS       0x01
#define BYTE_A2_DAY         0x02
#define BUFSIZE_ALARM2      0x03

#define BIT_HOUR12          (1 << 6)
#define BIT_HOURPM          (1 << 5)
#define BIT_CENTURY         (1 << 7)
#define BIT_ALARM_DAY       (1 << 6)
#define BIT_ALARM_MATCH     (1 << 7)

#define BIT_CONTROL_A1IE    (1 << 0)  // Bit 0: Alarm 1 Interrupt Enable (A1IE)
#define BIT_CONTROL_A2IE    (1 << 1)  // Bit 1: Alarm 2 Interrupt Enable (A2IE)
#define BIT_CONTROL_INTCN   (1 << 2)  // Bit 2: Interrupt Control (INTCN)
#define BIT_CONTROL_RS1     (1 << 3)  // Bits 3 and 4: Rate Select (RS1 and RS2)
#define BIT_CONTROL_RS2     (1 << 4)  
#define BIT_CONTROL_CONV    (1 << 5)  // Bit 5: Convert Temperature (CONV)
#define BIT_CONTROL_BBSQW   (1 << 6)  // Bit 6: Battery-Backed Square-Wave Enable (BBSQW)
#define BIT_CONTROL_EOSC    (1 << 7)  // Bit 7: Enable Oscillator (EOSC)

#define BIT_STATUS_A1F      (1 << 0)  // Bit 0: Alarm 1 Flag (A1F)
#define BIT_STATUS_A2F      (1 << 1)  // Bit 1: Alarm 2 Flag (A2F)
#define BIT_STATUS_BSY      (1 << 2)  // Bit 2: Busy (BSY)
#define BIT_STATUS_EN32KHZ  (1 << 3)  // Bit 3: Enable 32kHz Output (EN32kHz)
#define BIT_STATUS_OSF      (1 << 7)  // Bit 7: Oscillator Stop Flag (OSF)

#define BIT_SIGN            (1 << 7)

#define MASK_SECONDS        0x7F
#define MASK_MINUTES        0x7F
#define MASK_HOUR12         0x1F
#define MASK_HOUR24         0x3F
#define MASK_WDAY           0x07
#define MASK_MDAY           0x3F
#define MASK_MONTH          0x1F
#define MASK_YEAR           0xFF
#define MASK_SQW_RATE       0x18
#define MASK_AGING_OFFSET   0x7F
#define MASK_TEMP_LSB       0x7F
#define MASK_TEMP_MSB       0xC0

static uint8_t bcd2dec(uint8_t val)
{
  return (val >> 4) * 10 + (val & 0x0F);
}

static uint8_t dec2bcd(uint8_t val)
{
  return ((val / 10) << 4) + (val % 10);
}

reDS3231::reDS3231(i2c_port_t numI2C, uint8_t addrI2C)
{
  _numI2C = numI2C;
  _addrI2C = addrI2C;
}

reDS3231::~reDS3231()
{

}

bool reDS3231::read_register(uint8_t reg, uint8_t *val, uint8_t size)
{
  esp_err_t err = readI2C(_numI2C, _addrI2C, &reg, 1, val, size, 0, I2C_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to read DS3231 register #%d: %d (%s)", reg, err, esp_err_to_name(err));
    return false;
  };
  return true;
}

bool reDS3231::write_register(uint8_t reg, uint8_t *val, uint8_t size)
{
  esp_err_t err = writeI2C(_numI2C, _addrI2C, &reg, 1, val, size, I2C_TIMEOUT);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to write DS3231 register #%d: %d (%s)", reg, err, esp_err_to_name(err));
    return false;
  };
  return true;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Start --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reDS3231::is_running()
{
  return _enabled;  
}

bool reDS3231::start()
{
  _enabled = set_config(true, false);
  return _enabled;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Configure ------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reDS3231::set_config(bool bat_osc_enabled, bool bat_sqw_enabled)
{
  uint8_t control;
  if (read_register(REG_CONTROL, &control, 1)) {
    bat_osc_enabled ? control &= ~BIT_CONTROL_EOSC : control |= BIT_CONTROL_EOSC;
    bat_sqw_enabled ? control |= BIT_CONTROL_BBSQW : control &= ~BIT_CONTROL_BBSQW;
    if (write_register(REG_CONTROL, &control, 1)) {
      rlog_i(logTAG, "Set config: bat osc enabled=%d, bat sqw enabled=%d", bat_osc_enabled, bat_sqw_enabled);
      return true;
    };
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------- Oscillator Stop Flag ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reDS3231::get_oscillator_stop_flag(bool * flag)
{
  uint8_t buf;
  if (flag && read_register(REG_CONTROL, &buf, 1)) {
    *flag = (buf & BIT_STATUS_OSF);
    return true;
  };
  return false;
}

bool reDS3231::reset_oscillator_stop_flag()
{
  uint8_t buf;
  if (read_register(REG_CONTROL, &buf, 1)) {
    if (buf & BIT_STATUS_OSF) {
      buf &= ~BIT_STATUS_OSF;
      if (write_register(REG_CONTROL, &buf, 1)) {
        rlog_i(logTAG, "Reset oscillator stop flag");
      } else {
        return false;
      };
    };
    return true;
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------- Date and time ----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
bool reDS3231::get_time(struct tm *time)
{
  uint8_t buf[BUFSIZE_DATETIME];
  if (time && read_register(REG_DATETIME, (uint8_t*)&buf, sizeof(buf))) {
    // tm_sec - seconds after the minute (0 - 59) | register: 0 - 59
    time->tm_sec = bcd2dec(buf[BYTE_SECONDS] & MASK_SECONDS);
    // tm_min - minutes after the hour (0 - 59) | register: 0 - 59
    time->tm_min = bcd2dec(buf[BYTE_MINUTES] & MASK_MINUTES);
    // tm_hour - hours since midnight (from 0 to 23) | register: 1 - 12 (+ AM/PM) or 0 - 23
    if (buf[BYTE_HOURS] & BIT_HOUR12) {
      // 12-hour mode
      time->tm_hour = bcd2dec(buf[BYTE_HOURS] & MASK_HOUR12) - 1;
      if (buf[BYTE_HOURS] & BIT_HOURPM) time->tm_hour += 12;
    } else {
      // 24-hour mode
      time->tm_hour = bcd2dec(buf[BYTE_HOURS] & MASK_HOUR24);
    };
    // tm_wday - day of the week (0 - 6; Sunday = 0) | register: 1 - 7
    time->tm_wday = bcd2dec(buf[BYTE_WDAY] & MASK_WDAY) - 1;
    // tm_mday - day of the month (from 1 to 31) | register: 1 - 31
    time->tm_mday = bcd2dec(buf[BYTE_MDAY] & MASK_MDAY);
    // tm_mon - month (0 - 11; January = 0) | register: 1 - 12
    time->tm_mon  = bcd2dec(buf[BYTE_MONTH] & MASK_MONTH) - 1;
    // tm_year - year (current year minus 1900) | register: 0 - 99
    time->tm_year = bcd2dec(buf[BYTE_YEAR] & MASK_YEAR) + 100;

    rlog_i(logTAG, "Read time: year=%d, month=%d, mday=%d, wday=%d, hour=%d, min=%d, sec=%d",
      time->tm_year - 100, time->tm_mon + 1, time->tm_mday, time->tm_wday + 1, time->tm_hour, time->tm_min, time->tm_sec);
    return true;
  };
  return false;
}

bool reDS3231::set_time(struct tm *time)
{
  if (time) {
    uint8_t buf[BUFSIZE_DATETIME] = {0};
    // tm_sec - seconds after the minute (0 - 59) | register: 0 - 59
    buf[BYTE_SECONDS] = dec2bcd(time->tm_sec) & MASK_SECONDS;
    // tm_min - minutes after the hour (0 - 59) | register: 0 - 59
    buf[BYTE_MINUTES] = dec2bcd(time->tm_min) & MASK_MINUTES;
    // tm_hour - hours since midnight (from 0 to 23) | register: 0 - 23 in 24-hour mode
    buf[BYTE_HOURS] = dec2bcd(time->tm_hour) & MASK_HOUR24;
    // tm_wday - day of the week (0 - 6; Sunday = 0) | register: 1 - 7
    buf[BYTE_WDAY] = dec2bcd(time->tm_wday + 1) & MASK_WDAY;
    // tm_mday - day of the month (from 1 to 31) | register: 1 - 31
    buf[BYTE_MDAY] = dec2bcd(time->tm_mday) & MASK_MDAY;
    // tm_mon - month (0 - 11; January = 0) | register: 1 - 12
    buf[BYTE_MONTH] = dec2bcd(time->tm_mon + 1) & MASK_MONTH;
    // tm_year - year (current year minus 1900) | register: 0 - 99
    buf[BYTE_YEAR] = dec2bcd(time->tm_year - 100) & MASK_YEAR;

    if (write_register(REG_DATETIME, (uint8_t*)&buf, sizeof(buf))) {
      rlog_i(logTAG, "Set time: year=%d, month=%d, mday=%d, wday=%d, hour=%d, min=%d, sec=%d",
        time->tm_year - 100, time->tm_mon + 1, time->tm_mday, time->tm_wday + 1, time->tm_hour, time->tm_min, time->tm_sec);
      return true;
    };
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Alarms -------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

bool reDS3231::get_alarm1_config(ds3231_alarm1_t * alarm)
{
  uint8_t control;
  uint8_t buf[BUFSIZE_ALARM1];
  if (alarm && read_register(REG_ALARM1, (uint8_t*)&buf, sizeof(buf)) && read_register(REG_CONTROL, &control, 1)) {
    alarm->inerrupt_enabled = control & BIT_CONTROL_A1IE;
    // Seconds
    alarm->seconds_match = (buf[BYTE_A1_SECONDS] & BIT_ALARM_MATCH);
    alarm->seconds_value = bcd2dec(buf[BYTE_A1_SECONDS] & MASK_SECONDS);
    // Minutes
    alarm->minutes_match = (buf[BYTE_A1_MINUTES] & BIT_ALARM_MATCH);
    alarm->minutes_value = bcd2dec(buf[BYTE_A1_MINUTES] & MASK_MINUTES);
    // Hours
    alarm->hours_match = (buf[BYTE_A1_HOURS] & BIT_ALARM_MATCH);
    if (buf[BYTE_A1_HOURS] & BIT_HOUR12) {
      // 12-hour mode
      alarm->hours_value = bcd2dec(buf[BYTE_A1_HOURS] & MASK_HOUR12) - 1;
      if (buf[BYTE_A1_HOURS] & BIT_HOURPM) alarm->hours_value += 12;
    } else {
      // 24-hour mode
      alarm->hours_value = bcd2dec(buf[BYTE_A1_HOURS] & MASK_HOUR24);
    };
    // Days
    if (buf[BYTE_A1_DAY] & BIT_ALARM_DAY) {
      // Day of week (0..6 as tm_wday)
      alarm->wday_match = (buf[BYTE_A1_DAY] & BIT_ALARM_MATCH);
      alarm->wday_value = bcd2dec(buf[BYTE_A1_DAY] & MASK_WDAY) - 1;
      alarm->mday_match = false;
      alarm->mday_value = 0;
    } else {
      // Day of month
      alarm->wday_match = false;
      alarm->wday_value = 0;
      alarm->mday_match = (buf[BYTE_A1_DAY] & BIT_ALARM_MATCH);
      alarm->mday_value = bcd2dec(buf[BYTE_A1_DAY] & MASK_MDAY);
    };

    rlog_i(logTAG, "Read alarm 1: mday=%d (%d), wday=%d (%d), hour=%d (%d), min=%d (%d), sec=%d (%d)",
      alarm->mday_value, alarm->mday_match, alarm->wday_value, alarm->wday_match,
      alarm->hours_value, alarm->hours_match, alarm->minutes_value, alarm->minutes_match, alarm->seconds_value, alarm->seconds_match);
    return true;
  };
  return false;
}

bool reDS3231::set_alarm1_config(ds3231_alarm1_t * alarm)
{
  uint8_t control;
  uint8_t buf[BUFSIZE_ALARM1] = {0};
  if (alarm && read_register(REG_CONTROL, &control, 1)) {
    alarm->inerrupt_enabled ? control |= BIT_CONTROL_A1IE : control &= ~BIT_CONTROL_A1IE;
    // Seconds
    buf[BYTE_A1_SECONDS] = dec2bcd(alarm->seconds_value) & MASK_SECONDS;
    if (alarm->seconds_match) buf[BYTE_A1_SECONDS] |= BIT_ALARM_MATCH;
    // Minutes
    buf[BYTE_A1_MINUTES] = dec2bcd(alarm->minutes_value) & MASK_MINUTES;
    if (alarm->minutes_match) buf[BYTE_A1_MINUTES] |= BIT_ALARM_MATCH;
    // Hours
    buf[BYTE_A1_HOURS] = dec2bcd(alarm->hours_value) & MASK_HOUR24;
    if (alarm->hours_match) buf[BYTE_A1_HOURS] |= BIT_ALARM_MATCH;
    // Days
    if (alarm->wday_match) {
      buf[BYTE_A1_DAY] = dec2bcd(alarm->wday_value) & MASK_WDAY;
      buf[BYTE_A1_DAY] |= BIT_ALARM_DAY;
      buf[BYTE_A1_DAY] |= BIT_ALARM_MATCH;
    } else {
      buf[BYTE_A1_DAY] = dec2bcd(alarm->mday_value) & MASK_MDAY;
      if (alarm->mday_match) buf[BYTE_A1_DAY] |= BIT_ALARM_MATCH;
    };
    if (write_register(REG_ALARM1, (uint8_t*)&buf, sizeof(buf)) && write_register(REG_CONTROL, &control, 1)) {
      rlog_i(logTAG, "Set alarm 1: mday=%d (%d), wday=%d (%d), hour=%d (%d), min=%d (%d), sec=%d (%d)", 
        alarm->mday_value, alarm->mday_match, alarm->wday_value, alarm->wday_match,
        alarm->hours_value, alarm->hours_match, alarm->minutes_value, alarm->minutes_match, alarm->seconds_value, alarm->seconds_match);
      return true;
    };
  };
  return false;
}

bool reDS3231::get_alarm1_flag(bool * status)
{
  uint8_t buf;
  if (status && read_register(REG_CONTROL, &buf, 1)) {
    *status = (buf & BIT_STATUS_A1F);
    return true;
  };
  return false;
}

bool reDS3231::get_alarm2_config(ds3231_alarm2_t * alarm)
{
  uint8_t control;
  uint8_t buf[BUFSIZE_ALARM2];
  if (alarm && read_register(REG_ALARM2, (uint8_t*)&buf, sizeof(buf)) && read_register(REG_CONTROL, &control, 1)) {
    alarm->inerrupt_enabled = control & BIT_CONTROL_A2IE;
    // Minutes
    alarm->minutes_match = (buf[BYTE_A2_MINUTES] & BIT_ALARM_MATCH);
    alarm->minutes_value = bcd2dec(buf[BYTE_A2_MINUTES] & MASK_MINUTES);
    // Hours
    alarm->hours_match = (buf[BYTE_A2_HOURS] & BIT_ALARM_MATCH);
    if (buf[BYTE_A2_HOURS] & BIT_HOUR12) {
      // 12-hour mode
      alarm->hours_value = bcd2dec(buf[BYTE_A2_HOURS] & MASK_HOUR12) - 1;
      if (buf[BYTE_A2_HOURS] & BIT_HOURPM) alarm->hours_value += 12;
    } else {
      // 24-hour mode
      alarm->hours_value = bcd2dec(buf[BYTE_A2_HOURS] & MASK_HOUR24);
    };
    // Days
    if (buf[BYTE_A2_DAY] & BIT_ALARM_DAY) {
      // Day of week (0..6 as tm_wday)
      alarm->wday_match = (buf[BYTE_A2_DAY] & BIT_ALARM_MATCH);
      alarm->wday_value = bcd2dec(buf[BYTE_A2_DAY] & MASK_WDAY) - 1;
      alarm->mday_match = false;
      alarm->mday_value = 0;
    } else {
      // Day of month
      alarm->wday_match = false;
      alarm->wday_value = 0;
      alarm->mday_match = (buf[BYTE_A2_DAY] & BIT_ALARM_MATCH);
      alarm->mday_value = bcd2dec(buf[BYTE_A2_DAY] & MASK_MDAY);
    };

    rlog_i(logTAG, "Read alarm 2: mday=%d (%d), wday=%d (%d), hour=%d (%d), min=%d (%d)",
      alarm->mday_value, alarm->mday_match, alarm->wday_value, alarm->wday_match,
      alarm->hours_value, alarm->hours_match, alarm->minutes_value, alarm->minutes_match);
    return true;
  };
  return false;
}

bool reDS3231::set_alarm2_config(ds3231_alarm2_t * alarm)
{
  uint8_t control;
  uint8_t buf[BUFSIZE_ALARM2] = {0};
  if (alarm && read_register(REG_CONTROL, &control, 1)) {
    alarm->inerrupt_enabled ? control |= BIT_CONTROL_A2IE : control &= ~BIT_CONTROL_A2IE;
    // Minutes
    buf[BYTE_A2_MINUTES] = dec2bcd(alarm->minutes_value) & MASK_MINUTES;
    if (alarm->minutes_match) buf[BYTE_A2_MINUTES] |= BIT_ALARM_MATCH;
    // Hours
    buf[BYTE_A2_HOURS] = dec2bcd(alarm->hours_value) & MASK_HOUR24;
    if (alarm->hours_match) buf[BYTE_A2_HOURS] |= BIT_ALARM_MATCH;
    // Days
    if (alarm->wday_match) {
      buf[BYTE_A2_DAY] = dec2bcd(alarm->wday_value) & MASK_WDAY;
      buf[BYTE_A2_DAY] |= BIT_ALARM_DAY;
      buf[BYTE_A2_DAY] |= BIT_ALARM_MATCH;
    } else {
      buf[BYTE_A2_DAY] = dec2bcd(alarm->mday_value) & MASK_MDAY;
      if (alarm->mday_match) buf[BYTE_A2_DAY] |= BIT_ALARM_MATCH;
    };
    if (write_register(REG_ALARM2, (uint8_t*)&buf, sizeof(buf)) && write_register(REG_CONTROL, &control, 1)) {
      rlog_i(logTAG, "Set alarm 2: mday=%d (%d), wday=%d (%d), hour=%d (%d), min=%d (%d)", 
        alarm->mday_value, alarm->mday_match, alarm->wday_value, alarm->wday_match,
        alarm->hours_value, alarm->hours_match, alarm->minutes_value, alarm->minutes_match);
      return true;
    };
  };
  return false;
}

bool reDS3231::get_alarm2_flag(bool * status)
{
  uint8_t buf;
  if (status && read_register(REG_CONTROL, &buf, 1)) {
    *status = (buf & BIT_STATUS_A2F);
    return true;
  };
  return false;
}

// -----------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------ Squarewave -----------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------
bool reDS3231::get_squarewave(ds3231_sqw_t *freq)
{
  uint8_t control;
  if (freq && read_register(REG_CONTROL, &control, 1)) {
    if (control & BIT_CONTROL_INTCN) {
      *freq = DS3231_SQW_INTERRUPT;
    } else {
      if (control & BIT_CONTROL_RS1) {
        *freq = control & BIT_CONTROL_RS2 ? DS3231_SQW_8192HZ : DS3231_SQW_1024HZ;
      } else {
        *freq = control & BIT_CONTROL_RS2 ? DS3231_SQW_4096HZ : DS3231_SQW_1HZ;
      };      
    };
    rlog_i(logTAG, "Read squarewave: %d", *freq);
    return true;
  };
  return false;
}

bool reDS3231::set_squarewave(ds3231_sqw_t freq)
{
  uint8_t control;
  if (freq && read_register(REG_CONTROL, &control, 1)) {
    if (freq == DS3231_SQW_INTERRUPT) {
      // Set bit 2: Interrupt Control (INTCN)
      control |= BIT_CONTROL_INTCN;
    } else {
      // Clear bit 2: Interrupt Control (INTCN)
      control &= ~BIT_CONTROL_INTCN;
      if (freq == DS3231_SQW_1HZ) {
        control &= ~BIT_CONTROL_RS1; 
        control &= ~BIT_CONTROL_RS2;
      } else if (freq == DS3231_SQW_1024HZ) {
        control |=  BIT_CONTROL_RS1;
        control &= ~BIT_CONTROL_RS2;
      } else if (freq == DS3231_SQW_4096HZ) {
        control &= ~BIT_CONTROL_RS1;
        control |=  BIT_CONTROL_RS2;
      } else {
        control |=  BIT_CONTROL_RS1;
        control |=  BIT_CONTROL_RS2;
      };
    };
    if (write_register(REG_CONTROL, &control, 1)) {
      rlog_i(logTAG, "Set squarewave: %d", freq);
      return true;
    };
  };
  return false;
}

