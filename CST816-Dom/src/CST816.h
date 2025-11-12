#ifndef CST816_H
#define CST816_H

#include <Arduino.h>

// Touchpad mode
enum touchpad_mode { 
  mode_touch, 
  mode_change, 
  mode_fast, 
  mode_motion 
};

// Touch coordinates
struct TouchCoordinates {
  uint16_t points;
  uint16_t X_Pos;
  uint16_t Y_Pos;
};

// Touch gesture
struct TouchGesture {
  uint8_t gesture_id;
  uint8_t touch_state;
};

class CST816 {
public:
  // Constructor
  CST816(uint8_t sda_pin = 11, uint8_t scl_pin = 7, uint8_t rst_pin = 10, uint8_t irq_pin = 9);
  
  // Initialization
  bool begin(touchpad_mode mode = mode_touch);
  
  // Basic functions
  void reset();
  uint8_t getChipID();
  
  // Touch data
  TouchCoordinates getTouch();
  TouchGesture getGesture();
  void clearTouchData();
  
  // Configuration
  void disableAutoSleep(bool disable = true);
  void setTouchMode(touchpad_mode mode);
  
  // Status
  bool isTouched();
  bool available();

private:
  // Pin definitions
  uint8_t _sda_pin;
  uint8_t _scl_pin;
  uint8_t _rst_pin;
  uint8_t _irq_pin;
  
  // Device info
  uint8_t _chip_id;
  touchpad_mode _current_mode;
  uint8_t _touch_state;
  
  // Low-level GPIO control
  void SDA_OUT();
  void SDA_IN();
  void SDA_SET();
  void SDA_CLR();
  uint8_t SDA_GET();
  void SCL_SET();
  void SCL_CLR();
  void delay_us(uint32_t us);
  
  // I2C communication
  void i2c_start();
  void i2c_stop();
  void i2c_ack();
  void i2c_nack();
  uint8_t i2c_wait_ack();
  void i2c_send_byte(uint8_t byte);
  uint8_t i2c_receive_byte();
  
  // Register operations
  void write_register(uint8_t reg, uint8_t data);
  uint8_t read_register(uint8_t reg);
  void receive_byte(uint8_t addr, uint8_t* data);
  
  // Constants
  static const uint8_t CST816_ADDR = 0x2A;
  static const uint8_t REG_CHIP_ID = 0xA3;
  static const uint8_t REG_DIS_AUTO_SLEEP = 0xA5;
  static const uint8_t REG_IRQ_CTRL = 0xA6;
  static const uint8_t REG_MOTION_MASK = 0xA9;
  static const uint8_t REG_AUTO_SLEEP_TIME = 0xAB;
  static const uint8_t REG_GESTURE_ID = 0x01;
  static const uint8_t REG_AUTO_RESET = 0xA7;
  
  // IRQ enable bits
  static const uint8_t IRQ_EN_TOUCH = 0x01;
  static const uint8_t IRQ_EN_CHANGE = 0x02;
  static const uint8_t IRQ_EN_MOTION = 0x04;
  static const uint8_t IRQ_EN_LONGPRESS = 0x08;
  static const uint8_t MOTION_MASK_DOUBLE_CLICK = 0x02;
};

#endif
