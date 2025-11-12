#include "CST816.h"

/**
 * @brief Construct a new CST816::CST816 object
 * @param sda_pin GPIO pin number for I2C SDA
 * @param scl_pin GPIO pin number for I2C SCL
 * @param rst_pin GPIO pin number for Reset (can be -1 if not used)
 * @param irq_pin GPIO pin number for Interrupt (IRQ)
 */
CST816::CST816(uint8_t sda_pin, uint8_t scl_pin, uint8_t rst_pin, uint8_t irq_pin) {
  _sda_pin = sda_pin;
  _scl_pin = scl_pin;
  _rst_pin = rst_pin;
  _irq_pin = irq_pin;
  _current_mode = mode_touch; // Default mode
  _touch_state = 0;           // Internal state tracking
  _chip_id = 0;               // Cache for chip ID
}

/**
 * @brief Initialize the touch controller.
 * @param mode The desired operating mode (touch, change, fast, motion).
 * @return true if initialization is successful (always returns true in this implementation).
 */
bool CST816::begin(touchpad_mode mode) {
  _current_mode = mode;
  
  // Initialize GPIO pins
  if (_rst_pin != (uint8_t)-1) {
    pinMode(_rst_pin, OUTPUT); // Set Reset pin as output if it's connected
  }
  pinMode(_scl_pin, OUTPUT); // SCL (Clock) is always output for master
  pinMode(_sda_pin, OUTPUT); // SDA (Data) default to output
  pinMode(_irq_pin, INPUT);  // IRQ (Interrupt) is input from the chip

  // Reset device if a reset pin is available
  reset();
  
  // Disable auto-sleep mode on the chip by default
  disableAutoSleep(false);
  
  // Reset again for stability after configuration
  reset();
  
  // Configure chip registers based on the selected mode
  uint8_t irq_en = 0;
  uint8_t motion_mask = 0;
  
  switch (_current_mode) {
    case mode_touch: 
      irq_en = IRQ_EN_TOUCH; // Interrupt on touch
      break;
    case mode_change: 
      irq_en = IRQ_EN_CHANGE; // Interrupt on coordinates change
      break;
    case mode_fast: 
      irq_en = IRQ_EN_MOTION; // Interrupt on fast motion
      break;
    case mode_motion:
      // Interrupt on motion or long press
      irq_en = IRQ_EN_MOTION | IRQ_EN_LONGPRESS;
      motion_mask = MOTION_MASK_DOUBLE_CLICK; // Also enable double click
      break;
  }
  
  // Wait for the chip to stabilize
  delay(100);
  // Write the interrupt configuration to the register
  write_register(REG_IRQ_CTRL, irq_en);
  delay(100);
  // Write the motion mask configuration (e.g., enable/disable double click)
  write_register(REG_MOTION_MASK, motion_mask);
  delay(100);
  
  return true;
}

/**
 * @brief Resets the touch controller chip using the reset pin.
 * Does nothing if the reset pin was not specified (-1).
 */
void CST816::reset() {
  if (_rst_pin == (uint8_t)-1) {
    // No reset pin connected, skip reset
    return;
  }

  // Perform a hard reset by pulsing the reset pin LOW
  digitalWrite(_rst_pin, LOW);
  delay(50); // Hold low for 50ms
  digitalWrite(_rst_pin, HIGH);
  delay(100); // Wait 100ms for the chip to boot
}

/**
 * @brief Reads the chip ID from the register.
 * @return The 8-bit chip ID (should be 0xB5 for CST816S).
 */
uint8_t CST816::getChipID() {
  uint8_t id;
  receive_byte(REG_CHIP_ID, &id);
  _chip_id = id;
  return id;
}

/**
 * @brief Reads the current touch coordinates and state.
 * @return A TouchCoordinates struct containing X, Y, and points (fingers).
 */
TouchCoordinates CST816::getTouch() {
  uint8_t temp[7]; // Buffer to hold register data
  uint16_t x, y;
  static TouchCoordinates touch = {0, 0, 0}; // Static struct to return

  // Read registers 0x01–0x06 one by one
  // This is inefficient (multiple I2C START/STOPs) but functional.
  receive_byte(0x01, &temp[0]);  // Gesture ID / State
  receive_byte(0x02, &temp[1]);  // Touch points count
  receive_byte(0x03, &temp[2]);  // X high 4 bits
  receive_byte(0x04, &temp[3]);  // X low 8 bits
  receive_byte(0x05, &temp[4]);  // Y high 4 bits
  receive_byte(0x06, &temp[5]);  // Y low 8 bits

  // Store the raw state/gesture info
  _touch_state = temp[0];

  touch.points = temp[1];  // number of fingers (0 or 1)

  // Reconstruct 12-bit coordinates from high/low bytes
  // X = (X_high[3:0] << 8) | X_low[7:0]
  x = ((temp[2] & 0x0F) << 8) | temp[3];
  y = ((temp[4] & 0x0F) << 8) | temp[5];

  touch.X_Pos = x;
  touch.Y_Pos = y;

  return touch;
}

/**
 * @brief Reads the gesture ID from the device.
 * @return A TouchGesture struct containing the gesture ID.
 */
TouchGesture CST816::getGesture() {
  TouchGesture gesture;
  uint8_t temp;
  
  // Read the gesture ID register
  receive_byte(REG_GESTURE_ID, &temp);
  gesture.gesture_id = temp;
  
  // This register often reflects the general touch state as well
  gesture.touch_state = temp; 
  _touch_state = temp;
  
  return gesture;
}

/**
 * @brief Clears the internal touch state flag.
 */
void CST816::clearTouchData() {
  _touch_state = 0;
}

/**
 * @brief Enables or disables the chip's auto-sleep feature.
 * @param disable true to disable auto-sleep, false to enable.
 */
void CST816::disableAutoSleep(bool disable) {
  // 0xFF disables auto-sleep, 0x00 enables it
  write_register(REG_DIS_AUTO_SLEEP, disable ? 0xFF : 0x00);
}

/**
 * @brief Sets a new touch mode.
 * @param mode The new touchpad_mode to set.
 */
void CST816::setTouchMode(touchpad_mode mode) {
  _current_mode = mode;
  begin(mode); // Reinitialize the chip with the new mode
}

/**
 * @brief Checks if the screen is currently being touched.
 * @return true if a touch (or gesture) is active, false otherwise.
 */
bool CST816::isTouched() {
  // _touch_state is updated by getTouch() or getGesture()
  // 0 means no touch/gesture.
  return _touch_state != 0;
}

/**
 * @brief Checks if new touch data is available via the IRQ pin.
 * @return true if the IRQ pin is active (LOW), false otherwise.
 */
bool CST816::available() {
  // The IRQ pin is active LOW.
  return digitalRead(_irq_pin) == LOW;
}

// -------------------------------------------------------------------------
// Private methods - Low-level GPIO control (for software I2C)
// -------------------------------------------------------------------------

/** @brief Set SDA pin to OUTPUT mode */
void CST816::SDA_OUT() {
  pinMode(_sda_pin, OUTPUT);
}

/** @brief Set SDA pin to INPUT_PULLUP mode */
void CST816::SDA_IN() {
  pinMode(_sda_pin, INPUT_PULLUP);
}

/** @brief Drive SDA pin HIGH */
void CST816::SDA_SET() {
  digitalWrite(_sda_pin, HIGH);
}

/** @brief Drive SDA pin LOW */
void CST816::SDA_CLR() {
  digitalWrite(_sda_pin, LOW);
}

/** @brief Read the state of the SDA pin */
uint8_t CST816::SDA_GET() {
  return digitalRead(_sda_pin);
}

/** @brief Drive SCL pin HIGH */
void CST816::SCL_SET() {
  digitalWrite(_scl_pin, HIGH);
}

/** @brief Drive SCL pin LOW */
void CST816::SCL_CLR() {
  digitalWrite(_scl_pin, LOW);
}

/** @brief Wrapper for microsecond delay */
void CST816::delay_us(uint32_t us) {
  delayMicroseconds(us);
}

// -------------------------------------------------------------------------
// Private methods - Software I2C communication protocol
// -------------------------------------------------------------------------

/** @brief Generate I2C START condition: SDA high-to-low while SCL is high */
void CST816::i2c_start() {
  SDA_OUT(); // Ensure SDA is output
  SDA_SET();
  SCL_SET();
  delay_us(5);
  SDA_CLR(); // Data line falls while clock is high
  delay_us(5);
  SCL_CLR(); // Clock goes low, ready for data
}

/** @brief Generate I2C STOP condition: SDA low-to-high while SCL is high */
void CST816::i2c_stop() {
  SDA_OUT(); // Ensure SDA is output
  SCL_CLR();
  SDA_CLR(); // Data line is low
  delay_us(5);
  SCL_SET();
  SDA_SET(); // Data line rises while clock is high
  delay_us(5);
}

/** @brief Send an I2C ACK (Acknowledge) bit (SDA LOW) */
void CST816::i2c_ack() {
  SDA_OUT(); // Master drives SDA
  SCL_CLR();
  SDA_CLR(); // Pull SDA low for ACK
  delay_us(5);
  SCL_SET(); // Pulse clock
  delay_us(5);
  SCL_CLR();
}

/** @brief Send an I2C NACK (Not Acknowledge) bit (SDA HIGH) */
void CST816::i2c_nack() {
  SDA_OUT(); // Master drives SDA
  SCL_CLR();
  SDA_SET(); // Leave SDA high for NACK
  delay_us(5);
  SCL_SET(); // Pulse clock
  delay_us(5);
  SCL_CLR();
}

/**
 * @brief Wait for an ACK from the slave device.
 * @return 0 on success (ACK received), 1 on failure (NACK/timeout).
 */
uint8_t CST816::i2c_wait_ack() {
  uint8_t t = 0;
  SDA_IN();  // Set SDA to input to read from slave
  SDA_SET(); // Enable pullup
  delay_us(5);
  SCL_SET(); // Clock high, slave should drive SDA
  delay_us(5);

  while(SDA_GET()) { // Wait for slave to pull SDA LOW
    t++;
    if(t > 250) { // Timeout check
      i2c_stop(); // Abort transaction
      return 1;
    }
  }
  SCL_CLR(); // Clock low to end ACK bit
  return 0;
}

/**
 * @brief Send one byte of data over I2C.
 * @param byte The byte to send.
 */
void CST816::i2c_send_byte(uint8_t byte) {
  SDA_OUT(); // Master drives SDA
  SCL_CLR();
  for(uint8_t i = 0; i < 8; i++) {
    // Send MSB first
    if(byte & 0x80) {
      SDA_SET();
    } else {
      SDA_CLR();
    }
    byte <<= 1; // Shift to next bit
    
    // Pulse clock
    delay_us(5);
    SCL_SET();
    delay_us(5);
    SCL_CLR();
    delay_us(5);
  }
}

/**
 * @brief Receive one byte of data over I2C.
 * @return The received byte.
 */
uint8_t CST816::i2c_receive_byte() {
  uint8_t ret = 0;
  SDA_IN(); // Set SDA to input to read from slave
  for(uint8_t i = 0; i < 8; i++) {
    SCL_CLR();
    delay_us(5);
    SCL_SET(); // Clock high for slave to set data
    ret <<= 1; // Shift to make room for new bit
    if(SDA_GET()) {
      ret++; // Read the bit
    }
    delay_us(5);
  }
  // Note: This function does not send an ACK or NACK.
  // The caller is responsible for NACKing the last byte.
  // (Though this library doesn't seem to do that, it just STOPs)
  return ret;
}

// -------------------------------------------------------------------------
// Private methods - Register-level read/write operations
// -------------------------------------------------------------------------

/**
 * @brief Write a single byte to a device register.
 * @param reg The 8-bit register address.
 * @param data The 8-bit data to write.
 */
void CST816::write_register(uint8_t reg, uint8_t data) {
  i2c_start();
  i2c_send_byte(CST816_ADDR); // Send device address (Write)
  i2c_wait_ack();
  i2c_send_byte(reg);         // Send register address
  i2c_wait_ack();
  i2c_send_byte(data);        // Send data
  i2c_wait_ack();
  i2c_stop();
  delay(10); // Small delay after write
}

/**
 * @brief Read a single byte from a device register.
 * @param reg The 8-bit register address.
 * @return The 8-bit data read from the register.
 */
uint8_t CST816::read_register(uint8_t reg) {
  uint8_t data;
  
  // I2C "write" operation to set the register address
  i2c_start();
  i2c_send_byte(CST816_ADDR); // Device address (Write)
  i2c_wait_ack();
  i2c_send_byte(reg);         // Register address
  i2c_wait_ack();
  
  // I2C "read" operation to get the data
  i2c_start();                // Repeated START
  i2c_send_byte(CST816_ADDR | 0x01); // Device address (Read)
  i2c_wait_ack();
  data = i2c_receive_byte();  // Read the data byte
  // This implementation omits the NACK before STOP,
  // which is non-standard but often works for single-byte reads.
  i2c_stop();
  
  return data;
}

/**
 * @brief Read a single byte from a device register (pointer-based).
 * @param addr The 8-bit register address.
 * @param data Pointer to store the read data.
 */
void CST816::receive_byte(uint8_t addr, uint8_t* data) {
  // Set the register address to read from
  i2c_start();
  i2c_send_byte(CST816_ADDR); // Device address (Write)
  i2c_wait_ack();
  i2c_send_byte(addr);        // Register address
  i2c_wait_ack();
  
  // Read the data from the specified address
  i2c_start();                // Repeated START
  i2c_send_byte(CST816_ADDR | 0x01); // Device address (Read)
  i2c_wait_ack();
  *data = i2c_receive_byte(); // Read data and store in pointer
  i2c_stop();
}