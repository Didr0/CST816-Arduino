CST816 Touch Controller Library

A simple C++ library for interacting with the CST816 capacitive touch controller, commonly found in smartwatches and other small display devices. This library is designed for Arduino-compatible microcontrollers (like ESP32) and uses a software-based (bit-bang) I2C implementation.

Features

Read X/Y touch coordinates

Detect touch gestures (swipe up, down, left, right, etc.)

Interrupt-driven touch detection via the IRQ pin

Configurable operating modes (touch, motion, fast, etc.)

Hardware reset support

Software-based I2C: No hardware I2C peripheral dependency, works on any GPIO pin.

Hardware Requirements

To use this library, you need:

A microcontroller (e.g., ESP32, Arduino)

A CST816 touch controller

4 GPIO pins for:

SDA (I2C Data)

SCL (I2C Clock)

IRQ (Interrupt - Highly recommended for efficient operation)

RST (Reset - Optional, can be set to -1 if not used)

Installation

Download the CST816.h and CST816.cpp files.

Add them to your Arduino project folder.

For the Arduino IDE, you can place them in the same directory as your .ino file.

For PlatformIO, you can place them in your project's lib or src directory.

API Reference

Public Methods

CST816(uint8_t sda_pin, uint8_t scl_pin, uint8_t rst_pin, uint8_t irq_pin)

Constructor. Pass your GPIO pin definitions here. Set rst_pin to (uint8_t)-1 if not used.

bool begin(touchpad_mode mode)

Initializes the GPIO pins and the touch chip. Call this in your setup().

mode: The operating mode to start in (see below).

bool available()

Checks if the IRQ pin is active (LOW), indicating new touch data is ready.

TouchCoordinates getTouch()

Reads and returns the current touch data (X, Y, points).

Returns a TouchCoordinates struct:

struct TouchCoordinates {
  uint16_t X_Pos;
  uint16_t Y_Pos;
  uint8_t points; // Number of touch points (0 or 1)
};


TouchGesture getGesture()

Reads and returns the last detected gesture.

Returns a TouchGesture struct:

struct TouchGesture {
  uint8_t gesture_id;
  uint8_t touch_state;
};


Known Gesture IDs (defined in CST816.h):

GESTURE_NONE (0x00)

GESTURE_SWIPE_UP (0x01)

GESTURE_SWIPE_DOWN (0x02)

GESTURE_SWIPE_LEFT (0x03)

GESTURE_SWIPE_RIGHT (0x04)

GESTURE_LONG_PRESS (0x0B)

GESTURE_DOUBLE_CLICK (0x0C)

void setTouchMode(touchpad_mode mode)

Changes the operating mode of the chip (re-initializes).

uint8_t getChipID()

Returns the chip ID (should be 0xB5 for CST816S).

void reset()

Performs a hardware reset via the RST pin (if connected).

void disableAutoSleep(bool disable)

Disables or enables the chip's auto-sleep feature.

bool isTouched()

Returns true if the internal state flag indicates a touch.

Operating Modes

You can set the chip's behavior using touchpad_mode:

mode_touch: Standard touch detection. Interrupts on touch events.

mode_change: Interrupts only when touch coordinates change.

mode_fast: Faster reporting, interrupts on motion.

mode_motion: Optimized for motion and gesture detection.

Notes

Software I2C: This library uses a "bit-bang" or software I2C implementation. This means it does not use the microcontroller's hardware I2C peripherals (e.g., Wire.h) and can be used on any available GPIO pins.

Interrupt Pin (IRQ): For best performance, you must connect the IRQ pin. This pin signals the microcontroller when a touch event occurs, allowing your code to react instantly instead of constantly polling the chip. Polling with touch.available() in a tight loop is inefficient and not recommended. The correct approach is to use an external interrupt (e.g., attachInterrupt on Arduino/ESP32) triggered by the IRQ pin.
