# CST816 Arduino Touch Controller Library

---

A **C++ library** for seamless integration with the CST816 capacitive touch controller, widely used in smartwatches and small touch displays. Designed for **Arduino-compatible microcontrollers**, this library provides flexible features and easy setup.

---

## 📦 Features

- **Read touch coordinates (X, Y)**
- **Detect touch gestures**: swipe (up, down, left, right), long press, double click
- **Interrupt-driven detection**: Use the IRQ pin for instant touch response
- **Configurable operating modes**: Choose between touch, motion, fast, and other modes
- **Hardware reset support**
- **Software (bit-bang) I2C**: Use any GPIO pins—no hardware I2C needed

---

## 🛠️ Hardware Requirements

To use this library, you need:

- **Microcontroller** (e.g., ESP32, Arduino)
- **CST816 Touch Controller**
- **4 GPIO pins**:
  - **SDA** (I2C Data)
  - **SCL** (I2C Clock)
  - **IRQ** (Interrupt; highly recommended for best performance)
  - **RST** (Reset; optional—set to -1 if not used)

---

## 🚀 Installation

1. **Download** `CST816.h` and `CST816.cpp`.
2. **Add both files** to your project folder.
   - **Arduino IDE**: Place in the same folder as your `.ino` file.
   - **PlatformIO**: Place in your `lib` or `src` folder.

---

## 🧑‍💻 API Overview

### **Constructor**
```cpp
CST816(uint8_t sda_pin, uint8_t scl_pin, uint8_t rst_pin, uint8_t irq_pin)
```
- Pass GPIO pins. Use `(uint8_t)-1` if RST is not connected.

### **Touch Setup**
```cpp
bool begin(touchpad_mode mode)
```
- Initialize pins and chip. Call inside `setup()`.
- `mode`: Choose how the sensor operates (see below).

### **Touch Data**
```cpp
bool available()
```
- Returns `true` if touch data is ready (IRQ pin LOW).

```cpp
TouchCoordinates getTouch()
```
- Returns X/Y positions and number of touch points.

**TouchCoordinates struct:**
```cpp
struct TouchCoordinates {
  uint16_t X_Pos;
  uint16_t Y_Pos;
  uint8_t points; // 0 or 1
};
```

### **Gesture Detection**
```cpp
TouchGesture getGesture()
```
- Returns last detected gesture.

**TouchGesture struct:**
```cpp
struct TouchGesture {
  uint8_t gesture_id;  // See below for IDs
  uint8_t touch_state;
};
```

#### **Gesture Identifiers (from CST816.h)**

| Gesture           | ID    |
|-------------------|-------|
| None              | 0x00  |
| Swipe Up          | 0x01  |
| Swipe Down        | 0x02  |
| Swipe Left        | 0x03  |
| Swipe Right       | 0x04  |
| Long Press        | 0x0B  |
| Double Click      | 0x0C  |

### **Other Functions**

```cpp
void setTouchMode(touchpad_mode mode)
```
- Change and re-initialize sensor mode.

```cpp
uint8_t getChipID()
```
- Read chip ID (`0xB5` expected for CST816S).

```cpp
void reset()
```
- Perform hardware reset via `RST` pin.

```cpp
void disableAutoSleep(bool disable)
```
- Enable/disable auto-sleep for the controller.

```cpp
bool isTouched()
```
- Returns `true` if a touch is detected.

---

## ⚡ Operating Modes

Set sensor behavior using `touchpad_mode`:

- **mode_touch** — Interrupts on touch events (standard)
- **mode_change** — Interrupts only when coordinates change
- **mode_fast** — Faster reporting, interrupts on motion
- **mode_motion** — Optimized for motion/gesture detection

---

## ⚠️ Notes & Tips

- **Software I2C**: Library uses bit-bang I2C, so you can pick any GPIO pins—hardware I2C not required!
- **IRQ Pin**: For **instant response** to touch, connect the IRQ pin. This pin signals your microcontroller as soon as a touch or gesture is detected.

---

## 📚 Example Usage

```cpp
#include "CST816.h"

CST816 touch(16, 17, -1, 4); // SDA, SCL, RST (unused), IRQ

void setup() {
  touch.begin(mode_touch);
}

void loop() {
  if (touch.available()) {
    TouchCoordinates pos = touch.getTouch();
    // Use pos.X_Pos, pos.Y_Pos, pos.points
  }
  TouchGesture gesture = touch.getGesture();
  // Check gesture.gesture_id
}
```

---

**Enjoy rapid prototyping and touch interaction on your next Arduino project!**
