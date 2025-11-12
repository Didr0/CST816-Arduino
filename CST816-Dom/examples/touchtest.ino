#include <CST816.h>

// Crea oggetto touch con pin predefiniti
CST816 touch;

// Oppure con pin custom:
//CST816 touch(11, 7, 8, 9);

void setup() {
  Serial.begin(115200);
  
  if(touch.begin()) {
    Serial.println("CST816 Touch Initialized");
    Serial.print("Chip ID: 0x");
    Serial.println(touch.getChipID(), HEX);
  } else {
    Serial.println("CST816 Init Failed");
  }
}

void loop() {
  // Controlla se c'è un touch
  if(touch.available()) {
    TouchCoordinates pos = touch.getTouch();

    
    if(pos.X_Pos != 0 || pos.Y_Pos != 0) {
      Serial.print("Touch - X: ");
      Serial.print(pos.X_Pos);
      Serial.print(" Y: ");
      Serial.println(pos.Y_Pos);
      Serial.print("Fingers: ");
      Serial.println(pos.points);
    }
    
    // Puoi anche ottenere i gesti
    TouchGesture gesture = touch.getGesture();
    if(gesture.gesture_id != 0) {
      Serial.print("Gesture: 0x");
      Serial.println(gesture.gesture_id, HEX);
    }
  }
}
