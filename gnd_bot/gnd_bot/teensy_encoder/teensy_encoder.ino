#include <Encoder.h>

// Define encoder pins
Encoder myEnc(7, 8);  // Channel A -> pin 7, Channel B -> pin 8

void setup() {
  Serial.begin(115200);
  Serial.println("Encoder Test Started");
}

void loop() {
  static long oldPosition = -999;
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("Position: ");
    Serial.println(newPosition);
  }
}
