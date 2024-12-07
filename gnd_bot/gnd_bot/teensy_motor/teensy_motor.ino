const int dirPin = 2;
const int pwmhPin = 3;

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(pwmhPin, OUTPUT);

  // Set PWM frequency to 20 kHz
  analogWriteFrequency(pwmhPin, 20000);

  // Initialize motor stopped
  digitalWrite(dirPin, LOW);
  analogWrite(pwmhPin, 0);
}

void loop() {
  // Move motor forward at 25% speed
  digitalWrite(dirPin, HIGH);    // Forward direction
  analogWrite(pwmhPin, 64);      // 25% duty cycle (slow speed)
  delay(3000);                   // Run for 3 seconds

  // Move motor forward at 50% speed
  analogWrite(pwmhPin, 128);     // 50% duty cycle (medium speed)
  delay(3000);                   // Run for 3 seconds

  // Move motor forward at 75% speed
  analogWrite(pwmhPin, 192);     // 75% duty cycle (faster speed)
  delay(3000);                   // Run for 3 seconds

  // Move motor forward at full speed
  analogWrite(pwmhPin, 255);     // 100% duty cycle (maximum speed)
  delay(3000);                   // Run for 3 seconds

  // Stop the motor
  analogWrite(pwmhPin, 0);       // Motor off
  delay(3000);                   // Pause for 3 seconds
}
