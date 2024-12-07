/***************************************************************************
 * Example sketch for controlling a single ESC on Teensy
 * without using the DShot.h library.
 *
 * This example initiates the ESC, increases the throttle to maximum speed,
 * and then decreases the throttle to zero using IntervalTimer.
 *
 ***************************************************************************/

#include <Arduino.h>

// Constants for DShot protocol
constexpr uint8_t ESC_PIN = 8;       // Pin to control the ESC
constexpr uint16_t LOOP_HZ = 2000;  // 2 kHz control loop frequency
constexpr uint16_t MAX_THROTTLE = 1999;
constexpr uint16_t MIN_THROTTLE = 0;

IntervalTimer timer;  // Timer for maintaining loop timing

volatile int16_t throttle = 0;       // Throttle value
volatile int8_t throttleChange = 1;  // Direction of throttle change
volatile uint64_t counter = 0;       // Loop counter
volatile uint8_t ledState = false;   // LED state

// Send DShot signal
void sendDShotSignal(uint16_t throttle)
{
    uint16_t packet = throttle << 1; // Shift throttle by 1 bit for telemetry (always 0 here)
    uint16_t checksum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    packet = (packet << 4) | checksum; // Append checksum

    // Send packet using bit-banging
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(ESC_PIN, (packet & (1 << i)) ? HIGH : LOW);
        delayMicroseconds(1); // Timing for DShot600 (approx. 1.67 µs per bit)
    }
    digitalWriteFast(ESC_PIN, LOW);
}

void setup()
{
    // Setup ESC pin and LED
    pinMode(ESC_PIN, OUTPUT);
    digitalWrite(ESC_PIN, LOW);
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize ESC with zero throttle
    for (size_t i = 0; i < 4000; i++) {
        sendDShotSignal(0);
        delayMicroseconds(1000); // Ensure ESC receives initialization signals
    }

    // Setup IntervalTimer to control ESC
    timer.begin(controlLoop, 1'000'000 / LOOP_HZ);
}

void controlLoop()
{
    // Send throttle signal
    sendDShotSignal(throttle);

    // Reverse throttle direction if max or min reached
    if (throttle >= MAX_THROTTLE) {
        throttleChange = -1;
    } else if (throttle <= MIN_THROTTLE) {
        throttleChange = 1;
    }

    // Adjust throttle periodically
    if (counter % 20 == 0) {
        throttle += throttleChange;
    }

    // Blink the LED at 1 Hz
    if (counter % LOOP_HZ == 0) {
        digitalWrite(LED_BUILTIN, ledState ^= 1);
    }

    ++counter;
}

void loop()
{
    // Empty loop since control is handled by the timer
}
