


///////////////////////////////////////////////////////////////////////////








#include <AlfredoCRSF.h>
#include <Arduino.h>
#include <Wire.h>

// Pin and motor values
#define MOTOR1_PIN 3
#define ESC_FREQUENCY 250




// Instantiate AlfredoCRSF object
AlfredoCRSF crsf;
uint16_t channels[16];  // RC channel values
int PWM = 0;




void setup()
{
    Serial.begin(115200); // USB Serial for debugging
    while (!Serial); // Wait for the Serial Monitor to open
    Serial.println("USB Serial initialized");

    // Initialize Serial1 for CRSF communication
    Serial1.begin(420000); // Use default pins for Serial1
    if (!Serial1) {
        Serial.println("Failed to initialize Serial1");
        while (1); // Halt if Serial1 fails
    }
    delay(3000);

    // Initialize CRSF with Serial1
    crsf.begin(Serial1);
    Serial.println("CRSF initialized");

    crsf.update();
    printChannels();
    delay(1000);


    pinMode(MOTOR1_PIN, OUTPUT);
    analogWriteFrequency(MOTOR1_PIN, ESC_FREQUENCY);
    analogWriteResolution(12);



  }

void loop()
{
    // Update CRSF data
    crsf.update(); // Call update to process CRSF data

    // Print channel data as fast as possible
    printChannels();

    
    if (channels[2] !=0){
      analogWrite(MOTOR1_PIN, channels[2]);
      Serial.print("Channel Values: ");
      Serial.println(channels[2]); 


      


    }
    
    // analogWrite(MOTOR1_PIN, map(channels[2],1100,1900,1300,1864));
    Serial.println(channels[2]); 
    delay(1);
}




// Function to print all 16 CRSF channel values
void printChannels()
{
    for (int channelNum = 1; channelNum <= 16; channelNum++) {
        channels[channelNum-1] = crsf.getChannel(channelNum);
        // Serial.print(channels[channelNum-1]);
        // Serial.print(", ");
    }
    // Serial.println();
}
