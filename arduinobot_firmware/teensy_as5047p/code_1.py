#include <AS5047P.h>



#define AS5047P_CHIP_SELECT_PORT 9 

// Define the SPI bus speed
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// Initialize a new AS5047P sensor object
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// Define the baseline angle to compare (151 degrees)
const int BASE_ANGLE = 23;

// Arduino setup routine
void setup() {
               
  Serial.begin(115200);                    // Initialize serial communication

  // Initialize the AS5047P sensor and hold if the sensor can't be initialized
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }
}

// Arduino loop routine
void loop() {
          
  
  int currentAngle = int(as5047p.readAngleDegree()); // Read the integer angle from the sensor

  // Calculate the adjusted angle relative to the base angle of 151
  int adjustedAngle = currentAngle - BASE_ANGLE;

  // Normalize the adjusted angle to keep it within 0-360 degrees range
  if (adjustedAngle < 0) {
    adjustedAngle += 360;
  } else if (adjustedAngle >= 360) {
    adjustedAngle -= 360;
  }

  // Print the adjusted angle to the serial console
  Serial.print("Adjusted Angle: ");
  Serial.print(adjustedAngle);
  Serial.println(" degrees");

  delay(15);                                  // Wait for 500 milliseconds

                                // Wait for another 500 milliseconds
}
