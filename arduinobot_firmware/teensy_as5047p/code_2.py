#include <AS5047P.h>

#define AS5047P_CHIP_SELECT_PORT 9 

// Define the SPI bus speed
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// Initialize a new AS5047P sensor object
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// Define the baseline angle to compare (23 degrees)
const int BASE_ANGLE = 23;

void setup() {
  Serial.begin(115200);                    // Initialize serial communication

  // Initialize the AS5047P sensor and hold if the sensor can't be initialized
  while (!as5047p.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(5000);
  }
}

void loop() {
  int currentAngle = int(as5047p.readAngleDegree()); // Read the integer angle from the sensor

  // Calculate the adjusted angle relative to the base angle of 23
  int adjustedAngle = currentAngle - BASE_ANGLE;

  // Wrap the adjusted angle within the range 180 to 0 to -180
  if (adjustedAngle > 180) {
    adjustedAngle -= 360; // Wrap around to -180
  } else if (adjustedAngle < -180) {
    adjustedAngle += 360; // Wrap around to 180
  }

  // Print the adjusted angle to the serial console
  Serial.print("Adjusted Angle: ");
  Serial.print(adjustedAngle);
  Serial.println(" degrees");

  delay(15); // Short delay for smooth readings
}
