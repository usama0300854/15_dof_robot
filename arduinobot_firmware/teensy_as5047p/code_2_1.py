#include <AS5047P.h>

#define AS5047P_CS1 9  // Chip select for the first AS5047P
#define AS5047P_CS2 10 // Chip select for the second AS5047P

// Define the SPI bus speed
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// Initialize two AS5047P sensor objects
AS5047P as5047p1(AS5047P_CS1, AS5047P_CUSTOM_SPI_BUS_SPEED); // First encoder
AS5047P as5047p2(AS5047P_CS2, AS5047P_CUSTOM_SPI_BUS_SPEED); // Second encoder

// Define the baseline angles for each encoder
const int BASE_ANGLE_1 = 23;  // Base angle for the first encoder
const int BASE_ANGLE_2 = 45;  // Base angle for the second encoder (example value)

void setup() {
  Serial.begin(115200);                    // Initialize serial communication

  // Initialize the first AS5047P sensor
  while (!as5047p1.initSPI()) {
    Serial.println(F("Can't connect to the first AS5047P sensor! Please check the connection..."));
    delay(5000);
  }
  
  // Initialize the second AS5047P sensor
  while (!as5047p2.initSPI()) {
    Serial.println(F("Can't connect to the second AS5047P sensor! Please check the connection..."));
    delay(5000);
  }
}

void loop() {
  // Read the first encoder
  int currentAngle1 = int(as5047p1.readAngleDegree()); // Read angle from the first encoder
  int adjustedAngle1 = currentAngle1 - BASE_ANGLE_1;

  // Wrap the adjusted angle for the first encoder within the range 180 to 0 to -180
  if (adjustedAngle1 > 180) {
    adjustedAngle1 -= 360; // Wrap around to -180
  } else if (adjustedAngle1 < -180) {
    adjustedAngle1 += 360; // Wrap around to 180
  }

  // Print the adjusted angle for the first encoder
  Serial.print("Adjusted Angle 1: ");
  Serial.print(adjustedAngle1);
  Serial.println(" degrees");

  // Read the second encoder
  int currentAngle2 = int(as5047p2.readAngleDegree()); // Read angle from the second encoder
  int adjustedAngle2 = currentAngle2 - BASE_ANGLE_2;

  // Wrap the adjusted angle for the second encoder within the range 180 to 0 to -180
  if (adjustedAngle2 > 180) {
    adjustedAngle2 -= 360; // Wrap around to -180
  } else if (adjustedAngle2 < -180) {
    adjustedAngle2 += 360; // Wrap around to 180
  }

  // Print the adjusted angle for the second encoder
  Serial.print("Adjusted Angle 2: ");
  Serial.print(adjustedAngle2);
  Serial.println(" degrees");

  delay(15); // Short delay for smooth readings
}
