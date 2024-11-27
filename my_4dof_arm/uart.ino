#include <SCServo.h>

// Create an instance of the SCServo class
SCSCL sc;

void setup() {
    // Initialize Serial1 for SCServo communication
    Serial1.begin(1000000);  // Set baud rate to 1Mbps (default for SCServo)
    sc.pSerial = &Serial1;   // Assign Serial1 to the SCServo object
    while (!Serial1) {
        // Wait for Serial1 to initialize
    }

    // Optional: Print a message to indicate setup is complete
    Serial.begin(9600);
    Serial.println("SCServo Sweep Test Initialized!");
}

void loop() {
    // Sweep the SCServo between 0° and 180°
    int position1 = 0;      // SCServo position equivalent to 0°
    int position2 = 1000;   // SCServo position equivalent to 180°

    // Move SCServo to position1 (0°)
    sc.WritePos(1, position1, 0, 1500);  // ID = 1, position = 0, acceleration = 0, speed = 1500
    delay(1000);  // Wait for 1 second

    // Move SCServo to position2 (180°)
    sc.WritePos(1, position2, 0, 1500);  // ID = 1, position = 1000, acceleration = 0, speed = 1500
    delay(1000);  // Wait for 1 second
}
