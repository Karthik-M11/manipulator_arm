#include <ESP32Servo.h>
#include <AccelStepper.h>

// Standard servo instance
Servo myservo1;
Servo myservo2;

// Stepper motor pins
#define A_STEP_PIN1  14
#define A_DIR_PIN2   27
#define ENABLE_PIN1  26
#define MS1_PIN1     25

#define A_STEP_PIN2  15
#define A_DIR_PIN2   28
#define ENABLE_PIN2  29
#define MS1_PIN2     30

// Create an AccelStepper instance
AccelStepper stepper1(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

// Pin assignments for the servo
const int servo1_pin = 18;
const int servo2_pin = 19;

// Variables to store angles and actions
int angle1 = 0;
int angle2 = 0;
int angle3 = 0;
int angle4 = 0;
int action = 0;

void setup() {
    // Initialize SCServo communication
    Serial1.begin(1000000);  // SCServo communicates at 1Mbps
    sc.pSerial = &Serial1;
    while (!Serial1) {}  // Wait for Serial1 to initialize

    // Initialize standard servo
    Serial.begin(9600);
    ESP32PWM::allocateTimer(0);
    myservo1.setPeriodHertz(50); // Standard 50 Hz servo
    myservo1.attach(servo1_pin, 1000, 2000); // Attach servo to pin 18

    // Initialize stepper motor pins
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);

    // Enable the stepper motor driver
    digitalWrite(ENABLE_PIN, LOW); // LOW = Enabled

    // Configure microstepping (optional)
    digitalWrite(MS1_PIN, LOW); // LOW = Full step

    // Set motor speed and acceleration
    stepper.setMaxSpeed(1000);  // Steps per second
    stepper.setAcceleration(500);  // Steps per second^2
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read incoming data until a newline character
        parseData(input);  // Parse the received CSV data
        moveServos();    // Move SCServo motors
        moveStepper();     // Move stepper motor
        delay(1000);

        if (action == 1) {
            pickup();       // Perform pickup action
        } else {
            dropdown();     // Perform dropdown action
        }

        delay(1000);
        home();             // Return to home position
        delay(1000);
    }
}

void parseData(String data) {
    int commaIndex1 = data.indexOf(',');  // Find first comma
    int commaIndex2 = data.indexOf(',', commaIndex1 + 1);  // Find second comma
    int commaIndex3 = data.indexOf(',', commaIndex2 + 1);  // Find third comma
    int commaIndex4 = data.indexOf(',', commaIndex3 + 1);  // Find last comma

    // Extract each angle and action from the data
    angle1 = data.substring(0, commaIndex1).toInt();       // First angle
    angle2 = data.substring(commaIndex1 + 1, commaIndex2).toInt();  // Second angle
    angle3 = data.substring(commaIndex2 + 1, commaIndex3).toInt();  // Third angle
    angle4 = data.substring(commaIndex3 + 1, commaIndex4).toInt();  // Fourth angle
    action = data.substring(commaIndex4 + 1).toInt();      // Action
}

void moveServos() {
    // Control SCServo motors (assuming IDs 1 and 2 for SCServo motors)
    sc.WritePos(1, map(angle2, 0, 180, 0, 1000), 0, 1500);  // Move SCServo 1 to angle2
    sc.WritePos(2, map(angle3, 0, 180, 0, 1000), 0, 1500);  // Move SCServo 2 to angle3
    myservo1.write(angle4);
}

void moveStepper() {
    // Set the target position for the stepper motor
    stepper.moveTo(angle1);
}

void home() {
    // Move SCServos and standard servo to home position
    sc.WritePos(1, map(angle2, 0, 180, 0, 1000), 0, 1500);  // Move SCServo 1 to home position
    sc.WritePos(2, map(angle3, 0, 180, 0, 1000), 0, 1500);  // Move SCServo 2 to home position
    myservo1.write(angle4);  // Move standard servo to home position
}

void pickup() {
    // Perform pickup action with SCServos
    sc.WritePos(2, map(angle3 + 20, 0, 180, 0, 1000), 0, 1500);  // Adjust SCServo 2 position
}

void dropdown() {
    // Perform dropdown action with SCServos
    sc.WritePos(2, map(angle3 - 20, 0, 180, 0, 1000), 0, 1500);  // Adjust SCServo 2 position
}
