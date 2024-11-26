#include <ESP32Servo.h>
#include <AccelStepper.h>

Servo myservo1;  // create servo object to control a servo

#define A_STEP_PIN  14  // IO-OUT Pin 1 (A_STEP)
#define A_DIR_PIN   27  // IO-OUT Pin 2 (A_DIR)
#define ENABLE_PIN  26  // IO-OUT Pin 3 (ENABLE)
#define MS1_PIN     25  // IO-OUT Pin 4 (MS1, optional)

// Create an AccelStepper instance
AccelStepper stepper(AccelStepper::DRIVER, A_STEP_PIN, A_DIR_PIN);

int servo1_pos = 0;
int servo2_pos = 0;
int servo3_pos = 0;

const int servo1_pin = 18;
const int servo2_pin = 19;
const int servo3_pin = 20;

int angle1 = 0;
int angle2 = 0;
int angle3 = 0;
int angle4 = 0;
int action = 0;

void setup() {

    // Initialise serial
    Serial.begin(9600);
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo1.setPeriodHertz(50);    // standard 50 hz servo
	myservo1.attach(servo1_pin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
    // Do for other servos
    myservo2.setPeriodHertz(50);
    myservo2.attach(servo2_pin, 1000, 2000);

    myservo3.setPeriodHertz(50);
    myservo3.attach(servo3_pin, 1000, 2000);

    // Initialize pins for 
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);

    // Enable the stepper motor driver
    digitalWrite(ENABLE_PIN, LOW); // LOW = Enabled, HIGH = Disabled

    // Configure microstepping (optional)
    digitalWrite(MS1_PIN, LOW); // LOW = Full step, HIGH = Half step

    // Set motor speed and acceleration
    stepper.setMaxSpeed(1000);  // Steps per second
    stepper.setAcceleration(500);  // Steps per second^2
}

void loop() {
    if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read incoming data until a newline character
    parseData(input);  // Parse the received CSV data
    moveServos();
    moveStepper();
    delay(1000);
    if (action == 1) {
        pickup();
    }
    else {
        dropdown();
    }
    delay(1000);
    home();
    delay(1000);
    }
}


void parseData(String data) {
  int commaIndex1 = data.indexOf(',');  // Find first comma
  int commaIndex2 = data.indexOf(',', commaIndex1 + 1);  // Find second comma
  int commaIndex3 = data.indexOf(',', commaIndex2 + 1);  // Find third comma
  int commaIndex4 = data.indexOf(',', commaIndex3 + 1);  //Find last comma

  // Extract each angle and action from the data
  angle1 = data.substring(0, commaIndex1).toInt();  // First angle
  angle2 = data.substring(commaIndex1 + 1, commaIndex2).toInt();  // Second angle
  angle3 = data.substring(commaIndex2 + 1, commaIndex3).toInt();  // Third angle
  angle4 = data.substring(commaIndex3 + 1, commaIndex4).toInt();  // Fourth angle
  action = data.substring(commaIndex4 +1).toInt()  // Find the action
}


void moveServos() {
  // Constrain angles to be between 0 and 180 degrees
  angle1 = constrain(angle2, 0, 180);
  angle2 = constrain(angle3, 0, 180);
  angle3 = constrain(angle4, 0, 180);

  // Move servos to the specified angles
  myservo1.write(angle2);
  myservo2.write(angle3);
  myservo3.write(angle4);
}

void moveStepper() {
    // Set the target position for stepper motor
    stepper.moveTo(angle1);
}

void home() {
    // Set the params for home position
    myservo1.write(angle2);
    myservo2.write(angle3);
    myservo3.write(angle4);
}

void pickup() {
    // move servo2 to pick disk up
    myservo2.write(angle3+20);
}

void dropdown() {
    // move servo2 to drop disk
    myservo2.write(angle2-20);
}