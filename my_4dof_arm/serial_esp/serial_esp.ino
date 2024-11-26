#include <ESP32Servo.h>
#include <AccelStepper.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
Servo myservo3;

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

	
}
