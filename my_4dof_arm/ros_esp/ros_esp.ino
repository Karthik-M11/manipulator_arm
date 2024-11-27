#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/quaternion.h>  // Include Quaternion message type
#include <ESP32Servo.h>
#include <AccelStepper.h>

// Quaternion message and Servo declarations
geometry_msgs__msg__Quaternion msg;
geometry_msgs__msg__Quaternion pub_msg;  // Message for publisher

Servo servo1;
Servo servo2;

rcl_subscription_t subscriber;
rcl_publisher_t publisher;  // Single Publisher

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

const int servo_pin1 = 19;
const int servo_pin2 = 22;
const int servo_pin3 = 25;

#define A_STEP_PIN1  14  // IO-OUT Pin 1 (A_STEP)
#define A_DIR_PIN1   27  // IO-OUT Pin 2 (A_DIR)
#define ENABLE_PIN1  26  // IO-OUT Pin 3 (ENABLE)
#define MS1_PIN1     25  // IO-OUT Pin 4 (MS1, optional)

#define A_STEP_PIN2  14  // IO-OUT Pin 1 (A_STEP)
#define A_DIR_PIN2   27  // IO-OUT Pin 2 (A_DIR)
#define ENABLE_PIN2  26  // IO-OUT Pin 3 (ENABLE)
#define MS1_PIN2     25  // IO-OUT Pin 4 (MS1, optional)

// Create an AccelStepper instance
AccelStepper stepper1(AccelStepper::DRIVER, A_STEP_PIN1, A_DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, A_STEP_PIN2, A_DIR_PIN2);

// m1, m2, m3, and m4 variables
int m1 = 0, m2 = 0, m3 = 0, m4 = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { } }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Subscription callback function
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Quaternion *msg = (const geometry_msgs__msg__Quaternion *)msgin;

  // Extract the quaternion values into m1, m2, m3, m4
  m1 = msg->x;  // Assuming quaternion.x corresponds to m1
  m2 = msg->y;  // Assuming quaternion.y corresponds to m2
  m3 = msg->z;  // Assuming quaternion.z corresponds to m3
  m4 = msg->w;  // Assuming quaternion.w corresponds to m4

  // Print received values for debugging
  Serial.print("Received Quaternion: ");
  Serial.print("x = ");
  Serial.print(m1);
  Serial.print(", y = ");
  Serial.print(m2);
  Serial.print(", z = ");
  Serial.print(m3);
  Serial.print(", w = ");
  Serial.println(m4);

  // Update servos and stepper motor
  servo1.write(m1);  // Assuming m1 is in the valid range for servo
  servo2.write(m2);  // Assuming m2 corresponds to another servo
  stepper1.moveTo(m3);
  stepper2.moveTo(m4); // Assuming m4 corresponds to stepper position

  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
  }

  while (stepper2.distanceToGo() != 0) {
    stepper2.run();
  }

  // Prepare and publish the quaternion data
  pub_msg.x = m1;
  pub_msg.y = m2;
  pub_msg.z = m3;
  pub_msg.w = m4;  // Set quaternion values

  // Publish the data
  RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

void setup() {
  set_microros_wifi_transports("robocon24", "12345678", "192.168.122.245", 8888);
  
  pinMode(servo_pin1, OUTPUT);
  pinMode(servo_pin2, OUTPUT);

  // Initialize pins
  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(MS1_PIN1, OUTPUT);
  pinMode(MS1_PIN2, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);

  // Enable the stepper motor driver
  digitalWrite(ENABLE_PIN1, LOW); // LOW = Enabled, HIGH = Disabled
  digitalWrite(ENABLE_PIN2, LOW);

  // Configure microstepping (optional)
  digitalWrite(MS1_PIN1, LOW); // LOW = Full step, HIGH = Half step
  digitalWrite(MS1_PIN2, LOW);

  // Set motor speed and acceleration
  stepper1.setMaxSpeed(1000);  // Steps per second
  stepper1.setAcceleration(500);  // Steps per second^2

  stepper2.setMaxSpeed(1000);  // Steps per second
  stepper2.setAcceleration(500);  // Steps per second^2

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);

  Serial.begin(9600);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
      "joint"));  // Subscribe to joint_state topic

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
      "processed_joint_state"));  // Publish to processed_joint_state topic

  // Set up executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
