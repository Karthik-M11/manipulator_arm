#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/quaternion.h>
#include <SCServo.h>          // For SCServos
#include <ESP32Servo.h>       // For normal servo
#include <AccelStepper.h>     // For stepper motor

// Quaternion message and SCServo declarations
geometry_msgs__msg__Quaternion msg;
geometry_msgs__msg__Quaternion pub_msg;  // Message for publisher

SCSCL sc;                  // SCServo instance

Servo normal_servo;        // Normal servo instance

rcl_subscription_t subscriber;
rcl_publisher_t publisher;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Pin definitions
const int scservo_pin = 17;   // TX/RX pins for SCServo (connect to hardware Serial1)
const int normal_servo_pin = 22;  // Normal servo control pin
const int step_pin = 32;      // Stepper STEP pin
const int dir_pin = 33;       // Stepper DIR pin

AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);  // Stepper setup

// Variables for angles/positions
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

  // Extract quaternion values
  m1 = msg->x;  // SCServo 1 target position
  m2 = msg->y;  // SCServo 2 target position
  m3 = msg->z;  // Normal servo target position
  m4 = msg->w;  // Stepper motor position

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

  // Update SCServos, normal servo, and stepper motor
  sc.WritePos(1, m1, 0, 1500);  // Move SCServo 1 to position m1
  sc.WritePos(2, m2, 0, 1500);  // Move SCServo 2 to position m2
  normal_servo.write(m3);       // Move normal servo to angle m3
  stepper.moveTo(m4);           // Move stepper to position m4

  // Prepare and publish quaternion data
  pub_msg.x = m1;
  pub_msg.y = m2;
  pub_msg.z = m3;
  pub_msg.w = m4;

  // Publish the data
  RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
}

void setup() {
  set_microros_wifi_transports("robocon24", "12345678", "192.168.12.1", 8888);
  
  // Initialize Serial1 for SCServos
  Serial1.begin(1000000);
  sc.pSerial = &Serial1;

  // Initialize normal servo
  ESP32PWM::allocateTimer(0);
  normal_servo.setPeriodHertz(50);
  normal_servo.attach(normal_servo_pin, 1000, 2000);

  // Initialize stepper motor
  pinMode(dir_pin, OUTPUT);
  pinMode(step_pin, OUTPUT);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  Serial.begin(9600);

  // Micro-ROS setup
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
      "joint_state"));  // Subscribe to joint_state topic

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
  // Spin executor
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
