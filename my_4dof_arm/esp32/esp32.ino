#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>

// ROS 2 message
std_msgs__msg__Float32MultiArray msg;

// Servo setup
Servo servo1;
Servo servo2;
Servo servo3;

// Stepper setup
const int step_pin = 32;
const int dir_pin = 33;
AccelStepper stepper(AccelStepper::DRIVER, step_pin, dir_pin);

// ROS 2 setup
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// Pin definitions
const int servo_pin1 = 18;
const int servo_pin2 = 19;
const int servo_pin3 = 21; // Replaced invalid pin 20 with 21

// Servo range
int minUs = 1000;
int maxUs = 2000;

// Movement variables
int m1 = 0, m2 = 0, m3 = 0, m4 = 0;

// Macros for error handling
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

// Error handling loop
void error_loop() {
  Serial.println("Error encountered. Restarting...");
  delay(1000);
  ESP.restart(); // Restart ESP32 to recover
}

// Subscription callback
void subscription_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

  // Safety checks
  if (msg == NULL) {
    Serial.println("Callback received NULL message");
    return;
  }

  if (msg->data.size < 4) {
    Serial.println("Insufficient data in message");
    return;
  }

  // Assign data to movement variables
  m1 = msg->data.data[0];
  m2 = msg->data.data[1];
  m3 = msg->data.data[2];
  m4 = msg->data.data[3];

  // Log received values
  Serial.println("Received joint states:");
  Serial.println(m1);
  Serial.println(m2);
  Serial.println(m3);
  Serial.println(m4);

  // Update servos and stepper
  servo1.write(m1);
  servo2.write(m2);
  servo3.write(m3);
  stepper.moveTo(m4);
}

void setup() {
  // Serial monitor
  Serial.begin(115200);

  // Initialize Wi-Fi transport
  set_microros_wifi_transports("robocon24", "12345678", "192.168.122.245", 8888);
  Serial.println("Micro-ROS transport configured");

  // Servo setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo1.attach(servo_pin1, minUs, maxUs);
  servo2.attach(servo_pin2, minUs, maxUs);
  servo3.attach(servo_pin3, minUs, maxUs);

  // Stepper setup
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // ROS 2 initialization
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "joint_states"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  Serial.println("Setup complete");
}

void loop() {
  // Check Micro-ROS Agent connectivity
  if (!rmw_uros_ping_agent(1000, 1)) {
    Serial.println("Micro-ROS Agent not reachable");
    error_loop();
  }

  // Spin the executor
  rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  if (ret != RCL_RET_OK) {
    Serial.print("Executor spin failed with error code: ");
    Serial.println(ret);
    error_loop();
  } else {
    Serial.println("Executor spin successful");
  }

  // Allow time for operations
  delay(10);
}
