#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <Arduino.h>
#include <WiFi.h>

#include "hardware/pwm.h"


// === Pin assignments (confirmed) ===
#define RIGHT_A 8
#define RIGHT_B 9
#define FRONT_RIGHT_A 10
#define FRONT_RIGHT_B 11
#define FRONT_LEFT_A 14
#define FRONT_LEFT_B 15
#define LEFT_A 12
#define LEFT_B 13


// micro-ROS setup
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t sub;
geometry_msgs__msg__Twist cmd_vel_msg;

void setupPWM(int gpio) {
  Serial.print("Configuring PWM on GPIO ");
  Serial.println(gpio);
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(gpio);
  Serial.print("Enabling PWM slice ");
  Serial.println(slice);
  pwm_set_enabled(slice, true);
}

// Clamp to -1.0 .. 1.0 and convert to 0–255 PWM
int pwm_value(float speed) {
  float original = speed;
  speed = constrain(speed, -1.0f, 1.0f);
  int result = int(fabs(speed) * 255.0f);
  Serial.print("pwm_value(");
  Serial.print(original, 3);
  Serial.print(") => ");
  Serial.println(result);
  return result;
}

// Apply PWM to H-bridge motor driver
void driveMotor(int pinA, int pinB, float value) {
  int pwm = pwm_value(value);
  Serial.print("Driving motor with pinA=");
  Serial.print(pinA);
  Serial.print(" pinB=");
  Serial.print(pinB);
  Serial.print(" value=");
  Serial.print(value, 3);
  Serial.print(" pwm=");
  Serial.println(pwm);

  if (value >= 0.0f) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, pwm);
  }
}

// Callback when Twist message received
void cmd_vel_callback(const void *msgin) {
  const auto *twist = static_cast<const geometry_msgs__msg__Twist*>(msgin);
  float linear = twist->linear.x;
  float angular = twist->angular.z;

  // Differential (tank) drive mix
  float left_speed = linear - angular;
  float right_speed = linear + angular;

  Serial.println("cmd_vel_callback: received Twist message");
  Serial.print("  linear.x = ");
  Serial.println(linear, 3);
  Serial.print("  angular.z = ");
  Serial.println(angular, 3);
  Serial.print("  left_speed = ");
  Serial.println(left_speed, 3);
  Serial.print("  right_speed = ");
  Serial.println(right_speed, 3);

  // Drive motors
  driveMotor(LEFT_A, LEFT_B, left_speed);
  driveMotor(RIGHT_A, RIGHT_B, right_speed);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);

  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting micro-ROS rover (StoRPer rear motors)...");

  setupPWM(12);
  setupPWM(13);

  // Set up micro-ROS transport
  IPAddress agent_ip(192, 168, 8, 5);
  Serial.println("Setting up micro-ROS WiFi transport...");
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, agent_ip, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;
  rc = rclc_support_init(&support, 0, NULL, &allocator);
  Serial.print("Support init: "); Serial.println(rc);

  rc = rclc_node_init_default(&node, "rover_node", "", &support);
  Serial.print("Node init: "); Serial.println(rc);

  rc = rclc_subscription_init_default(
    &sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "rt/cmd_vel"
  );
  Serial.print("Sub init: "); Serial.println(rc);

  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  Serial.print("Exec init: "); Serial.println(rc);

  rc = rclc_executor_add_subscription(&executor, &sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  Serial.print("Add sub: "); Serial.println(rc);

  Serial.println("Rover ready and listening for Twist messages.");
}

void loop() {
  static int counter = 0;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);

  if (++counter % 50 == 0) {
    Serial.println("Loop heartbeat: spinning executor");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

