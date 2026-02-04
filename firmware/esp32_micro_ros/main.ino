#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <Wire.h>
#include <std_msgs/msg/float32_multi_array.h>

// --- CONFIGURATION ---
#define SLAVE_ADDR 0x10  // The I2C Address of hardware ESP32
#define SDA_PIN 21       // Standard ESP32 SDA
#define SCL_PIN 22       // Standard ESP32 SCL

// --- DATA STRUCTURE ---
struct Packet {
  float vx;
  float vy;
  float wz;
  float trigger_seq;
};
Packet txData;

struct FeedbackPacket {
  float heading;
  float tof_distance;
  float rpm[4];
};
FeedbackPacket rxSensors;

// --- ROS OBJECTS ---
rcl_subscription_t sub_vel; 
rcl_subscription_t sub_grip;
geometry_msgs__msg__Twist msg_vel;
std_msgs__msg__Int32 msg_grip;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray sensor_msg;
float sensor_data_buffer[6];

// --- CALLBACK ---
void vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // 1. Prepare Data
  txData.vx = msg->linear.x * 100.0; 
  txData.vy = msg->linear.y * 100.0;
  txData.wz = msg->angular.z * 5.0; 

  // 2. Send via I2C
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t *)&txData, sizeof(Packet));
  Wire.endTransmission();
}

void grip_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  txData.trigger_seq = (float)msg->data;

  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t *)&txData, sizeof(Packet));
  Wire.endTransmission();
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN); 
  Wire.setClock(100000);

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_comms", "", &support);

  rclc_subscription_init_default(
    &sub_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  rclc_subscription_init_default(
    &sub_grip, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/gripper_cmd");

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/robot_status"
  );

  sensor_msg.data.capacity = 6;
  sensor_msg.data.size = 6;
  sensor_msg.data.data = sensor_data_buffer;

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_vel, &msg_vel, &vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_grip, &msg_grip, &grip_callback, ON_NEW_DATA);
}

void loop() {
  uint8_t bytesReceived = Wire.requestFrom(SLAVE_ADDR, sizeof(FeedbackPacket));

  if (Wire.available() == sizeof(FeedbackPacket)) {
    Wire.readBytes((uint8_t *)&rxSensors, sizeof(FeedbackPacket));

    sensor_msg.data.data[0] = rxSensors.heading;
    sensor_msg.data.data[1] = rxSensors.tof_distance;
    sensor_msg.data.data[2] = rxSensors.rpm[0];
    sensor_msg.data.data[3] = rxSensors.rpm[1];
    sensor_msg.data.data[4] = rxSensors.rpm[2];
    sensor_msg.data.data[5] = rxSensors.rpm[3];

    rcl_publish(&publisher, &sensor_msg, NULL);
    
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}
