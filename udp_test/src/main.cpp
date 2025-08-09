#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#define LED_PIN 2

// Wi-Fi credentials
const char ssid[] = "BARANI_ON";
const char pass[] = "S.BARANI333@2713";

// micro-ROS Agent details
const char agent_ip[] = "192.168.1.101"; // Replace with your agent's IP
const uint16_t agent_port = 8888;        // Replace with your agent's port

// ROS 2 publisher variables
rcl_publisher_t publisher;
std_msgs__msg__String msg;
char msg_buffer[50];

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

void setup() {
  Serial.begin(115200);
  delay(2000); // Allow time for Serial Monitor to start
  pinMode(LED_PIN, OUTPUT);

  Serial.println("Connecting to Wi-Fi and micro-ROS agent...");

  // Connect micro-ROS over Wi-Fi UDP
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Error initializing micro-ROS support!");
    return;
  }

  // Create node
  if (rclc_node_init_default(&node, "esp32_simple_pub", "", &support) != RCL_RET_OK) {
    Serial.println("Error creating micro-ROS node!");
    return;
  }

  // Create publisher
  if (rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "esp32_chatter") != RCL_RET_OK) {
    Serial.println("Error creating publisher!");
    return;
  }

  // Prepare message memory
  msg.data.data = msg_buffer;
  msg.data.capacity = sizeof(msg_buffer);
}

void loop() {
  // Fill the message
  snprintf(msg_buffer, sizeof(msg_buffer), "Hello from ESP32! Time: %lu", millis());
  msg.data.size = strlen(msg_buffer);

  // Publish message
  if (rcl_publish(&publisher, &msg, NULL) != RCL_RET_OK) {
    Serial.println("Publish failed!");
  } else {
    Serial.println(msg_buffer);
  }

  // Blink LED to show activity
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  delay(1000); // 1-second loop
}
