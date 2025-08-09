#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Adafruit_PWMServoDriver.h>

// ==== Configuration ====
#define LED_PIN 2                 // ESP32 onboard LED
#define PCA9685_ADDR 0x40
#define NUM_CHANNELS 16           // Total servo channels (PCA9685 supports 16)
#define SERVO_MIN 150             // PCA9685 pulse value for 0°
#define SERVO_MAX 600             // PCA9685 pulse value for 180°
#define PCA9685_TRY_ATTEMPTS 5    // startup retries
#define PCA9685_RECHECK_MS 2000   // check interval in loop

// ==== Hardware ====
Adafruit_PWMServoDriver pwmDriver = Adafruit_PWMServoDriver(PCA9685_ADDR);
bool pca9685Connected = false;

// ==== micro-ROS objects ====
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray recv_msg;
std_msgs__msg__Int32MultiArray send_msg;
rclc_executor_t executor;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// ==== Static buffers for message sequences ====
static int32_t recv_data[NUM_CHANNELS];
static int32_t send_data[NUM_CHANNELS];

// ==== Check if PCA9685 is connected on I2C ====
bool checkPCA9685() {
  Wire.beginTransmission(PCA9685_ADDR);
  return (Wire.endTransmission() == 0);
}

// ==== Try to connect PCA9685 with retries ====
bool tryConnectPCA9685() {
  for (int attempt = 1; attempt <= PCA9685_TRY_ATTEMPTS; attempt++) {
    if (checkPCA9685()) {
      pwmDriver.begin();
      pwmDriver.setPWMFreq(50);
      Serial.println("PCA9685 detected!");
      return true;
    }
    Serial.printf("PCA9685 not found, attempt %d/%d\n", attempt, PCA9685_TRY_ATTEMPTS);
    delay(500);
  }
  Serial.println("PCA9685 not found after attempts");
  return false;
}

// ==== Subscription callback: receive servo angles and drive servos ====
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  Serial.print("Received servo angles: ");
  size_t count = msg->data.size;
  if(count > NUM_CHANNELS) count = NUM_CHANNELS;  // safety limit

  for (size_t i = 0; i < count; i++) {
    int32_t angle = msg->data.data[i];
    Serial.printf("[%d]=%ld ", (int)i, (long)angle);

    if (pca9685Connected) {
      angle = constrain(angle, 0, 180);
      int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
      pwmDriver.setPWM((uint8_t)i, 0, pulse);
    }

    // Copy to feedback array
    send_data[i] = angle;
  }
  Serial.println();

  send_msg.data.size = count;
  // Publish feedback
  rcl_ret_t ret = rcl_publish(&publisher, &send_msg, NULL);
  if (ret != RCL_RET_OK) {
    Serial.printf("Failed to publish feedback: %d\n", (int)ret);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {} // wait for Serial

  Wire.begin();

  set_microros_serial_transports(Serial);
  delay(2000);

  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_PIN, 0);

  pca9685Connected = tryConnectPCA9685();
  ledcWrite(0, pca9685Connected ? 255 : 0);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_pca9685_node", "", &support);

  // Subscriber to servo_cmds
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_cmds"
  );

  // Publisher for servo_feedback
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_feedback"
  );

  // Setup static message buffers
  recv_msg.data.data = recv_data;
  recv_msg.data.size = 0;
  recv_msg.data.capacity = NUM_CHANNELS;

  send_msg.data.data = send_data;
  send_msg.data.size = 0;
  send_msg.data.capacity = NUM_CHANNELS;

  // Setup executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA);

  Serial.println("Setup complete, waiting for servo commands...");
}

void loop() {
  // Periodically check PCA9685 connection
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= PCA9685_RECHECK_MS) {
    lastCheck = millis();
    bool connectedNow = checkPCA9685();
    if (connectedNow != pca9685Connected) {
      pca9685Connected = connectedNow;
      if (pca9685Connected) {
        pwmDriver.begin();
        pwmDriver.setPWMFreq(50);
        ledcWrite(0, 255);
        Serial.println("PCA9685 reconnected and reinitialized.");
      } else {
        ledcWrite(0, 0);
        Serial.println("PCA9685 disconnected.");
      }
    }
  }

  // Process incoming micro-ROS messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
