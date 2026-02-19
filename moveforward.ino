#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define RXD2 16 // Conectar ao TX do Wave Rover
#define TXD2 17 // Conectar ao RX do Wave Rover

// ========== FUNCTION PROTOTYPES ==========
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
void sendToWaveRover(float left, float right);
void led_control_callback(const void* msgin);
void getSpeed_callback(const void* msgin);

rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg_float;
std_msgs__msg__Bool msg_bool;

rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t speed_sub;

// Speed from ESP-NOW or ROS (used for Serial2)
float speedLeft = 0.0f;
float speedRight = 0.0f;

void setup() {
  Serial.begin(115200);

  // Inicializa a Serial2 para comunicação com o Wave Rover
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Moving slow...");
  delay(2000);

  // ---------- ESP-NOW (receive speed from gateway) ----------
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW ready (receive)");

  // ---------- micro-ROS (must init before creating subscriptions) ----------
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "moveforward_node", "", &support);

  // Create subscribers (node is now initialized)
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control"
  );
  rclc_subscription_init_default(
    &speed_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "getSpeed"
  );

  // Executor: add subscriptions and init
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_bool, &led_control_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &speed_sub, &msg_float, &getSpeed_callback, ON_NEW_DATA);
  Serial.println("Micro-ROS initialized");
}

void led_control_callback(const void* msgin) {
  (void)msgin;
  // Optional: drive an LED from "led_control" topic
}

void getSpeed_callback(const void* msgin) {
  const std_msgs__msg__Float64* msg = (const std_msgs__msg__Float64*)msgin;
  speedLeft = (float)msg->data;
  speedRight = (float)msg->data;
  sendToWaveRover(speedLeft, speedRight);
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  (void)info;
  if (len <= 0 || len >= 80) return;
  char buf[80];
  memcpy(buf, incomingData, len);
  buf[len] = '\0';
  float left = 0.0f, right = 0.0f;
  if (sscanf(buf, "speedRight=%f speedLeft=%f", &right, &left) == 2) {
    speedLeft = left;
    speedRight = right;
    sendToWaveRover(speedLeft, speedRight);
  }
}

void sendToWaveRover(float left, float right) {
  // Wave Rover format: {"T":1,"L":left,"R":right}
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}", (double)left, (double)right);
  Serial2.println(cmd);
}



void loop() {
  // Spin micro-ROS executor so subscriptions get callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  delay(10);
}