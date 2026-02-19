// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>

#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x26, 0xFB, 0x28};

// Default speed sent to robot (used until a /speed_cmd message is received from ROS)
#define DEFAULT_SPEED_LEFT  0.2f
#define DEFAULT_SPEED_RIGHT 0.2f

rcl_node_t node;
rcl_publisher_t publisher;
rcl_publisher_t speed;
rclc_support_t support;
rcl_allocator_t allocator;


rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t speed_cmd_sub;
std_msgs__msg__Float64 msg;
std_msgs__msg__Float64 speed_cmd_msg;

float speedRight;
float speedLeft;

void speed_cmd_callback(const void* msgin);

// ========== SETUP FUNCTION ==========
void setup() {
  Serial.begin(115200);
  

  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  esp_now_init();

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  addPeer(peerMAC, NULL);
  
  // Serial.println("Setup complete");
  // Serial.print("MAC Address: ");
  // Serial.println(WiFi.macAddress());

  //   // Initialize micro-ROS
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  
  rclc_node_init_default(
    &node,
    "sonar_node1",
    "",
    &support
  );

  // Create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "distanceProject"
  );
  rclc_publisher_init_default(
    &speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "sendspeed"
  );

  // Subscription to receive speed command from ROS (one value used for both L and R)
  rclc_subscription_init_default(
    &speed_cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "speed_cmd"
  );
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &speed_cmd_sub, &speed_cmd_msg, &speed_cmd_callback, ON_NEW_DATA);

  speedRight = DEFAULT_SPEED_RIGHT;
  speedLeft = DEFAULT_SPEED_LEFT;
  Serial.println("Micro-ROS initialized");
  Serial.print("Default speed: L=");
  Serial.print(speedLeft);
  Serial.print(" R=");
  Serial.println(speedRight);
  Serial.println("(Publish to /speed_cmd to override)");
}

void speed_cmd_callback(const void* msgin) {
  const std_msgs__msg__Float64* m = (const std_msgs__msg__Float64*)msgin;
  speedRight = (float)m->data;
  speedLeft = (float)m->data;
  Serial.print("Received speed command: ");
  Serial.print(speedLeft);
  Serial.print(" (both wheels)");
  Serial.println();
  // Immediately send to moveforward board
  sendSpeed(speedRight, speedLeft);
}

// ========== MAIN LOOP ==========
void loop() {
  // Spin micro-ROS so we receive speed_cmd and stay connected to the agent
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(50);
  // Note: Speed is sent immediately in speed_cmd_callback when received
  // This periodic send ensures the robot keeps moving even if no new commands arrive
  sendSpeed(speedRight, speedLeft);
}

void sendSpeed(float speedRight, float speedLeft){
  char messageBuffer[80];

  int len = snprintf(messageBuffer, sizeof(messageBuffer),
    "speedRight=%06.2f speedLeft=%+06.2f",
    speedRight, speedLeft);

  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)messageBuffer, len);

  if (result == ESP_OK){
    Serial.print("Sent speed to moveforward: L=");
    Serial.print(speedLeft);
    Serial.print(" R=");
    Serial.println(speedRight);
  } else {
    Serial.println("Failed to send speed via ESP-NOW");
  }
}



void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  
  
  char msg[len+1] = {0};
  double number;
  memcpy(&number, (void*)incomingData, sizeof(double));

  Serial.print("Received data: ");
  Serial.println(number);
}
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  return;
}

esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0;
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}