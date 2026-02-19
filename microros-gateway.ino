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

// ========== DATA STRUCTURE ==========
// Structure to send velocity commands via ESP-NOW
typedef struct __attribute__((packed)) {
  float linearVel;   // X: linear velocity in m/s
  float angularVel;  // Z: angular velocity in rad/s
} velocity_command_t;

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables
uint8_t peerMAC[6] = {0x28, 0x05, 0xA5, 0x26, 0xFB, 0x28};

// Default velocities sent to robot (used until ROS messages are received)
#define DEFAULT_LINEAR_VEL  0.1f   // m/s
#define DEFAULT_ANGULAR_VEL 0.0f   // rad/s

rcl_node_t node;
rcl_publisher_t publisher;
rcl_publisher_t speed;
rclc_support_t support;
rcl_allocator_t allocator;


rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t linear_vel_sub;
rcl_subscription_t angular_vel_sub;
std_msgs__msg__Float64 msg;
std_msgs__msg__Float64 linear_vel_msg;
std_msgs__msg__Float64 angular_vel_msg;

float linearVelocity;   // X: linear velocity in m/s
float angularVelocity;  // Z: angular velocity in rad/s

void linear_vel_callback(const void* msgin);
void angular_vel_callback(const void* msgin);

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

  // Subscriptions to receive velocity commands from ROS
  // /linear_velocity: X value (linear velocity in m/s)
  rclc_subscription_init_default(
    &linear_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "linear_velocity"
  );
  // /angular_velocity: Z value (angular velocity in rad/s)
  rclc_subscription_init_default(
    &angular_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "angular_velocity"
  );
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &linear_vel_sub, &linear_vel_msg, &linear_vel_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &angular_vel_sub, &angular_vel_msg, &angular_vel_callback, ON_NEW_DATA);

  linearVelocity = DEFAULT_LINEAR_VEL;
  angularVelocity = DEFAULT_ANGULAR_VEL;
  Serial.println("Micro-ROS initialized");
  Serial.print("Default velocities: X=");
  Serial.print(linearVelocity);
  Serial.print(" m/s, Z=");
  Serial.print(angularVelocity);
  Serial.println(" rad/s");
  Serial.println("(Publish to /linear_velocity and /angular_velocity to override)");
}

void linear_vel_callback(const void* msgin) {
  const std_msgs__msg__Float64* m = (const std_msgs__msg__Float64*)msgin;
  linearVelocity = (float)m->data;
  Serial.print("Received linear velocity (X): ");
  Serial.print(linearVelocity);
  Serial.println(" m/s");
  // Immediately send to moveforward board
  sendVelocity(linearVelocity, angularVelocity);
}

void angular_vel_callback(const void* msgin) {
  const std_msgs__msg__Float64* m = (const std_msgs__msg__Float64*)msgin;
  angularVelocity = (float)m->data;
  Serial.print("Received angular velocity (Z): ");
  Serial.print(angularVelocity);
  Serial.println(" rad/s");
  // Immediately send to moveforward board
  sendVelocity(linearVelocity, angularVelocity);
}

// ========== MAIN LOOP ==========
void loop() {
  // Spin micro-ROS so we receive velocity commands and stay connected to the agent
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  delay(50);
  // Note: Velocity is sent immediately in callbacks when received
  // This periodic send ensures the robot keeps moving even if no new commands arrive
  sendVelocity(linearVelocity, angularVelocity);
}

void sendVelocity(float linearVel, float angularVel){
  // Create struct with velocity data
  velocity_command_t velCmd;
  velCmd.linearVel = linearVel;
  velCmd.angularVel = angularVel;

  // Send struct as binary data via ESP-NOW
  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)&velCmd, sizeof(velocity_command_t));

  if (result == ESP_OK){
    Serial.print("Sent velocity to moveforward: X=");
    Serial.print(linearVel);
    Serial.print(" m/s, Z=");
    Serial.print(angularVel);
    Serial.println(" rad/s");
  } else {
    Serial.println("Failed to send velocity via ESP-NOW");
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