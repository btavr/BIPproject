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
#include <std_msgs/msg/string.h>

// ========== DATA STRUCTURE ==========
// Structure to send velocity commands via ESP-NOW
typedef struct __attribute__((packed)) {
  float targetX;      // Target X coordinate (meters)
  float targetY;      // Target Y coordinate (meters)
  float targetZ;      // Target Z coordinate (meters) or orientation
  float linearVel;    // Linear velocity in m/s
  float angularVel;   // Angular velocity in rad/s
} velocity_command_t;


typedef struct __attribute__((packed)) {
  float currentX;      // Target X coordinate 
  float currentY;      // Target Y coordinate 
  float currentZ;      // Target Z coordinate ) or orientation
  float distance;
  float orientation;   
} current_coordinates_t;

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
rcl_publisher_t linear_vel_pub;   // Publisher for linear velocity
rcl_publisher_t angular_vel_pub;  // Publisher for angular velocity
rcl_publisher_t current_coordinates_pub;  // Publisher for all coordinates
rclc_support_t support;
rcl_allocator_t allocator;


rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t linear_vel_sub;
rcl_subscription_t angular_vel_sub;
std_msgs__msg__Float64 msg;
std_msgs__msg__Float64 linear_vel_msg;
std_msgs__msg__Float64 angular_vel_msg;
std_msgs__msg__Float64 linear_vel_pub_msg;   // Message for publishing linear velocity
std_msgs__msg__Float64 angular_vel_pub_msg;  // Message for publishing angular velocity
std_msgs__msg__String coordinates_msg;       // Message for publishing all coordinates

float linearVelocity;   // X: linear velocity in m/s
float angularVelocity;  // Z: angular velocity in rad/s

// Current coordinates received from coordinates.ino
float currentX = 0.0f;
float currentY = 0.0f;
float currentZ = 0.0f;
int qualityFactor = 0;
char coordinates_str[128];  // Buffer for coordinate string message

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

  // Publishers to publish current velocity values to ROS topics
  rclc_publisher_init_default(
    &linear_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "linear_velocity"
  );
  rclc_publisher_init_default(
    &angular_vel_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "angular_velocity"
  );

  // Publisher to publish all current coordinates to ROS topic
  rclc_publisher_init_default(
    &current_coordinates_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "current_coordinates"
  );
  
  // Initialize string message to use static buffer
  coordinates_msg.data.data = coordinates_str;
  coordinates_msg.data.size = 0;
  coordinates_msg.data.capacity = sizeof(coordinates_str);

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
  // Serial.println("Micro-ROS initialized");
  // Serial.print("Default velocities: X=");
  // Serial.print(linearVelocity);
  // Serial.print(" m/s, Z=");
  // Serial.print(angularVelocity);
  // Serial.println(" rad/s");
  // Serial.println("(Publish to /linear_velocity and /angular_velocity to override)");
}

void linear_vel_callback(const void* msgin) {
  const std_msgs__msg__Float64* m = (const std_msgs__msg__Float64*)msgin;
  linearVelocity = (float)m->data;
  // Serial.print("Received linear velocity (X): ");
  // Serial.print(linearVelocity);
  // Serial.println(" m/s");
  // Immediately send to moveforward board
  sendVelocity(linearVelocity, angularVelocity);
}

void angular_vel_callback(const void* msgin) {
  const std_msgs__msg__Float64* m = (const std_msgs__msg__Float64*)msgin;
  angularVelocity = (float)m->data;
  // Serial.print("Received angular velocity (Z): ");
  // Serial.print(angularVelocity);
  // Serial.println(" rad/s");
  // Immediately send to moveforward board
  sendVelocity(linearVelocity, angularVelocity);
}

// ========== MAIN LOOP ==========
void loop() {
  // Spin micro-ROS so we receive velocity commands and stay connected to the agent
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  
  // Publish current velocity values to ROS topics
  linear_vel_pub_msg.data = (double)linearVelocity;
  rcl_publish(&linear_vel_pub, &linear_vel_pub_msg, NULL);
  
  angular_vel_pub_msg.data = (double)angularVelocity;
  rcl_publish(&angular_vel_pub, &angular_vel_pub_msg, NULL);
  
  // Publish all coordinates as a single string message
  int str_len = snprintf(coordinates_str, sizeof(coordinates_str), "{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"qf\":%d}", 
                         currentX, currentY, currentZ, qualityFactor);
  if (str_len > 0 && str_len < sizeof(coordinates_str)) {
    coordinates_msg.data.data = coordinates_str;
    coordinates_msg.data.size = str_len;
    rcl_publish(&current_coordinates_pub, &coordinates_msg, NULL);
  }
  
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

  // if (result == ESP_OK){
  //   Serial.print("Sent velocity to moveforward: X=");
  //   Serial.print(linearVel);
  //   Serial.print(" m/s, Z=");
  //   Serial.print(angularVel);
  //   Serial.println(" rad/s");
  // } else {
  //   Serial.println("Failed to send velocity via ESP-NOW");
  // }
}



void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // Check if this is JSON coordinate data from coordinates.ino
  // Format: {"x":1234, "y":5678, "z":90, "qf":100}
  if (len > 0 && incomingData[0] == '{') {
    // This is JSON coordinate data
    char jsonStr[len + 1];
    memcpy(jsonStr, incomingData, len);
    jsonStr[len] = '\0';
    
    // Parse JSON: {"x":1234, "y":5678, "z":90, "qf":100}
    int x = 0, y = 0, z = 0, qf = 0;
    if (sscanf(jsonStr, "{\"x\":%d, \"y\":%d, \"z\":%d, \"qf\":%d}", &x, &y, &z, &qf) == 4) {
      // Convert from millimeters to meters (Qorvo typically sends in mm)
      currentX = x / 1000.0f;
      currentY = y / 1000.0f;
      currentZ = z / 1000.0f;
      qualityFactor = qf;
      
      // Serial.print("Received coordinates: X=");
      // Serial.print(currentX);
      // Serial.print(" m, Y=");
      // Serial.print(currentY);
      // Serial.print(" m, Z=");
      // Serial.print(currentZ);
      // Serial.print(" m, QF=");
      // Serial.println(qualityFactor);
    } else {
      // Serial.print("Failed to parse coordinates JSON: ");
      // Serial.println(jsonStr);
    }
  } else {
    // Handle other data types if needed
    // Serial.print("Received non-JSON data, length: ");
    // Serial.println(len);
  }
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