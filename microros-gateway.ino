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
// Structure to send velocity commands via ESP-NOW (must match moveforward.ino)
typedef struct __attribute__((packed)) {
  float targetX;      // Target X coordinate (meters)
  float targetY;      // Target Y coordinate (meters)
  float targetZ;      // Target Z coordinate (meters) or orientation
  float linearVel;    // Linear velocity in m/s
  float angularVel;   // Angular velocity in rad/s
} velocity_command_t;

// Structure to receive current coordinates from moveforward board
typedef struct __attribute__((packed)) {
  float currentX;      // Current X coordinate (meters)
  float currentY;      // Current Y coordinate (meters)
  float currentZ;      // Current Z coordinate (meters) or orientation
  float distance;      // Distance measurement
  float orientation;   // Orientation in radians
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
rcl_publisher_t current_x_pub;    // Publisher for current X coordinate
rcl_publisher_t current_y_pub;    // Publisher for current Y coordinate
rcl_publisher_t current_z_pub;    // Publisher for current Z coordinate
rcl_publisher_t distance_pub;      // Publisher for distance
rcl_publisher_t orientation_pub;  // Publisher for orientation
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
std_msgs__msg__Float64 current_x_pub_msg;    // Message for publishing current X
std_msgs__msg__Float64 current_y_pub_msg;    // Message for publishing current Y
std_msgs__msg__Float64 current_z_pub_msg;    // Message for publishing current Z
std_msgs__msg__Float64 distance_pub_msg;      // Message for publishing distance
std_msgs__msg__Float64 orientation_pub_msg;  // Message for publishing orientation

float linearVelocity;   // X: linear velocity in m/s
float angularVelocity;  // Z: angular velocity in rad/s

void linear_vel_callback(const void* msgin);
void angular_vel_callback(const void* msgin);

// ========== SETUP FUNCTION ==========
void setup() {
  Serial.begin(115200);
  

  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  if (addPeer(peerMAC, NULL) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
    return;
  }
  
  Serial.println("Setup complete");
  Serial.print("Gateway MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Target peer MAC: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (peerMAC[i] < 0x10) Serial.print("0");
    Serial.print(peerMAC[i], HEX);
  }
  Serial.println();

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
  
  // Publishers for current coordinates from robot
  rclc_publisher_init_default(
    &current_x_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "current_x"
  );
  rclc_publisher_init_default(
    &current_y_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "current_y"
  );
  rclc_publisher_init_default(
    &current_z_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "current_z"
  );
  rclc_publisher_init_default(
    &distance_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "distance"
  );
  rclc_publisher_init_default(
    &orientation_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "orientation"
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
  
  // Publish current velocity values to ROS topics
  linear_vel_pub_msg.data = (double)linearVelocity;
  rcl_publish(&linear_vel_pub, &linear_vel_pub_msg, NULL);
  
  angular_vel_pub_msg.data = (double)angularVelocity;
  rcl_publish(&angular_vel_pub, &angular_vel_pub_msg, NULL);
  
  delay(50);
  // Note: Velocity is sent immediately in callbacks when received
  // This periodic send ensures the robot keeps moving even if no new commands arrive
  sendVelocity(linearVelocity, angularVelocity);
}

void sendVelocity(float linearVel, float angularVel){
  // Create struct with velocity data
  velocity_command_t velCmd;
  velCmd.targetX = 0.0f;      // Not used for velocity-only commands
  velCmd.targetY = 0.0f;      // Not used for velocity-only commands
  velCmd.targetZ = 0.0f;      // Not used for velocity-only commands
  velCmd.linearVel = linearVel;
  velCmd.angularVel = angularVel;

  // Send struct as binary data via ESP-NOW
  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)&velCmd, sizeof(velocity_command_t));

  if (result == ESP_OK){
    Serial.print("Sent velocity to moveforward: X=");
    Serial.print(linearVel);
    Serial.print(" m/s, Z=");
    Serial.print(angularVel);
    Serial.print(" rad/s (size=");
    Serial.print(sizeof(velocity_command_t));
    Serial.println(" bytes)");
  } else {
    Serial.print("Failed to send velocity via ESP-NOW. Error code: ");
    Serial.println(result);
  }
}



void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  Serial.print("ESP-NOW: Received ");
  Serial.print(len);
  Serial.print(" bytes from MAC: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (info->src_addr[i] < 0x10) Serial.print("0");
    Serial.print(info->src_addr[i], HEX);
  }
  Serial.println();
  
  // Check if this is coordinates data (from moveforward board)
  if (len == sizeof(current_coordinates_t)) {
    current_coordinates_t coords;
    memcpy(&coords, incomingData, sizeof(current_coordinates_t));
    
    Serial.print("Received coordinates: X=");
    Serial.print(coords.currentX);
    Serial.print(", Y=");
    Serial.print(coords.currentY);
    Serial.print(", Z=");
    Serial.print(coords.currentZ);
    Serial.print(", distance=");
    Serial.print(coords.distance);
    Serial.print(", orientation=");
    Serial.println(coords.orientation);
    
    // Publish coordinates to ROS topics
    current_x_pub_msg.data = (double)coords.currentX;
    rcl_publish(&current_x_pub, &current_x_pub_msg, NULL);
    
    current_y_pub_msg.data = (double)coords.currentY;
    rcl_publish(&current_y_pub, &current_y_pub_msg, NULL);
    
    current_z_pub_msg.data = (double)coords.currentZ;
    rcl_publish(&current_z_pub, &current_z_pub_msg, NULL);
    
    distance_pub_msg.data = (double)coords.distance;
    rcl_publish(&distance_pub, &distance_pub_msg, NULL);
    
    orientation_pub_msg.data = (double)coords.orientation;
    rcl_publish(&orientation_pub, &orientation_pub_msg, NULL);
    
    Serial.println("Published coordinates to ROS topics");
  } else {
    // Legacy data format (for backward compatibility)
    Serial.print("Received legacy data (");
    Serial.print(len);
    Serial.println(" bytes)");
  }
}
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW: Message sent successfully");
  } else {
    Serial.println("ESP-NOW: Message send failed!");
  }
}

esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  esp_now_peer_info_t* localPeerInfo;
  if(peerInfo == NULL){
    localPeerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(localPeerInfo, 0, sizeof(esp_now_peer_info_t));
  } else {
    localPeerInfo = peerInfo;
  }
  memcpy(localPeerInfo->peer_addr, mac, 6);
  localPeerInfo->channel = 1;  // Match the channel set in setup()
  localPeerInfo->encrypt = false;
  esp_err_t result = esp_now_add_peer(localPeerInfo);
  if (peerInfo == NULL) {
    free(localPeerInfo);  // Free allocated memory
  }
  return result;
}