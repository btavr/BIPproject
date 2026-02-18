// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

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

rcl_node_t node;
rcl_publisher_t publisher;
rcl_publisher_t speed;
rclc_support_t support;
rcl_allocator_t allocator;


rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__Float64 msg;

float speedRight;
float speedLeft;

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
  Serial.println("Micro-ROS initialized");
}

// ========== MAIN LOOP ==========
void loop() {

  delay(1000);
  char msg[16] = "Hello";
  esp_now_send(peerMAC, (uint8_t*)msg, strlen(msg));
  sendSpeed(speedRight, speedLeft);
}

void sendSpeed(float speedRight, float speedLeft){
  char messageBuffer[80];

  int len = snprintf(messageBuffer, sizeof(messageBuffer),
    "speedRight=%06.2f speedLeft=%+06.2f", // simplify string and create onDataRec
    speedRight, speedLeft);

  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)messageBuffer, len);

  if (result == ESP_OK){
    Serial.print("Sent successfully");
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