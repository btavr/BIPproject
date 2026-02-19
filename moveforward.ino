#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>

#define RXD2 25 // Conectar ao TX do Wave Rover
#define TXD2 26 // Conectar ao RX do Wave Rover

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
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void sendToWaveRover(float linearVel, float angularVel);
void sendCoordenates(float x, float y, float z, float distance, float orientation);
void readCoordinatesFromSerial2();

// Gateway MAC address (to send coordinates back)
uint8_t gatewayMAC[6] = {0x28, 0x05, 0xA5, 0x26, 0xFB, 0x28};  // TODO: Update with actual gateway MAC

// Velocity received from gateway via ESP-NOW
float linearVelocity = 0.0f;   // X: linear velocity in m/s
float angularVelocity = 0.0f;  // Z: angular velocity in rad/s

void setup() {
  Serial.begin(115200);

  // Inicializa a Serial2 para comunicação com o Wave Rover
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Wave Rover controller ready");
  delay(1000);

  // ---------- ESP-NOW (receive speed commands from gateway) ----------
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  
  // Add gateway as peer to send coordinates back
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayMAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add gateway as ESP-NOW peer!");
  }
  
  Serial.println("ESP-NOW ready - waiting for speed commands from gateway");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  (void)info;
  
  Serial.print("ESP-NOW: Received ");
  Serial.print(len);
  Serial.print(" bytes from MAC: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (info->src_addr[i] < 0x10) Serial.print("0");
    Serial.print(info->src_addr[i], HEX);
  }
  Serial.println();
  
  // Check if received data matches our struct size
  if (len != sizeof(velocity_command_t)) {
    Serial.print("ERROR: Invalid data length. Expected ");
    Serial.print(sizeof(velocity_command_t));
    Serial.print(" bytes, received ");
    Serial.println(len);
    return;
  }
  
  // Copy received data into struct
  velocity_command_t velCmd;
  memcpy(&velCmd, incomingData, sizeof(velocity_command_t));
  
  // Extract velocity values from struct
  linearVelocity = velCmd.linearVel;
  angularVelocity = velCmd.angularVel;
  
  // Note: targetX, targetY, targetZ are available but not used for velocity control
  
  Serial.print("Received velocity: X=");
  Serial.print(linearVelocity);
  Serial.print(" m/s, Z=");
  Serial.print(angularVelocity);
  Serial.println(" rad/s");
  
  // Send command to Wave Rover robot using ROS Control format
  sendToWaveRover(linearVelocity, angularVelocity);
}

void sendToWaveRover(float linearVel, float angularVel) {
  // ROS Control format: {"T":13,"X":linearVel,"Z":angularVel}
  // X = linear velocity in m/s, Z = angular velocity in rad/s
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", (double)linearVel, (double)angularVel);
  
  // Check if Serial2 is available before sending
  if (Serial2.availableForWrite() > 0) {
    Serial2.println(cmd);
    Serial.print("Sent to robot via Serial2: ");
    Serial.println(cmd);
  } else {
    Serial.println("ERROR: Serial2 not available for writing!");
  }
}

void sendCoordenates(float x, float y, float z, float distance, float orientation){
  // Create struct with coordinate data
  current_coordinates_t coordCmd;
  coordCmd.currentX = x;
  coordCmd.currentY = y;
  coordCmd.currentZ = z;
  coordCmd.distance = distance;
  coordCmd.orientation = orientation;

  // Send struct as binary data via ESP-NOW
  esp_err_t result = esp_now_send(gatewayMAC, (uint8_t *)&coordCmd, sizeof(current_coordinates_t));

  if (result == ESP_OK){
    Serial.print("Sent coordinates to gateway: X=");
    Serial.print(x);
    Serial.print(", Y=");
    Serial.print(y);
    Serial.print(", Z=");
    Serial.print(z);
    Serial.print(", distance=");
    Serial.print(distance);
    Serial.print(", orientation=");
    Serial.print(orientation);
    Serial.print(" (size=");
    Serial.print(sizeof(current_coordinates_t));
    Serial.println(" bytes)");
  } else {
    Serial.print("Failed to send coordinates via ESP-NOW. Error code: ");
    Serial.println(result);
  }
}

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  // Only log failures
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("ESP-NOW: Coordinate send failed!");
  }
}

void readCoordinatesFromSerial2() {
  // Check if we have enough data (15 bytes: header + position data)
  if (Serial2.available() >= 15) { 
    byte header_type = Serial2.read();   
    byte header_len = Serial2.read();    
    byte err_code = Serial2.read();      

    if (err_code == 0) {
      byte pos_type = Serial2.read();    
      byte pos_len = Serial2.read();     

      int32_t x = 0, y = 0, z = 0;
      byte qf = 0;

      // Read 32-bit integers (little-endian)
      x = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      y = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      z = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      qf = Serial2.read();

      // Convert int32_t to float (assuming coordinates are in millimeters, convert to meters)
      // Adjust the scale factor if your robot uses a different unit
      float x_m = (float)x / 1000.0f;  // Convert mm to meters (adjust if needed)
      float y_m = (float)y / 1000.0f;
      float z_m = (float)z / 1000.0f;
      
      // Use qf as quality factor, you can use it as distance or orientation
      // For now, we'll use it as a quality/distance metric
      float distance = (float)qf;
      float orientation = 0.0f;  // You may need to calculate this separately

      Serial.print("Received coordinates from robot: x=");
      Serial.print(x);
      Serial.print(" (raw), y=");
      Serial.print(y);
      Serial.print(" (raw), z=");
      Serial.print(z);
      Serial.print(" (raw), qf=");
      Serial.println(qf);
      
      Serial.print("Converted to meters: x=");
      Serial.print(x_m, 6);
      Serial.print(", y=");
      Serial.print(y_m, 6);
      Serial.print(", z=");
      Serial.print(z_m, 6);
      Serial.println();

      // Send coordinates via ESP-NOW using the structured format
      sendCoordenates(x_m, y_m, z_m, distance, orientation);
      
      // Alternative: Send as JSON if you prefer (uncomment below and comment above)
      /*
      char payload[100]; 
      snprintf(payload, sizeof(payload), "{\"x\":%d, \"y\":%d, \"z\":%d, \"qf\":%d}", x, y, z, qf);
      Serial.print("Sending JSON: ");
      Serial.println(payload);
      esp_err_t result = esp_now_send(gatewayMAC, (uint8_t *)payload, strlen(payload) + 1);
      if (result != ESP_OK) {
        Serial.print("Failed to send JSON coordinates. Error code: ");
        Serial.println(result);
      }
      */
    } else {
      Serial.print("Error reading coordinates. Error code: ");
      Serial.println(err_code);
    }
    
    // Clear any remaining data from buffer
    while(Serial2.available()) Serial2.read();
  }
}

void loop() {
  // Read coordinates from Wave Rover robot
  readCoordinatesFromSerial2();
  
  // ESP-NOW callbacks are handled asynchronously
  delay(50);  // Reduced delay for more responsive coordinate reading
}