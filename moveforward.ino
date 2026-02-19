#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>

#define RXD2 16 // Conectar ao TX do Wave Rover
#define TXD2 17 // Conectar ao RX do Wave Rover

// ========== DATA STRUCTURE ==========
// Structure to receive velocity commands via ESP-NOW (must match gateway structure)
typedef struct __attribute__((packed)) {
  float linearVel;   // X: linear velocity in m/s
  float angularVel;  // Z: angular velocity in rad/s
} velocity_command_t;

// ========== FUNCTION PROTOTYPES ==========
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
void sendToWaveRover(float linearVel, float angularVel);

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

void loop() {
  // ESP-NOW callbacks are handled asynchronously, so loop can be minimal
  delay(100);
}