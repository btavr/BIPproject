#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>

#define RXD2 16 // Conectar ao TX do Wave Rover
#define TXD2 17 // Conectar ao RX do Wave Rover

// ========== FUNCTION PROTOTYPES ==========
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
void sendToWaveRover(float left, float right);

// Speed received from gateway via ESP-NOW
float speedLeft = 0.0f;
float speedRight = 0.0f;

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
  if (len <= 0 || len >= 80) return;
  
  char buf[80];
  memcpy(buf, incomingData, len);
  buf[len] = '\0';
  
  // Parse the speed command from gateway: "speedRight=XX.XX speedLeft=XX.XX"
  float left = 0.0f, right = 0.0f;
  if (sscanf(buf, "speedRight=%f speedLeft=%f", &right, &left) == 2) {
    speedLeft = left;
    speedRight = right;
    Serial.print("Received speed: L=");
    Serial.print(speedLeft);
    Serial.print(" R=");
    Serial.println(speedRight);
    // Send command to Wave Rover robot
    sendToWaveRover(speedLeft, speedRight);
  } else {
    Serial.print("Failed to parse speed command: ");
    Serial.println(buf);
  }
}

void sendToWaveRover(float left, float right) {
  // Wave Rover format: {"T":1,"L":left,"R":right}
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}", (double)left, (double)right);
  Serial2.println(cmd);
  Serial.print("Sent to robot: ");
  Serial.println(cmd);
}

void loop() {
  // ESP-NOW callbacks are handled asynchronously, so loop can be minimal
  delay(100);
}