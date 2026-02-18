// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
byte SONAR_TRIGGER_PIN = 14;
byte SONAR_ECHO_PIN = 12;
#include <HCSR04.h>


// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

// Variables
uint8_t peerMAC[6] = {0xE0, 0x8C, 0xFE, 0x36, 0x62, 0xA8};

// ========== SETUP FUNCTION ==========
void setup() {

  Serial.begin(115200);
  
  // Initialize serial communication for debugging
  HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
  Serial.println("Sensor initialized");
  

  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  addPeer(peerMAC, NULL);
  
  Serial.println("Remote sensor ready");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// ========== MAIN LOOP ==========
void loop() {

   double* distances = HCSR04.measureDistanceCm();

  esp_now_send(peerMAC, (uint8_t*)distances, sizeof(double));
  delay(500);
  Serial.println(distances[0]);

}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy(msg,(void*)incomingData, len);

  Serial.print("Received data: ");
  Serial.println((char*) msg);
}

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Last Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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