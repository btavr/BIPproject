#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// ========== PINOS E CONFIGURAÇÕES ==========
#define RXD2 16
#define TXD2 17

// Endereço MAC do ESP32 que vai RECEBER os dados
uint8_t peerMAC[6] = {0xE0, 0x8C, 0xFE, 0x36, 0x62, 0xA8};

// ========== PROTÓTIPOS DE FUNÇÕES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

typedef struct __attribute__((packed)) {
  float targetX;      // Target X coordinate (meters)
  float targetY;      // Target Y coordinate (meters)
  float targetZ;      // Target Z coordinate (meters) or orientation
  float linearVel;    // Linear velocity in m/s
  float angularVel;   // Angular velocity in rad/s
} robot_command_t;

typedef struct __attribute__((packed)) {
  float currentX;      // Target X coordinate 
  float currentY;      // Target Y coordinate 
  float currentZ;      // Target Z coordinate ) or orientation
  float distance;
  float orientation;   
} current_coordinates_t;

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  // Configuração Wi-Fi para ESP-NOW
  WiFi.mode(WIFI_STA); // Modo station é o correto para ESP-NOW
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Iniciar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Registar callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Adicionar o dispositivo recetor
  addPeer(peerMAC, NULL);
  
  Serial.println("System Ready. Requesting Qorvo position...");
  Serial.print("My MAC Address: ");
  Serial.println(WiFi.macAddress());
}

// ========== LOOP PRINCIPAL ==========
void loop() {
  // 1. Enviar comando binário dwm_pos_get (0x02 0x00)
  Serial2.write(0x02);
  Serial2.write(0x00);

  delay(150); // Pausa para a Tag processar

  // 2. Ler a resposta
  if (Serial2.available() >= 15) { 
    byte header_type = Serial2.read();   
    byte header_len = Serial2.read();    
    byte err_code = Serial2.read();      

    if (err_code == 0) {
      byte pos_type = Serial2.read();    
      byte pos_len = Serial2.read();     

      int32_t x = 0, y = 0, z = 0;
      byte qf = 0;

      x = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      y = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      z = Serial2.read() | (Serial2.read() << 8) | (Serial2.read() << 16) | (Serial2.read() << 24);
      qf = Serial2.read();

      // Formatar a mensagem numa String tipo JSON
      char payload[100]; 
      snprintf(payload, sizeof(payload), "{\"x\":%d, \"y\":%d, \"z\":%d, \"qf\":%d}", x, y, z, qf);

      // Imprimir localmente
      Serial.print("Sending: ");
      Serial.println(payload);

      // Enviar via ESP-NOW
      // Usamos strlen(payload) + 1 para garantir que enviamos o carácter de fim de string '\0'
      esp_err_t result = esp_now_send(peerMAC, (uint8_t *)payload, strlen(payload) + 1);

    }
    
    // Limpar lixo do buffer
    while(Serial2.available()) Serial2.read();
  }

  delay(500); // Atualiza a cada meio segundo
}

// ========== FUNÇÕES DE CALLBACK E AUXILIARES ==========

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  char msg[len+1] = {0};
  memcpy(msg, (void*)incomingData, len);

  Serial.print("Received back: ");
  Serial.println((char*) msg);
}

esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0; // O canal tem de coincidir com o configurado
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}
