#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <string.h>

// ========== PINOS E CONFIGURAÇÕES ==========

// Pinos para a Qorvo (Usando Serial2 - Igual ao teu script 2)
#define RX_QORVO 16
#define TX_QORVO 17

// Pinos para o Wave Rover (Usando Serial1)
#define RX_ROVER 26 // Conectar ao TX do Wave Rover
#define TX_ROVER 25 // Conectar ao RX do Wave Rover

// Endereço MAC do ESP32 que vai RECEBER as coordenadas da Qorvo (microros-gateway)
// Este é o MAC do dispositivo que executa microros-gateway.ino
uint8_t peerMAC[6] = {0xE0, 0x8C, 0xFE, 0x36, 0x62, 0xA8};

// NOTA IMPORTANTE: 
// - final.ino envia coordenadas PARA este MAC (microros-gateway)
// - final.ino RECEBE comandos de velocidade DE microros-gateway (não precisa adicionar peer para receber)
// - microros-gateway.ino deve ter peerMAC apontando para o MAC deste dispositivo (final.ino)
// - Para descobrir o MAC deste dispositivo, ver Serial.print("My MAC Address:") no setup

// ========== DATA STRUCTURES (WAVE ROVER) ==========
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

float linearVelocity = 0.0f;
float angularVelocity = 0.0f;

// ========== VARIÁVEIS DE CONTROLO DO TEMPO (QORVO) ==========
unsigned long qorvoTimer = 0;
int qorvoState = 0; // 0 = Espera 500ms para pedir, 1 = Espera 150ms para ler

// ========== PROTÓTIPOS DE FUNÇÕES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);
void sendToWaveRover(float linearVel, float angularVel);



// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  
  // Iniciar Serial1 para o Wave Rover (Pinos 26 e 25)
  Serial1.begin(115200, SERIAL_8N1, RX_ROVER, TX_ROVER);
  
  // Iniciar Serial2 para a Qorvo (Pinos 16 e 17)
  Serial2.begin(115200, SERIAL_8N1, RX_QORVO, TX_QORVO);
  
  delay(1000);

  // Configuração Wi-Fi para ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  
  // Iniciar ESP-NOW
  esp_now_init();
  
  // Registar callbacks (Juntámos os dois)
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Adicionar o dispositivo recetor para enviar as coordenadas
  addPeer(peerMAC, NULL);
  
  // Inicia o relógio para a Qorvo
  qorvoTimer = millis(); 
}

// ========== LOOP PRINCIPAL ==========
void loop() {
  
  // --- LÓGICA DA QORVO (Sem usar delays que bloqueiem o código) ---
  
  // Estado 0: Enviar comando dwm_pos_get (0x02 0x00) a cada 500ms
  if (qorvoState == 0 && (millis() - qorvoTimer >= 500)) {
    Serial2.write(0x02);
    Serial2.write(0x00);
    qorvoTimer = millis();
    qorvoState = 1; // Passa ao estado de leitura
  }
  
  // Estado 1: Ler a resposta após dar 150ms para a Tag processar
  if (qorvoState == 1 && (millis() - qorvoTimer >= 150)) {
    
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

        // Enviar via ESP-NOW as coordenadas
        esp_now_send(peerMAC, (uint8_t *)payload, strlen(payload) + 1);
      }
      
      // Limpar lixo do buffer
      while(Serial2.available()) Serial2.read();
    }
    
    qorvoTimer = millis();
    qorvoState = 0; // Volta ao estado de esperar 500ms para pedir novamente
  }
  
  // O ESP-NOW trata das velocidades do Wave Rover autonomamente por trás!
}

// ========== FUNÇÕES DE CALLBACK (ESP-NOW E WAVE ROVER) ==========

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // Como recebemos dois tipos de dados diferentes, vamos distingui-los pelo tamanho
  
  if (len == sizeof(velocity_command_t)) {
    // 1. DADOS PARA O WAVE ROVER (A struct das velocidades)
    velocity_command_t velCmd;
    memcpy(&velCmd, incomingData, sizeof(velocity_command_t));
    
    linearVelocity = velCmd.linearVel;
    angularVelocity = velCmd.angularVel;
    
    // Debug: Check if speed is being received
    Serial.print("RX: L=");
    Serial.print(linearVelocity);
    Serial.print(" A=");
    Serial.print(angularVelocity);
    Serial.print(" LEN=");
    Serial.println(len);
    
    sendToWaveRover(linearVelocity, angularVelocity);
    
  } else {
    // 2. TEXTO DE RETORNO (O que vias no teu Script 2)
    char msg[len+1] = {0};
    memcpy(msg, (void*)incomingData, len);
    Serial.print("RX: Unknown len=");
    Serial.println(len);
  }
}

void sendToWaveRover(float linearVel, float angularVel) {
  // Formato: {"T":13,"X":linearVel,"Z":angularVel}
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", (double)linearVel, (double)angularVel);
  
  // Envia para o Wave Rover usando a Serial1
  Serial1.println(cmd);
  
  // Debug: Check if command is sent to robot
  Serial.print("ROBOT: ");
  Serial.println(cmd);
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
