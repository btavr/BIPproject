#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#include <micro_ros_arduino.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float64.h>

#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>

#define RXD2 16 // Conectar ao TX do Wave Rover
#define TXD2 17 // Conectar ao RX do Wave Rover

rcl_node_t node;
rcl_publisher_t publisher;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Float64 msg;

rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_subscription_t speed;

void setup() {
  // Inicializa a comunicação serial com o monitor serial (PC)
  Serial.begin(115200);
 
  // Inicializa a Serial2 para comunicação com o Wave Rover
  // Configuração: 115200 baud, 8 bits de dados, sem paridade, 1 stop bit
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  Serial.println("Moving slow...");
  delay(2000);

  // Create subscriber
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "led_control"
  );

    // Create subscriber
  rclc_subscription_init_default(
    &speed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "getSpeed"
  );
}



void loop() {
  // Enviar comando JSON para ir para frente (velocidade 80)
  // O formato esperado pelo Wave Rover geralmente é {"T":1,"L":80,"R":80}
  //Serial2.println("{\"T\":1,\"L\":0.1,\"R\":0.1}");
  Serial.println("moving: front slow");

  delay(300);

  // Comando para parar
  // O formato enviado é {"T":1,"L":0,"R":0}
  //Serial2.println("{\"T\":1,\"L\":0,\"R\":0}");
  //Serial.println("stop");

  delay(500);
}