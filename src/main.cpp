#include <Arduino.h>

// Complete Instructions to Get and Change ESP MAC Address: https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/

#include "WiFi.h"
#include <esp_now.h>
#include <esp_wifi.h>
 
uint8_t broadcastAddress[] = {0x10, 0x97, 0xBD, 0xB7, 0x4F, 0x90}; // Endereço da PCB
// uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xD1, 0x63, 0x50}; // Endereço do devkit

esp_now_peer_info_t peerInfo;
String success;

#define WIFI_CHANNEL 6

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);

void setup(){
  Serial.begin(115200);
  // WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  
  
  
  WiFi.mode( WIFI_STA );
  WiFi.disconnect();
  int chan = WIFI_CHANNEL;
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan,WIFI_SECOND_CHAN_NONE));
  int a = esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR );
  if (esp_now_init() != ESP_OK) { ESP.restart(); return; }
  peerInfo.channel = chan;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) == ESP_OK){Serial.printf("# Peer Added\r\n");}
  else {Serial.printf("# Unable to add peer \r\n");}

  // int a = esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_LR );
  
  // // Init ESP-NOW
  // if (esp_now_init() != ESP_OK) 
  // {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 6;  
  // peerInfo.encrypt = false;
  
  // Add peer        
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  
  pinMode(2, OUTPUT);
}
 
void loop(){
  String teste = "olá!";
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &teste, sizeof(teste));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(200);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  digitalWrite(2, HIGH);
  delay(200);
  digitalWrite(2, LOW);
}
