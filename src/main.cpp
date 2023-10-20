/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include "AiEsp32RotaryEncoder.h"

// Defines for LED
#define PIN 25
#define PIXELCOUNT 8

#define ENCODER_CLK 26
#define ENCODER_DT 27
#define ENCODER_SW 14
#define ROTARY_ENCODER_STEPS 4

const int led = 2;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELCOUNT, PIN, NEO_RGB + NEO_KHZ800);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ENCODER_DT, ENCODER_CLK, ENCODER_SW, -1, ROTARY_ENCODER_STEPS);

bool occupied;
bool rotaryPressed = false;
bool debugMode = true;
int distancReceived = 0;

uint8_t receiverAddress[] = {0x08, 0xD1, 0xF9, 0xD3, 0x7A, 0xCC};
esp_now_peer_info_t peerInfo;

// Data for ESP-Now connection
typedef struct receivedMessage {
  bool statusWC;
  int distanzWC;
} receivedMessage;

typedef struct messageToBeSent {
  bool changeDistance;
  int setDistance;
} messageToBeSent;

messageToBeSent myMessageToBeSent;
receivedMessage myReceivedMessage;

void readValueOfRotary(){
  if(debugMode){Serial.println("Rotary Value");Serial.println(rotaryEncoder.readEncoder());}
  myMessageToBeSent.setDistance = rotaryEncoder.readEncoder();
  myMessageToBeSent.changeDistance = true;
  digitalWrite(led, HIGH);

  delay(100);
}

void messageReceived(const uint8_t* macAddr, const uint8_t* incomingData, int len){
memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));

  occupied = myReceivedMessage.statusWC;
  distancReceived = myReceivedMessage.distanzWC;
  if(debugMode){
    Serial.println("--------------------------------------------------");
    Serial.println("Message received");
    Serial.print("Status von ESP32\t");
    Serial.println(myReceivedMessage.statusWC);
    Serial.print("Distanz gemessen\t");
    Serial.println(myReceivedMessage.distanzWC);
    Serial.print("Eingestellter Wert\t");
    Serial.println(rotaryEncoder.readEncoder());
    }

  if (myReceivedMessage.statusWC){
    digitalWrite(led, HIGH);
  }else{
    digitalWrite(led, LOW);
  }

  esp_err_t result = esp_now_send(receiverAddress, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
    if (result != ESP_OK) {
        Serial.println("Sending error");
    }
}

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set Pin to Pin 2
  pinMode(led, OUTPUT);

  // Init for roatary encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);

  bool circleValues = false;

  rotaryEncoder.setBoundaries(0, 1000, circleValues);
  rotaryEncoder.disableAcceleration();


  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(messageReceived);
    
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  strip.begin();
  strip.setBrightness(100);
  strip.show();
}
 
void loop() {

  if(rotaryEncoder.isEncoderButtonClicked() && rotaryPressed == true){
   rotaryPressed = false;
   myMessageToBeSent.changeDistance = false;
   Serial.println("Modus Ausschalten");
   digitalWrite(led, LOW);
   delay(1000);
  }

  if(rotaryEncoder.isEncoderButtonClicked() && rotaryPressed == false){
    rotaryPressed = true;
    delay(1000);
    Serial.println("Distanz einstellen Mode");
  }

  if(rotaryPressed){
    readValueOfRotary();
    Serial.println(myMessageToBeSent.setDistance);
  }
  delay(100);

  if (occupied) 
  {
    int count = 0;
    do{
      strip.setPixelColor(count, 0,255,0);
      strip.show();
      count++;
    }while(count <= PIXELCOUNT);
  }else{
    int count = 0;
    do{
      strip.setPixelColor(count, 255,0,0);
      strip.show();
      count++;
    }while(count <= PIXELCOUNT);
  }

  /*if(debugMode){
    Serial.print("StatusWC");
    Serial.println(myReceivedMessage.statusWC);
    Serial.print("DistanzWC");
    Serial.println(distancReceived);
    Serial.print("Eingestellte Distanz");
    Serial.println(rotaryEncoder.readEncoder());
    Serial.print("rotaryPressed");
    Serial.println(rotaryEncoder.isEncoderButtonClicked());
  }*/

}