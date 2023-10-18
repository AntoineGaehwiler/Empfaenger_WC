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

// Defines for LED
#define PIN 25
#define PIXELCOUNT 8
const int led = 2;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXELCOUNT, PIN, NEO_RGB + NEO_KHZ800);

bool occupied;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int a;
    bool b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  Serial.print("Bool: ");
  Serial.println(myData.b);
  occupied = myData.b;

  if (myData.b)
  {
    digitalWrite(led, HIGH);
  }else
  {
    digitalWrite(led, LOW);
  }
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set Pin to Pin 2
  pinMode(led, OUTPUT);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  strip.begin();
  strip.setBrightness(100);
  strip.show();


}
 
void loop() {
    if (myData.b) 
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
}