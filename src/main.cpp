
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
   Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  -
  - ivm problemen met data transfer (espnow) van Esp8266_EspNow_Tx naar 
    Esp32_espnow_Rx heb ik (van Scratch) een andere Transmittor gebouwd 
    (ESP32_BME280_Server). Documentatie: in de /Lib/Docs map van de respectievelijke programmaś 
  - Laatste wijziging: 11 jan 2023 

*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address  (Esp32_espnow_Rx)
uint8_t broadcastAddress[] = {0xc0, 0x49, 0xef, 0xcd, 0x0b, 0x9c};

typedef struct struct_message {
  char a[32];
  float t;  // temperatuur
  float p;  // luchtdruk
  float h;  // vochtigheid
  int l;  // LDR 
  int b;    // bewegingsdetetie
  int r;    // voor inschakelen relais door Receiver (Rx)
} struct_message;

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

// Create a struct_message called myData
//struct_message myData;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message BME280Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;


esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Espnow  callback function when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received from ESP32_espnow_Rx: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(incomingReadings.a);
  Serial.print("T ");
  Serial.println(incomingReadings.t);
  Serial.print("P: ");
  Serial.println(incomingReadings.p);
  Serial.print("H: ");
  Serial.println(incomingReadings.h);
  Serial.print("LDR: ");
  Serial.println(incomingReadings.l);
  Serial.print("Beweging detectie (0,1): ");
  Serial.println(incomingReadings.b);
  Serial.print("Relais Rood: ");  
  Serial.println(incomingReadings.r);
  Serial.println();
}

/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

#define timeSeconds 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

int pinLedGroen = 18;
int pinLedRood = 16;
const int analogInPin = 32;  // ESP32 pin 32 = Touch 9 / ADC 4

// beweging vastleggen
const int motionSensor = 19; // PIR Motion Sensor (D5)
bool motionDetected = false;
int beweging =0;  //gekoppeld aan motiondetection

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  digitalWrite(pinLedGroen, HIGH);
  beweging = 1; 
  startTimer = true;
  lastTrigger = millis();
}


void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.print("Lichtsterkte = ");
  Serial.print(analogRead(analogInPin));
  Serial.println("lux");

    
  if(incomingReadings.r==1){
    digitalWrite(pinLedRood,HIGH);
  } else {digitalWrite(pinLedRood,LOW);}

}

void setup() {
  Serial.begin(115200);
  Serial.println(F("BME280 test"));

  bool status;

 // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT_PULLUP);

 // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
 attachInterrupt(digitalPinToInterrupt(motionSensor), detectsMovement, RISING);

 pinMode(pinLedGroen,OUTPUT);
 digitalWrite(pinLedGroen,LOW);
 pinMode(pinLedRood,OUTPUT);
 digitalWrite(pinLedRood,LOW);
  
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}


void loop() { 
   

if ((millis() - lastTime) > timerDelay) {
 lastTime = millis();
// Set values to send
    //strcpy(myData.a, "THIS IS een Test");
    BME280Readings.p = bme.readPressure()/100.0F;
    BME280Readings.t = bme.readTemperature();
    BME280Readings.h = bme.readHumidity();
    BME280Readings.l = analogRead(analogInPin);
    BME280Readings.b = beweging;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  printValues();
}
now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
    digitalWrite(pinLedGroen, LOW);
    beweging = 0;
    startTimer = false;
  }
}


