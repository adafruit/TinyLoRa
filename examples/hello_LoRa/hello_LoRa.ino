// Hello LoRa - ABP TTN Packet Sender (Multi-Channel)
// Tutorial Link: https://learn.adafruit.com/the-things-network-for-feather/using-a-feather-32u4
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Written by Brent Rubell for Adafruit Industries
// Copyright 2018 Adafruit Industries
// Copyright 2015, 2016 Ideetron B.V.
/************************** Configuration ***********************************/
#include <TinyLoRa.h>
#include <SPI.h>

// Visit your thethingsnetwork.org device console
// to create an account, or if you need your session keys.

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x9B, 0x27, 0x5D, 0xF6, 0x65, 0xB8, 0x1A, 0xD7, 0x95, 0xF1, 0x97, 0xA1, 0x73, 0xCB, 0xA4, 0xFC };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x84, 0x9C, 0x26, 0x0E, 0xC3, 0x4A, 0x40, 0x98, 0x3B, 0x7E, 0x4C, 0x4F, 0x03, 0x96, 0xB6, 0xD0 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x26, 0x02, 0x14, 0x58 };

/************************** Example Begins Here ***********************************/
// Data Packet to Send to TTN
unsigned char loraData[11] = {"hello LoRa"};

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather 32u4 LoRa
TinyLoRa lora = TinyLoRa(7, 8);

// Pinout for Adafruit Feather M0 LoRa
//TinyLoRa lora = TinyLoRa(3, 8);

void setup()
{
  delay(2000);
  while (! Serial);
  Serial.begin(9600);
 
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LoRa
  Serial.println("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  lora.begin();
}

void loop()
{
  Serial.println("Sending LoRa Data...");
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("delaying...");
  delay(sendInterval * 1000);
}
