// Hello LoRa - ABP TTN Packet Sender (Multi-Channel)
// Tutorial Link: https://learn.adafruit.com/the-things-network-for-feather/using-a-feather-32u4
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Copyright 2015, 2016 Ideetron B.V.
//
// Modified by Brent Rubell for Adafruit Industries, 2018
/************************** Configuration ***********************************/
#include <TinyLoRa.h>
#include <SPI.h>

// Visit your thethingsnetwork.org device console
// to create an account, or if you need your session keys.

// Network Session Key (MSB)
uint8_t NwkSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Application Session Key (MSB)
uint8_t AppSkey[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Device Address (MSB)
uint8_t DevAddr[4] = { 0x00, 0x00, 0x00, 0x00 };

/************************** Example Begins Here ***********************************/
// Start TxPower settings //
uint8_t const TxPower = 0xFF; // default value. Don't modify this value [default: 0xFF]

// uncomment this section if you want to control the TxPower, and comment the above line.
/*
bool PaBoost = 1; // 8th bit [default: 1]
uint8_t OutputPower = 1; // 1-4 bit (DEC values: 0-15) [default: 15] +2 to +17dBm.
uint8_t MaxPower = 4; // 5-7 bit, (DEC values: 0-7) [default: 4] BUG? Range -4 to 0 but DOC says -4 to +15dBm

// function to pack the data for TXpower.
uint8_t packDataPower(){
  // According to HOPE RFM9x documentation (p. 80 section 5.4.3), if PA_LF or PA_HF is used instead of PaBoost, output power must be disabled!
  if ( PaBoost == 0 && OutputPower > 0 ) {
     OutputPower = 0; // make sure OutputPower is disabled if PA_LF or PA_HF is used.
    }

  uint8_t DataPower = (PaBoost << 7) + (MaxPower << 4) + OutputPower;
  return DataPower;
}

// necessary for the initial setup.
uint8_t TxPower = packDataPower();
*/

// Port number.
uint8_t FramePort = 1;

// Data Packet to Send to TTN
unsigned char loraData[11] = {"hello LoRa"};

// How many times data transfer should occur, in seconds
const unsigned int sendInterval = 30;

// Pinout for Adafruit Feather 32u4 LoRa
TinyLoRa lora = TinyLoRa(7, 8, 4);

// Pinout for Adafruit Feather M0 LoRa
//TinyLoRa lora = TinyLoRa(3, 8, 4);

void setup()
{
  delay(2000);
  Serial.begin(9600);
  while (! Serial);
  
  // Initialize pin LED_BUILTIN as an output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize LoRa
  Serial.print("Starting LoRa...");
  // define multi-channel sending
  lora.setChannel(MULTI);
  // set datarate
  lora.setDatarate(SF7BW125);
  
  if(!lora.begin())
  {
    Serial.println("Failed");
    Serial.println("Check your radio");
    while(true);
  }
  
  // ATTN: setPower *after* lora.begin or feather 32u4 hangs, untested with M0.
  lora.setPower(TxPower);
  
  Serial.println("OK");
}

void loop()
{
  Serial.println("Sending LoRa Data...");
  lora.sendData(loraData, sizeof(loraData), lora.frameCounter, FramePort);
  Serial.print("Frame Counter: ");Serial.println(lora.frameCounter);
  lora.frameCounter++;

  // blink LED to indicate packet sent
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  Serial.println("delaying...");
  delay(sendInterval * 1000);
}
