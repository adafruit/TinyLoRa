/*!
 * @file TinyLoRa.h
 *
 * This is part of Adafruit's TinyLoRa library for the Arduino platform. It is
 * designed specifically to work with the Adafruit Feather 32u4 RFM95 LoRa:
 * https://www.adafruit.com/product/3078
 *
 * This library uses SPI to communicate, 4 pins (SCL, SDA, IRQ, SS)
 * are required to interface with the HopeRF RFM95/96 breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright 2015, 2016 Ideetron B.V.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified by Brent Rubell for Adafruit Industries.
 *
 * LGPL license, all text here must be included in any redistribution.
 *
 */

#ifndef TINY_LORA_H
#define TINY_LORA_H

#include <Arduino.h>
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#elif defined(ARDUINO_ARCH_ESP32)
  #include <pgmspace.h>
#endif

// uncomment for debug output
// #define DEBUG

/** RFM channel options */
typedef enum rfm_channels
{
  CH0,
  CH1,
  CH2,
  CH3,
  CH4,
  CH5,
  CH6,
  CH7,
  MULTI,
} rfm_channels_t;

/** RFM fixed datarate, dependent on region */
typedef enum rfm_datarates
{
  SF7BW125,
  SF7BW250,
  SF8BW125,
  SF9BW125,
  SF10BW125,
  SF11BW125,
  SF12BW125,
} rfm_datarates_t;

/** Region configuration*/
#if !defined(EU863) && !defined(AU915) && !defined(AS920)  
  #define US902 ///< Used in USA, Canada and South America
#endif
//#define EU863 ///< Used in Europe
//#define AU915 ///< Used in Australia
//#define AS920 ///< Used in Asia

#define RFM9x_VER   0x12 ///<Expected RFM9x RegVersion

/* RFM Modes */
#define MODE_SLEEP  0x00  ///<low-power mode
#define MODE_LORA   0x80  ///<LoRa operating mode
#define MODE_STDBY  0x01  ///<Osc. and baseband disabled
#define MODE_TX     0x83  ///<Configures and transmits packet

/* RFM Registers */
#define REG_PA_CONFIG              0x09 ///<PA selection and Output Power control
#define REG_PA_DAC                 0x4D ///<PA Higher Power Settings
#define REG_PREAMBLE_MSB           0x20 ///<Preamble Length, MSB
#define REG_PREAMBLE_LSB           0x21 ///<Preamble Length, LSB
#define REG_FRF_MSB                0x06 ///<RF Carrier Frequency MSB
#define REG_FRF_MID                0x07 ///<RF Carrier Frequency Intermediate
#define REG_FRF_LSB                0x08 ///<RF Carrier Frequency LSB
#define REG_FEI_LSB                0x1E ///<Info from Prev. Header
#define REG_FEI_MSB                0x1D ///<Number of received bytes
#define REG_MODEM_CONFIG           0x26 ///<Modem configuration register
#define REG_VER                    0x42 ///<RFM9x version register

/**************************************************************************/
/*! 
    @brief  TinyLoRa Class
*/
/**************************************************************************/
class TinyLoRa
{
	public:
		uint8_t txrandomNum;  ///<random number for AES
		uint16_t frameCounter;  ///<frame counter
		void setChannel(rfm_channels_t channel);
		void setDatarate(rfm_datarates_t datarate);
		void setPower(int8_t Tx_Power = 17);
		TinyLoRa(int8_t rfm_dio0, int8_t rfm_nss, int8_t rfm_rst);
		bool begin(void);
		void sendData(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter_Tx, uint8_t Frame_Port = 1);

	private:
		uint8_t randomNum;
		int8_t _cs, _irq, _rst;
		bool _isMultiChan;
		unsigned char _rfmMSB, _rfmMID, _rfmLSB, _sf, _bw, _modemcfg;
		static const unsigned char LoRa_Frequency[8][3];
		static const unsigned char S_Table[16][16];
		void RFM_Send_Package(unsigned char *RFM_Tx_Package, unsigned char Package_Length);
		void RFM_Write(unsigned char RFM_Address, unsigned char RFM_Data);
		uint8_t RFM_Read(uint8_t RFM_Address);
		void Encrypt_Payload(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
		void Calculate_MIC(unsigned char *Data, unsigned char *Final_MIC, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction);
		void Generate_Keys(unsigned char *K1, unsigned char *K2);
		void Shift_Left(unsigned char *Data);
		void XOR(unsigned char *New_Data, unsigned char *Old_Data);
		void AES_Encrypt(unsigned char *Data, unsigned char *Key);
		void AES_Add_Round_Key(unsigned char *Round_Key, unsigned char(*State)[4]);
		unsigned char AES_Sub_Byte(unsigned char Byte);
		void AES_Shift_Rows(unsigned char(*State)[4]);
		void AES_Mix_Collums(unsigned char(*State)[4]);
		void AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key);

};

/*
*****************************************************************************************
* Description: TTN regional frequency plans
*****************************************************************************************
*/

#if defined(NOFREQ)
// We're being included from TinyLora.cpp - don't define frequencies, to allow the sketch to define them
#elif defined(AU915)
#pragma message "Using AU915 frequencies"
const unsigned char PROGMEM TinyLoRa::LoRa_Frequency[8][3] = {
	{ 0xE5, 0x33, 0x5A },	//Channel 0 916.800 MHz / 61.035 Hz = 15020890 = 0xE5335A
	{ 0xE5, 0x40, 0x26 },	//Channel 2 917.000 MHz / 61.035 Hz = 15024166 = 0xE54026
	{ 0xE5, 0x4C, 0xF3 },	//Channel 3 917.200 MHz / 61.035 Hz = 15027443 = 0xE54CF3
	{ 0xE5, 0x59, 0xC0 },	//Channel 4 917.400 MHz / 61.035 Hz = 15030720 = 0xE559C0
	{ 0xE5, 0x66, 0x8D },	//Channel 5 917.600 MHz / 61.035 Hz = 15033997 = 0xE5668D
	{ 0xE5, 0x73, 0x5A },	//Channel 6 917.800 MHz / 61.035 Hz = 15037274 = 0xE5735A
	{ 0xE5, 0x80, 0x27 },	//Channel 7 918.000 MHz / 61.035 Hz = 15040551 = 0xE58027
	{ 0xE5, 0x8C, 0xF3 }	//Channel 8 918.200 MHz / 61.035 Hz = 15043827 = 0xE58CF3
};
#elif defined(EU863)
#pragma message "Using EU863 frequencies"
const unsigned char PROGMEM TinyLoRa::LoRa_Frequency[8][3] = {
	{ 0xD9, 0x06, 0x8B },	//Channel 0 868.100 MHz / 61.035 Hz = 14222987 = 0xD9068B
	{ 0xD9, 0x13, 0x58 },	//Channel 1 868.300 MHz / 61.035 Hz = 14226264 = 0xD91358
	{ 0xD9, 0x20, 0x24 },	//Channel 2 868.500 MHz / 61.035 Hz = 14229540 = 0xD92024
	{ 0xD8, 0xC6, 0x8B },	//Channel 3 867.100 MHz / 61.035 Hz = 14206603 = 0xD8C68B
	{ 0xD8, 0xD3, 0x58 },	//Channel 4 867.300 MHz / 61.035 Hz = 14209880 = 0xD8D358
	{ 0xD8, 0xE0, 0x24 },	//Channel 5 867.500 MHz / 61.035 Hz = 14213156 = 0xD8E024
	{ 0xD8, 0xEC, 0xF1 },	//Channel 6 867.700 MHz / 61.035 Hz = 14216433 = 0xD8ECF1
	{ 0xD8, 0xF9, 0xBE }	//Channel 7 867.900 MHz / 61.035 Hz = 14219710 = 0xD8F9BE

};
#elif defined(US902)
#pragma message "Using US902 frequencies"
const unsigned char PROGMEM TinyLoRa::LoRa_Frequency[8][3] = {
	{ 0xE1, 0xF9, 0xC0 },		//Channel 0 903.900 MHz / 61.035 Hz = 14809536 = 0xE1F9C0
	{ 0xE2, 0x06, 0x8C },		//Channel 1 904.100 MHz / 61.035 Hz = 14812812 = 0xE2068C
	{ 0xE2, 0x13, 0x59 },		//Channel 2 904.300 MHz / 61.035 Hz = 14816089 = 0xE21359
	{ 0xE2, 0x20, 0x26 },		//Channel 3 904.500 MHz / 61.035 Hz = 14819366 = 0xE22026
	{ 0xE2, 0x2C, 0xF3 },		//Channel 4 904.700 MHz / 61.035 Hz = 14822643 = 0xE22CF3
	{ 0xE2, 0x39, 0xC0 },		//Channel 5 904.900 MHz / 61.035 Hz = 14825920 = 0xE239C0
	{ 0xE2, 0x46, 0x8C },		//Channel 6 905.100 MHz / 61.035 Hz = 14829196 = 0xE2468C
	{ 0xE2, 0x53, 0x59 }		//Channel 7 905.300 MHz / 61.035 Hz = 14832473 = 0xE25359
};
#elif defined(AS920)
#pragma message "Using AS920 frequencies"
const unsigned char PROGMEM TinyLoRa::LoRa_Frequency[8][3] = {
	{ 0xE6, 0xCC, 0xF4 },		//Channel 0 868.100 MHz / 61.035 Hz = 15125748 = 0xE6CCF4
	{ 0xE6, 0xD9, 0xC0 },		//Channel 1 868.300 MHz / 61.035 Hz = 15129024 = 0xE6D9C0
	{ 0xE6, 0x8C, 0xF3 },		//Channel 2 868.500 MHz / 61.035 Hz = 15109363 = 0xE68CF3
	{ 0xE6, 0x99, 0xC0 },		//Channel 3 867.100 MHz / 61.035 Hz = 15112640 = 0xE699C0
	{ 0xE6, 0xA6, 0x8D },		//Channel 4 867.300 MHz / 61.035 Hz = 15115917 = 0xE6A68D
	{ 0xE6, 0xB3, 0x5A },		//Channel 5 867.500 MHz / 61.035 Hz = 15119194 = 0xE6B35A
	{ 0xE6, 0xC0, 0x27 },		//Channel 6 867.700 MHz / 61.035 Hz = 15122471 = 0xE6C027
	{ 0xE6, 0x80, 0x27 }		//Channel 7 867.900 MHz / 61.035 Hz = 15106087 = 0xE68027
};
#else
#error No Frequencies defined!
#endif

#endif
