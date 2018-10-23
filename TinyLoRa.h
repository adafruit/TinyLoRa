#ifndef TINY_LORA_H
#define TINY_LORA_H

#include <Arduino.h>
#include <avr/pgmspace.h>

// debugging, unset if you don't need this.
#define DEBUG

// Multi-Channel Package Sending (default)
//#define MULTICH
#define SGLCH

/**************************************************************************/
/*! 
    @brief  RFM Channel List
*/
/**************************************************************************/
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
  MUTLI,
} rfm_channels_t;

/* TTN Configuration */
// Set TTN frequecy plan EU863, AU915, AS920, US902
#define US902
//#define EU863
//#define AU915
//#define AS920
// Define fixed datarate
#define SF7BW125
//#define SF12BW125
//#define SF11BW125
//#define SF10BW125
//#define SF9BW125
//#define SF8BW125
//#define SF7BW250

/* RFM Modes */
#define MODE_SLEEP  0x00
#define MODE_LORA   0x80
#define MODE_STDBY  0x01
#define MODE_TX     0x83

/* RFM Registers */
#define REG_PA_CONFIG            0x09
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define RegFrfMsb                0x06
#define RegFrfMid                0x07
#define RegFrfLsb                0x08

/* TinyLoRa Class */
class TinyLoRa
{
	public:
		uint8_t txrandomNum;
		uint16_t frameCounter;
    void setChannel(rfm_channels_t channel);
    TinyLoRa(int8_t rfm_dio0, int8_t rfm_nss);
		void begin(void);
		void sendData(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter_Tx);

	private:
		uint8_t randomNum;
		int8_t _cs, _irq;
    unsigned char _rfmMSB, _rfmMID, _rfmLSB;
    static const unsigned char LoRa_Frequency[8][3];
		static const unsigned char S_Table[16][16];
		void RFM_Send_Package(unsigned char *RFM_Tx_Package, unsigned char Package_Length);
		void RFM_Write(unsigned char RFM_Address, unsigned char RFM_Data);
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

#endif
