/*!
 * @file TinyLoRa.cpp
 *
 * @mainpage TinyLoRa RFM95/96W breakout driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's Feather LoRa for the
 * Arduino platform. It is designed specifically to work with the
 * Adafruit Feather 32u4 LoRa: 
 * This is the documentation for Adafruit's FXOS8700 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXOS8700 breakout: https://www.adafruit.com/product/3078
 *
 * This library uses SPI to communicate, 4 pins (SCL, SDA, IRQ, SS)
 * are required to interface with the HopeRF RFM95/96 breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library has no dependencies.
 *
 * @section author Author
 *
 * Written by Ideetron B.V., modified by Brent Rubell for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "TinyLoRa.h"
#include <SPI.h>

extern uint8_t NwkSkey[16]; ///< Network Session Key
extern uint8_t AppSkey[16]; ///< Application Session Key
extern uint8_t DevAddr[4]; ///< Device Address

static SPISettings RFM_spisettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

/*
*****************************************************************************************
* Description: TTN regional frequency plans
*****************************************************************************************
*/

#ifdef AU915
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
#endif

#ifdef EU863
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
#endif

#ifdef US902
const unsigned char PROGMEM TinyLoRa::LoRa_Frequency[8][3] = {
	{ 0xE1, 0xF9, 0xC0 },		//Channel 0 903.900 MHz / 61.035 Hz = 14809536 = 0xE1F9C0
	{ 0xE2, 0x06, 0x8C },		//Channel 1 904.100 MHz / 61.035 Hz = 14812812 = 0xE2068C
	{ 0xE2, 0x13, 0x59},		//Channel 2 904.300 MHz / 61.035 Hz = 14816089 = 0xE21359
	{ 0xE2, 0x20, 0x26 },		//Channel 3 904.500 MHz / 61.035 Hz = 14819366 = 0xE22026
	{ 0xE2, 0x2C, 0xF3 },		//Channel 4 904.700 MHz / 61.035 Hz = 14822643 = 0xE22CF3
	{ 0xE2, 0x39, 0xC0 },		//Channel 5 904.900 MHz / 61.035 Hz = 14825920 = 0xE239C0
	{ 0xE2, 0x46, 0x8C },		//Channel 6 905.100 MHz / 61.035 Hz = 14829196 = 0xE2468C
	{ 0xE2, 0x53, 0x59 }		//Channel 7 905.300 MHz / 61.035 Hz = 14832473 = 0xE25359
};
#endif

#ifdef AS920
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
#endif


/*
*****************************************************************************************
* Description: S_Table used for AES encription
*****************************************************************************************
*/

const unsigned char PROGMEM TinyLoRa::S_Table[16][16] = {
	  {0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76},
	  {0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0},
	  {0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15},
	  {0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75},
	  {0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84},
	  {0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF},
	  {0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8},
	  {0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2},
	  {0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73},
	  {0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB},
	  {0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79},
	  {0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08},
	  {0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A},
	  {0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E},
	  {0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF},
	  {0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16}
	};

/***************************************************************************
 CONSTRUCTORS
 ***************************************************************************/

/**************************************************************************/
/*! 
    @brief Sets the RFM datarate
    @param datarate Bandwidth and Frequency plan.
*/
/**************************************************************************/
void TinyLoRa::setDatarate(rfm_datarates_t datarate) {
  _sf, _bw, _modemcfg = 0;
  switch(datarate) {
    case SF7BW125:
      _sf = 0x74;
      _bw = 0x72;
      _modemcfg = 0x04;
      break;
    case SF7BW250:
      _sf = 0x74;
      _bw = 0x82;
      _modemcfg = 0x04;
      break;
    case SF8BW125:
      _sf = 0x84;
      _bw = 0x72;
      _modemcfg = 0x04;
      break;
    case SF9BW125:
      _sf = 0x94;
      _bw = 0x72;
      _modemcfg = 0x04;
      break;
    case SF10BW125:
      _sf = 0xA4;
      _bw = 0x72;
      _modemcfg = 0x04;
      break;
    case SF11BW125:
      _sf = 0xB4;
      _bw = 0x72;
      _modemcfg = 0x0C;
      break;
    case SF12BW125:
      _sf = 0xC4;
      _bw = 0x72;
      _modemcfg = 0x0C;
      break;
    default:
      _sf = 0x74;
      _bw = 0x72;
      _modemcfg = 0x04;
      break;
  }
}

/**************************************************************************/
/*! 
    @brief Sets the RFM channel.
    @param channel Which channel to send data
*/
/**************************************************************************/
void TinyLoRa::setChannel(rfm_channels_t channel) {
  _rfmMSB, _rfmLSB, _rfmMID = 0;
  switch (channel)
  {
    case CH0:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[0][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[0][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[0][0]));
      _isMultiChan = 0;
      break;
    case CH1:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[1][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[1][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[1][0]));
      _isMultiChan = 0;
      break;
    case CH2:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[2][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[2][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[2][0]));
      _isMultiChan = 0;
      break;
    case CH3:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[3][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[3][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[3][0]));
      _isMultiChan = 0;
      break;
    case CH4:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[4][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[4][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[4][0]));
      _isMultiChan = 0;
      break;
    case CH5:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[5][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[5][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[5][0]));
      _isMultiChan = 0;
      break;
    case CH6:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[6][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[6][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[6][0]));
      _isMultiChan = 0;
      break;
    case CH7:
      _rfmLSB = pgm_read_byte(&(LoRa_Frequency[7][2]));
      _rfmMID = pgm_read_byte(&(LoRa_Frequency[7][1]));
      _rfmMSB = pgm_read_byte(&(LoRa_Frequency[7][0]));
      _isMultiChan = 0;
      break;
    case MULTI:
      _isMultiChan = 1;
      break;
    default:
      _isMultiChan = 1;
      break;
  }
}

/**************************************************************************/
/*!
    @brief  Instanciates a new TinyLoRa class, including assigning
            irq and cs pins to the RFM breakout.
    @param    rfm_irq
              The RFM module's interrupt pin (rfm_nss).
    @param    rfm_nss
              The RFM module's slave select pin (rfm_nss).
*/
/**************************************************************************/
TinyLoRa::TinyLoRa(int8_t rfm_irq, int8_t rfm_nss) {
  _irq = rfm_irq;
  _cs = rfm_nss;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

 /**************************************************************************/
 /*!
     @brief  Initializes the RFM, including configuring SPI, configuring
             the frameCounter and txrandomNum. 
 */
 /**************************************************************************/
void TinyLoRa::begin() 
{

  // start and configure SPI
  SPI.begin();
  
  // RFM95 ss as output
  pinMode(_cs, OUTPUT);

  // RFM95 _irq as input
  pinMode(_irq, OUTPUT);

  //Switch RFM to sleep
  RFM_Write(0x01,MODE_SLEEP);

  //Set RFM in LoRa mode
  RFM_Write(0x01,MODE_LORA);

  //PA pin (maximal power)
  RFM_Write(0x09,0xFF);

  //Rx Timeout set to 37 symbols
  RFM_Write(0x1F,0x25);

  //Preamble length set to 8 symbols
  //0x0008 + 4 = 12
  RFM_Write(REG_PREAMBLE_MSB,0x00);
  RFM_Write(REG_PREAMBLE_LSB,0x08);

  //Low datarate optimization off AGC auto on
  RFM_Write(0x26,0x0C);

  //Set LoRa sync word
  RFM_Write(0x39,0x34);

  //Set IQ to normal values
  RFM_Write(0x33,0x27);
  RFM_Write(0x3B,0x1D);

  //Set FIFO pointers
  //TX base adress
  RFM_Write(0x0E,0x80);
  //Rx base adress
  RFM_Write(0x0F,0x00);

  // init frame counter
  uint16_t frameCounter = 0x0000;

  // init tx random number for first use
  uint8_t txrandomNum = 0x00;
  #ifdef DEBUG
    Serial.println("> RFM module initialized"); 
  #endif

}

/**************************************************************************/
/*!
    @brief  Sends a package with the RFM module.
    @param    *RFM_Tx_Package
              Pointer to array containing data to be sent.
    @param    Package_Length
              Length of the package to be sent.
*/
/**************************************************************************/
void TinyLoRa::RFM_Send_Package(unsigned char *RFM_Tx_Package, unsigned char Package_Length)
{
  unsigned char i;

  //Set RFM in Standby mode wait on mode ready
  RFM_Write(MODE_STDBY,0x81);
  
  // wait for standby mode
  delay(10);
  
  //Switch _irq to TxDone
  RFM_Write(0x40,0x40);

  // select rfm channel
  if (_isMultiChan == 1) {
    RFM_Write(REG_FRF_MSB, pgm_read_byte(&(LoRa_Frequency[randomNum][0])));
    RFM_Write(REG_FRF_MID, pgm_read_byte(&(LoRa_Frequency[randomNum][1])));
    RFM_Write(REG_FRF_LSB, pgm_read_byte(&(LoRa_Frequency[randomNum][2])));
  } else {
    RFM_Write(REG_FRF_MSB, _rfmMSB);
    RFM_Write(REG_FRF_MID, _rfmMID);
    RFM_Write(REG_FRF_LSB, _rfmLSB);
  }

  /* Set RFM Datarate */
  RFM_Write(REG_FEI_LSB, _sf);
  RFM_Write(REG_FEI_MSB, _bw);
  RFM_Write(REG_MODEM_CONFIG, _modemcfg);

  //Set payload length to the right length
  RFM_Write(0x22,Package_Length);

  //Set SPI pointer to start of Tx part in FiFo
  RFM_Write(0x0D,0x80);

  //Write Payload to FiFo
  for (i = 0;i < Package_Length; i++)
  {
    RFM_Write(0x00,*RFM_Tx_Package);
    RFM_Tx_Package++;
  }
  //Switch RFM to Tx
  RFM_Write(0x01,MODE_TX);

  //Wait _irq to pull high
  while(digitalRead(_irq) == LOW)
  {
    
  }
  //Switch RFM to sleep
  RFM_Write(0x01,MODE_SLEEP);
}

/**************************************************************************/
/*!
    @brief    Function which writes to a register from the RFM.
    @param    RFM_Address
              An address of the register to be written.
    @param    RFM_Data
              Data to be written to the register.
*/
/**************************************************************************/
void TinyLoRa::RFM_Write(unsigned char RFM_Address, unsigned char RFM_Data) 
{
  // br: SPI Transfer Debug
  #ifdef DEBUG
    Serial.print("SPI Write ADDR: ");
    Serial.print(RFM_Address, HEX);
    Serial.print(" DATA: ");
    Serial.println(RFM_Data, HEX);
  #endif

  SPI.beginTransaction(RFM_spisettings);

  //Set NSS pin Low to start communication
  digitalWrite(_cs, LOW);

  //Send Address with MSB 1 to make it a writ command
  SPI.transfer(RFM_Address | 0x80);
  //Send Data
  SPI.transfer(RFM_Data);

  //Set NSS pin High to end communication
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    @brief    Function to assemble and send a LoRaWAN package.
    @param    *Data
              Pointer to the array of data to be transmitted.
    @param    Frame_Counter_Tx
              Frame counter for transfer frames.
    @param    Data_Length
              Length of data to be sent.
*/
/**************************************************************************/
void TinyLoRa::sendData(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter_Tx)
{
  
  //Define variables
  unsigned char i;

  //Direction of frame is up
  unsigned char Direction = 0x00;

  unsigned char RFM_Data[64];
  unsigned char RFM_Package_Length;

  unsigned char MIC[4];

  //Unconfirmed data up
  unsigned char Mac_Header = 0x40;

  unsigned char Frame_Control = 0x00;
  unsigned char Frame_Port = 0x01;

  //Encrypt the data
  Encrypt_Payload(Data, Data_Length, Frame_Counter_Tx, Direction);

  
  //Build the Radio Package
  RFM_Data[0] = Mac_Header;
  RFM_Data[1] = DevAddr[3];
  RFM_Data[2] = DevAddr[2];
  RFM_Data[3] = DevAddr[1];
  RFM_Data[4] = DevAddr[0];
  RFM_Data[5] = Frame_Control;
  RFM_Data[6] = (Frame_Counter_Tx & 0x00FF);
  RFM_Data[7] = ((Frame_Counter_Tx >> 8) & 0x00FF);
  RFM_Data[8] = Frame_Port;

  //Set Current package length
  RFM_Package_Length = 9;

  //Load Data
  for(i = 0; i < Data_Length; i++)
  {
    RFM_Data[RFM_Package_Length + i] = Data[i];
  }

  //Add data Lenth to package length
  RFM_Package_Length = RFM_Package_Length + Data_Length;
  #ifdef DEBUG
    Serial.print("Package length: ");
    Serial.println(RFM_Package_Length);
  #endif
  
  //Calculate MIC
  Calculate_MIC(RFM_Data, MIC, RFM_Package_Length, Frame_Counter_Tx, Direction);

  //Load MIC in package
  for(i = 0; i < 4; i++)
  {
    RFM_Data[i + RFM_Package_Length] = MIC[i];
  }

  //Add MIC length to RFM package length
  RFM_Package_Length = RFM_Package_Length + 4;
 
  //Send Package
  RFM_Send_Package(RFM_Data, RFM_Package_Length);
  #ifdef DEBUG
    Serial.println("sent package!");
  #endif
}

/**************************************************************************/
/*!
    @brief    Function used to encrypt and decrypt the data in a LoRaWAN
              data packet.
    @param    *Data
              Pointer to the data to decrypt or encrypt.
    @param    Data_Length
              Number of bytes to be transmitted.
    @param    Frame_Counter
              Counts upstream frames.
    @param    Direction
              Direction of message (is up).
*/
/**************************************************************************/
void TinyLoRa::Encrypt_Payload(unsigned char *Data, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction)
{
  unsigned char i = 0x00;
  unsigned char j;
  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;

  unsigned char Block_A[16];

  //Calculate number of blocks
  Number_of_Blocks = Data_Length / 16;
  Incomplete_Block_Size = Data_Length % 16;
  if(Incomplete_Block_Size != 0)
  {
    Number_of_Blocks++;
  }

  for(i = 1; i <= Number_of_Blocks; i++)
  {
    Block_A[0] = 0x01;
    Block_A[1] = 0x00;
    Block_A[2] = 0x00;
    Block_A[3] = 0x00;
    Block_A[4] = 0x00;

    Block_A[5] = Direction;

    Block_A[6] = DevAddr[3];
    Block_A[7] = DevAddr[2];
    Block_A[8] = DevAddr[1];
    Block_A[9] = DevAddr[0];

    Block_A[10] = (Frame_Counter & 0x00FF);
    Block_A[11] = ((Frame_Counter >> 8) & 0x00FF);

    Block_A[12] = 0x00; //Frame counter upper Bytes
    Block_A[13] = 0x00;

    Block_A[14] = 0x00;

    Block_A[15] = i;

    //Calculate S
    AES_Encrypt(Block_A,AppSkey); //original
    

    //Check for last block
    if(i != Number_of_Blocks)
    {
      for(j = 0; j < 16; j++)
      {
        *Data = *Data ^ Block_A[j];
        Data++;
      }
    }
    else
    {
      if(Incomplete_Block_Size == 0)
      {
        Incomplete_Block_Size = 16;
      }
      for(j = 0; j < Incomplete_Block_Size; j++)
      {
        *Data = *Data ^ Block_A[j];
        Data++;
      }
    }
  }
}

/**************************************************************************/
/*!
    @brief    Function used to calculate the validity of data messages.
    @param    *Data
              Pointer to the data to decrypt or encrypt.
    @param    Data_Length
              Number of bytes to be transmitted.
    @param    *Final_Mic
              Pointer to MIC array (4 bytes).
    @param    Frame_Counter
              Frame counter of upstream frames.
    @param    Direction
              Direction of message (is up?).
*/
/**************************************************************************/
void TinyLoRa::Calculate_MIC(unsigned char *Data, unsigned char *Final_MIC, unsigned char Data_Length, unsigned int Frame_Counter, unsigned char Direction)
{
  unsigned char i;
  unsigned char Block_B[16];
  
  unsigned char Key_K1[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char Key_K2[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  //unsigned char Data_Copy[16];

  unsigned char Old_Data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  unsigned char New_Data[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  
  
  unsigned char Number_of_Blocks = 0x00;
  unsigned char Incomplete_Block_Size = 0x00;
  unsigned char Block_Counter = 0x01;

  //Create Block_B
  Block_B[0] = 0x49;
  Block_B[1] = 0x00;
  Block_B[2] = 0x00;
  Block_B[3] = 0x00;
  Block_B[4] = 0x00;

  Block_B[5] = Direction;

  Block_B[6] = DevAddr[3];
  Block_B[7] = DevAddr[2];
  Block_B[8] = DevAddr[1];
  Block_B[9] = DevAddr[0];

  Block_B[10] = (Frame_Counter & 0x00FF);
  Block_B[11] = ((Frame_Counter >> 8) & 0x00FF);

  Block_B[12] = 0x00; //Frame counter upper bytes
  Block_B[13] = 0x00;

  Block_B[14] = 0x00;
  Block_B[15] = Data_Length;

  //Calculate number of Blocks and blocksize of last block
  Number_of_Blocks = Data_Length / 16;
  Incomplete_Block_Size = Data_Length % 16;

  if(Incomplete_Block_Size != 0)
  {
    Number_of_Blocks++;
  }

  Generate_Keys(Key_K1, Key_K2);

  //Preform Calculation on Block B0

  //Preform AES encryption
  AES_Encrypt(Block_B,NwkSkey);

  //Copy Block_B to Old_Data
  for(i = 0; i < 16; i++)
  {
    Old_Data[i] = Block_B[i];
  }

  //Preform full calculating until n-1 messsage blocks
  while(Block_Counter < Number_of_Blocks)
  {
    //Copy data into array
    for(i = 0; i < 16; i++)
    {
      New_Data[i] = *Data;
      Data++;
    }

    //Preform XOR with old data
    XOR(New_Data,Old_Data);

    //Preform AES encryption
    AES_Encrypt(New_Data,NwkSkey);

    //Copy New_Data to Old_Data
    for(i = 0; i < 16; i++)
    {
      Old_Data[i] = New_Data[i];
    }

    //Raise Block counter
    Block_Counter++;
  }

  //Perform calculation on last block
  //Check if Datalength is a multiple of 16
  if(Incomplete_Block_Size == 0)
  {
    //Copy last data into array
    for(i = 0; i < 16; i++)
    {
      New_Data[i] = *Data;
      Data++;
    }

    //Preform XOR with Key 1
    XOR(New_Data,Key_K1);

    //Preform XOR with old data
    XOR(New_Data,Old_Data);
    
    //Preform last AES routine
    // read NwkSkey from PROGMEM
    AES_Encrypt(New_Data,NwkSkey);
  }
  else
  {
    //Copy the remaining data and fill the rest
    for(i =  0; i < 16; i++)
    {
      if(i < Incomplete_Block_Size)
      {
        New_Data[i] = *Data;
        Data++;
      }
      if(i == Incomplete_Block_Size)
      {
        New_Data[i] = 0x80;
      }
      if(i > Incomplete_Block_Size)
      {
        New_Data[i] = 0x00;
      }
    }

    //Preform XOR with Key 2
    XOR(New_Data,Key_K2);

    //Preform XOR with Old data
    XOR(New_Data,Old_Data);

    //Preform last AES routine
    AES_Encrypt(New_Data,NwkSkey);
  }

  Final_MIC[0] = New_Data[0];
  Final_MIC[1] = New_Data[1];
  Final_MIC[2] = New_Data[2];
  Final_MIC[3] = New_Data[3];
	
  // Generate a random number between 0 and 7 to select next transmit channel 
  randomNum = Final_MIC[3] & 0x07;

  // Generate a random number between 0 and 7 to randomise next transmit message schedule
  txrandomNum = Final_MIC[2] & 0x07;

}

/**************************************************************************/
/*!
    @brief    Function used to generate keys for the MIC calculation.
    @param    *K1
              Pointer to Key1.
    @param    *K2
              Pointer to Key2.
*/
/**************************************************************************/
void TinyLoRa::Generate_Keys(unsigned char *K1, unsigned char *K2)
{
  unsigned char i;
  unsigned char MSB_Key;

  //Encrypt the zeros in K1 with the NwkSkey
  AES_Encrypt(K1,NwkSkey);

  //Create K1
  //Check if MSB is 1
  if((K1[0] & 0x80) == 0x80)
  {
    MSB_Key = 1;
  }
  else
  {
    MSB_Key = 0;
  }

  //Shift K1 one bit left
  Shift_Left(K1);

  //if MSB was 1
  if(MSB_Key == 1)
  {
    K1[15] = K1[15] ^ 0x87;
  }

  //Copy K1 to K2
  for( i = 0; i < 16; i++)
  {
    K2[i] = K1[i];
  }

  //Check if MSB is 1
  if((K2[0] & 0x80) == 0x80)
  {
    MSB_Key = 1;
  }
  else
  {
    MSB_Key = 0;
  }

  //Shift K2 one bit left
  Shift_Left(K2);

  //Check if MSB was 1
  if(MSB_Key == 1)
  {
    K2[15] = K2[15] ^ 0x87;
  }
}
void TinyLoRa::Shift_Left(unsigned char *Data)
{
  unsigned char i;
  unsigned char Overflow = 0;
  //unsigned char High_Byte, Low_Byte;

  for(i = 0; i < 16; i++)
  {
    //Check for overflow on next byte except for the last byte
    if(i < 15)
    {
      //Check if upper bit is one
      if((Data[i+1] & 0x80) == 0x80)
      {
        Overflow = 1;
      }
      else
      {
        Overflow = 0;
      }
    }
    else
    {
      Overflow = 0;
    }

    //Shift one left
    Data[i] = (Data[i] << 1) + Overflow;
  }
}

/**************************************************************************/
/*!
    @brief    Function to XOR two character arrays.
    @param    *New_Data
              A pointer to the calculated data.
    @param    *Old_Data
              A pointer to the data to be xor'd.
*/
/**************************************************************************/
void TinyLoRa::XOR(unsigned char *New_Data,unsigned char *Old_Data)
{
  unsigned char i;

  for(i = 0; i < 16; i++)
  {
    New_Data[i] = New_Data[i] ^ Old_Data[i];
  }
}

/**************************************************************************/
/*!
    @brief    Function used to perform AES encryption.
    @param    *Data
              Pointer to the data to decrypt or encrypt.
    @param    *Key
              Pointer to AES encryption key.
*/
/**************************************************************************/
void TinyLoRa::AES_Encrypt(unsigned char *Data, unsigned char *Key)
{
  unsigned char Row, Column, Round = 0;
  unsigned char Round_Key[16];
  unsigned char State[4][4];

  //  Copy input to State arry
  for( Column = 0; Column < 4; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      State[Row][Column] = Data[Row + (Column << 2)];
    }
  }

  //  Copy key to round key
  memcpy( &Round_Key[0], &Key[0], 16 );

  //  Add round key
  AES_Add_Round_Key( Round_Key, State );

  //  Preform 9 full rounds with mixed collums
  for( Round = 1 ; Round < 10 ; Round++ )
  {
    //  Perform Byte substitution with S table
    for( Column = 0 ; Column < 4 ; Column++ )
    {
      for( Row = 0 ; Row < 4 ; Row++ )
      {
        State[Row][Column] = AES_Sub_Byte( State[Row][Column] );
      }
    }

    //  Perform Row Shift
    AES_Shift_Rows(State);

    //  Mix Collums
    AES_Mix_Collums(State);

    //  Calculate new round key
    AES_Calculate_Round_Key(Round, Round_Key);

        //  Add the round key to the Round_key
    AES_Add_Round_Key(Round_Key, State);
  }

  //  Perform Byte substitution with S table whitout mix collums
  for( Column = 0 ; Column < 4 ; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      State[Row][Column] = AES_Sub_Byte(State[Row][Column]);
    }
  }
    
  //  Shift rows
  AES_Shift_Rows(State);

  //  Calculate new round key
  AES_Calculate_Round_Key( Round, Round_Key );

    //  Add round key
  AES_Add_Round_Key( Round_Key, State );

  //  Copy the State into the data array
  for( Column = 0; Column < 4; Column++ )
  {
    for( Row = 0; Row < 4; Row++ )
    {
      Data[Row + (Column << 2)] = State[Row][Column];
    }
  }
}

/**************************************************************************/
/*!
    @brief    Function performs AES AddRoundKey step.
    @param    *Round_Key
              Pointer to the round subkey.
    @param    *State
              Pointer to bytes of the states-to-be-xor'd.
*/
/**************************************************************************/
void TinyLoRa::AES_Add_Round_Key(unsigned char *Round_Key, unsigned char (*State)[4])
{
  unsigned char Row, Collum;

  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      State[Row][Collum] ^= Round_Key[Row + (Collum << 2)];
    }
  }
}

/**************************************************************************/
/*!
    @brief    Function performs AES SubBytes step.
    @param    Byte
              Individual byte, from state array.
*/
/**************************************************************************/
unsigned char TinyLoRa::AES_Sub_Byte(unsigned char Byte)
{
//  unsigned char S_Row,S_Collum;
//  unsigned char S_Byte;
//
//  S_Row    = ((Byte >> 4) & 0x0F);
//  S_Collum = ((Byte >> 0) & 0x0F);
//  S_Byte   = S_Table [S_Row][S_Collum];

  //return S_Table [ ((Byte >> 4) & 0x0F) ] [ ((Byte >> 0) & 0x0F) ]; // original
  return pgm_read_byte(&(S_Table [((Byte >> 4) & 0x0F)] [((Byte >> 0) & 0x0F)]));
}

/**************************************************************************/
/*!
    @brief    Function performs AES ShiftRows step.
    @param    *State
              Pointer to state array.
*/
/**************************************************************************/
void TinyLoRa::AES_Shift_Rows(unsigned char (*State)[4])
{
  unsigned char Buffer;

  //Store firt byte in buffer
  Buffer      = State[1][0];
  //Shift all bytes
  State[1][0] = State[1][1];
  State[1][1] = State[1][2];
  State[1][2] = State[1][3];
  State[1][3] = Buffer;

  Buffer      = State[2][0];
  State[2][0] = State[2][2];
  State[2][2] = Buffer;
  Buffer      = State[2][1];
  State[2][1] = State[2][3];
  State[2][3] = Buffer;

  Buffer      = State[3][3];
  State[3][3] = State[3][2];
  State[3][2] = State[3][1];
  State[3][1] = State[3][0];
  State[3][0] = Buffer;
}

/**************************************************************************/
/*!
    @brief    Function performs AES MixColumns step.
    @param    *State
              Pointer to state array.
*/
/**************************************************************************/
void TinyLoRa::AES_Mix_Collums(unsigned char (*State)[4])
{
  unsigned char Row,Collum;
  unsigned char a[4], b[4];
    
    
  for(Collum = 0; Collum < 4; Collum++)
  {
    for(Row = 0; Row < 4; Row++)
    {
      a[Row] =  State[Row][Collum];
      b[Row] = (State[Row][Collum] << 1);

      if((State[Row][Collum] & 0x80) == 0x80)
      {
        b[Row] ^= 0x1B;
      }
    }
        
    State[0][Collum] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
    State[1][Collum] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
    State[2][Collum] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
    State[3][Collum] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
  }
}   //  AES_Mix_Collums



/**************************************************************************/
/*!
    @brief    Function performs AES Round Key Calculation.
    @param    Round
              Number of rounds to perform (depends on key size).
    @param    Round_Key
              Pointer to round key.
*/
/**************************************************************************/
void TinyLoRa::AES_Calculate_Round_Key(unsigned char Round, unsigned char *Round_Key)
{
  unsigned char i, j, b, Rcon;
  unsigned char Temp[4];

    
    //Calculate Rcon
  Rcon = 0x01;
  while(Round != 1)
  {
    b = Rcon & 0x80;
    Rcon = Rcon << 1;
        
    if(b == 0x80)
    {
      Rcon ^= 0x1b;
    }
    Round--;
  }
    
  //  Calculate first Temp
  //  Copy laste byte from previous key and subsitute the byte, but shift the array contents around by 1.
    Temp[0] = AES_Sub_Byte( Round_Key[12 + 1] );
    Temp[1] = AES_Sub_Byte( Round_Key[12 + 2] );
    Temp[2] = AES_Sub_Byte( Round_Key[12 + 3] );
    Temp[3] = AES_Sub_Byte( Round_Key[12 + 0] );

  //  XOR with Rcon
  Temp[0] ^= Rcon;

  //  Calculate new key
  for(i = 0; i < 4; i++)
  {
    for(j = 0; j < 4; j++)
    {
      Round_Key[j + (i << 2)]  ^= Temp[j];
      Temp[j]                   = Round_Key[j + (i << 2)];
    }
  }
}