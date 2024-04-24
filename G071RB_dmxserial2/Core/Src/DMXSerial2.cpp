
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstring>
#include "main.h"
//*h24 #include "eeprom.h"
#include "DMXSerial2.h"

#define SWAPINT(i) (((i&0x00FF)<<8) | ((i&0xFF00)>>8))
#define SWAPINT32(i) ((i&0x000000ff)<<24) | ((i&0x0000ff00)<<8) | ((i&0x00ff0000)>>8) | ((i&0xff000000)>>24)
#define READINT(p) ((p[0]<<8) | (p[1]))
#define WRITEINT(p, d) (p)[0] = (d&0xFF00)>>8; (p)[1] = (d&0x00FF);
#define INTERPACKETDELAY 100
#define htons(n) __REV16(n)
#define ntohs(n) __REV16(n)

bool dmxchanged=false;

char eeprom__devicelabel[32];
char eprom_deviceid[32];
void save_settings();


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

extern unsigned long tickcounter;
volatile unsigned char data_tx[DMXSERIAL_MAX+1];
void DMX_Send_9Data(uint8_t tempdata);
void DMX_Break();
void DMX_BreakRDM();
void delayMicroseconds(uint16_t delay);

bool8 myresponse=false;


extern uint16_t eepromid;
extern uint16_t start_address;
extern uint16_t footprint;
extern uint16_t _newpersonality;
extern char label[32];;


void CalculateID()
{
	  char buf[10];
	  unsigned long *id = (unsigned long *)0x1FFFF7AC;
	  char* uniqueid = 	 utoa(id[0],buf, 10);
	  strcpy (label,"SkyLine Q35");
	  strcat (label,uniqueid);
	  printf("This Device unique part no is : ");
	  printf("%s",label);
	  printf("\n");
}


void init_settings(void)
{
	eepromid = 1234;
	start_address=1;
	footprint=24;
	_newpersonality=1;
	CalculateID();
	save_settings();
}

void load_settings(void)
{
	uint16_t id= readEEPROMHalfWord(0);
	 printf("%i\n",id);
	 if(id !=1234){

		 printf("loading defaults\n");
         init_settings();
	 }
	 else{
		 printf("loading configuration\n");
		 start_address= readEEPROMHalfWord(2);
		 footprint= readEEPROMHalfWord(4);
		 _newpersonality= readEEPROMHalfWord(6);

		 printf("Start Address:=%i\n",start_address);
		 printf("Footprint:=%i\n",footprint);
		 printf("New Personality:=%i\n",_newpersonality);
		 printf("default label:=%s\n",label);
	 	 }
}


void save_settings(void)
{
	printf("new address%i\n",start_address);
	 enableEEPROMWriting();
	 writeEEPROMHalfWord(0, 1234 );
	 writeEEPROMHalfWord(2, start_address );
	 writeEEPROMHalfWord(4, footprint );
	 writeEEPROMHalfWord(6, _newpersonality );

	 //label starts @ 8
	 disableEEPROMWriting();
}


//PRETTY ACURATE WITHIN A FEW US
void delayMicroseconds(uint16_t delay)
{
	    uint32_t i=5* delay;
	    while(i--);
}


unsigned int micros()
{
	return tickcounter;
}


unsigned int millis()
{
	return HAL_GetTick();
}


#define led2_Pin GPIO_PIN_9
#define led2_GPIO_Port GPIOA
#define DMX_TX_High  HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET)
#define DMX_TX_Low  HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_RESET)

void GPIO_Tx_Config_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = led2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(led2_GPIO_Port, &GPIO_InitStruct);
}

void GPIO_Tx_Config_AF(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = led2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(led2_GPIO_Port, &GPIO_InitStruct);
}

void DMX_Send_9Data(uint8_t tempdata)
{
    if(USART1->SR & (1<<6))
    {
        USART1->DR = 0x0100 | tempdata;
    }
    while((USART1->SR&0X40)==0);//waiting for Send data over
}

void DMX_Break()
{
    GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
    DMX_TX_Low;
    delayMicroseconds(DMXBREAK);        //180 = 199us
    DMX_TX_High;
    delayMicroseconds(DMXMAB);         //20 = 32uS
    GPIO_Tx_Config_AF();
    DMX_Send_9Data(0);			  // START CODE
}


void DMX_BreakRDM()
{
    GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
    DMX_TX_Low;
    delayMicroseconds(DMXBREAK+200);        //180 = 199us
    DMX_TX_High;
    delayMicroseconds(DMXMAB);         //20 = 32uS
    GPIO_Tx_Config_AF();
    DMX_Send_9Data(0x0100 |RDM_START_CODE);	   // START CODE
}

void DMX_BreakRDMNOSTARTCODE()
{
    GPIO_Tx_Config_OUT();     //Set UART TX pin mode to OUTPUT
    DMX_TX_Low;
    delayMicroseconds(DMXBREAK+200);    //180 = 199us
    DMX_TX_High;
    delayMicroseconds(DMXMAB);         //20 = 32uS
    GPIO_Tx_Config_AF();
}


void DMXSerialClass2::Send_Packet()
{
	 HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_SET);
    uint16_t i=0;
    DMX_Break();        //Break and Start Code

    while(i < 512)  //1-512
    {
        DMX_Send_9Data(0x0100 |data_tx[i]);
        i++;
    }

    HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_RESET);
}



unsigned long _timingReceiveEnd; // when the last incoming byte was received


typedef enum {
  IDLE,      // ignoring everything and wait for the next BREAK
             // or a valid RDM packet arrived, need for processing !
  BREAK,     // received a BREAK: now a new packet will start
  DMXDATA,   // receiving DMX data into the _dmxData buffer
  RDMDATA,   // receiving RDM data into the _rdm.buffer
  CHECKSUMH, // received the High byte of the _rdm.buffer checksum
  CHECKSUML  // received the Low byte of the _rdm.buffer checksum
} DMXReceivingState;



// the special discovery response message
struct DISCOVERYMSG {
  byte headerFE[7];
  byte headerAA;
  byte maskedDevID[12];
  byte checksum[4];
}__attribute__((packed));



// The DEVICEINFO structure (length = 19) has to be responded for E120_DEVICE_INFO
// See http://rdm.openlighting.org/pid/display?manufacturer=0&pid=96
struct DEVICEINFO {
  byte protocolMajor;
  byte protocolMinor;
  uint16_t deviceModel;
  uint16_t productCategory;
  uint32_t softwareVersion;
  uint16_t footprint;
  byte currentPersonality;
  byte personalityCount;
  uint16_t startAddress;
  uint16_t subDeviceCount;
  byte sensorCount;
}__attribute__((packed));


DEVICEID _devID = { 0x09, 0x87, 0x20, 0x12, 0x00, 0x00 };
// The Device ID for addressing all devices of a manufacturer.
DEVICEID _devIDGroup = { 0x09, 0x87, 0xFF, 0xFF, 0xFF, 0xFF };
// The Device ID for addressing all devices: 6 times 0xFF.
DEVICEID _devIDAll = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


// This is the buffer for RDM packets being received and sent.
// this structure is needed to RDM data separate from DMX data.
union RDMMEM {
  // the most common RDM packet layout for commands
  struct RDMDATA packet;
  // the layout of the RDM packet when returning a discovery message
  struct DISCOVERYMSG discovery;
  // the byte array used while receiving and sending.
  byte buffer[60];
}__attribute__((packed))  _rdm; // union RDMMEM


// This flag will be set when a full RDM packet was received.
bool8 _rdmAvailable;

bool8 mysponse=false;

// This is the current 16 bit checksum for RDM commands, used by the interrupt routines.
uint16_t _rdmCheckSum;

// static data that is not needed externally so it is not put into the class definition.
bool8 _isMute;    // is set to true when RDM discovery command muted this device.

// compare 2 DeviceIDs
#define DeviceIDCmp(id1, id2) memcmp(id1, id2, sizeof(DEVICEID))

// copy an DeviceID id2 to id1
#define DeviceIDCpy(id1, id2) memcpy(id1, id2, sizeof(DEVICEID))


// ----- DMXSerial Private variables -----
// These variables are not class members because they have to be reached by the interrupt implementations.
// don't use these variable from outside, use the appropriate methods.

volatile uint8_t  _dmxState;          // Current State of receiving DMX Bytes
volatile int      _dmxPos;            // the current read or write position in a DMX/RDM packet transmission.
unsigned long     _gotLastPacket = 0; // the last time (using the millis function) a packet was received.

volatile byte *_dmxSendBuffer;
volatile int _dmxSendLen;

// Array of DMX values (raw).
// Entry 0 will never be used for DMX data but will store the startbyte (0 for DMX mode).
volatile byte _dmxData[DMXSERIAL_MAX+1];

// Create a single class instance. Multiple class instances (multiple simultaneous DMX ports) are not supported.
DMXSerialClass2 DMXSerial2;


void _DMXSerialBaud(uint16_t baud_setting, uint8_t format);
void _DMXSerialWriteByte(uint8_t data);

void respondMessage(bool8 isHandled, uint16_t nackReason = E120_NR_UNKNOWN_PID);
int random255();


void DMXSerialClass2::init(struct RDMINIT *initData, RDMCallbackFunction func, RDMGetSensorValue sensorFunc, uint8_t modePin, uint8_t modeIn, uint8_t modeOut)
{
  // save the given initData for later use.
  _initData = initData;
  _rdmFunc = func;
  _sensorFunc = sensorFunc;

  _baseInit();

  CalculateID();
  // now initialize RDM specific elements
  _isMute = false;
  _rdmAvailable = false;
  _identifyMode = false;
  _softwareLabel = "STM32 RDM 1.0";
   strcpy (deviceLabel,label);

  // now start
  HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_RESET);

  _dmxSendBuffer = _rdm.buffer;
  // _dmxSendLen = ... will be set individually

  // Enable receiver and RX COMPLETE interrupts
   USART1->CR1 |= ( USART_CR1_RE |USART_CR1_RXNEIE );
   printf("Listening on DMX address #");

   printf("%i\n",start_address);
}


// Read the current value of a channel.
uint8_t DMXSerialClass2::read(int channel)
{
  // adjust parameter
  if (channel < 1) channel = 1;
  if (channel > DMXSERIAL_MAX) channel = DMXSERIAL_MAX;
  // read value from buffer
  return(_dmxData[channel]);
} // read()


// get the deviceID
void DMXSerialClass2::getDeviceID (DEVICEID id) {
  DeviceIDCpy(id, _devID);
} // getDeviceID()


uint8_t DMXSerialClass2::readRelative(unsigned int channel)
{
  uint8_t val = 0;
  //if (channel < _initData->footprint) {
    // channel is in a valid range! (channel >= 0) is assumed by (unsigned int) type.
    val = _dmxData[start_address + channel];
  //} // if
  return(val);
} // readRelative()



void DMXSerialClass2::write(int channel, uint8_t value)
{
  // adjust parameters
  if (channel < 1) channel = 1;
  if (channel > DMXSERIAL_MAX) channel = DMXSERIAL_MAX;
  // store value for later sending
  _dmxData[channel] = value;
} // write()


// Register a self implemented function for RDM callbacks
void DMXSerialClass2::attachRDMCallback(RDMCallbackFunction newFunction)
{
  _rdmFunc = newFunction;
}


void DMXSerialClass2::attachSensorCallback(RDMGetSensorValue newFunction)
{
  _sensorFunc = newFunction;
} // attachSensorCallback

// some functions to hide the internal variables from being changed

unsigned long DMXSerialClass2::noDataSince() {
  /* Make sure we don't load partial updates of this multi-byte value */
	  __disable_irq();
  unsigned long lastPacket = _gotLastPacket;
  __enable_irq();
  return(HAL_GetTick() - lastPacket);
}

bool8 DMXSerialClass2::isIdentifyMode() { return(_identifyMode); }

uint16_t DMXSerialClass2::getStartAddress()
{
	uint16_t dmx_start_address =start_address;

	return(dmx_start_address);
}

uint16_t DMXSerialClass2::getFootprint() { return(_initData->footprint); }



void DMXSerialClass2::tick(void)
{
  if (((_dmxState == IDLE) || (_dmxState == BREAK)) && (_rdmAvailable)) { // 15.06.2015
    // never process twice.
    _rdmAvailable = false;

    // respond to RDM commands now.
    bool8 packetIsForMe = false;
    bool8 packetIsForGroup = false;
    bool8 packetIsForAll = false;
    bool8 isHandled = false;

    struct RDMDATA *rdm = &_rdm.packet;

    byte     CmdClass  = rdm->CmdClass;  // command class

    uint16_t Parameter=ntohs(rdm->Parameter);// parameter ID

    // in the ISR only some global conditions are checked: DestID
    if (DeviceIDCmp(rdm->DestID, _devIDAll) == 0) {
      packetIsForAll = true;
    } else if (DeviceIDCmp(rdm->DestID, _devIDGroup) == 0) {
      packetIsForGroup = true;
    } else if (DeviceIDCmp(rdm->DestID, _devID) == 0) {
      packetIsForMe = true;
    } // if

      if ((! packetIsForMe) && (! packetIsForGroup) && (! packetIsForAll)) {
        // ignore this packet

      } else if (CmdClass == E120_DISCOVERY_COMMAND) { // 0x10
        // handle all Discovery commands locally
        if (Parameter == E120_DISC_UNIQUE_BRANCH) { // 0x0001
          // not tested here for pgm space reasons: rdm->Length must be 24+6+6 = 36
          // not tested here for pgm space reasons: rdm->_DataLength must be 6+6 = 12

          if (! _isMute) {
            // check if my _devID is in the discovery range
            if ((DeviceIDCmp(rdm->Data, _devID) <= 0) && (DeviceIDCmp(_devID, rdm->Data+6) <= 0)) {

              // respond a special discovery message !
              struct DISCOVERYMSG *disc = &_rdm.discovery;
              _rdmCheckSum = 6 * 0xFF;

              // fill in the _rdm.discovery response structure
              for (byte i = 0; i < 7; i++)
                disc->headerFE[i] = 0xFE;
              disc->headerAA = 0xAA;
              for (byte i = 0; i < 6; i++) {
                disc->maskedDevID[i+i]   = _devID[i] | 0xAA;
                disc->maskedDevID[i+i+1] = _devID[i] | 0x55;
                _rdmCheckSum += _devID[i];
              }
              disc->checksum[0] = (_rdmCheckSum >> 8)   | 0xAA;
              disc->checksum[1] = (_rdmCheckSum >> 8)   | 0x55;
              disc->checksum[2] = (_rdmCheckSum & 0xFF) | 0xAA;
              disc->checksum[3] = (_rdmCheckSum & 0xFF) | 0x55;

             unsigned int _rdmBufferLen = sizeof(_rdm.discovery);
         	 uint16_t i=0;
         	  HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_SET);
              DMX_BreakRDM();       //Break and Start Code

             while(i < _rdmBufferLen)
             {
                 DMX_Send_9Data( _rdm.buffer[i]);
                 i++;
             }
             HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_RESET);
         	 delayMicroseconds(INTERPACKETDELAY*6);

              // Re-enable receiver and Receive interrupt
              _dmxState= IDLE; // initial state
             // USART1->CR1 |= (USART_CR1_RXNEIE );

            } // if
          } // if

        } else if (Parameter == E120_DISC_UN_MUTE) { // 0x0003
          isHandled = true;
          if (_rdm.packet.DataLength > 0) {
            // Unexpected data
            // Do nothing
          } else {
            _isMute = false;
            if (packetIsForMe) { // Only actually respond if it's sent direct to us
              // Control field
              _rdm.packet.Data[0] = 0b00000000;
              _rdm.packet.Data[1] = 0b00000000;
              _rdm.packet.DataLength = 2;
              respondMessage(true); // 21.11.2013
            }
          }

        } else if (Parameter == E120_DISC_MUTE) { // 0x0002
          isHandled = true;
          if (_rdm.packet.DataLength > 0) {
            // Unexpected data
            // Do nothing
          } else {
            _isMute = true;
            if (packetIsForMe) { // Only actually respond if it's sent direct to us
              // Control field
              _rdm.packet.Data[0] = 0b00000000;
              _rdm.packet.Data[1] = 0b00000000;
              _rdm.packet.DataLength = 2;
              respondMessage(true); // 21.11.2013
            }
          }
        } // if

      } else {
        // don't ignore packets not sent directly but via broadcasts.
        // Only send an answer on directly sent packages.
        DMXSerial2._processRDMMessage(CmdClass, Parameter, isHandled, packetIsForMe);
      }
  }
}

bool tempflag=false;

//------------------------------------------------------------------------------------

void DMXSerialClass2::_baseInit() {
  _dmxPos = 0;
  _dmxState= IDLE; // initial state
  _gotLastPacket = millis(); // remember current (relative) time in msecs.
  // initialiSe the DMX buffer
  for (int n = 0; n < DMXSERIAL_MAX+1; n++)
    _dmxData[n] = 0;
}


// Interrupt Service Routine, called when a byte or frame error was received.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  uint8_t  USARTstate= USART1->SR;    //get state before data!
  uint8_t  DmxByte   = USART1->DR;      //get data
  uint8_t  DmxState  = _dmxState; //just load once from SRAM to increase speed

  if (USARTstate  & UART_FLAG_FE) { //check for break
    _dmxState = BREAK; // break condition detected.
    _dmxPos= 0;        // The next data byte is the start byte

  } else if (DmxState == IDLE) {
    // wait on...

  } else if (DmxState == BREAK) {
    if (DmxByte == 0) {
      _dmxState = DMXDATA; // DMX data start code detected
      // _dmxData[_dmxPos++] = DmxByte;  // store in DMX buffer
      _dmxPos = 1;
      _gotLastPacket = millis(); // remember current (relative) time in msecs.

    } else if (DmxByte == E120_SC_RDM) {
    	myresponse=true;
      _dmxState = RDMDATA;  // RDM command start code
      _rdm.buffer[_dmxPos++] = DmxByte;  // store in RDM buffer (in StartCode)
      _rdmCheckSum = DmxByte;

    } else {
      // This might be a non RDM command -> not implemented so wait for next BREAK !
      _dmxState= IDLE;
    } // if

  } else if (DmxState == DMXDATA) {
    // another DMX byte
    _dmxData[_dmxPos++]= DmxByte;  // store received data into DMX data buffer.

    if (_dmxPos > DMXSERIAL_MAX) { // all channels done.
      _dmxState = IDLE; // wait for next break
    } // if

  } else if (DmxState == RDMDATA) {
    // another RDM byte
    if (_dmxPos >= (int)sizeof(_rdm.buffer)) {
      // too much data ...
      _dmxState = IDLE; // wait for next break
    } else {
      _rdm.buffer[_dmxPos++] = DmxByte;
      _rdmCheckSum += DmxByte;

      if (_dmxPos == _rdm.packet.Length) {
        // all data received. Now getting checksum !
        _dmxState = CHECKSUMH; // wait for checksum High Byte
      } // if
    } // if

  } else if (DmxState == CHECKSUMH) {
    // High byte of RDM checksum -> subtract from checksum
    _rdmCheckSum -= DmxByte << 8;
    _dmxState = CHECKSUML;

  } else if (DmxState == CHECKSUML) {
    // Low byte of RDM checksum -> subtract from checksum
    _rdmCheckSum -= DmxByte;

    // now check some error conditions and addressing issues
    if ((_rdmCheckSum == 0) && (_rdm.packet.SubStartCode == E120_SC_SUB_MESSAGE)) { // 0x01
      // prepare for answering when tick() is called
      _rdmAvailable = true;
      _gotLastPacket = millis(); // remember current (relative) time in msecs.
      // TIMING: remember when the last byte was sent
      _timingReceiveEnd = micros();
    } // if

    _dmxState = IDLE; // wait for next break or RDM package processing.
  } // if

}

// send back original Message including changed data in some cases
void respondMessage(bool8 isHandled, uint16_t nackReason)
{

  uint16_t checkSum = 0;

  // no need to set these data fields:
  // StartCode, SubStartCode
  if (isHandled) {
    _rdm.packet.ResponseType = E120_RESPONSE_TYPE_ACK; // 0x00
  } else {
    _rdm.packet.ResponseType = E120_RESPONSE_TYPE_NACK_REASON; // 0x00
    _rdm.packet.DataLength = 2;
    _rdm.packet.Data[0] = (nackReason >> 8) & 0xFF;
    _rdm.packet.Data[1] = nackReason & 0xFF;
  } // if
  _rdm.packet.Length = _rdm.packet.DataLength + 24; // total packet length

  // swap SrcID into DestID for sending back.
  DeviceIDCpy(_rdm.packet.DestID, _rdm.packet.SourceID);
  DeviceIDCpy(_rdm.packet.SourceID, _devID);

  _rdm.packet.CmdClass++;

  // prepare buffer and Checksum
  _dmxSendLen = _rdm.packet.Length;
  for (byte i = 0; i < _dmxSendLen; i++) {
    checkSum += _dmxSendBuffer[i];
  } // for

  unsigned long d = micros() - _timingReceiveEnd;
  if (d <190)
    delayMicroseconds(190 - d);

 USART1->CR1 &= ~(  USART_CR1_TCIE );

  uint16_t i=0;
  HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_SET);
   DMX_BreakRDMNOSTARTCODE();

  while(i < _dmxSendLen)
  {
      DMX_Send_9Data( _rdm.buffer[i]);
      // printf("val"); printf("%i\n", _rdm.buffer[i]);
     i++;
  }

  // send High Byte
   DMX_Send_9Data(checkSum >> 8);

  // send Low Byte
  DMX_Send_9Data(checkSum & 0xFF);

  HAL_GPIO_WritePin(DmxDirection_GPIO_Port, DmxDirection_Pin, GPIO_PIN_RESET);

  // Re-enable receiver and Receive interrupt ??
  _dmxState= IDLE; // initial state

}

// generate a number 0..255
int random255() {
  int num = 240;
  return(num);
}



//-------------------------------------------------------------------------------
//				    		new PROCESS RDM MESSAGE
//-------------------------------------------------------------------------------

void DMXSerialClass2::_processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 handled, bool8 doRespond)
{
	uint16_t nackReason = E120_NR_UNKNOWN_PID;

	// call the device specific method
	if ((!handled) && (_rdmFunc)) {
		handled = _rdmFunc(&_rdm.packet, &nackReason);
	} // if

	// if not already handled the command: handle it using this implementation
	if (!handled) {

		//-------------------------------------------------------------------------------
		//				    		IDENTIFY_DEVICE
		//-------------------------------------------------------------------------------
		if (Parameter == E120_IDENTIFY_DEVICE) { // 0x1000
			if (CmdClass == E120_SET_COMMAND) { // 0x30
				if (_rdm.packet.DataLength != 1) {
					// Oversized data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if ((_rdm.packet.Data[0] != 0) && (_rdm.packet.Data[0] != 1)) {
					// Out of range data
					nackReason = E120_NR_DATA_OUT_OF_RANGE;
				}
				else {
					_identifyMode = _rdm.packet.Data[0] != 0;
					_rdm.packet.DataLength = 0;
					handled = true;
				}
			}
			else if (CmdClass == E120_GET_COMMAND) { // 0x20
				if (_rdm.packet.DataLength > 0) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					_rdm.packet.Data[0] = _identifyMode;
					_rdm.packet.DataLength = 1;
					handled = true;
				}
			} // if

		}

		//-------------------------------------------------------------------------------
		//				    		GET DEVICE_INFO
		//-------------------------------------------------------------------------------
		else if ((CmdClass == E120_GET_COMMAND) && (Parameter == E120_DEVICE_INFO)) { // 0x0060
			if (_rdm.packet.DataLength > 0) {
				// Unexpected data
				nackReason = E120_NR_FORMAT_ERROR;
			}
			else if (_rdm.packet.SubDev != 0) {
				// No sub-devices supported
				nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
			}
			else {
				// return all device info data
				DEVICEINFO *devInfo = (DEVICEINFO *)(_rdm.packet.Data); // The data has to be responded in the Data buffer.

				devInfo->protocolMajor = 1;
				devInfo->protocolMinor = 0;
				devInfo->deviceModel = SWAPINT(_initData->deviceModelId);
				devInfo->productCategory = SWAPINT(E120_PRODUCT_CATEGORY_DIMMER_CS_LED);
				devInfo->softwareVersion = SWAPINT32(0x01000000);// 0x04020900;
				devInfo->footprint = SWAPINT(footprint);
				//devInfo->currentPersonality = 1;
				devInfo->currentPersonality = _newpersonality;
				devInfo->personalityCount = 6;
		        devInfo->startAddress = SWAPINT(start_address);
				devInfo->subDeviceCount = 0;
				devInfo->sensorCount = _initData->sensorsLength;

				_rdm.packet.DataLength = sizeof(DEVICEINFO);
				handled = true;
			}

		}
		//-------------------------------------------------------------------------------
		//				    		GET _MANUFACTURER_LABEL
		//-------------------------------------------------------------------------------
		else if ((CmdClass == E120_GET_COMMAND) && (Parameter == E120_MANUFACTURER_LABEL)) { // 0x0081
			if (_rdm.packet.DataLength > 0) {
				// Unexpected data
				nackReason = E120_NR_FORMAT_ERROR;
			}
			else if (_rdm.packet.SubDev != 0) {
				// No sub-devices supported
				nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
			}
			else {
				// return the manufacturer label
				_rdm.packet.DataLength = strlen(_initData->manufacturerLabel);
				memcpy(_rdm.packet.Data, _initData->manufacturerLabel, _rdm.packet.DataLength);
				handled = true;
			}

		}
		//-------------------------------------------------------------------------------
		//				    		GET DEVICE_MODEL_DESCRIPTION
		//-------------------------------------------------------------------------------
		else if ((CmdClass == E120_GET_COMMAND) && (Parameter ==E120_DEVICE_MODEL_DESCRIPTION)) { // 0x0080
			if (_rdm.packet.DataLength > 0) {
				// Unexpected data
				nackReason = E120_NR_FORMAT_ERROR;
			}
			else if (_rdm.packet.SubDev != 0) {
				// No sub-devices supported
				nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
			}
			else {
				// return the DEVICE MODEL DESCRIPTION
				_rdm.packet.DataLength = strlen(_initData->deviceModel);
				memcpy(_rdm.packet.Data, _initData->deviceModel, _rdm.packet.DataLength);
				handled = true;
			}

		}
		//-------------------------------------------------------------------------------
		//				    		SET DEVICE_LABEL
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_DEVICE_LABEL) { // 0x0082
		      if (CmdClass == E120_SET_COMMAND) {
		        if (_rdm.packet.DataLength > DMXSERIAL_MAX_RDM_STRING_LENGTH) {
		          // Oversized data
		          nackReason = E120_NR_FORMAT_ERROR;
		        } else {
		          memcpy(deviceLabel, _rdm.packet.Data, _rdm.packet.DataLength);
		          deviceLabel[_rdm.packet.DataLength] = '\0';
		          _rdm.packet.DataLength = 0;
		          // persist in EEPROM
		          save_settings();
		          handled = true;

				}
			}
			//-------------------------------------------------------------------------------
			//				    		GET DEVICE_LABEL
			//-------------------------------------------------------------------------------
			else if (CmdClass == E120_GET_COMMAND) {
				if (_rdm.packet.DataLength > 0) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					_rdm.packet.DataLength = strlen(deviceLabel);
					memcpy(_rdm.packet.Data, deviceLabel, _rdm.packet.DataLength);
					handled = true;
				}
			} // if

		}
		//-------------------------------------------------------------------------------
		//				    		GET SOFTWARE_VERSION_LABEL
		//-------------------------------------------------------------------------------
		else if ((CmdClass == E120_GET_COMMAND) && (Parameter == E120_SOFTWARE_VERSION_LABEL)) { // 0x00C0
			if (_rdm.packet.DataLength > 0) {
				// Unexpected data
				nackReason = E120_NR_FORMAT_ERROR;
			}
			else if (_rdm.packet.SubDev != 0) {
				// No sub-devices supported
				nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
			}
			else {
				// return the SOFTWARE_VERSION_LABEL
				_rdm.packet.DataLength = strlen(_softwareLabel);
				memcpy(_rdm.packet.Data, _softwareLabel, _rdm.packet.DataLength);
				handled = true;
			}

		}
		//-------------------------------------------------------------------------------
		//				    		SET DMX_START_ADDRESS
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_DMX_START_ADDRESS) { // 0x00F0
			if (CmdClass == E120_SET_COMMAND) {
				if (_rdm.packet.DataLength != 2) {
					// Oversized data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else {
					uint16_t newStartAddress = READINT(_rdm.packet.Data);
					if ((newStartAddress <= 0) || (newStartAddress > DMXSERIAL_MAX)) {
						// Out of range start address
						// TODO(Peter): Should it be newStartAddress less footprint?
						nackReason = E120_NR_DATA_OUT_OF_RANGE;
					}
					else {
						start_address = newStartAddress;
						_rdm.packet.DataLength = 0;
						// persist in EEPROM
			            save_settings();
						handled = true;
					}
				}
			}
			//-------------------------------------------------------------------------------
			//				    		GET DMX_START_ADDRESS
			//-------------------------------------------------------------------------------
			else if (CmdClass == E120_GET_COMMAND) {
				if (_rdm.packet.DataLength > 0) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					 WRITEINT(_rdm.packet.Data, start_address);
					_rdm.packet.DataLength = 2;
					handled = true;
				}
			} // if

		}

		//-------------------------------------------------------------------------------
		//				    		SET PERSONALITY NUMBER
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_DMX_PERSONALITY) { // 0x00E0
		if (CmdClass == E120_SET_COMMAND) {
			printf("SET PERSONALITY");
			printf("datalength:");
		    printf("%i",_rdm.packet.DataLength);

			if (_rdm.packet.DataLength != 1) {
				// Oversized data
				nackReason = E120_NR_FORMAT_ERROR;
			}
				else {
					uint8_t newPersonality = _rdm.packet.Data[0];
					_newpersonality = newPersonality;
					printf("personality:");
					printf("%i",_newpersonality);
					_rdm.packet.DataLength = 0;
					// persist in EEPROM
			        save_settings();
					//nackReason = E120_RESPONSE_TYPE_ACK;
					handled = true;
				}
			//}
		}
		//-------------------------------------------------------------------------------
		//				    		GET PERSONALITY NUMBER
		//-------------------------------------------------------------------------------
		else if (CmdClass == E120_GET_COMMAND) {
			printf("GET PERSONALITY");
			if (_rdm.packet.DataLength > 0) {
				// Unexpected data
				nackReason = E120_NR_FORMAT_ERROR;
			}
			else if (_rdm.packet.SubDev != 0) {
				// No sub-devices supported
				nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
			}
			else {
				printf("GOT PERSONALITY: ");
				printf("%i",_newpersonality);
				_rdm.packet.Data[0]=_newpersonality;
				_rdm.packet.DataLength = 1;
				handled = true;
			}
		} // if

		}

		//-------------------------------------------------------------------------------
		//				    		GET PERSONALITY_DESCRIPTION
		//-------------------------------------------------------------------------------


		else if ((CmdClass == E120_GET_COMMAND) && (Parameter == E120_DMX_PERSONALITY_DESCRIPTION)) { // 0x00E1
		//Serial.println("GET DMX_PERSONALITY_DESCRIPTION");
		if (_rdm.packet.DataLength > 0) {
			// Unexpected data
			nackReason = E120_NR_FORMAT_ERROR;
		}
		else if (_rdm.packet.SubDev != 0) {
			// No sub-devices supported
			nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
		}
		else {
			// return the PERSONALITY DESCRIPTIONS
			//uint8_t persno = 0;
			//_rdm.packet.DataLength = 2 + strlen(_initData->sensors[persno].description);
			//_rdm.packet.Data[0] = 0;	//fp hi
			//_rdm.packet.Data[1] = 1;		//fp lo
			//memcpy(_rdm.packet.Data + 2, _initData->sensors[persno].description, _rdm.packet.DataLength - 2);


			struct PERSONALITIES *per ;
			_rdm.packet.DataLength = sizeof (per);
			memcpy(_rdm.packet.Data, per, _rdm.packet.DataLength);

			handled = true;
		}

		}
		//-------------------------------------------------------------------------------
		//				    		GET SUPPORTED_PARAMETERS
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_SUPPORTED_PARAMETERS) { // 0x0050
			if (CmdClass == E120_GET_COMMAND) {
				if (_rdm.packet.DataLength > 0) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					// Some supported PIDs shouldn't be returned as per the standard, these are:
					// E120_DISC_UNIQUE_BRANCH
					// E120_DISC_MUTE
					// E120_DISC_UN_MUTE
					// E120_SUPPORTED_PARAMETERS
					// E120_IDENTIFY_DEVICE
					// E120_DEVICE_INFO
					// E120_DMX_START_ADDRESS
					// E120_SOFTWARE_VERSION_LABEL
					_rdm.packet.DataLength = 2 * (3 + _initData->additionalCommandsLength);
					WRITEINT(_rdm.packet.Data, E120_MANUFACTURER_LABEL);
					WRITEINT(_rdm.packet.Data + 2, E120_DEVICE_MODEL_DESCRIPTION);
					WRITEINT(_rdm.packet.Data + 4, E120_DEVICE_LABEL);
					WRITEINT(_rdm.packet.Data + 6, E120_DMX_PERSONALITY);

					uint8_t offset = 8;
					if (_initData->sensorsLength > 0) {
						_rdm.packet.DataLength += 2 * 2;
						offset += 2 * 2;
						WRITEINT(_rdm.packet.Data + 8, E120_SENSOR_DEFINITION);
						WRITEINT(_rdm.packet.Data + 10, E120_SENSOR_VALUE);
					}
					for (uint16_t n = 0; n < _initData->additionalCommandsLength; n++) {
						WRITEINT(_rdm.packet.Data + offset + n + n, _initData->additionalCommands[n]);
					}
					handled = true;
				}
			}
			else if (CmdClass == E120_SET_COMMAND) {
				// Unexpected set
				nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
			}

			// ADD: PARAMETER_DESCRIPTION

		}
		//-------------------------------------------------------------------------------
		//				    		GET SENSOR_DEFINITION
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_SENSOR_DEFINITION && _initData->sensorsLength > 0) { // 0x0200
			if (CmdClass == E120_GET_COMMAND) {
				if (_rdm.packet.DataLength != 1) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					uint8_t sensorNr = _rdm.packet.Data[0];
					if (sensorNr >= _initData->sensorsLength) {
						// Out of range sensor
						nackReason = E120_NR_DATA_OUT_OF_RANGE;
					}
					else {
						_rdm.packet.DataLength = 13 + strlen(_initData->sensors[sensorNr].description);
						_rdm.packet.Data[0] = sensorNr;
						_rdm.packet.Data[1] = _initData->sensors[sensorNr].type;
						_rdm.packet.Data[2] = _initData->sensors[sensorNr].unit;
						_rdm.packet.Data[3] = _initData->sensors[sensorNr].prefix;
						WRITEINT(_rdm.packet.Data + 4, _initData->sensors[sensorNr].rangeMin);
						WRITEINT(_rdm.packet.Data + 6, _initData->sensors[sensorNr].rangeMax);
						WRITEINT(_rdm.packet.Data + 8, _initData->sensors[sensorNr].normalMin);
						WRITEINT(_rdm.packet.Data + 10, _initData->sensors[sensorNr].normalMax);
						_rdm.packet.Data[12] = (_initData->sensors[sensorNr].lowHighSupported ? 2 : 0) | (_initData->sensors[sensorNr].recordedSupported ? 1 : 0);
						memcpy(_rdm.packet.Data + 13, _initData->sensors[sensorNr].description, _rdm.packet.DataLength - 13);
						handled = true;
					}
				}
			}
			else if (CmdClass == E120_SET_COMMAND) {
				// Unexpected set
				nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
			}
		}
		//-------------------------------------------------------------------------------
		//				    		GET SENSOR_VALUE
		//-------------------------------------------------------------------------------
		else if (Parameter == E120_SENSOR_VALUE && _initData->sensorsLength > 0) { // 0x0201
			if (CmdClass == E120_GET_COMMAND) {
				if (_rdm.packet.DataLength != 1) {
					// Unexpected data
					nackReason = E120_NR_FORMAT_ERROR;
				}
				else if (_rdm.packet.SubDev != 0) {
					// No sub-devices supported
					nackReason = E120_NR_SUB_DEVICE_OUT_OF_RANGE;
				}
				else {
					uint8_t sensorNr = _rdm.packet.Data[0];
					if (sensorNr >= _initData->sensorsLength) {
						// Out of range sensor
						nackReason = E120_NR_DATA_OUT_OF_RANGE;
					}
					else {
						int16_t sensorValue = 0;
						int16_t lowestValue = 0;
						int16_t highestValue = 0;
						int16_t recordedValue = 0;
						bool8 res = false;
						if (_sensorFunc) {
							res = _sensorFunc(sensorNr, &sensorValue, &lowestValue, &highestValue, &recordedValue);
						}
						if (res) {
							_rdm.packet.DataLength = 9;
							_rdm.packet.Data[0] = sensorNr;
							WRITEINT(_rdm.packet.Data + 1, sensorValue);
							WRITEINT(_rdm.packet.Data + 3, lowestValue);
							WRITEINT(_rdm.packet.Data + 5, highestValue);
							WRITEINT(_rdm.packet.Data + 7, recordedValue);
							handled = true;
						}
						else {
							nackReason = E120_NR_HARDWARE_FAULT;
						}
					}
				}
			}
			else if (CmdClass == E120_SET_COMMAND) {
				// Unhandled set. Set on a sensor is used to reset stats.
				// User should process it in own handler when sensor supports high/low or recorded value.
				nackReason = E120_NR_UNSUPPORTED_COMMAND_CLASS;
			}
		}
		else {
			handled = false;

		}  // if
	}  // if

	if (doRespond)
		respondMessage(handled, nackReason);
} // _processRDMMessage



