
#ifndef DmxSerial_h
#define DmxSerial_h

#include "rdm.h"

// ----- Constants -----
#define DMXSERIAL_MAX 512                  ///< The max. number of supported DMX data channels
#define DMXSERIAL_MIN_SLOT_VALUE 0         ///< The min. value a DMX512 slot can take
#define DMXSERIAL_MAX_SLOT_VALUE 255       ///< The max. value a DMX512 slot can take
#define DMXSERIAL_MAX_RDM_STRING_LENGTH 32 ///< The max. length of a string in RDM

#define RDM_START_CODE  E120_SC_RDM
#define DMXBREAK 180
#define RDMBREAK 280
#define DMXMAB 20

// ----- Types -----

typedef uint8_t bool8;
typedef uint8_t byte;
typedef byte DEVICEID[6];

// ----- structures -----
struct RDMDATA {
  byte     StartCode;    // Start Code 0xCC for RDM
  byte     SubStartCode; // Start Code 0x01 for RDM
  byte     Length;       // packet length
  byte     DestID[6];
  byte     SourceID[6];

  byte     _TransNo;     // transaction number, not checked
  byte     ResponseType;    // ResponseType
  byte     _unknown;     // I don't know, ignore this
  uint16_t SubDev;      // sub device number (root = 0)
  byte     CmdClass;     // command class
  uint16_t Parameter;	   // parameter ID
  byte     DataLength;   // parameter data length in bytes
  byte     Data[231];   // data byte field
}__attribute__((packed));


//Callback function types

extern "C" {

  typedef bool8 (*RDMCallbackFunction)(struct RDMDATA *buffer, uint16_t *nackReason);
  typedef bool8 (*RDMGetSensorValue)(uint8_t sensorNr, int16_t *value, int16_t *lowestValue, int16_t *highestValue, int16_t *recordedValue);
}

// ----- Library Class -----
struct RDMPERSONALITY {
  uint16_t footprint;
 // const char          *deviceModel;
}__attribute__((packed));

struct RDMSENSOR {
  uint8_t type;
  uint8_t unit;
  uint8_t prefix;
  int16_t rangeMin;
  int16_t rangeMax;
  int16_t normalMin;
  int16_t normalMax;
  bool8 lowHighSupported;
  bool8 recordedSupported;
  char *description;
}__attribute__((packed));


struct RDMINIT {
  const char          *manufacturerLabel;
  const uint16_t      deviceModelId;
  const char          *deviceModel;
  uint16_t footprint;
  //uint8_t personalityCount;
  //RDMPERSONALITY *personalities;
  const uint16_t        additionalCommandsLength;
  const uint16_t       *additionalCommands;
  const uint8_t sensorsLength;
  const RDMSENSOR *sensors;
}__attribute__((packed));


class DMXSerialClass2
{
  public:

    void    init (struct RDMINIT *initData, RDMCallbackFunction func, uint8_t modePin = 2, uint8_t modeIn = 0, uint8_t modeOut = 1) {
      init(initData, func, NULL, modePin, modeIn, modeOut);
    }

    void    init (struct RDMINIT *initData, RDMCallbackFunction func, RDMGetSensorValue sensorFunc, uint8_t modePin = 2, uint8_t modeIn = 0, uint8_t modeOut = 1);
    uint8_t read       (int channel);

    // Read the last known value of a channel by using the startAddress and footprint range.
    uint8_t readRelative(unsigned int channel);

    void write(int channel, uint8_t value);

    unsigned long noDataSince();

    // ----- RDM specific members -----
    // Return true when identify mode was set on by controller.
    bool8 isIdentifyMode();

    /// Returns the Device ID. Copies the UID to the buffer passed through the uid argument.
    void getDeviceID(DEVICEID id);

    /// Return the current DMX start address that is the first dmx address used by the device.
    uint16_t getStartAddress();

    /// Return the current DMX footprint, that is the number of dmx addresses used by the device.
    uint16_t getFootprint();

    /// Register a device-specific implemented function for RDM callbacks
    void    attachRDMCallback (RDMCallbackFunction newFunction);

    /// Register a device-specific implemented function for getting sensor values
    void    attachSensorCallback (RDMGetSensorValue newFunction);

    /// check for unprocessed RDM Command.
    void    tick(void);

    /// A short custom label given to the device.
    char deviceLabel[DMXSERIAL_MAX_RDM_STRING_LENGTH];

    /// don't use that method from extern.
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled);


    void Filltxbuffer();
    void inittestbuffer();
    void Send_Packet();
    void Send_Packet2();
    void Testsub();

  private:
    /// process a relevant message
    void _processRDMMessage(byte CmdClass, uint16_t Parameter, bool8 isHandled, bool8 doRespond);

    /// common internal initialiSation function.
    void _baseInit();

    // callback function to device specific code
    RDMCallbackFunction _rdmFunc;

    // callback function to get sensor value
    RDMGetSensorValue _sensorFunc;

    /// remember the given manufacturer label and device model strings during init
    struct RDMINIT *_initData;
    struct PERSONALITY *_initData2;

    /// intern parameter settings
    const char *_softwareLabel;
	const char *_MypersonalityLabel;
    bool8  _identifyMode;


}; // class DMXSerialClass2


// Use the DMXSerial2 library through the DMXSerial2 object.
extern DMXSerialClass2 DMXSerial2;

#endif


