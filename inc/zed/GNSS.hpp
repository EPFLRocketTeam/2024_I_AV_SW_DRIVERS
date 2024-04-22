#ifndef GNSS_H
#define GNSS_H


#include "GNSS_DEF.hpp"






static int turut(0);

class GNSSDeviceBus
{
  public:
    // For I2C, ping the _address
    // Not Applicable for SPI and Serial
    virtual bool ping() = 0;

    // For Serial, return Serial.available()
    // For I2C, read registers 0xFD and 0xFE. Return bytes available as uint16_t
    // Not Applicable for SPI
    virtual uint16_t available() = 0;

    // For Serial, do Serial.write
    // For I2C, push data to register 0xFF. Chunkify if necessary. Prevent single byte writes as these are illegal
    // For SPI, writing bytes will also read bytes simultaneously. Read data is _ignored_ here. Use writeReadBytes
    virtual uint8_t writeBytes(uint8_t *data, uint8_t length) = 0;

    // For SPI, writing bytes will also read bytes simultaneously. Read data is returned in readData
    virtual uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length) = 0;
    virtual void startWriteReadByte() = 0;                                  // beginTransaction
    virtual void writeReadByte(const uint8_t *data, uint8_t *readData) = 0; // transfer
    virtual void writeReadByte(const uint8_t data, uint8_t *readData) = 0;  // transfer
    virtual void endWriteReadByte() = 0;                                    // endTransaction

    // For Serial, attempt Serial.read
    // For I2C, read from register 0xFF
    // For SPI, read the byte while writing 0xFF
    virtual uint8_t readBytes(uint8_t *data, uint8_t length) = 0;
};

class SfeI2C : public GNSSDeviceBus
{
  public:
    SfeI2C(void);

    bool init(uint8_t address);

    bool init(I2C_INTERFACE  &wirePort, uint8_t address, bool bInit = false);

    bool ping();

    uint16_t available();

    uint8_t writeBytes(uint8_t *data, uint8_t length);

    uint8_t writeReadBytes(const uint8_t *data, uint8_t *readData, uint8_t length)
    { (void)data; (void)readData; (void)length; return 0; }

    void startWriteReadByte(){};
    void writeReadByte(const uint8_t *data, uint8_t *readData){ (void)data; (void)readData; }
    void writeReadByte(const uint8_t data, uint8_t *readData){ (void)data; (void)readData; }
    void endWriteReadByte(){};

    uint8_t readBytes(uint8_t *data, uint8_t length);

  private:
    I2C_INTERFACE  *_i2cPort;
    uint8_t _address;
};

class SFE_UBLOX_GNSS 
{
public:
  SFE_UBLOX_GNSS() { _commType = COMM_TYPE_I2C; }



  // Depending on the sentence type the processor will load characters into different arrays
  enum sfe_ublox_sentence_types_e
  {
    SFE_UBLOX_SENTENCE_TYPE_NONE = 0,
    SFE_UBLOX_SENTENCE_TYPE_NMEA,
    SFE_UBLOX_SENTENCE_TYPE_UBX,
    SFE_UBLOX_SENTENCE_TYPE_RTCM
  } currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;
#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  struct
  {
    union
    {
      uint8_t all;
      struct
      {
        uint8_t dataValid1005 : 1;
        uint8_t dataRead1005 : 1;
        uint8_t dataValid1006 : 1;
        uint8_t dataRead1006 : 1;
      } bits;
    } flags;
    RTCM_1005_data_t rtcm1005; // Latest RTCM 1005 parsed from pushRawData
    RTCM_1006_data_t rtcm1006; // Latest RTCM 1006 parsed from pushRawData
    void (*rtcm1005CallbackPointer)(RTCM_1005_data_t *);
    void (*rtcm1006CallbackPointer)(RTCM_1006_data_t *);
    void init(void) // Initializer / constructor
    {
      flags.all = 0;                     // Clear the RTCM Input flags
      rtcm1005CallbackPointer = nullptr; // Clear the callback pointers
      rtcm1006CallbackPointer = nullptr;
    }
  } rtcmInputStorage; // Latest RTCM parsed from pushRawData
#endif


  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the SFE_UBLOX_GNSS library and connect to
  // the GNSS device. This method must be called before calling any other method
  // that interacts with the device.
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an I2C object which then defaults to Wire.
  bool begin(uint8_t deviceAddress = kUBLOXGNSSDefaultAddress, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus);

    // Initialize the I2C buss class i.e. setup default Wire port
    _i2cBus.init(deviceAddress);

    // Initialize the system - return results
    return this->init(maxWait, assumeSuccess);
  }

  // Version 2:
  //  User passes in an I2C object and an address (optional).
  bool begin(I2C_INTERFACE  &wirePort, uint8_t deviceAddress = kUBLOXGNSSDefaultAddress, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool assumeSuccess = false)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus);

    // Give the I2C port provided by the user to the I2C bus class.
    _i2cBus.init(wirePort, deviceAddress);

    // Initialize the system - return results
    return this->init(maxWait, assumeSuccess);
  }
  bool init(uint16_t maxWait, bool assumeSuccess);
  void setCommunicationBus(GNSSDeviceBus &theBus);
  bool createLock(void) { return true; }
  bool createFileBuffer(void);         
  bool setPacketCfgPayloadSize(size_t payloadSize); // Set packetCfgPayloadSize
  bool createRTCMBuffer(void);                                  // Create the RTCM buffer. Called by .begin
  bool isConnected(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  void setFileBufferSize(uint16_t bufferSize);                             // Set the size of the file buffer. This must be called _before_ .begin.
  bool ping();
  bool getVal8(uint32_t key, uint8_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);        // Returns the value at a given key location
  uint8_t getVal8(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Unsafe overload - for backward compatibility only
  sfe_ublox_status_e getVal(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Load payload with response
  sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait, bool expectACKonly = false); // Given a packet and payload, send everything including CRC bytes, return true if we got a response
  void calcChecksum(ubxPacket *msg);
  sfe_ublox_status_e sendI2cCommand(ubxPacket *outgoingUBX);
  sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait); // Poll the module until a config packet is received
  sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = kUBLOXGNSSDefaultMaxWait);   // Poll the module until a config packet and an ACK is received, or just an ACK
  uint8_t writeBytes(uint8_t *data, uint8_t length);
  uint8_t extractByte(ubxPacket *msg, uint16_t spotToStart);
  bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass = 0, uint8_t requestedID = 0); // Checks module with user selected commType
  bool checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);    // Method for I2C polling of data, passing any new bytes to process()
  uint16_t available();
  bool getModuleInfo(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);             // Queries module, extracts info. Returns true if MON-VER was read successfully
  bool initModuleSWVersion(); // Allocate RAM for moduleSWVersion and initialize it
 
  uint8_t getFirmwareVersionHigh(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Returns the FWVER XX.00 from UBX-MON-VER register
  uint8_t getFirmwareVersionLow(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);  // Returns the FWVER 00.XX from UBX-MON-VER register
  const char *getFirmwareType(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Returns the firmware type (SPG, HPG, ADR, etc.) from UBX-MON-VER register
  bool prepareModuleInfo(uint16_t maxWait);
  int8_t extractSignedChar(ubxPacket *msg, uint16_t spotToStart); // Get signed 8-bit value from payload
  bool getNavigationFrequency(uint8_t *navFreq, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);    // Get the number of nav solutions sent per second currently being output by module
  uint8_t getNavigationFrequency(uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                   // Unsafe overload
  bool getHPPOSLLH(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                                                                          // NAV HPPOSLLH
  uint32_t getTimeOfWeekFromHPPOSLLH(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getHighResLongitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getHighResLatitude(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getElipsoid(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int32_t getMeanSeaLevel(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getHighResLongitudeHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getHighResLatitudeHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getElipsoidHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  int8_t getMeanSeaLevelHp(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getHorizontalAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  uint32_t getVerticalAccuracy(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);
  bool initPacketUBXNAVHPPOSLLH();      // Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
  bool getVal16(uint32_t key, uint16_t *val, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Returns the value at a given key location
  uint16_t getVal16(uint32_t key, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                 // Unsafe overload - for backward compatibility only
  
  bool setI2COutput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure I2C port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setI2CInput(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);   // Configure I2C port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART2Input(uint8_t comSettings, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Configure UART2 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode = SFE_UBLOX_DGNSS_MODE_FIXED, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the DGNSS differential mode
  bool setNavigationFrequency(uint8_t navFreq, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Set the number of nav solutions sent per second
  bool setVal8(uint32_t key, uint8_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);             // Sets the 8-bit value at a given group/id/size location
  bool setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Sets the N-byte value at a given group/id/size location
  bool setVal16(uint32_t key, uint16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);           // Sets the 16-bit value at a given group/id/size location
  bool newCfgValset(uint8_t layer = VAL_LAYER_RAM_BBR);                                                         // Create a new, empty UBX-CFG-VALSET. Add entries with addCfgValset8/16/32/64
  bool addCfgValset8(uint32_t key, uint8_t value);     // Add a new key and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
  bool sendCfgValset(uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                              // Send the CfgValset (UBX-CFG-VALSET) construct
  bool addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N);                                                  // Add a new key and N-byte value to an existing UBX-CFG-VALSET ubxPacket
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key);
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2);
  bool setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait); // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoPVT(bool enabled, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                                                // Enable/disable automatic PVT reports at the navigation frequency
  bool setAutoPVT(bool enabled, bool implicitUpdate, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                           // Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoPVTrate(uint8_t rate, bool implicitUpdate = true, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);                // Set the rate for automatic PVT reports
  
  bool setRXMCORcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_COR_data_t *)); // RXM COR
  bool initPacketUBXRXMCOR();           // Allocate RAM for packetUBXRXMCOR and initialize it
  template <typename T>
  bool addCfgValset(uint32_t key, T value) // Add a new key and value to an existing UBX-CFG-VALSET ubxPacket
  {
    uint8_t val[8];

    uint32_t k1 = key & ~UBX_CFG_SIZE_MASK; // Convert key back into an actual key

    switch (key & UBX_CFG_SIZE_MASK)
    {
    case UBX_CFG_L:
      val[0] = (bool)value;
      return (addCfgValsetN(k1, val, 1));
      break;
    case UBX_CFG_U1:
    case UBX_CFG_E1:
    case UBX_CFG_X1:
      val[0] = (uint8_t)value;
      return (addCfgValsetN(k1, val, 1));
      break;
    case UBX_CFG_I1:
      unsignedSigned8 usVal8;
      usVal8.signed8 = (int8_t)value;
      return (addCfgValsetN(k1, &usVal8.unsigned8, 1));
      break;
    case UBX_CFG_U2:
    case UBX_CFG_E2:
    case UBX_CFG_X2:
      for (uint8_t i = 0; i < 2; i++)
        val[i] = (uint8_t)(((uint16_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 2));
      break;
    case UBX_CFG_I2:
      unsignedSigned16 usVal16;
      usVal16.signed16 = (int16_t)value;
      for (uint8_t i = 0; i < 2; i++)
        val[i] = (uint8_t)(usVal16.unsigned16 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 2));
      break;
    case UBX_CFG_U4:
    case UBX_CFG_E4:
    case UBX_CFG_X4:
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(((uint32_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_I4:
      unsignedSigned32 usVal32;
      usVal32.signed32 = (int32_t)value;
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(usVal32.unsigned32 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_R4:
      unsigned32float us32flt;
      us32flt.flt = (float)value;
      for (uint8_t i = 0; i < 4; i++)
        val[i] = (uint8_t)(us32flt.unsigned32 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 4));
      break;
    case UBX_CFG_U8:
    case UBX_CFG_X8:
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(((uint64_t)value) >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    case UBX_CFG_I8:
      unsignedSigned64 usVal64;
      usVal64.signed64 = (int64_t)value;
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(usVal64.unsigned64 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    case UBX_CFG_R8:
      unsigned64double us64dbl;
      us64dbl.dbl = (float)value;
      for (uint8_t i = 0; i < 8; i++)
        val[i] = (uint8_t)(us64dbl.unsigned64 >> (8 * i)); // Value
      return (addCfgValsetN(k1, val, 8));
      break;
    default:
      return false;
      break;
    }
    return false;
  }
  void softwareResetGNSSOnly();                                     // Controlled Software Reset (GNSS only) only restarts the GNSS tasks, without reinitializing the full system or reloading any stored configuration.
  bool setRXMPMPmessageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *)); // Use this if you want all of the PMP message (including sync chars, checksum, etc.) to push to a GNSS
  void cfgRst(uint8_t *data, uint8_t len);                          // Common method for CFG RST
  bool initPacketUBXRXMPMPmessage();    // Allocate RAM for packetUBXRXMPMPRaw and initialize it
  bool checkUblox(uint8_t requestedClass = 0, uint8_t requestedID = 0); // Checks module with user selected commType

  // Check if any callbacks need to be called
  void checkCallbacks(void);
 void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);             // Processes NMEA and UBX binary sentences one byte at a time
  bool autoLookup(uint8_t Class, uint8_t ID, uint16_t *maxSize = nullptr);
  bool logThisUBX(uint8_t UBX_CLASS, uint8_t UBX_ID);                      // Returns true if this UBX should be added to the logging buffer
  void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);          // Given a character, file it away into the uxb packet structure
  bool initPacketUBXNAVPVT();           // Allocate RAM for packetUBXNAVPVT and initialize it
  bool isNMEAHeaderValid();    // Return true if the six byte NMEA header appears valid. Used to set _signsOfLife
  bool isThisNMEAauto();                 // Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has RAM allocated for it)
  uint8_t *getNMEAWorkingLengthPtr();    // Get a pointer to the working copy length
  uint8_t *getNMEAWorkingNMEAPtr();      // Get a pointer to the working copy NMEA data
  uint8_t getNMEAMaxLength();            // Get the maximum length of this NMEA message
  bool logThisNMEA();          // Return true if we should log this NMEA message
  bool processThisNMEA();      // Return true if we should pass this NMEA message to processNMEA
  void processNMEA(char incoming) __attribute__((weak));                                                           // Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
  uint8_t *getNMEACompleteLengthPtr();   // Get a pointer to the complete copy length
  uint8_t *getNMEACompleteNMEAPtr();     // Get a pointer to the complete copy NMEA data
  nmeaAutomaticFlags *getNMEAFlagsPtr(); // Get a pointer to the flags
  bool doesThisNMEAHaveCallback();       // Do we need to copy the data into the callback copy?
  uint8_t *getNMEACallbackLengthPtr();   // Get a pointer to the callback copy length
  uint8_t *getNMEACallbackNMEAPtr();     // Get a pointer to the callback copy NMEA data
  bool storeFileBytes(uint8_t *theBytes, uint16_t numBytes);    // Add theBytes to the file buffer
  uint16_t rtcmBufferSpaceAvailable(void);                      // Check how much space is available in the buffer
  bool storeRTCMBytes(uint8_t *theBytes, uint16_t numBytes);    // Add theBytes to the buffer
  void extractRTCM1005(RTCM_1005_data_t *destination, uint8_t *source); // Extract RTCM 1005 from source into destination
  void extractRTCM1006(RTCM_1006_data_t *destination, uint8_t *source); // Extract RTCM 1006 from source into destination
  sfe_ublox_sentence_types_e processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter) __attribute__((weak)); // Monitor the incoming bytes for start and length bytes
  void addToChecksum(uint8_t incoming);    
  void printPacket(ubxPacket *packet, bool alwaysPrintPayload = false); // Useful for debugging
  void processUBXpacket(ubxPacket *msg);                                                                           // Once a packet has been received and validated, identify this packet's class/id and update internal flags
  uint32_t extractLong(ubxPacket *msg, uint16_t spotToStart);          // Combine four bytes from payload into long
  uint16_t extractInt(ubxPacket *msg, uint16_t spotToStart);           // Combine two bytes from payload into int
  bool storePacket(ubxPacket *msg);                             // Add a UBX packet to the file buffer
  bool setValSigned16(uint32_t key, int16_t value, uint8_t layer = VAL_LAYER_RAM_BBR, uint16_t maxWait = kUBLOXGNSSDefaultMaxWait);      // Sets the 16-bit value at a given group/id/size location
  uint8_t readBytes(uint8_t *data, uint8_t length);
  bool initStorageNMEA(); // Allocate RAM for incoming non-Auto NMEA messages and initialize it
  uint16_t fileBufferSpaceAvailable(void);                      // Check how much space is available in the buffer
  uint16_t fileBufferSpaceUsed(void);                           // Check how much space is used in the buffer
  void writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the file buffer
  uint16_t rtcmBufferSpaceUsed(void);                           // Check how much space is used in the buffer
  void writeToRTCMBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the buffer
  int64_t extractSignedBits(uint8_t *ptr, uint16_t start, uint16_t width);
  void processRTCM(uint8_t incoming) __attribute__((weak));                                                        // Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
  uint64_t extractUnsignedBits(uint8_t *ptr, uint16_t start, uint16_t width);
  int32_t extractSignedLong(ubxPacket *msg, uint16_t spotToStart);     // Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
  int16_t extractSignedInt(ubxPacket *msg, uint16_t spotToStart);
  bool pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool callProcessBuffer = true);
  void parseRTCM1005(uint8_t *dataBytes, size_t numDataBytes);
  void parseRTCM1006(uint8_t *dataBytes, size_t numDataBytes);
  bool lock(void) { return true; }





  typedef union
  {
    uint16_t unsigned16;
    int16_t signed16;
  } unsignedSigned16;

  typedef union
  {
    uint32_t unsigned32;
    int32_t signed32;
  } unsignedSigned32;

  typedef union
  {
    uint64_t unsigned64;
    int64_t signed64;
  } unsignedSigned64;

  typedef union
  {
    uint64_t unsigned64;
    double dbl;
  } unsigned64double;

private:
  bool _signsOfLife;
  size_t packetCfgPayloadSize = 0;
  bool _printDebug = false;                      // Flag to print the serial commands we are sending to the Serial port for debug
  bool _printLimitedDebug = false;               // Flag to print limited debug messages. Useful for I2C debugging or high navigation rates
  uint16_t fileBufferSize = 0;                                  // The size of the file buffer. This can be changed by calling setFileBufferSize _before_ .begin
  uint16_t fileBufferHead = 0;                                  // The incoming byte is written into the file buffer at this location
  uint16_t fileBufferTail = 0;                                  // The next byte to be read from the buffer will be read from this location
  uint8_t *ubxFileBuffer = nullptr;                             // Pointer to the file buffer. RAM is allocated for this if required in .begin
  uint8_t *payloadCfg = nullptr;
  uint16_t rtcmBufferSize = 0;                                  // The size of the RTCM buffer. This can be changed by calling setRTCMBufferSize _before_ .begin
  uint8_t *rtcmBuffer = nullptr;                                // Pointer to the RTCM buffer. RAM is allocated for this if required in .begin
  uint16_t rtcmBufferHead = 0;                                  // The incoming byte is written into the buffer at this location
  uint16_t rtcmBufferTail = 0;                                  // The next byte to be read from the buffer will be read from this location
  uint8_t payloadBuf[2];           // Temporary buffer used to screen incoming packets or dump unrequested packets
  uint8_t payloadAck[2];           // Holds the requested ACK/NACK
  // Set the max number of bytes set in a given I2C transaction
  uint8_t i2cTransactionSize = 32; // Default to ATmega328 limit
  uint8_t i2cPollingWait = 100;    // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
  unsigned long lastCheck = 0;
  uint8_t nmeaAddressField[6]; // NMEA Address Field - includes the start character (*)

  // Flag to indicate if currentSentence should be reset on a (I2C) bus error
  bool _resetCurrentSentenceOnBusError = true;
  moduleSWVersion_t *moduleSWVersion = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  uint8_t i2cPollingWaitNAV = 100; // We need to record the desired polling rate for standard nav messages
  uint8_t i2cPollingWaitHNR = 100; // and for HNR too so we can set i2cPollingWait to the lower of the two
  uint8_t *payloadAuto = nullptr;

  // Keep track of how many keys have been added to CfgValset
  uint8_t _numCfgKeys = 0;

  // Send the current CFG_VALSET message when packetCfg has less than this many bytes available
  size_t _autoSendAtSpaceRemaining = 0;


  // Init the packet structures and init them with pointers to the payloadAck, payloadCfg, payloadBuf and payloadAuto arrays
  ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetAuto = {0, 0, 0, 0, 0, payloadAuto, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  enum commTypes
  {
    COMM_TYPE_I2C = 0,
    COMM_TYPE_SERIAL,
    COMM_TYPE_SPI
  } _commType = COMM_TYPE_I2C; // Controls which port we look to for incoming bytes
 
  // I2C bus class
  SfeI2C _i2cBus;
  GNSSDeviceBus *_sfeBus;
  I2C_INTERFACE  *_i2cPort;
  uint8_t _address;
  volatile bool checkCallbacksReentrant = false;
  uint16_t ubxFrameCounter; // Count all UBX frame bytes. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]
  // Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
  bool ignoreThisPayload = false;
  sfe_ublox_packet_buffer_e activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
  int8_t nmeaByteCounter; // Count all NMEA message bytes.
  uint16_t rtcmFrameCounter = 0; // Tracks the type of incoming byte inside RTCM frame
  uint8_t rollingChecksumA; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
  uint8_t rollingChecksumB; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
  // UBX logging
  sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_head = nullptr; // Linked list of which messages to log
  // UBX_NAV_SAT_MAX_LEN is just > UBX_RXM_RAWX_MAX_LEN
  const uint16_t SFE_UBX_MAX_LENGTH = UBX_NAV_SAT_MAX_LEN;
  NMEA_STORAGE_t *_storageNMEA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  int8_t maxNMEAByteCount = SFE_UBLOX_MAX_NMEA_BYTE_COUNT;
  RTCM_1005_t *storageRTCM1005 = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessar
  uint16_t fileBufferMaxAvail = 0;                              // The maximum number of bytes the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
  size_t spiBufferSize = SFE_UBLOX_SPI_BUFFER_DEFAULT_SIZE;    // Default size of the SPI buffer

  const char *statusString(sfe_ublox_status_e stat); // Pretty print the return value









#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  RTCM_FRAME_t *_storageRTCM = nullptr;              // Pointer to struct. RAM will be allocated for this if/when necessary
  void crc24q(uint8_t incoming, uint32_t *checksum); // Add incoming to checksum as per CRC-24Q
#endif

  // RTCM logging
  sfe_ublox_rtcm_filtering_t _logRTCM; // Flags to indicate which NMEA messages should be added to the file buffer for logging


  NMEA_GPGGA_t *storageNMEAGPGGA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNGGA_t *storageNMEAGNGGA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPVTG_t *storageNMEAGPVTG = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNVTG_t *storageNMEAGNVTG = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPRMC_t *storageNMEAGPRMC = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNRMC_t *storageNMEAGNRMC = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPZDA_t *storageNMEAGPZDA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNZDA_t *storageNMEAGNZDA = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  // NMEA logging / Auto support
  sfe_ublox_nmea_filtering_t _logNMEA;     // Flags to indicate which NMEA messages should be added to the file buffer for logging
  sfe_ublox_nmea_filtering_t _processNMEA; // Flags to indicate which NMEA messages should be passed to processNMEA


  UBX_NAV_POSECEF_t *packetUBXNAVPOSECEF = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_STATUS_t *packetUBXNAVSTATUS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_DOP_t *packetUBXNAVDOP = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_EOE_t *packetUBXNAVEOE = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ATT_t *packetUBXNAVATT = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVT_t *packetUBXNAVPVT = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ODO_t *packetUBXNAVODO = nullptr;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMEUTC_t *packetUBXNAVTIMEUTC = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELECEF_t *packetUBXNAVVELECEF = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELNED_t *packetUBXNAVVELNED = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSECEF_t *packetUBXNAVHPPOSECEF = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSLLH_t *packetUBXNAVHPPOSLLH = nullptr;   // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVAT_t *packetUBXNAVPVAT = nullptr;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_CLOCK_t *packetUBXNAVCLOCK = nullptr;         // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMELS_t *packetUBXNAVTIMELS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SVIN_t *packetUBXNAVSVIN = nullptr;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_RELPOSNED_t *packetUBXNAVRELPOSNED = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_AOPSTATUS_t *packetUBXNAVAOPSTATUS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary




#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  UBX_NAV_SAT_t *packetUBXNAVSAT = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SIG_t *packetUBXNAVSIG = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_PMP_t *packetUBXRXMPMP = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_PMP_message_t *packetUBXRXMPMPmessage = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_QZSSL6_message_t *packetUBXRXMQZSSL6message = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_COR_t *packetUBXRXMCOR = nullptr;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_SFRBX_t *packetUBXRXMSFRBX = nullptr;                  // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_RAWX_t *packetUBXRXMRAWX = nullptr;                    // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_MEASX_t *packetUBXRXMMEASX = nullptr;                  // Pointer to struct. RAM will be allocated for this if/when necessary
#endif



  UBX_TIM_TM2_t *packetUBXTIMTM2 = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_TIM_TP_t *packetUBXTIMTP = nullptr;   // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_MON_HW_t *packetUBXMONHW = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_ESF
  UBX_ESF_ALG_t *packetUBXESFALG = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_INS_t *packetUBXESFINS = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_MEAS_t *packetUBXESFMEAS = nullptr;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_RAW_t *packetUBXESFRAW = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_STATUS_t *packetUBXESFSTATUS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif
#ifndef SFE_UBLOX_DISABLE_HNR
  UBX_HNR_PVT_t *packetUBXHNRPVT = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_ATT_t *packetUBXHNRATT = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_INS_t *packetUBXHNRINS = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

  UBX_MGA_ACK_DATA0_t *packetUBXMGAACK = nullptr; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_MGA_DBD_t *packetUBXMGADBD = nullptr;       // Pointer to struct. RAM will be allocated for this if/when necessary


};







#endif