#include"GNSS.hpp"




//UBX_NAV_PVT_data_t


// Data is stored inside a UBX_NAV_PVT_data_t inside of the UBX_NAV_PVT_t in the SFE_UBLOX_GNSS instance.
// We have 3 options:
// 1. Point-perfect for only the ZED. We need WiFi for it to work.
// 2. Point-perfect for the ZED with L-band.
// 3. The usual ZED alone, no extra features.
// For the first option, look at the example "point_perfect_zed". We have a list of objects we need to set in order to use and set up the automatic callback. Everything is explained in the example.
// For option 2, it's the same process. We need to set some values in the GNSS_DEF and set it up the same way as the example "point_perfect_neo", then set up an L-band instance for the NEO.
// For the last one, we just need to set up the ZED, then call myGNSS.getHPPOSLLH(). The example "zed_high_acc" is essential. the data is present in a UBX_NAV_HPPOSLLH_t type of sturct











// ***** HPPOSLLH Helper Functions

uint32_t SFE_UBLOX_GNSS::getTimeOfWeekFromHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.iTOW);
}

int32_t SFE_UBLOX_GNSS::getHighResLongitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lon);
}

int32_t SFE_UBLOX_GNSS::getHighResLatitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lat);
}

int32_t SFE_UBLOX_GNSS::getElipsoid(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.height);
}

int32_t SFE_UBLOX_GNSS::getMeanSeaLevel(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSL);
}

int8_t SFE_UBLOX_GNSS::getHighResLongitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lonHp);
}

int8_t SFE_UBLOX_GNSS::getHighResLatitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.latHp);
}

int8_t SFE_UBLOX_GNSS::getElipsoidHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.heightHp);
}

int8_t SFE_UBLOX_GNSS::getMeanSeaLevelHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSLHp);
}

uint32_t SFE_UBLOX_GNSS::getHorizontalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hAcc);
}

uint32_t SFE_UBLOX_GNSS::getVerticalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.vAcc);
}















void SFE_UBLOX_GNSS::parseRTCM1005(uint8_t *dataBytes, size_t numDataBytes)
{
  // This is called from inside pushRawData. It thoroughly examines dataBytes and will copy any RTCM 1005 messages it finds into storage.
  // It keeps a local copy of the data so it does not matter if the message spans multiple calls to pushRawData.

  static uint8_t rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 6];
  static uint8_t bytesStored;

  enum parse1005states
  {
    waitingForD3,
    expecting00,
    expecting13,
    expecting3E,
    expectingDn,
    storingBytes,
  };
  static parse1005states parse1005state = waitingForD3;

  for (size_t i = 0; i < numDataBytes; i++) // Step through each byte
  {
    switch (parse1005state)
    {
    case waitingForD3:
      if (*(dataBytes + i) == 0xD3)
      {
        rtcm1005store[0] = 0xD3;
        parse1005state = expecting00;
      }
      break;
    case expecting00:
      if (*(dataBytes + i) == 0x00)
      {
        rtcm1005store[1] = 0x00;
        parse1005state = expecting13;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expecting13:
      if (*(dataBytes + i) == 0x13)
      {
        rtcm1005store[2] = 0x13;
        parse1005state = expecting3E;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expecting3E:
      if (*(dataBytes + i) == 0x3E)
      {
        rtcm1005store[3] = 0x3E;
        parse1005state = expectingDn;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case expectingDn:
      if (((*(dataBytes + i)) & 0xF0) == 0xD0)
      {
        rtcm1005store[4] = *(dataBytes + i);
        parse1005state = storingBytes;
        bytesStored = 5;
      }
      else
      {
        parse1005state = waitingForD3;
      }
      break;
    case storingBytes:
      rtcm1005store[bytesStored++] = *(dataBytes + i);
      if (bytesStored == RTCM_1005_MSG_LEN_BYTES + 6) // All data received?
      {
        parse1005state = waitingForD3;
        uint32_t checksum = 0;
        for (size_t j = 0; j < (RTCM_1005_MSG_LEN_BYTES + 3); j++)
          crc24q(rtcm1005store[j], &checksum);
        if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 3] == ((checksum >> 16) & 0xFF)) // Check the checksum
          if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 4] == ((checksum >> 8) & 0xFF))
            if (rtcm1005store[RTCM_1005_MSG_LEN_BYTES + 5] == (checksum & 0xFF))
            {
              extractRTCM1005(&rtcmInputStorage.rtcm1005, &rtcm1005store[3]);
              rtcmInputStorage.flags.bits.dataValid1005 = 1;
              rtcmInputStorage.flags.bits.dataRead1005 = 0;
              return; // Return now - to avoid processing the remainder of the data
            }
      }
      break;
    }
  }
}

void SFE_UBLOX_GNSS::parseRTCM1006(uint8_t *dataBytes, size_t numDataBytes)
{
  // This is called from inside pushRawData. It thoroughly examines dataBytes and will copy any RTCM 1006 messages it finds into storage.
  // It keeps a local copy of the data so it does not matter if the message spans multiple calls to pushRawData.

  static uint8_t rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 6];
  static uint8_t bytesStored;

  enum parse1006states
  {
    waitingForD3,
    expecting00,
    expecting15,
    expecting3E,
    expectingEn,
    storingBytes,
  };
  static parse1006states parse1006state = waitingForD3;

  for (size_t i = 0; i < numDataBytes; i++) // Step through each byte
  {
    switch (parse1006state)
    {
    case waitingForD3:
      if (*(dataBytes + i) == 0xD3)
      {
        rtcm1006store[0] = 0xD3;
        parse1006state = expecting00;
      }
      break;
    case expecting00:
      if (*(dataBytes + i) == 0x00)
      {
        rtcm1006store[1] = 0x00;
        parse1006state = expecting15;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expecting15:
      if (*(dataBytes + i) == 0x15)
      {
        rtcm1006store[2] = 0x15;
        parse1006state = expecting3E;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expecting3E:
      if (*(dataBytes + i) == 0x3E)
      {
        rtcm1006store[3] = 0x3E;
        parse1006state = expectingEn;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case expectingEn:
      if (((*(dataBytes + i)) & 0xF0) == 0xE0)
      {
        rtcm1006store[4] = *(dataBytes + i);
        parse1006state = storingBytes;
        bytesStored = 5;
      }
      else
      {
        parse1006state = waitingForD3;
      }
      break;
    case storingBytes:
      rtcm1006store[bytesStored++] = *(dataBytes + i);
      if (bytesStored == RTCM_1006_MSG_LEN_BYTES + 6) // All data received?
      {
        parse1006state = waitingForD3;
        uint32_t checksum = 0;
        for (size_t j = 0; j < (RTCM_1006_MSG_LEN_BYTES + 3); j++)
          crc24q(rtcm1006store[j], &checksum);

        if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 3] == ((checksum >> 16) & 0xFF)) // Check the checksum
          if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 4] == ((checksum >> 8) & 0xFF))
            if (rtcm1006store[RTCM_1006_MSG_LEN_BYTES + 5] == (checksum & 0xFF))
            {
              extractRTCM1006(&rtcmInputStorage.rtcm1006, &rtcm1006store[3]);
              rtcmInputStorage.flags.bits.dataValid1006 = 1;
              rtcmInputStorage.flags.bits.dataRead1006 = 0;
              return; // Return now - to avoid processing the remainder of the data
            }
      }
      break;
    }
  }
}


// Push (e.g.) RTCM data directly to the module
// Returns true if all numDataBytes were pushed successfully
// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
bool SFE_UBLOX_GNSS::pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool callProcessBuffer)
{
  // Return now if numDataBytes is zero
  if (numDataBytes == 0)
    return false; // Indicate to the user that there was no data to push

#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
  parseRTCM1005(dataBytes, numDataBytes);
  parseRTCM1006(dataBytes, numDataBytes);
#endif

  if (!lock())
    return false;

  bool ok = false;
  if (_commType == COMM_TYPE_I2C)
  {
    // We can not write a single data byte to I2C as it would look like the address of a random read.
    // If numDataBytes is 1, we should probably just reject the data and return false.
    // But we'll be nice and store the byte until the next time pushRawData is called.

    // Storage just in case the user tries to push a single byte using pushRawBytes
    static bool _pushSingleByte = false;
    static uint8_t _pushThisSingleByte = 0;

    if ((numDataBytes == 1) && (_pushSingleByte == false))
    {
      _pushThisSingleByte = *dataBytes;
      _pushSingleByte = true;
      ok = false; // Indicate to the user that their data has not been pushed yet
    }
    else
    {

      // I2C: split the data up into packets of i2cTransactionSize
      size_t bytesLeftToWrite = numDataBytes;
      size_t bytesWrittenTotal = 0;

      if (_pushSingleByte == true) // Increment bytesLeftToWrite if we have a single byte waiting to be pushed
        bytesLeftToWrite++;

      while (bytesLeftToWrite > 0)
      {
        size_t bytesToWrite; // Limit bytesToWrite to i2cTransactionSize

        if (bytesLeftToWrite > i2cTransactionSize)
          bytesToWrite = i2cTransactionSize;
        else
          bytesToWrite = bytesLeftToWrite;

        // If there would be one byte left to be written next time, send one byte less now
        if ((bytesLeftToWrite - bytesToWrite) == 1)
          bytesToWrite--;

        size_t bytesWritten = 0;

        if (_pushSingleByte == true)
        {
          uint8_t buf[i2cTransactionSize];

          buf[0] = _pushThisSingleByte;

          for (uint16_t x = 1; x < bytesToWrite; x++)
            buf[x] = dataBytes[x - 1];

          bytesWritten += writeBytes(buf, bytesToWrite); // Write the bytes
          dataBytes += bytesToWrite - 1;                 // Point to fresh data
          _pushSingleByte = false;                       // Clear the flag
        }
        else
        {
          bytesWritten += writeBytes(dataBytes, bytesToWrite); // Write the bytes
          dataBytes += bytesToWrite;                           // Point to fresh data
        }

        bytesWrittenTotal += bytesWritten; // Update the totals
        bytesLeftToWrite -= bytesToWrite;
      }

      ok = (bytesWrittenTotal == numDataBytes); // Return true if the correct number of bytes were written
    }
  }

  return ok;
}


void SFE_UBLOX_GNSS::extractRTCM1006(RTCM_1006_data_t *destination, uint8_t *source)
{
  destination->MessageNumber = extractUnsignedBits(source, 0, 12);
  destination->ReferenceStationID = extractUnsignedBits(source, 12, 12);
  destination->ITRFRealizationYear = extractUnsignedBits(source, 24, 6);
  destination->GPSIndicator = extractUnsignedBits(source, 30, 1);
  destination->GLONASSIndicator = extractUnsignedBits(source, 31, 1);
  destination->GalileoIndicator = extractUnsignedBits(source, 32, 1);
  destination->ReferenceStationIndicator = extractUnsignedBits(source, 33, 1);
  destination->AntennaReferencePointECEFX = extractSignedBits(source, 34, 38);
  destination->SingleReceiverOscillatorIndicator = extractUnsignedBits(source, 72, 1);
  destination->Reserved = extractUnsignedBits(source, 73, 1);
  destination->AntennaReferencePointECEFY = extractSignedBits(source, 74, 38);
  destination->QuarterCycleIndicator = extractUnsignedBits(source, 112, 2);
  destination->AntennaReferencePointECEFZ = extractSignedBits(source, 114, 38);
  destination->AntennaHeight = extractUnsignedBits(source, 152, 16);
}



const char *SFE_UBLOX_GNSS::statusString(sfe_ublox_status_e stat)
{
  switch (stat)
  {
  case SFE_UBLOX_STATUS_SUCCESS:
    return "Success";
    break;
  case SFE_UBLOX_STATUS_FAIL:
    return "General Failure";
    break;
  case SFE_UBLOX_STATUS_CRC_FAIL:
    return "CRC Fail";
    break;
  case SFE_UBLOX_STATUS_TIMEOUT:
    return "Timeout";
    break;
  case SFE_UBLOX_STATUS_COMMAND_NACK:
    return "Command not acknowledged (NACK)";
    break;
  case SFE_UBLOX_STATUS_OUT_OF_RANGE:
    return "Out of range";
    break;
  case SFE_UBLOX_STATUS_INVALID_ARG:
    return "Invalid Arg";
    break;
  case SFE_UBLOX_STATUS_INVALID_OPERATION:
    return "Invalid operation";
    break;
  case SFE_UBLOX_STATUS_MEM_ERR:
    return "Memory Error";
    break;
  case SFE_UBLOX_STATUS_HW_ERR:
    return "Hardware Error";
    break;
  case SFE_UBLOX_STATUS_DATA_SENT:
    return "Data Sent";
    break;
  case SFE_UBLOX_STATUS_DATA_RECEIVED:
    return "Data Received";
    break;
  case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
    return "I2C Comm Failure";
    break;
  case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
    return "Data Packet Overwritten";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

//done
// PRIVATE: Returns true if this UBX should be added to the logging buffer
bool SFE_UBLOX_GNSS::logThisUBX(uint8_t UBX_CLASS, uint8_t UBX_ID)
{
  // If the list is empty
  if (sfe_ublox_ubx_logging_list_head == nullptr)
    return false;

  // Step through the list, check for CLASS + ID
  sfe_ublox_ubx_logging_list_t *sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_head;
  bool keepGoing = true;
  while (keepGoing)
  {
    if ((sfe_ublox_ubx_logging_list_ptr->UBX_CLASS == UBX_CLASS) // Check for a match
        && (sfe_ublox_ubx_logging_list_ptr->UBX_ID == UBX_ID))
    {
      return sfe_ublox_ubx_logging_list_ptr->enable;
    }

    if (sfe_ublox_ubx_logging_list_ptr->next == nullptr)
      keepGoing = false;
    else
      sfe_ublox_ubx_logging_list_ptr = sfe_ublox_ubx_logging_list_ptr->next;
  }

  return false;
}


void SFE_UBLOX_GNSS::crc24q(uint8_t incoming, uint32_t *checksum)
{
  uint32_t crc = *checksum; // Seed is 0

  crc ^= ((uint32_t)incoming) << 16; // XOR-in incoming

  for (uint8_t i = 0; i < 8; i++)
  {
    crc <<= 1;
    if (crc & 0x1000000)
      // CRC-24Q Polynomial:
      // gi = 1 for i = 0, 1, 3, 4, 5, 6, 7, 10, 11, 14, 17, 18, 23, 24
      // 0b 1 1000 0110 0100 1100 1111 1011
      crc ^= 0x1864CFB; // CRC-24Q
  }

  *checksum = crc & 0xFFFFFF;
}




// ***** NAV HPPOSLLH automatic support

bool SFE_UBLOX_GNSS::getHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == nullptr)
    initPacketUBXNAVHPPOSLLH();        // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Don't overwrite the requested Class and ID
    return packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && !packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSLLH;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}



bool SFE_UBLOX_GNSS::getVal16(uint32_t key, uint16_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractInt(&packetCfg, 8);
  return result;
}
uint16_t SFE_UBLOX_GNSS::getVal16(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint16_t result = 0;
  getVal16(key, &result, layer, maxWait);
  return result;
}
// PRIVATE: Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVHPPOSLLH()
{
  packetUBXNAVHPPOSLLH = new UBX_NAV_HPPOSLLH_t; // Allocate RAM for the main struct
  if (packetUBXNAVHPPOSLLH == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initPacketUBXNAVHPPOSLLH: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVHPPOSLLH->automaticFlags.flags.all = 0;
  packetUBXNAVHPPOSLLH->callbackPointerPtr = nullptr;
  packetUBXNAVHPPOSLLH->callbackData = nullptr;
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0;
  return (true);
}


// Get the rate at which the module is outputting nav solutions
// Note: if the measurementRate (which is actually a period) is less than 1000ms, this will return zero
bool SFE_UBLOX_GNSS::getNavigationFrequency(uint8_t *navFreq, uint8_t layer, uint16_t maxWait)
{
  uint16_t measurementRate = 0;

  bool result = getVal16(UBLOX_CFG_RATE_MEAS, &measurementRate, layer, maxWait);

  if ((result) && (measurementRate > 0))
    *navFreq = 1000 / measurementRate; // This may return an int when it's a float, but I'd rather not return 4 bytes

  return result;
}
uint8_t SFE_UBLOX_GNSS::getNavigationFrequency(uint8_t layer, uint16_t maxWait) // Unsafe overload...
{
  uint8_t navFreq = 0;

  getNavigationFrequency(&navFreq, layer, maxWait);

  return navFreq;
}
// For I2C, ping the _address
// Not Applicable for SPI and Serial
bool SFE_UBLOX_GNSS::ping()
{

  bool ok = _sfeBus->ping();
  return ok;
}


// Processes NMEA, RTCM and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void SFE_UBLOX_GNSS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // Update storedClass and storedID if either requestedClass or requestedID is non-zero,
  // otherwise leave unchanged. This allows calls of checkUblox() (which defaults to checkUblox(0,0))
  // by other threads without overwriting the requested / expected Class and ID.
  volatile static uint8_t storedClass = 0;
  volatile static uint8_t storedID = 0;
  if (requestedClass || requestedID) // If either is non-zero, store the requested Class and ID
  {
    storedClass = requestedClass;
    storedID = requestedID;
  }

 // Serial.write(incoming); // Echo this byte to the serial port

  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) || (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA))
  {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a stored class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    }
    else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    }
    else
    {
      // This character is unknown or we missed the previous start of a sentence
      // Or it could be a 0xFF from a SPI transaction
    }
  }

  uint16_t maxPayload = 0;

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX)
  {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        bool logBecauseAuto = autoLookup(packetBuf.cls, packetBuf.id, &maxPayload);
        bool logBecauseEnabled = logThisUBX(packetBuf.cls, packetBuf.id);

        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == storedClass) && (packetBuf.id == storedID))
        {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own storage defined
        else if (logBecauseAuto || logBecauseEnabled)
        {
          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
          // reception is complete.
          if (logBecauseAuto && (maxPayload == 0))
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              Serial.print(F("process: autoLookup returned ZERO maxPayload!! Class: 0x"));
              Serial.print(packetBuf.cls, HEX);
              Serial.print(F(" ID: 0x"));
              Serial.println(packetBuf.id, HEX);
            }
#endif
          }
          if (payloadAuto != nullptr) // Check if memory is already allocated - this should be impossible!
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              Serial.println(F("process: memory is already allocated for payloadAuto! Deleting..."));
            }
#endif
            delete[] payloadAuto; // Created with new[]
            payloadAuto = nullptr;
            packetAuto.payload = payloadAuto;
          }
          if ((!logBecauseAuto) && (logBecauseEnabled))
            maxPayload = SFE_UBX_MAX_LENGTH;
          payloadAuto = new uint8_t[maxPayload]; // Allocate RAM for payloadAuto
          packetAuto.payload = payloadAuto;
          if (payloadAuto == nullptr) // Check if the alloc failed
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              Serial.print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
              Serial.print(packetBuf.cls, HEX);
              Serial.print(F(" ID: 0x"));
              Serial.println(packetBuf.id, HEX);
              Serial.println(F("process: \"automatic\" message could overwrite data"));
            }
#endif
            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
          }
          else
          {
            // The RAM allocation was successful so we start diverting data into packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if (_printDebug == true)
            {
              Serial.print(F("process: incoming \"automatic\" message: Class: 0x"));
              Serial.print(packetBuf.cls, HEX);
              Serial.print(F(" ID: 0x"));
              Serial.print(packetBuf.id, HEX);
              Serial.print(F(" logBecauseAuto:"));
              Serial.print(logBecauseAuto);
              Serial.print(F(" logBecauseEnabled:"));
              Serial.println(logBecauseEnabled);
            }
#endif
          }
        }
        else
        {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          Serial.print(F("process: ZERO LENGTH packet received: Class: 0x"));
          Serial.print(packetBuf.cls, HEX);
          Serial.print(F(" ID: 0x"));
          Serial.println(packetBuf.id, HEX);
        }
#endif
        // If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == storedClass)           // and if the class matches
          && (packetBuf.payload[1] == storedID))             // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.print(F("process: ACK received with .len != 2: Class: 0x"));
            Serial.print(packetBuf.payload[0], HEX);
            Serial.print(F(" ID: 0x"));
            Serial.print(packetBuf.payload[1], HEX);
            Serial.print(F(" len: "));
            Serial.println(packetBuf.len);
          }
#endif
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, storedClass, storedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, storedClass, storedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, storedClass, storedID);

    // If user has assigned an output port then pipe the characters there,
    // but only if the port is different (otherwise we'll output each character twice!)
    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA) // Process incoming NMEA mesages. Selectively log if desired.
  {
    if ((nmeaByteCounter == 0) && (incoming != '$'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset. (Almost certainly redundant!)
    }
    else if ((nmeaByteCounter == 1) && (incoming != 'G'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    }
    else if ((nmeaByteCounter >= 0) && (nmeaByteCounter <= 5))
    {
      nmeaAddressField[nmeaByteCounter] = incoming; // Store the start character and NMEA address field
    }

    if (nmeaByteCounter == 5)
    {
      if (!_signsOfLife) // If _signsOfLife is not already true, set _signsOfLife to true if the NMEA header is valid
      {
        _signsOfLife = isNMEAHeaderValid();
      }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      // Check if we have automatic storage for this message
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        *lengthPtr = 6;                           // Set the working copy length
        memset(nmeaPtr, 0, nmeaMaxLength);        // Clear the working copy
        memcpy(nmeaPtr, &nmeaAddressField[0], 6); // Copy the start character and address field into the working copy
      }
      else
#endif
      {
        // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        // {
        //   Serial.println(F("process: non-auto NMEA message"));
        // }
      }

      // We've just received the end of the address field. Check if it is selected for logging
      if (logThisNMEA())
      {
        memcpy(_storageNMEA->data, &nmeaAddressField[0], 6); // Add start character and address field to the storage
        _storageNMEA->length = 6;
      }
      // Check if it should be passed to processNMEA
      if (processThisNMEA())
      {
        for (uint8_t i = 0; i < 6; i++)
        {
          processNMEA(nmeaAddressField[i]); // Process the start character and address field
          // If user has assigned an output port then pipe the characters there,
          // but only if the port is different (otherwise we'll output each character twice!)
        }
      }
    }

    if ((nmeaByteCounter > 5) || (nmeaByteCounter < 0)) // Should we add incoming to the file buffer and/or pass it to processNMEA?
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        if (*lengthPtr < nmeaMaxLength)
        {
          *(nmeaPtr + *lengthPtr) = incoming; // Store the character
          *lengthPtr = *lengthPtr + 1;        // Increment the length
          if (*lengthPtr == nmeaMaxLength)
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              Serial.println(F("process: NMEA buffer is full!"));
            }
          }
        }
      }
#endif
      if (logThisNMEA())
      {
        // This check is probably redundant.
        // currentSentence is set to SFE_UBLOX_SENTENCE_TYPE_NONE below if nmeaByteCounter == maxNMEAByteCount
        if (_storageNMEA->length < maxNMEAByteCount) // Check we have room for it
        {
          _storageNMEA->data[_storageNMEA->length] = incoming; // Store the byte
          _storageNMEA->length = _storageNMEA->length + 1;
        }
      }
      if (processThisNMEA())
      {
        processNMEA(incoming); // Pass incoming to processNMEA
        // If user has assigned an output port then pipe the characters there,
        // but only if the port is different (otherwise we'll output each character twice!)
      }
    }

    if (incoming == '*')
      nmeaByteCounter = -5; // We are expecting * plus two checksum bytes plus CR and LF

    nmeaByteCounter++; // Increment the byte counter

    if (nmeaByteCounter == maxNMEAByteCount)          // Check if we have processed too many bytes
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.

    if (nmeaByteCounter == 0) // Check if we are done
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *workingLengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *workingNMEAPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();

        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        uint8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < (nmeaMaxLength - 1)) && (charsChecked < ((*workingLengthPtr) - 4)) && (thisChar != '*'))
        {
          thisChar = *(workingNMEAPtr + charsChecked); // Get a char from the working copy
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == *(workingNMEAPtr + charsChecked)) && (expectedChecksum2 == *(workingNMEAPtr + charsChecked + 1)))
          {
            uint8_t *completeLengthPtr = getNMEACompleteLengthPtr();    // Get a pointer to the complete copy length
            uint8_t *completeNMEAPtr = getNMEACompleteNMEAPtr();        // Get a pointer to the complete copy NMEA data
            memset(completeNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous complete copy
            memcpy(completeNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the complete copy
            *completeLengthPtr = *workingLengthPtr;                     // Update the length
            nmeaAutomaticFlags *flagsPtr = getNMEAFlagsPtr();           // Get a pointer to the flags
            nmeaAutomaticFlags flagsCopy = *flagsPtr;
            flagsCopy.flags.bits.completeCopyValid = 1; // Set the complete copy valid flag
            flagsCopy.flags.bits.completeCopyRead = 0;  // Clear the complete copy read flag
            *flagsPtr = flagsCopy;                      // Update the flags
            // Callback
            if (doesThisNMEAHaveCallback()) // Do we need to copy the data into the callback copy?
            {
              if (flagsCopy.flags.bits.callbackCopyValid == 0) // Has the callback copy valid flag been cleared (by checkCallbacks)
              {
                uint8_t *callbackLengthPtr = getNMEACallbackLengthPtr();    // Get a pointer to the callback copy length
                uint8_t *callbackNMEAPtr = getNMEACallbackNMEAPtr();        // Get a pointer to the callback copy NMEA data
                memset(callbackNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous callback copy
                memcpy(callbackNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the callback copy
                *callbackLengthPtr = *workingLengthPtr;                     // Update the length
                flagsCopy.flags.bits.callbackCopyValid = 1;                 // Set the callback copy valid flag
                *flagsPtr = flagsCopy;                                      // Update the flags
              }
            }
          }
          else
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              Serial.print(F("process: NMEA checksum fail (2)! Expected "));
              Serial.write(expectedChecksum1);
              Serial.write(expectedChecksum2);
              Serial.print(F(" Got "));
              Serial.write(*(workingNMEAPtr + charsChecked));
              Serial.write(*(workingNMEAPtr + charsChecked + 1));
              Serial.println();
            }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.println(F("process: NMEA checksum fail (1)!"));
          }
        }
      }
#endif
      if (logThisNMEA())
      {
        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        int8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < maxNMEAByteCount) && (charsChecked < (_storageNMEA->length - 4)) && (thisChar != '*'))
        {
          thisChar = _storageNMEA->data[charsChecked]; // Get a char from the storage
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == _storageNMEA->data[charsChecked]) && (expectedChecksum2 == _storageNMEA->data[charsChecked + 1]))
          {
            storeFileBytes(_storageNMEA->data, _storageNMEA->length); // Add NMEA to the file buffer
          }
          else if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.println(F("process: _storageNMEA checksum fail!"));
          }
        }
      }
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // All done!
    }
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_RTCM)
  {

    // RTCM Logging
#ifndef SFE_UBLOX_DISABLE_RTCM_LOGGING
    if (_storageRTCM != nullptr) // Check if RTCM logging storage exists
    {
      if (rtcmFrameCounter == 0)
      {
        _storageRTCM->dataMessage[0] = incoming;
        _storageRTCM->rollingChecksum = 0; // Initialize the checksum. Seed is 0x000000
      }
      else if (rtcmFrameCounter == 1)
      {
        _storageRTCM->dataMessage[1] = incoming;
        _storageRTCM->messageLength = (uint16_t)(incoming & 0x03) << 8;
      }
      else if (rtcmFrameCounter == 2)
      {
        _storageRTCM->dataMessage[2] = incoming;
        _storageRTCM->messageLength |= incoming;
      }

      // Store the mesage data (and CRC) - now that the message length is known
      if ((rtcmFrameCounter >= 3) && (rtcmFrameCounter < (_storageRTCM->messageLength + 6)) && (rtcmFrameCounter < (3 + SFE_UBLOX_MAX_RTCM_MSG_LEN + 3)))
        _storageRTCM->dataMessage[rtcmFrameCounter] = incoming;

      // Add incoming header and data bytes to the checksum
      if ((rtcmFrameCounter < 3) || ((rtcmFrameCounter >= 3) && (rtcmFrameCounter < (_storageRTCM->messageLength + 3))))
        crc24q(incoming, &_storageRTCM->rollingChecksum);

      // Check if all bytes have been received
      if ((rtcmFrameCounter >= 3) && (rtcmFrameCounter == _storageRTCM->messageLength + 5))
      {
        uint32_t expectedChecksum = _storageRTCM->dataMessage[_storageRTCM->messageLength + 3];
        expectedChecksum <<= 8;
        expectedChecksum |= _storageRTCM->dataMessage[_storageRTCM->messageLength + 4];
        expectedChecksum <<= 8;
        expectedChecksum |= _storageRTCM->dataMessage[_storageRTCM->messageLength + 5];

        if (expectedChecksum == _storageRTCM->rollingChecksum) // Does the checksum match?
        {
          // Extract the message type and check if it should be logged

          // Extract the message number from the first 12 bits
          uint16_t messageType = ((uint16_t)_storageRTCM->dataMessage[3]) << 4;
          messageType |= _storageRTCM->dataMessage[4] >> 4;
          uint16_t messageSubType = ((uint16_t)_storageRTCM->dataMessage[4] & 0x0F) << 8;
          messageSubType |= _storageRTCM->dataMessage[5];
          bool logThisRTCM = false;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if (_printDebug == true)
          {
            Serial.print(F("process: valid RTCM message type: "));
            Serial.print(messageType);
            if (messageType == 4072)
            {
              Serial.print(F("_"));
              Serial.print(messageSubType);
            }
            Serial.println(F(""));
          }
#endif

          if (!logThisRTCM)
            logThisRTCM = (messageType == 1001) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1001 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1002) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1002 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1003) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1003 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1004) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1004 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1005) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1005 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1006) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1006 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1007) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1007 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1009) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1009 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1010) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1010 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1011) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1011 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1012) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1012 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1033) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1033 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1074) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1074 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1075) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1075 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1077) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1077 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1084) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1084 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1085) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1085 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1087) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1087 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1094) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1094 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1095) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1095 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1097) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1097 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1124) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1124 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1125) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1125 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1127) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1127 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 1230) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE1230 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 4072) && (messageSubType == 0) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE4072_0 == 1));
          if (!logThisRTCM)
            logThisRTCM = (messageType == 4072) && (messageSubType == 1) && ((_logRTCM.bits.all == 1) || (_logRTCM.bits.UBX_RTCM_TYPE4072_1 == 1));

          if (logThisRTCM) // Should we log this message?
          {
            storeFileBytes(_storageRTCM->dataMessage, _storageRTCM->messageLength + 6);
          }

          // If there is space in the RTCM buffer, store the data there too
          if (rtcmBufferSpaceAvailable() >= _storageRTCM->messageLength + 6)
            storeRTCMBytes(_storageRTCM->dataMessage, _storageRTCM->messageLength + 6);

          // Check "Auto" RTCM
          if ((messageType == 1005) && (_storageRTCM->messageLength == RTCM_1005_MSG_LEN_BYTES) && (storageRTCM1005 != nullptr))
          {
            extractRTCM1005(&storageRTCM1005->data, &_storageRTCM->dataMessage[3]);

            storageRTCM1005->automaticFlags.flags.bits.dataValid = 1; // Mark the data as valid and unread
            storageRTCM1005->automaticFlags.flags.bits.dataRead = 0;

            if (storageRTCM1005->callbackData != nullptr)                              // Should we copy the data for the callback?
              if (storageRTCM1005->callbackPointerPtr != nullptr)                      // Has the callback been defined?
                if (storageRTCM1005->automaticFlags.flags.bits.callbackDataValid == 0) // Only overwrite the callback copy if it has been read
                {
                  memcpy(storageRTCM1005->callbackData, &storageRTCM1005->data, sizeof(RTCM_1005_data_t));
                  storageRTCM1005->automaticFlags.flags.bits.callbackDataValid = 1;
                }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.println(F("process: RTCM checksum fail!"));
          }
        }
      }
    }
#endif

    currentSentence = processRTCMframe(incoming, &rtcmFrameCounter); // Deal with RTCM bytes

    // If user has assigned an output port then pipe the characters there,
    // but only if the port is different (otherwise we'll output each character twice!)

  }
}



// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
// Also calculate how much RAM is needed to store the payload for a given automatic message
bool SFE_UBLOX_GNSS::autoLookup(uint8_t Class, uint8_t ID, uint16_t *maxSize)
{
  if (maxSize != nullptr)
    *maxSize = 0;

  switch (Class)
  {
  case UBX_CLASS_NAV:
    if (ID == UBX_NAV_POSECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_POSECEF_LEN;
      return (packetUBXNAVPOSECEF != nullptr);
    }
    else if (ID == UBX_NAV_STATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_STATUS_LEN;
      return (packetUBXNAVSTATUS != nullptr);
    }
    else if (ID == UBX_NAV_DOP)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_DOP_LEN;
      return (packetUBXNAVDOP != nullptr);
    }
    else if (ID == UBX_NAV_ATT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_ATT_LEN;
      return (packetUBXNAVATT != nullptr);
    }
    else if (ID == UBX_NAV_PVT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_PVT_LEN;
      return (packetUBXNAVPVT != nullptr);
    }
    else if (ID == UBX_NAV_ODO)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_ODO_LEN;
      return (packetUBXNAVODO != nullptr);
    }
    else if (ID == UBX_NAV_VELECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_VELECEF_LEN;
      return (packetUBXNAVVELECEF != nullptr);
    }
    else if (ID == UBX_NAV_VELNED)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_VELNED_LEN;
      return (packetUBXNAVVELNED != nullptr);
    }
    else if (ID == UBX_NAV_HPPOSECEF)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_HPPOSECEF_LEN;
      return (packetUBXNAVHPPOSECEF != nullptr);
    }
    else if (ID == UBX_NAV_HPPOSLLH)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_HPPOSLLH_LEN;
      return (packetUBXNAVHPPOSLLH != nullptr);
    }
    else if (ID == UBX_NAV_PVAT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_PVAT_LEN;
      return (packetUBXNAVPVAT != nullptr);
    }
    else if (ID == UBX_NAV_TIMEUTC)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_TIMEUTC_LEN;
      return (packetUBXNAVTIMEUTC != nullptr);
    }
    else if (ID == UBX_NAV_CLOCK)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_CLOCK_LEN;
      return (packetUBXNAVCLOCK != nullptr);
    }
    else if (ID == UBX_NAV_TIMELS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_TIMELS_LEN;
      return (packetUBXNAVTIMELS != nullptr);
    }
    else if (ID == UBX_NAV_SVIN)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SVIN_LEN;
      return (packetUBXNAVSVIN != nullptr);
    }
    else if (ID == UBX_NAV_RELPOSNED)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_RELPOSNED_LEN_F9;
      return (packetUBXNAVRELPOSNED != nullptr);
    }
    else if (ID == UBX_NAV_AOPSTATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_AOPSTATUS_LEN;
      return (packetUBXNAVAOPSTATUS != nullptr);
    }
    else if (ID == UBX_NAV_EOE)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_EOE_LEN;
      return (packetUBXNAVEOE != nullptr);
    }
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    else if (ID == UBX_NAV_SAT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SAT_MAX_LEN;
      return (packetUBXNAVSAT != nullptr);
    }
    else if (ID == UBX_NAV_SIG)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_NAV_SIG_MAX_LEN;
      return (packetUBXNAVSIG != nullptr);
    }
#endif
    break;
  case UBX_CLASS_RXM:
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    if (ID == UBX_RXM_SFRBX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_SFRBX_MAX_LEN;
      return (packetUBXRXMSFRBX != nullptr);
    }
    else if (ID == UBX_RXM_RAWX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_RAWX_MAX_LEN;
      return (packetUBXRXMRAWX != nullptr);
    }
    else if (ID == UBX_RXM_QZSSL6)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_QZSSL6_MAX_LEN;
      return (packetUBXRXMQZSSL6message != nullptr);
    }
    else if (ID == UBX_RXM_COR)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_COR_LEN;
      return (packetUBXRXMCOR != nullptr);
    }
    else if (ID == UBX_RXM_MEASX)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_MEASX_MAX_LEN;
      return (packetUBXRXMMEASX != nullptr);
    }
    else if (ID == UBX_RXM_PMP)
    {
      // PMP is a special case as it has both struct and message packages
      if (maxSize != nullptr)
        *maxSize = UBX_RXM_PMP_MAX_LEN;
      return ((packetUBXRXMPMP != nullptr) || (packetUBXRXMPMPmessage != nullptr));
    }
#endif
    break;
  case UBX_CLASS_TIM:
    if (ID == UBX_TIM_TM2)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_TIM_TM2_LEN;
      return (packetUBXTIMTM2 != nullptr);
    }
    else if (ID == UBX_TIM_TP)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_TIM_TP_LEN;
      return (packetUBXTIMTP != nullptr);
    }
    break;
  case UBX_CLASS_MON:
    if (ID == UBX_MON_HW)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MON_HW_LEN;
      return (packetUBXMONHW != nullptr);
    }
    break;
  case UBX_CLASS_ESF:
#ifndef SFE_UBLOX_DISABLE_ESF
    if (ID == UBX_ESF_ALG)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_ALG_LEN;
      return (packetUBXESFALG != nullptr);
    }
    else if (ID == UBX_ESF_INS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_INS_LEN;
      return (packetUBXESFINS != nullptr);
    }
    else if (ID == UBX_ESF_MEAS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_MEAS_MAX_LEN;
      return (packetUBXESFMEAS != nullptr);
    }
    else if (ID == UBX_ESF_RAW)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_RAW_MAX_LEN;
      return (packetUBXESFRAW != nullptr);
    }
    else if (ID == UBX_ESF_STATUS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_ESF_STATUS_MAX_LEN;
      return (packetUBXESFSTATUS != nullptr);
    }
#endif
    break;
  case UBX_CLASS_MGA:
    if (ID == UBX_MGA_ACK_DATA0)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MGA_ACK_DATA0_LEN;
      return (packetUBXMGAACK != nullptr);
    }
    else if (ID == UBX_MGA_DBD)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_MGA_DBD_LEN;
      return (packetUBXMGADBD != nullptr);
    }
    break;
  case UBX_CLASS_HNR:
#ifndef SFE_UBLOX_DISABLE_HNR
    if (ID == UBX_HNR_PVT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_PVT_LEN;
      return (packetUBXHNRPVT != nullptr);
    }
    else if (ID == UBX_HNR_ATT)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_ATT_LEN;
      return (packetUBXHNRATT != nullptr);
    }
    else if (ID == UBX_HNR_INS)
    {
      if (maxSize != nullptr)
        *maxSize = UBX_HNR_INS_LEN;
      return (packetUBXHNRINS != nullptr);
    }
#endif
    break;
  default:
    return false;
    break;
  }
  return false;
}




// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array size is packetCfgPayloadSize bytes.
// startingSpot can be set so we only record a subset of bytes within a larger packet.
void SFE_UBLOX_GNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // If incomingUBX is a user-defined custom packet, then the payload size could be different to packetCfgPayloadSize.
  // TO DO: update this to prevent an overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
  {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{

    bool logBecauseAuto = autoLookup(incomingUBX->cls, incomingUBX->id, &maximum_payload_size);
    bool logBecauseEnabled = logThisUBX(incomingUBX->cls, incomingUBX->id);
    if ((!logBecauseAuto) && (logBecauseEnabled))
      maximum_payload_size = SFE_UBX_MAX_LENGTH;
    if (maximum_payload_size == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        Serial.print(F("processUBX: autoLookup returned ZERO maxPayload!! Class: 0x"));
        Serial.print(incomingUBX->cls, HEX);
        Serial.print(F(" ID: 0x"));
        Serial.println(incomingUBX->id, HEX);
      }
#endif
    }
    //}
    // else
    //  maximum_payload_size = 2;
  }
  else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < (incomingUBX->len + 4))
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife = true;                                  // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          Serial.print(F("processUBX: NACK received: Requested Class: 0x"));
          Serial.print(incomingUBX->payload[0], HEX);
          Serial.print(F(" Requested ID: 0x"));
          Serial.println(incomingUBX->payload[1], HEX);
        }
#endif
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if ((autoLookup(incomingUBX->cls, incomingUBX->id)) || (logThisUBX(incomingUBX->cls, incomingUBX->id)))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          Serial.print(F("processUBX: incoming \"automatic\" message: Class: 0x"));
          Serial.print(incomingUBX->cls, HEX);
          Serial.print(F(" ID: 0x"));
          Serial.println(incomingUBX->id, HEX);
        }
#endif
      }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        Serial.print(F("Incoming: Size: "));
        Serial.print(incomingUBX->len);
        Serial.print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          Serial.println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          Serial.println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          Serial.println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          Serial.println(F("packetAck classAndIDmatch"));
        }
      }
#endif

      // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        // Drive an external pin to allow for easier logic analyzation
        if (debugPin >= 0)
        {
          digitalWrite((uint8_t)debugPin, LOW);
          delay(10);
          digitalWrite((uint8_t)debugPin, HIGH);
        }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        Serial.print(F("Checksum failed:"));
        Serial.print(F(" checksumA: "));
        Serial.print(incomingUBX->checksumA);
        Serial.print(F(" checksumB: "));
        Serial.print(incomingUBX->checksumB);

        Serial.print(F(" rollingChecksumA: "));
        Serial.print(rollingChecksumA);
        Serial.print(F(" rollingChecksumB: "));
        Serial.print(rollingChecksumB);
        Serial.println();
#endif
      }
    }

    // Now that the packet is complete and has been processed, we need to delete the memory
    // allocated for packetAuto
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
    {
      delete[] payloadAuto; // Created with new[]
      payloadAuto = nullptr;
      packetAuto.payload = payloadAuto;
    }
  }
  else // Load this byte into the payload array
  {
    // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (autoLookup(incomingUBX->cls, incomingUBX->id))
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      // Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        // Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < maximum_payload_size) // If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] = incoming; // Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
  if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) && (ignoreThisPayload == false)))
  {
    // Something has gone very wrong
    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      if (overrun)
        Serial.print(F("processUBX: buffer overrun detected!"));
      else
        Serial.print(F("processUBX: counter hit maximum_payload_size + 6!"));
      Serial.print(F(" activePacketBuffer: "));
      Serial.print(activePacketBuffer);
      Serial.print(F(" maximum_payload_size: "));
      Serial.println(maximum_payload_size);
    }
#endif
  }

  // Increment the counter
  incomingUBX->counter++;
}





// PRIVATE: Return true if we should add this NMEA message to the file buffer for logging
bool SFE_UBLOX_GNSS::logThisNMEA()
{
  bool logMe = false;
  if (_logNMEA.bits.all == 1)
    logMe = true;
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_DTM == 1))
    logMe = true;
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GAQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GBQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GBS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GGA == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_logNMEA.bits.UBX_NMEA_GLL == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GLQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GNQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GNS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GPQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GQQ == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GRS == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GSA == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_GST == 1))
      logMe = true;
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_logNMEA.bits.UBX_NMEA_GSV == 1))
      logMe = true;
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_RLM == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_logNMEA.bits.UBX_NMEA_RMC == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_THS == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_TXT == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_logNMEA.bits.UBX_NMEA_VLW == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_logNMEA.bits.UBX_NMEA_VTG == 1))
    logMe = true;
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_ZDA == 1))
    logMe = true;

  if (logMe)                   // Message should be logged.
    logMe = initStorageNMEA(); // Check we have non-Auto storage for it
  return (logMe);
}


// Get the maximum length of this NMEA message
uint8_t SFE_UBLOX_GNSS::getNMEAMaxLength()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  return 0;
}



// Get a pointer to the working copy length
uint8_t *SFE_UBLOX_GNSS::getNMEAWorkingLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.length;
  }

  return nullptr;
}

// PRIVATE: Return true if we should pass this NMEA message to processNMEA
bool SFE_UBLOX_GNSS::processThisNMEA()
{
  if (_processNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_processNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_processNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_processNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_THS == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_processNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_processNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}

void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  (void)incoming;
}

uint8_t *SFE_UBLOX_GNSS::getNMEAWorkingNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.nmea[0];
  }

  return nullptr;
}


// Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has dedicated RAM allocated for it)
bool SFE_UBLOX_GNSS::isThisNMEAauto()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != nullptr)
      return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != nullptr)
      return true;
  }

  return false;
}


// PRIVATE: Return true if the NMEA header is valid
bool SFE_UBLOX_GNSS::isNMEAHeaderValid()
{
  if (nmeaAddressField[0] != '*')
    return (false);
  if (nmeaAddressField[1] != 'G')
    return (false);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M'))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V'))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M'))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'H') && (nmeaAddressField[5] == 'S'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G'))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A'))
    return (true);
  return (false);
}



bool SFE_UBLOX_GNSS::initPacketUBXNAVPVT()
{
  packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVT == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initPacketUBXNAVPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVT->automaticFlags.flags.all = 0;
  packetUBXNAVPVT->callbackPointerPtr = nullptr;
  packetUBXNAVPVT->callbackData = nullptr;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}








// Get a pointer to the complete copy length
uint8_t *SFE_UBLOX_GNSS::getNMEACompleteLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.length;
  }

  return nullptr;
}






// Get a pointer to the complete copy NMEA data
uint8_t *SFE_UBLOX_GNSS::getNMEACompleteNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.nmea[0];
  }

  return nullptr;
}





// Get a pointer to the automatic NMEA flags
nmeaAutomaticFlags *SFE_UBLOX_GNSS::getNMEAFlagsPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->automaticFlags;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->automaticFlags;
  }

  return nullptr;
}






// Do we need to copy the data into the callback copy?
bool SFE_UBLOX_GNSS::doesThisNMEAHaveCallback()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != nullptr)
      if (storageNMEAGPGGA->callbackCopy != nullptr)
        if (storageNMEAGPGGA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != nullptr)
      if (storageNMEAGNGGA->callbackCopy != nullptr)
        if (storageNMEAGNGGA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != nullptr)
      if (storageNMEAGPVTG->callbackCopy != nullptr)
        if (storageNMEAGPVTG->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != nullptr)
      if (storageNMEAGNVTG->callbackCopy != nullptr)
        if (storageNMEAGNVTG->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != nullptr)
      if (storageNMEAGPRMC->callbackCopy != nullptr)
        if (storageNMEAGPRMC->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != nullptr)
      if (storageNMEAGNRMC->callbackCopy != nullptr)
        if (storageNMEAGNRMC->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != nullptr)
      if (storageNMEAGPZDA->callbackCopy != nullptr)
        if (storageNMEAGPZDA->callbackPointerPtr != nullptr)
          return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != nullptr)
      if (storageNMEAGNZDA->callbackCopy != nullptr)
        if (storageNMEAGNZDA->callbackPointerPtr != nullptr)
          return true;
  }

  return false;
}



// Get a pointer to the callback copy length
uint8_t *SFE_UBLOX_GNSS::getNMEACallbackLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->length;
  }

  return nullptr;
}


// Get a pointer to the callback copy NMEA data
uint8_t *SFE_UBLOX_GNSS::getNMEACallbackNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->nmea[0];
  }

  return nullptr;
}



// PRIVATE: Add theBytes to the file buffer
bool SFE_UBLOX_GNSS::storeFileBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == nullptr) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      Serial.println(F("storeFileBytes: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("storeFileBytes: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToFileBuffer(theBytes, numBytes);

  return (true);
}




// PRIVATE: Check how much space is available in the buffer
uint16_t SFE_UBLOX_GNSS::rtcmBufferSpaceAvailable(void)
{
  return (rtcmBufferSize - rtcmBufferSpaceUsed());
}




// PRIVATE: Add theBytes to the RTCM buffer
bool SFE_UBLOX_GNSS::storeRTCMBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((rtcmBuffer == nullptr) || (rtcmBufferSize == 0))
  {
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > rtcmBufferSpaceAvailable())
  {
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToRTCMBuffer(theBytes, numBytes);

  return (true);
}




void SFE_UBLOX_GNSS::extractRTCM1005(RTCM_1005_data_t *destination, uint8_t *source)
{
  destination->MessageNumber = extractUnsignedBits(source, 0, 12);
  destination->ReferenceStationID = extractUnsignedBits(source, 12, 12);
  destination->ITRFRealizationYear = extractUnsignedBits(source, 24, 6);
  destination->GPSIndicator = extractUnsignedBits(source, 30, 1);
  destination->GLONASSIndicator = extractUnsignedBits(source, 31, 1);
  destination->GalileoIndicator = extractUnsignedBits(source, 32, 1);
  destination->ReferenceStationIndicator = extractUnsignedBits(source, 33, 1);
  destination->AntennaReferencePointECEFX = extractSignedBits(source, 34, 38);
  destination->SingleReceiverOscillatorIndicator = extractUnsignedBits(source, 72, 1);
  destination->Reserved = extractUnsignedBits(source, 73, 1);
  destination->AntennaReferencePointECEFY = extractSignedBits(source, 74, 38);
  destination->QuarterCycleIndicator = extractUnsignedBits(source, 112, 2);
  destination->AntennaReferencePointECEFZ = extractSignedBits(source, 114, 38);
}

// Given a spot, extract a signed 8-bit value from the payload
int8_t SFE_UBLOX_GNSS::extractSignedChar(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned8 converter;
  converter.unsigned8 = extractByte(msg, spotToStart);
  return (converter.signed8);
}





SFE_UBLOX_GNSS::sfe_ublox_sentence_types_e SFE_UBLOX_GNSS::processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter)
{
  static uint16_t rtcmLen = 0; // Static - length is retained between calls

  if (*rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; // Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (*rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; // Bits 0-7 of packet length
    rtcmLen += 6;        // There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  *rtcmFrameCounter = *rtcmFrameCounter + 1; // Increment rtcmFrameCounter

  processRTCM(incoming); // Here is where we expose this byte to the user

  // If rtcmLen is not yet known, return SFE_UBLOX_SENTENCE_TYPE_RTCM
  if (*rtcmFrameCounter <= 2) // If this is header byte 0 or 1 (rtcmFrameCounter has been incremented)
    return SFE_UBLOX_SENTENCE_TYPE_RTCM;

  // Reset and start looking for next sentence type when done
  return (*rtcmFrameCounter == rtcmLen) ? SFE_UBLOX_SENTENCE_TYPE_NONE : SFE_UBLOX_SENTENCE_TYPE_RTCM;
}



// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
void SFE_UBLOX_GNSS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}



// Pretty prints the current ubxPacket
void SFE_UBLOX_GNSS::printPacket(ubxPacket *packet, bool alwaysPrintPayload)
{
  // Only print the payload is ignoreThisPayload is false otherwise
  // we could be printing gibberish from beyond the end of packetBuf
  // (These two lines get rid of a pesky compiler warning)
  bool printPayload = (ignoreThisPayload == false);
  printPayload |= (alwaysPrintPayload == true);

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    Serial.print(F("CLS:"));
    if (packet->cls == UBX_CLASS_NAV) // 1
      Serial.print(F("NAV"));
    else if (packet->cls == UBX_CLASS_ACK) // 5
      Serial.print(F("ACK"));
    else if (packet->cls == UBX_CLASS_CFG) // 6
      Serial.print(F("CFG"));
    else if (packet->cls == UBX_CLASS_MON) // 0x0A
      Serial.print(F("MON"));
    else
    {
      Serial.print(F("0x"));
      Serial.print(packet->cls, HEX);
    }

    Serial.print(F(" ID:"));
    if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
      Serial.print(F("PVT"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
      Serial.print(F("SAVE"));
    else
    {
      Serial.print(F("0x"));
      Serial.print(packet->id, HEX);
    }

    Serial.print(F(" Len: 0x"));
    Serial.print(packet->len, HEX);

    if (printPayload)
    {
      Serial.print(F(" Payload:"));

      for (uint16_t x = 0; x < packet->len; x++)
      {
        Serial.print(F(" "));
        Serial.print(packet->payload[x], HEX);
      }
    }
    else
    {
      Serial.print(F(" Payload: IGNORED"));
    }
    Serial.println();
  }
#else
  if (_printDebug == true)
  {
    Serial.print(F("Len: 0x"));
    Serial.print(packet->len, HEX);
  }
#endif
}


void SFE_UBLOX_GNSS::processUBXpacket(ubxPacket *msg)
{
  bool addedToFileBuffer = false;
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_POSECEF && msg->len == UBX_NAV_POSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPOSECEF != nullptr)
      {
        packetUBXNAVPOSECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPOSECEF->data.ecefX = extractSignedLong(msg, 4);
        packetUBXNAVPOSECEF->data.ecefY = extractSignedLong(msg, 8);
        packetUBXNAVPOSECEF->data.ecefZ = extractSignedLong(msg, 12);
        packetUBXNAVPOSECEF->data.pAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPOSECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPOSECEF->callbackData->iTOW, &packetUBXNAVPOSECEF->data.iTOW, sizeof(UBX_NAV_POSECEF_data_t));
          packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_STATUS && msg->len == UBX_NAV_STATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSTATUS != nullptr)
      {
        packetUBXNAVSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVSTATUS->data.gpsFix = extractByte(msg, 4);
        packetUBXNAVSTATUS->data.flags.all = extractByte(msg, 5);
        packetUBXNAVSTATUS->data.fixStat.all = extractByte(msg, 6);
        packetUBXNAVSTATUS->data.flags2.all = extractByte(msg, 7);
        packetUBXNAVSTATUS->data.ttff = extractLong(msg, 8);
        packetUBXNAVSTATUS->data.msss = extractLong(msg, 12);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSTATUS->callbackData->iTOW, &packetUBXNAVSTATUS->data.iTOW, sizeof(UBX_NAV_STATUS_data_t));
          packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_DOP && msg->len == UBX_NAV_DOP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVDOP != nullptr)
      {
        packetUBXNAVDOP->data.iTOW = extractLong(msg, 0);
        packetUBXNAVDOP->data.gDOP = extractInt(msg, 4);
        packetUBXNAVDOP->data.pDOP = extractInt(msg, 6);
        packetUBXNAVDOP->data.tDOP = extractInt(msg, 8);
        packetUBXNAVDOP->data.vDOP = extractInt(msg, 10);
        packetUBXNAVDOP->data.hDOP = extractInt(msg, 12);
        packetUBXNAVDOP->data.nDOP = extractInt(msg, 14);
        packetUBXNAVDOP->data.eDOP = extractInt(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVDOP->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVDOP->callbackData->iTOW, &packetUBXNAVDOP->data.iTOW, sizeof(UBX_NAV_DOP_data_t));
          packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ATT && msg->len == UBX_NAV_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVATT != nullptr)
      {
        packetUBXNAVATT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVATT->data.version = extractByte(msg, 4);
        packetUBXNAVATT->data.roll = extractSignedLong(msg, 8);
        packetUBXNAVATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXNAVATT->data.heading = extractSignedLong(msg, 16);
        packetUBXNAVATT->data.accRoll = extractLong(msg, 20);
        packetUBXNAVATT->data.accPitch = extractLong(msg, 24);
        packetUBXNAVATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXNAVATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVATT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVATT->callbackData->iTOW, &packetUBXNAVATT->data.iTOW, sizeof(UBX_NAV_ATT_data_t));
          packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVT != nullptr)
      {
        packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVT->data.year = extractInt(msg, 4);
        packetUBXNAVPVT->data.month = extractByte(msg, 6);
        packetUBXNAVPVT->data.day = extractByte(msg, 7);
        packetUBXNAVPVT->data.hour = extractByte(msg, 8);
        packetUBXNAVPVT->data.min = extractByte(msg, 9);
        packetUBXNAVPVT->data.sec = extractByte(msg, 10);
        packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
        packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
        packetUBXNAVPVT->data.nano = extractSignedLong(msg, 16); // Includes milliseconds
        packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
        packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
        packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
        packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
        packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
        packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
        packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
        packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
        packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
        packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
        packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
        packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
        packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
        packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
        packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
        packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
        packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
        packetUBXNAVPVT->data.flags3.all = extractByte(msg, 78);
        packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
        packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
        packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVT->callbackData->iTOW, &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ODO && msg->len == UBX_NAV_ODO_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVODO != nullptr)
      {
        packetUBXNAVODO->data.version = extractByte(msg, 0);
        packetUBXNAVODO->data.iTOW = extractLong(msg, 4);
        packetUBXNAVODO->data.distance = extractLong(msg, 8);
        packetUBXNAVODO->data.totalDistance = extractLong(msg, 12);
        packetUBXNAVODO->data.distanceStd = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVODO->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVODO->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVODO->callbackData->version, &packetUBXNAVODO->data.version, sizeof(UBX_NAV_ODO_data_t));
          packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELECEF && msg->len == UBX_NAV_VELECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELECEF != nullptr)
      {
        packetUBXNAVVELECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELECEF->data.ecefVX = extractSignedLong(msg, 4);
        packetUBXNAVVELECEF->data.ecefVY = extractSignedLong(msg, 8);
        packetUBXNAVVELECEF->data.ecefVZ = extractSignedLong(msg, 12);
        packetUBXNAVVELECEF->data.sAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELECEF->callbackData->iTOW, &packetUBXNAVVELECEF->data.iTOW, sizeof(UBX_NAV_VELECEF_data_t));
          packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELNED && msg->len == UBX_NAV_VELNED_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELNED != nullptr)
      {
        packetUBXNAVVELNED->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELNED->data.velN = extractSignedLong(msg, 4);
        packetUBXNAVVELNED->data.velE = extractSignedLong(msg, 8);
        packetUBXNAVVELNED->data.velD = extractSignedLong(msg, 12);
        packetUBXNAVVELNED->data.speed = extractLong(msg, 16);
        packetUBXNAVVELNED->data.gSpeed = extractLong(msg, 20);
        packetUBXNAVVELNED->data.heading = extractSignedLong(msg, 24);
        packetUBXNAVVELNED->data.sAcc = extractLong(msg, 28);
        packetUBXNAVVELNED->data.cAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELNED->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELNED->callbackData->iTOW, &packetUBXNAVVELNED->data.iTOW, sizeof(UBX_NAV_VELNED_data_t));
          packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSECEF && msg->len == UBX_NAV_HPPOSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSECEF != nullptr)
      {
        packetUBXNAVHPPOSECEF->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSECEF->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSECEF->data.ecefX = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSECEF->data.ecefY = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSECEF->data.ecefZ = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSECEF->data.ecefXHp = extractSignedChar(msg, 20);
        packetUBXNAVHPPOSECEF->data.ecefYHp = extractSignedChar(msg, 21);
        packetUBXNAVHPPOSECEF->data.ecefZHp = extractSignedChar(msg, 22);
        packetUBXNAVHPPOSECEF->data.flags.all = extractByte(msg, 23);
        packetUBXNAVHPPOSECEF->data.pAcc = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSECEF->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSECEF->callbackData->version, &packetUBXNAVHPPOSECEF->data.version, sizeof(UBX_NAV_HPPOSECEF_data_t));
          packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == UBX_NAV_HPPOSLLH_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSLLH != nullptr)
      {
        packetUBXNAVHPPOSLLH->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSLLH->data.flags.all = extractByte(msg, 3);
        packetUBXNAVHPPOSLLH->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSLLH->data.lon = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSLLH->data.lat = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSLLH->data.height = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSLLH->data.hMSL = extractSignedLong(msg, 20);
        packetUBXNAVHPPOSLLH->data.lonHp = extractSignedChar(msg, 24);
        packetUBXNAVHPPOSLLH->data.latHp = extractSignedChar(msg, 25);
        packetUBXNAVHPPOSLLH->data.heightHp = extractSignedChar(msg, 26);
        packetUBXNAVHPPOSLLH->data.hMSLHp = extractSignedChar(msg, 27);
        packetUBXNAVHPPOSLLH->data.hAcc = extractLong(msg, 28);
        packetUBXNAVHPPOSLLH->data.vAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSLLH->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSLLH->callbackData->version, &packetUBXNAVHPPOSLLH->data.version, sizeof(UBX_NAV_HPPOSLLH_data_t));
          packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVAT && msg->len == UBX_NAV_PVAT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVAT != nullptr)
      {
        packetUBXNAVPVAT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVAT->data.version = extractByte(msg, 4);
        packetUBXNAVPVAT->data.valid.all = extractByte(msg, 5);
        packetUBXNAVPVAT->data.year = extractInt(msg, 6);
        packetUBXNAVPVAT->data.month = extractByte(msg, 8);
        packetUBXNAVPVAT->data.day = extractByte(msg, 9);
        packetUBXNAVPVAT->data.hour = extractByte(msg, 10);
        packetUBXNAVPVAT->data.min = extractByte(msg, 11);
        packetUBXNAVPVAT->data.sec = extractByte(msg, 12);
        packetUBXNAVPVAT->data.tAcc = extractLong(msg, 16);
        packetUBXNAVPVAT->data.nano = extractSignedLong(msg, 20); // Includes milliseconds
        packetUBXNAVPVAT->data.fixType = extractByte(msg, 24);
        packetUBXNAVPVAT->data.flags.all = extractByte(msg, 25);
        packetUBXNAVPVAT->data.flags2.all = extractByte(msg, 26);
        packetUBXNAVPVAT->data.numSV = extractByte(msg, 27);
        packetUBXNAVPVAT->data.lon = extractSignedLong(msg, 28);
        packetUBXNAVPVAT->data.lat = extractSignedLong(msg, 32);
        packetUBXNAVPVAT->data.height = extractSignedLong(msg, 36);
        packetUBXNAVPVAT->data.hMSL = extractSignedLong(msg, 40);
        packetUBXNAVPVAT->data.hAcc = extractLong(msg, 44);
        packetUBXNAVPVAT->data.vAcc = extractLong(msg, 48);
        packetUBXNAVPVAT->data.velN = extractSignedLong(msg, 52);
        packetUBXNAVPVAT->data.velE = extractSignedLong(msg, 56);
        packetUBXNAVPVAT->data.velD = extractSignedLong(msg, 60);
        packetUBXNAVPVAT->data.gSpeed = extractSignedLong(msg, 64);
        packetUBXNAVPVAT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVAT->data.vehRoll = extractSignedLong(msg, 72);
        packetUBXNAVPVAT->data.vehPitch = extractSignedLong(msg, 76);
        packetUBXNAVPVAT->data.vehHeading = extractSignedLong(msg, 80);
        packetUBXNAVPVAT->data.motHeading = extractSignedLong(msg, 84);
        packetUBXNAVPVAT->data.accRoll = extractInt(msg, 88);
        packetUBXNAVPVAT->data.accPitch = extractInt(msg, 90);
        packetUBXNAVPVAT->data.accHeading = extractInt(msg, 92);
        packetUBXNAVPVAT->data.magDec = extractSignedInt(msg, 94);
        packetUBXNAVPVAT->data.magAcc = extractInt(msg, 96);
        packetUBXNAVPVAT->data.errEllipseOrient = extractInt(msg, 98);
        packetUBXNAVPVAT->data.errEllipseMajor = extractLong(msg, 100);
        packetUBXNAVPVAT->data.errEllipseMinor = extractLong(msg, 104);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVAT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVAT->callbackData->iTOW, &packetUBXNAVPVAT->data.iTOW, sizeof(UBX_NAV_PVAT_data_t));
          packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMEUTC && msg->len == UBX_NAV_TIMEUTC_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMEUTC != nullptr)
      {
        packetUBXNAVTIMEUTC->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMEUTC->data.tAcc = extractLong(msg, 4);
        packetUBXNAVTIMEUTC->data.nano = extractSignedLong(msg, 8);
        packetUBXNAVTIMEUTC->data.year = extractInt(msg, 12);
        packetUBXNAVTIMEUTC->data.month = extractByte(msg, 14);
        packetUBXNAVTIMEUTC->data.day = extractByte(msg, 15);
        packetUBXNAVTIMEUTC->data.hour = extractByte(msg, 16);
        packetUBXNAVTIMEUTC->data.min = extractByte(msg, 17);
        packetUBXNAVTIMEUTC->data.sec = extractByte(msg, 18);
        packetUBXNAVTIMEUTC->data.valid.all = extractByte(msg, 19);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVTIMEUTC->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVTIMEUTC->callbackData->iTOW, &packetUBXNAVTIMEUTC->data.iTOW, sizeof(UBX_NAV_TIMEUTC_data_t));
          packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_CLOCK && msg->len == UBX_NAV_CLOCK_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVCLOCK != nullptr)
      {
        packetUBXNAVCLOCK->data.iTOW = extractLong(msg, 0);
        packetUBXNAVCLOCK->data.clkB = extractSignedLong(msg, 4);
        packetUBXNAVCLOCK->data.clkD = extractSignedLong(msg, 8);
        packetUBXNAVCLOCK->data.tAcc = extractLong(msg, 12);
        packetUBXNAVCLOCK->data.fAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVCLOCK->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVCLOCK->callbackData->iTOW, &packetUBXNAVCLOCK->data.iTOW, sizeof(UBX_NAV_CLOCK_data_t));
          packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMELS && msg->len == UBX_NAV_TIMELS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMELS != nullptr)
      {
        packetUBXNAVTIMELS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMELS->data.version = extractByte(msg, 4);
        packetUBXNAVTIMELS->data.srcOfCurrLs = extractByte(msg, 8);
        packetUBXNAVTIMELS->data.currLs = extractSignedChar(msg, 9);
        packetUBXNAVTIMELS->data.srcOfLsChange = extractByte(msg, 10);
        packetUBXNAVTIMELS->data.lsChange = extractSignedChar(msg, 11);
        packetUBXNAVTIMELS->data.timeToLsEvent = extractSignedLong(msg, 12);
        packetUBXNAVTIMELS->data.dateOfLsGpsWn = extractInt(msg, 16);
        packetUBXNAVTIMELS->data.dateOfLsGpsDn = extractInt(msg, 18);
        packetUBXNAVTIMELS->data.valid.all = extractSignedChar(msg, 23);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    else if (msg->id == UBX_NAV_SVIN && msg->len == UBX_NAV_SVIN_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSVIN != nullptr)
      {
        packetUBXNAVSVIN->data.version = extractByte(msg, 0);
        packetUBXNAVSVIN->data.iTOW = extractLong(msg, 4);
        packetUBXNAVSVIN->data.dur = extractLong(msg, 8);
        packetUBXNAVSVIN->data.meanX = extractSignedLong(msg, 12);
        packetUBXNAVSVIN->data.meanY = extractSignedLong(msg, 16);
        packetUBXNAVSVIN->data.meanZ = extractSignedLong(msg, 20);
        packetUBXNAVSVIN->data.meanXHP = extractSignedChar(msg, 24);
        packetUBXNAVSVIN->data.meanYHP = extractSignedChar(msg, 25);
        packetUBXNAVSVIN->data.meanZHP = extractSignedChar(msg, 26);
        packetUBXNAVSVIN->data.meanAcc = extractLong(msg, 28);
        packetUBXNAVSVIN->data.obs = extractLong(msg, 32);
        packetUBXNAVSVIN->data.valid = extractSignedChar(msg, 36);
        packetUBXNAVSVIN->data.active = extractSignedChar(msg, 37);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSVIN->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSVIN->callbackData->version, &packetUBXNAVSVIN->data.version, sizeof(UBX_NAV_SVIN_data_t));
          packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
    else if (msg->id == UBX_NAV_SAT) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSAT != nullptr)
      {
        packetUBXNAVSAT->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSAT->data.header.version = extractByte(msg, 4);
        packetUBXNAVSAT->data.header.numSvs = extractByte(msg, 5);

        // The NAV SAT message could contain data for 255 SVs max. (numSvs is uint8_t. UBX_NAV_SAT_MAX_BLOCKS is 255)
        for (uint16_t i = 0; (i < UBX_NAV_SAT_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSAT->data.header.numSvs)) && ((i * 12) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 12) + 8;
          packetUBXNAVSAT->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSAT->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSAT->data.blocks[i].cno = extractByte(msg, offset + 2);
          packetUBXNAVSAT->data.blocks[i].elev = extractSignedChar(msg, offset + 3);
          packetUBXNAVSAT->data.blocks[i].azim = extractSignedInt(msg, offset + 4);
          packetUBXNAVSAT->data.blocks[i].prRes = extractSignedInt(msg, offset + 6);
          packetUBXNAVSAT->data.blocks[i].flags.all = extractLong(msg, offset + 8);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSAT->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSAT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSAT->callbackData->header.iTOW, &packetUBXNAVSAT->data.header.iTOW, sizeof(UBX_NAV_SAT_data_t));
          packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_SIG) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSIG != nullptr)
      {
        packetUBXNAVSIG->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSIG->data.header.version = extractByte(msg, 4);
        packetUBXNAVSIG->data.header.numSigs = extractByte(msg, 5);

        // The NAV SIG message could potentially contain data for 255 signals. (numSigs is uint8_t. UBX_NAV_SIG_MAX_BLOCKS is 92)
        for (uint16_t i = 0; (i < UBX_NAV_SIG_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSIG->data.header.numSigs)) && ((i * 16) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 16) + 8;
          packetUBXNAVSIG->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSIG->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSIG->data.blocks[i].sigId = extractByte(msg, offset + 2);
          packetUBXNAVSIG->data.blocks[i].freqId = extractByte(msg, offset + 3);
          packetUBXNAVSIG->data.blocks[i].prRes = extractSignedInt(msg, offset + 4);
          packetUBXNAVSIG->data.blocks[i].cno = extractByte(msg, offset + 6);
          packetUBXNAVSIG->data.blocks[i].qualityInd = extractByte(msg, offset + 7);
          packetUBXNAVSIG->data.blocks[i].corrSource = extractByte(msg, offset + 8);
          packetUBXNAVSIG->data.blocks[i].ionoModel = extractByte(msg, offset + 9);
          packetUBXNAVSIG->data.blocks[i].sigFlags.all = extractInt(msg, offset + 10);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSIG->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSIG->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSIG->callbackData->header.iTOW, &packetUBXNAVSIG->data.header.iTOW, sizeof(UBX_NAV_SIG_data_t));
          packetUBXNAVSIG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSIG->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
#endif
    else if (msg->id == UBX_NAV_RELPOSNED && ((msg->len == UBX_NAV_RELPOSNED_LEN) || (msg->len == UBX_NAV_RELPOSNED_LEN_F9)))
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVRELPOSNED != nullptr)
      {
        // Note:
        //   RELPOSNED on the M8 is only 40 bytes long
        //   RELPOSNED on the F9 is 64 bytes long and contains much more information

        packetUBXNAVRELPOSNED->data.version = extractByte(msg, 0);
        packetUBXNAVRELPOSNED->data.refStationId = extractInt(msg, 2);
        packetUBXNAVRELPOSNED->data.iTOW = extractLong(msg, 4);
        packetUBXNAVRELPOSNED->data.relPosN = extractSignedLong(msg, 8);
        packetUBXNAVRELPOSNED->data.relPosE = extractSignedLong(msg, 12);
        packetUBXNAVRELPOSNED->data.relPosD = extractSignedLong(msg, 16);

        if (msg->len == UBX_NAV_RELPOSNED_LEN)
        {
          // The M8 version does not contain relPosLength or relPosHeading
          packetUBXNAVRELPOSNED->data.relPosLength = 0;
          packetUBXNAVRELPOSNED->data.relPosHeading = 0;
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 21);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 22);
          packetUBXNAVRELPOSNED->data.relPosHPLength = 0; // The M8 version does not contain relPosHPLength
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 24);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 28);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 32);
          // The M8 version does not contain accLength or accHeading
          packetUBXNAVRELPOSNED->data.accLength = 0;
          packetUBXNAVRELPOSNED->data.accHeading = 0;
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 36);
        }
        else
        {
          packetUBXNAVRELPOSNED->data.relPosLength = extractSignedLong(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHeading = extractSignedLong(msg, 24);
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 32);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 33);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 34);
          packetUBXNAVRELPOSNED->data.relPosHPLength = extractSignedChar(msg, 35);
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 36);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 40);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 44);
          packetUBXNAVRELPOSNED->data.accLength = extractLong(msg, 48);
          packetUBXNAVRELPOSNED->data.accHeading = extractLong(msg, 52);
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 60);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVRELPOSNED->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVRELPOSNED->callbackData->version, &packetUBXNAVRELPOSNED->data.version, sizeof(UBX_NAV_RELPOSNED_data_t));
          packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_AOPSTATUS && msg->len == UBX_NAV_AOPSTATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVAOPSTATUS != nullptr)
      {
        packetUBXNAVAOPSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVAOPSTATUS->data.aopCfg.all = extractByte(msg, 4);
        packetUBXNAVAOPSTATUS->data.status = extractByte(msg, 5);

        // Mark all datums as fresh (not read before)
        packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVAOPSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVAOPSTATUS->callbackData->iTOW, &packetUBXNAVAOPSTATUS->data.iTOW, sizeof(UBX_NAV_AOPSTATUS_data_t));
          packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_EOE && msg->len == UBX_NAV_EOE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVEOE != nullptr)
      {
        packetUBXNAVEOE->data.iTOW = extractLong(msg, 0);

        // Mark all datums as fresh (not read before)
        packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVEOE->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVEOE->callbackData->iTOW, &packetUBXNAVEOE->data.iTOW, sizeof(UBX_NAV_EOE_data_t));
          packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_RAWX_SFRBX_PMP_QZSS_SAT
  case UBX_CLASS_RXM:
    if (msg->id == UBX_RXM_PMP)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it.
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMP != nullptr) && (packetUBXRXMPMP->callbackData != nullptr)
          //&& (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMP->callbackData->version = extractByte(msg, 0);
        packetUBXRXMPMP->callbackData->numBytesUserData = extractInt(msg, 2);
        packetUBXRXMPMP->callbackData->timeTag = extractLong(msg, 4);
        packetUBXRXMPMP->callbackData->uniqueWord[0] = extractLong(msg, 8);
        packetUBXRXMPMP->callbackData->uniqueWord[1] = extractLong(msg, 12);
        packetUBXRXMPMP->callbackData->serviceIdentifier = extractInt(msg, 16);
        packetUBXRXMPMP->callbackData->spare = extractByte(msg, 18);
        packetUBXRXMPMP->callbackData->uniqueWordBitErrors = extractByte(msg, 19);

        if (packetUBXRXMPMP->callbackData->version == 0x00)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 524);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 526);
        }
        else // if (packetUBXRXMPMP->data.version == 0x01)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 20);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 22);
        }

        uint16_t userDataStart = (packetUBXRXMPMP->callbackData->version == 0x00) ? 20 : 24;
        uint16_t userDataLength = (packetUBXRXMPMP->callbackData->version == 0x00) ? 504 : (packetUBXRXMPMP->callbackData->numBytesUserData);
        for (uint16_t i = 0; (i < userDataLength) && (i < 504); i++)
        {
          packetUBXRXMPMP->callbackData->userData[i] = extractByte(msg, i + userDataStart);
        }

        packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }

      // Full PMP message, including Class, ID and checksum
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMPmessage != nullptr) && (packetUBXRXMPMPmessage->callbackData != nullptr)
          //&& (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMPmessage->callbackData->sync1 = UBX_SYNCH_1;
        packetUBXRXMPMPmessage->callbackData->sync2 = UBX_SYNCH_2;
        packetUBXRXMPMPmessage->callbackData->cls = UBX_CLASS_RXM;
        packetUBXRXMPMPmessage->callbackData->ID = UBX_RXM_PMP;
        packetUBXRXMPMPmessage->callbackData->lengthLSB = msg->len & 0xFF;
        packetUBXRXMPMPmessage->callbackData->lengthMSB = msg->len >> 8;

        memcpy(packetUBXRXMPMPmessage->callbackData->payload, msg->payload, msg->len);

        packetUBXRXMPMPmessage->callbackData->checksumA = msg->checksumA;
        packetUBXRXMPMPmessage->callbackData->checksumB = msg->checksumB;

        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_QZSSL6)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Full QZSSL6 message, including Class, ID and checksum
      for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
      {
        if (0 == (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)))
        {

          packetUBXRXMQZSSL6message->callbackData[ch].sync1 = UBX_SYNCH_1;
          packetUBXRXMQZSSL6message->callbackData[ch].sync2 = UBX_SYNCH_2;
          packetUBXRXMQZSSL6message->callbackData[ch].cls = UBX_CLASS_RXM;
          packetUBXRXMQZSSL6message->callbackData[ch].ID = UBX_RXM_QZSSL6;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthLSB = msg->len & 0xFF;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthMSB = msg->len >> 8;

          memcpy(packetUBXRXMQZSSL6message->callbackData[ch].payload, msg->payload, msg->len);

          packetUBXRXMQZSSL6message->callbackData[ch].checksumA = msg->checksumA;
          packetUBXRXMQZSSL6message->callbackData[ch].checksumB = msg->checksumB;

          packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid |= (1 << ch);
          break; // abort when added
        }
      }
    }
    else if (msg->id == UBX_RXM_COR)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if ((packetUBXRXMCOR != nullptr) && (packetUBXRXMCOR->callbackData != nullptr)
          //&& (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMCOR->callbackData->version = extractByte(msg, 0);
        packetUBXRXMCOR->callbackData->ebno = extractByte(msg, 1);
        packetUBXRXMCOR->callbackData->statusInfo.all = extractLong(msg, 4);
        packetUBXRXMCOR->callbackData->msgType = extractInt(msg, 8);
        packetUBXRXMCOR->callbackData->msgSubType = extractInt(msg, 10);

        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_SFRBX)
    // Note: length is variable
    // Note: on protocol version 17: numWords is (0..16)
    //       on protocol version 18+: numWords is (0..10)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMSFRBX != nullptr)
      {
        packetUBXRXMSFRBX->data.gnssId = extractByte(msg, 0);
        packetUBXRXMSFRBX->data.svId = extractByte(msg, 1);
        packetUBXRXMSFRBX->data.freqId = extractByte(msg, 3);
        packetUBXRXMSFRBX->data.numWords = extractByte(msg, 4);
        packetUBXRXMSFRBX->data.chn = extractByte(msg, 5);
        packetUBXRXMSFRBX->data.version = extractByte(msg, 6);

        for (uint8_t i = 0; (i < UBX_RXM_SFRBX_MAX_WORDS) && (i < packetUBXRXMSFRBX->data.numWords) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXRXMSFRBX->data.dwrd[i] = extractLong(msg, 8 + (i * 4));
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMSFRBX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if (packetUBXRXMSFRBX->callbackData != nullptr) // If RAM has been allocated for the copies of the data
        {
          for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++) // Check all available buffers
          {
            if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) == 0) // AND the buffer is empty
            {
              memcpy(&packetUBXRXMSFRBX->callbackData[i].gnssId, &packetUBXRXMSFRBX->data.gnssId, sizeof(UBX_RXM_SFRBX_data_t));
              packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid |= (1 << i);
              break; // Only copy once - into first available buffer
            }
          }
        }

        // Check if we need to copy the data for the message callbacks
        if (packetUBXRXMSFRBX->callbackMessageData != nullptr) // If RAM has been allocated for the copies of the data
        {
          for (uint32_t i = 0; i < UBX_RXM_SFRBX_CALLBACK_BUFFERS; i++) // Check all available buffers
          {
            if ((packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid & (1 << i)) == 0) // AND the buffer is empty
            {
              packetUBXRXMSFRBX->callbackMessageData[i].sync1 = UBX_SYNCH_1;
              packetUBXRXMSFRBX->callbackMessageData[i].sync2 = UBX_SYNCH_2;
              packetUBXRXMSFRBX->callbackMessageData[i].cls = UBX_CLASS_RXM;
              packetUBXRXMSFRBX->callbackMessageData[i].ID = UBX_RXM_SFRBX;
              packetUBXRXMSFRBX->callbackMessageData[i].lengthLSB = msg->len & 0xFF;
              packetUBXRXMSFRBX->callbackMessageData[i].lengthMSB = msg->len >> 8;

              memcpy(&packetUBXRXMSFRBX->callbackMessageData[i].payload, msg->payload, msg->len);

              packetUBXRXMSFRBX->callbackMessageData[i].checksumA = msg->checksumA;
              packetUBXRXMSFRBX->callbackMessageData[i].checksumB = msg->checksumB;

              packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackMessageCopyValid |= (1 << i);
              break; // Only copy once - into first available buffer
            }
          }
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_RAWX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMRAWX != nullptr)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          packetUBXRXMRAWX->data.header.rcvTow[i] = extractByte(msg, i);
        }
        packetUBXRXMRAWX->data.header.week = extractInt(msg, 8);
        packetUBXRXMRAWX->data.header.leapS = extractSignedChar(msg, 10);
        packetUBXRXMRAWX->data.header.numMeas = extractByte(msg, 11);
        packetUBXRXMRAWX->data.header.recStat.all = extractByte(msg, 12);
        packetUBXRXMRAWX->data.header.version = extractByte(msg, 13);

        for (uint8_t i = 0; (i < UBX_RXM_RAWX_MAX_BLOCKS) && (i < packetUBXRXMRAWX->data.header.numMeas) && ((((uint16_t)i) * 32) < (msg->len - 16)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 32) + 16;
          for (uint8_t j = 0; j < 8; j++)
          {
            packetUBXRXMRAWX->data.blocks[i].prMes[j] = extractByte(msg, offset + j);
            packetUBXRXMRAWX->data.blocks[i].cpMes[j] = extractByte(msg, offset + 8 + j);
            if (j < 4)
              packetUBXRXMRAWX->data.blocks[i].doMes[j] = extractByte(msg, offset + 16 + j);
          }
          packetUBXRXMRAWX->data.blocks[i].gnssId = extractByte(msg, offset + 20);
          packetUBXRXMRAWX->data.blocks[i].svId = extractByte(msg, offset + 21);
          packetUBXRXMRAWX->data.blocks[i].sigId = extractByte(msg, offset + 22);
          packetUBXRXMRAWX->data.blocks[i].freqId = extractByte(msg, offset + 23);
          packetUBXRXMRAWX->data.blocks[i].lockTime = extractInt(msg, offset + 24);
          packetUBXRXMRAWX->data.blocks[i].cno = extractByte(msg, offset + 26);
          packetUBXRXMRAWX->data.blocks[i].prStdev = extractByte(msg, offset + 27);
          packetUBXRXMRAWX->data.blocks[i].cpStdev = extractByte(msg, offset + 28);
          packetUBXRXMRAWX->data.blocks[i].doStdev = extractByte(msg, offset + 29);
          packetUBXRXMRAWX->data.blocks[i].trkStat.all = extractByte(msg, offset + 30);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMRAWX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMRAWX->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMRAWX->callbackData->header.rcvTow[0], &packetUBXRXMRAWX->data.header.rcvTow[0], sizeof(UBX_RXM_RAWX_data_t));
          packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_MEASX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMMEASX != nullptr)
      {
        packetUBXRXMMEASX->data.header.version = extractByte(msg, 0);
        packetUBXRXMMEASX->data.header.gpsTOW = extractLong(msg, 4);
        packetUBXRXMMEASX->data.header.gloTOW = extractLong(msg, 8);
        packetUBXRXMMEASX->data.header.bdsTOW = extractLong(msg, 12);
        packetUBXRXMMEASX->data.header.qzssTOW = extractLong(msg, 20);
        packetUBXRXMMEASX->data.header.gpsTOWacc = extractInt(msg, 24);
        packetUBXRXMMEASX->data.header.gloTOWacc = extractInt(msg, 26);
        packetUBXRXMMEASX->data.header.bdsTOWacc = extractInt(msg, 28);
        packetUBXRXMMEASX->data.header.qzssTOWacc = extractInt(msg, 32);
        packetUBXRXMMEASX->data.header.numSV = extractByte(msg, 34);
        packetUBXRXMMEASX->data.header.flags.all = extractByte(msg, 35);

        for (uint8_t i = 0; (i < UBX_RXM_MEASX_MAX_BLOCKS) && (i < packetUBXRXMMEASX->data.header.numSV) && ((((uint16_t)i) * 24) < (msg->len - 44)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 24) + 44;
          packetUBXRXMMEASX->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXRXMMEASX->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXRXMMEASX->data.blocks[i].cNo = extractByte(msg, offset + 2);
          packetUBXRXMMEASX->data.blocks[i].mpathIndic = extractByte(msg, offset + 3);
          packetUBXRXMMEASX->data.blocks[i].dopplerMS = extractSignedLong(msg, offset + 4);
          packetUBXRXMMEASX->data.blocks[i].dopplerHz = extractSignedLong(msg, offset + 8);
          packetUBXRXMMEASX->data.blocks[i].wholeChips = extractInt(msg, offset + 12);
          packetUBXRXMMEASX->data.blocks[i].fracChips = extractInt(msg, offset + 14);
          packetUBXRXMMEASX->data.blocks[i].codePhase = extractLong(msg, offset + 16);
          packetUBXRXMMEASX->data.blocks[i].intCodePhase = extractByte(msg, offset + 20);
          packetUBXRXMMEASX->data.blocks[i].pseuRangeRMSErr = extractByte(msg, offset + 21);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMMEASX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMMEASX->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMMEASX->callbackData->header.version, &packetUBXRXMMEASX->data.header.version, sizeof(UBX_RXM_MEASX_data_t));
          packetUBXRXMMEASX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMMEASX->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
    break;
#endif
  case UBX_CLASS_TIM:
    if (msg->id == UBX_TIM_TM2 && msg->len == UBX_TIM_TM2_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTM2 != nullptr)
      {
        packetUBXTIMTM2->data.ch = extractByte(msg, 0);
        packetUBXTIMTM2->data.flags.all = extractByte(msg, 1);
        packetUBXTIMTM2->data.count = extractInt(msg, 2);
        packetUBXTIMTM2->data.wnR = extractInt(msg, 4);
        packetUBXTIMTM2->data.wnF = extractInt(msg, 6);
        packetUBXTIMTM2->data.towMsR = extractLong(msg, 8);
        packetUBXTIMTM2->data.towSubMsR = extractLong(msg, 12);
        packetUBXTIMTM2->data.towMsF = extractLong(msg, 16);
        packetUBXTIMTM2->data.towSubMsF = extractLong(msg, 20);
        packetUBXTIMTM2->data.accEst = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTM2->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTM2->callbackData->ch, &packetUBXTIMTM2->data.ch, sizeof(UBX_TIM_TM2_data_t));
          packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_TIM_TP && msg->len == UBX_TIM_TP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTP != nullptr)
      {
        packetUBXTIMTP->data.towMS = extractLong(msg, 0);
        packetUBXTIMTP->data.towSubMS = extractLong(msg, 4);
        packetUBXTIMTP->data.qErr = extractSignedLong(msg, 8);
        packetUBXTIMTP->data.week = extractInt(msg, 12);
        packetUBXTIMTP->data.flags.all = extractByte(msg, 14);
        packetUBXTIMTP->data.refInfo.all = extractByte(msg, 15);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTP->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTP->callbackData->towMS, &packetUBXTIMTP->data.towMS, sizeof(UBX_TIM_TP_data_t));
          packetUBXTIMTP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTP->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_MON:
    if (msg->id == UBX_MON_HW && msg->len == UBX_MON_HW_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMONHW != nullptr)
      {
        packetUBXMONHW->data.pinSel = extractLong(msg, 0);
        packetUBXMONHW->data.pinBank = extractLong(msg, 4);
        packetUBXMONHW->data.pinDir = extractLong(msg, 8);
        packetUBXMONHW->data.pinVal = extractLong(msg, 12);
        packetUBXMONHW->data.noisePerMS = extractInt(msg, 16);
        packetUBXMONHW->data.agcCnt = extractInt(msg, 18);
        packetUBXMONHW->data.aStatus = extractByte(msg, 20);
        packetUBXMONHW->data.aPower = extractByte(msg, 21);
        packetUBXMONHW->data.flags.all = extractByte(msg, 22);
        packetUBXMONHW->data.usedMask = extractLong(msg, 24);
        for (uint8_t i = 0; i < 17; i++)
          packetUBXMONHW->data.VP[i] = extractByte(msg, 28 + i);
        packetUBXMONHW->data.jamInd = extractByte(msg, 45);
        packetUBXMONHW->data.pinIrq = extractLong(msg, 48);
        packetUBXMONHW->data.pullH = extractLong(msg, 52);
        packetUBXMONHW->data.pullL = extractLong(msg, 56);

        // Mark all datums as fresh (not read before)
        packetUBXMONHW->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXMONHW->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXMONHW->callbackData->pinSel, &packetUBXMONHW->data.pinSel, sizeof(UBX_MON_HW_data_t));
          packetUBXMONHW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXMONHW->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_ESF
  case UBX_CLASS_ESF:
    if (msg->id == UBX_ESF_ALG && msg->len == UBX_ESF_ALG_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFALG != nullptr)
      {
        packetUBXESFALG->data.iTOW = extractLong(msg, 0);
        packetUBXESFALG->data.version = extractByte(msg, 4);
        packetUBXESFALG->data.flags.all = extractByte(msg, 5);
        packetUBXESFALG->data.error.all = extractByte(msg, 6);
        packetUBXESFALG->data.yaw = extractLong(msg, 8);
        packetUBXESFALG->data.pitch = extractSignedInt(msg, 12);
        packetUBXESFALG->data.roll = extractSignedInt(msg, 14);

        // Mark all datums as fresh (not read before)
        packetUBXESFALG->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFALG->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFALG->callbackData->iTOW, &packetUBXESFALG->data.iTOW, sizeof(UBX_ESF_ALG_data_t));
          packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_INS && msg->len == UBX_ESF_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFINS != nullptr)
      {
        packetUBXESFINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXESFINS->data.iTOW = extractLong(msg, 8);
        packetUBXESFINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXESFINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXESFINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXESFINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXESFINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXESFINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXESFINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFINS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFINS->callbackData->bitfield0.all, &packetUBXESFINS->data.bitfield0.all, sizeof(UBX_ESF_INS_data_t));
          packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_MEAS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFMEAS != nullptr)
      {
        packetUBXESFMEAS->data.timeTag = extractLong(msg, 0);
        packetUBXESFMEAS->data.flags.all = extractInt(msg, 4);
        packetUBXESFMEAS->data.id = extractInt(msg, 6);
        for (uint16_t i = 0; (i < DEF_MAX_NUM_ESF_MEAS) && (i < packetUBXESFMEAS->data.flags.bits.numMeas) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXESFMEAS->data.data[i].data.all = extractLong(msg, 8 + (i * 4));
        }
        if ((uint16_t)msg->len > (uint16_t)(8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4)))
          packetUBXESFMEAS->data.calibTtag = extractLong(msg, 8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4));

        // Check if we need to copy the data for the callback
        if (packetUBXESFMEAS->callbackData != nullptr) // If RAM has been allocated for the copy of the data
        {
          for (uint16_t i = 0; i < UBX_ESF_MEAS_CALLBACK_BUFFERS; i++)
          {
            if ((packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid & (1 << i)) == 0) // AND the data is stale
            {
              memcpy(&packetUBXESFMEAS->callbackData[i].timeTag, &packetUBXESFMEAS->data.timeTag, sizeof(UBX_ESF_MEAS_data_t));
              packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid |= (1 << i);
              break; // Only copy once
            }
          }
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_RAW)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFRAW != nullptr)
      {
        packetUBXESFRAW->data.numEsfRawBlocks = (msg->len - 4) / 8; // Record how many blocks were received. Could be 7 or 70 (ZED-F9R vs. NEO-M8U)
        for (uint16_t i = 0; (i < (DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS)) && ((i * 8) < (msg->len - 4)); i++)
        {
          packetUBXESFRAW->data.data[i].data.all = extractLong(msg, 4 + (i * 8));
          packetUBXESFRAW->data.data[i].sTag = extractLong(msg, 8 + (i * 8));
        }

        // Check if we need to copy the data for the callback
        if ((packetUBXESFRAW->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFRAW->callbackData->data[0].data.all, &packetUBXESFRAW->data.data[0].data.all, sizeof(UBX_ESF_RAW_data_t));
          packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_STATUS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFSTATUS != nullptr)
      {
        packetUBXESFSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXESFSTATUS->data.version = extractByte(msg, 4);
        packetUBXESFSTATUS->data.fusionMode = extractByte(msg, 12);
        packetUBXESFSTATUS->data.numSens = extractByte(msg, 15);
        for (uint16_t i = 0; (i < DEF_NUM_SENS) && (i < packetUBXESFSTATUS->data.numSens) && ((i * 4) < (msg->len - 16)); i++)
        {
          packetUBXESFSTATUS->data.status[i].sensStatus1.all = extractByte(msg, 16 + (i * 4) + 0);
          packetUBXESFSTATUS->data.status[i].sensStatus2.all = extractByte(msg, 16 + (i * 4) + 1);
          packetUBXESFSTATUS->data.status[i].freq = extractByte(msg, 16 + (i * 4) + 2);
          packetUBXESFSTATUS->data.status[i].faults.all = extractByte(msg, 16 + (i * 4) + 3);
        }

        // Mark all datums as fresh (not read before)
        packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFSTATUS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFSTATUS->callbackData->iTOW, &packetUBXESFSTATUS->data.iTOW, sizeof(UBX_ESF_STATUS_data_t));
          packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#endif
  case UBX_CLASS_MGA:
    if (msg->id == UBX_MGA_ACK_DATA0 && msg->len == UBX_MGA_ACK_DATA0_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGAACK != nullptr)
      {
        // Calculate how many ACKs are already stored in the ring buffer
        uint8_t ackBufferContains;
        if (packetUBXMGAACK->head >= packetUBXMGAACK->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          ackBufferContains = packetUBXMGAACK->head - packetUBXMGAACK->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_ACK_RINGBUFFER_LEN)
          ackBufferContains = ((uint8_t)(((uint16_t)packetUBXMGAACK->head + (uint16_t)UBX_MGA_ACK_DATA0_RINGBUFFER_LEN) - (uint16_t)packetUBXMGAACK->tail));
        }
        // Have we got space to store this ACK?
        if (ackBufferContains < (UBX_MGA_ACK_DATA0_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          packetUBXMGAACK->data[packetUBXMGAACK->head].type = extractByte(msg, 0);
          packetUBXMGAACK->data[packetUBXMGAACK->head].version = extractByte(msg, 1);
          packetUBXMGAACK->data[packetUBXMGAACK->head].infoCode = extractByte(msg, 2);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgId = extractByte(msg, 3);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[0] = extractByte(msg, 4);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[1] = extractByte(msg, 5);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[2] = extractByte(msg, 6);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[3] = extractByte(msg, 7);
          // Increment the head
          packetUBXMGAACK->head++;
          if (packetUBXMGAACK->head == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
            packetUBXMGAACK->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.println(F("processUBXpacket: packetUBXMGAACK is full. ACK will be lost!"));
          }
#endif
        }
      }
    }
    else if (msg->id == UBX_MGA_DBD && msg->len <= UBX_MGA_DBD_LEN) // Message length may be less than UBX_MGA_DBD_LEN. UBX_MGA_DBD_LEN is the maximum it will be.
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGADBD != nullptr)
      {
        // Calculate how many DBDs are already stored in the ring buffer
        uint8_t dbdBufferContains;
        if (packetUBXMGADBD->head >= packetUBXMGADBD->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          dbdBufferContains = packetUBXMGADBD->head - packetUBXMGADBD->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_DBD_RINGBUFFER_LEN)
          dbdBufferContains = ((uint8_t)(((uint16_t)packetUBXMGADBD->head + (uint16_t)UBX_MGA_DBD_RINGBUFFER_LEN) - (uint16_t)packetUBXMGADBD->tail));
        }
        // Have we got space to store this DBD?
        if (dbdBufferContains < (UBX_MGA_DBD_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          // We need to save the entire message - header, payload and checksum
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader1 = UBX_SYNCH_1;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader2 = UBX_SYNCH_2;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryClass = UBX_CLASS_MGA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryID = UBX_MGA_DBD;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenLSB = (uint8_t)(msg->len & 0xFF); // We need to store the length of the DBD entry. The entry itself does not contain a length...
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenMSB = (uint8_t)((msg->len >> 8) & 0xFF);
          for (uint16_t i = 0; i < msg->len; i++)
          {
            packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntry[i] = extractByte(msg, i);
          }
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumA = msg->checksumA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumB = msg->checksumB;
          // Increment the head
          packetUBXMGADBD->head++;
          if (packetUBXMGADBD->head == UBX_MGA_DBD_RINGBUFFER_LEN)
            packetUBXMGADBD->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            Serial.println(F("processUBXpacket: packetUBXMGADBD is full. DBD data will be lost!"));
          }
#endif
        }
      }
    }
    break;
#ifndef SFE_UBLOX_DISABLE_HNR
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_PVT && msg->len == UBX_HNR_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRPVT != nullptr)
      {
        packetUBXHNRPVT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRPVT->data.year = extractInt(msg, 4);
        packetUBXHNRPVT->data.month = extractByte(msg, 6);
        packetUBXHNRPVT->data.day = extractByte(msg, 7);
        packetUBXHNRPVT->data.hour = extractByte(msg, 8);
        packetUBXHNRPVT->data.min = extractByte(msg, 9);
        packetUBXHNRPVT->data.sec = extractByte(msg, 10);
        packetUBXHNRPVT->data.valid.all = extractByte(msg, 11);
        packetUBXHNRPVT->data.nano = extractSignedLong(msg, 12);
        packetUBXHNRPVT->data.gpsFix = extractByte(msg, 16);
        packetUBXHNRPVT->data.flags.all = extractByte(msg, 17);
        packetUBXHNRPVT->data.lon = extractSignedLong(msg, 20);
        packetUBXHNRPVT->data.lat = extractSignedLong(msg, 24);
        packetUBXHNRPVT->data.height = extractSignedLong(msg, 28);
        packetUBXHNRPVT->data.hMSL = extractSignedLong(msg, 32);
        packetUBXHNRPVT->data.gSpeed = extractSignedLong(msg, 36);
        packetUBXHNRPVT->data.speed = extractSignedLong(msg, 40);
        packetUBXHNRPVT->data.headMot = extractSignedLong(msg, 44);
        packetUBXHNRPVT->data.headVeh = extractSignedLong(msg, 48);
        packetUBXHNRPVT->data.hAcc = extractLong(msg, 52);
        packetUBXHNRPVT->data.vAcc = extractLong(msg, 56);
        packetUBXHNRPVT->data.sAcc = extractLong(msg, 60);
        packetUBXHNRPVT->data.headAcc = extractLong(msg, 64);

        // Mark all datums as fresh (not read before)
        packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRPVT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRPVT->callbackData->iTOW, &packetUBXHNRPVT->data.iTOW, sizeof(UBX_HNR_PVT_data_t));
          packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_ATT && msg->len == UBX_HNR_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRATT != nullptr)
      {
        packetUBXHNRATT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRATT->data.version = extractByte(msg, 4);
        packetUBXHNRATT->data.roll = extractSignedLong(msg, 8);
        packetUBXHNRATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXHNRATT->data.heading = extractSignedLong(msg, 16);
        packetUBXHNRATT->data.accRoll = extractLong(msg, 20);
        packetUBXHNRATT->data.accPitch = extractLong(msg, 24);
        packetUBXHNRATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXHNRATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRATT->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRATT->callbackData->iTOW, &packetUBXHNRATT->data.iTOW, sizeof(UBX_HNR_ATT_data_t));
          packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_INS && msg->len == UBX_HNR_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRINS != nullptr)
      {
        packetUBXHNRINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXHNRINS->data.iTOW = extractLong(msg, 8);
        packetUBXHNRINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXHNRINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXHNRINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXHNRINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXHNRINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXHNRINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXHNRINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRINS->callbackData != nullptr)                                  // If RAM has been allocated for the copy of the data
            && (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRINS->callbackData->bitfield0.all, &packetUBXHNRINS->data.bitfield0.all, sizeof(UBX_HNR_INS_data_t));
          packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          addedToFileBuffer = storePacket(msg);
        }
      }
    }
    break;
#endif
  }

  // Check if this UBX message should be added to the file buffer - if it has not been added already
  if ((!addedToFileBuffer) && (logThisUBX(msg->cls, msg->id)))
    storePacket(msg);
}




// Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GNSS::extractLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint32_t val = 0;
  for (uint8_t i = 0; i < 4; i++)
    val |= (uint32_t)msg->payload[spotToStart + i] << (8 * i);
  return (val);
}

// Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GNSS::extractInt(ubxPacket *msg, uint16_t spotToStart)
{
  uint16_t val = (uint16_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)msg->payload[spotToStart + 1] << 8 * 1;
  return (val);
}





// PRIVATE: Add a UBX packet to the file buffer
bool SFE_UBLOX_GNSS::storePacket(ubxPacket *msg)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == nullptr) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      Serial.println(F("storePacket: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  uint16_t totalLength = msg->len + 8; // Total length. Include sync chars, class, id, length and checksum bytes
  if (totalLength > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("storePacket: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // Store the two sync chars
  uint8_t sync_chars[] = {UBX_SYNCH_1, UBX_SYNCH_2};
  writeToFileBuffer(sync_chars, 2);

  // Store the Class & ID
  writeToFileBuffer(&msg->cls, 1);
  writeToFileBuffer(&msg->id, 1);

  // Store the length. Ensure length is little-endian
  uint8_t msg_length[2];
  msg_length[0] = msg->len & 0xFF;
  msg_length[1] = msg->len >> 8;
  writeToFileBuffer(msg_length, 2);

  // Store the payload
  writeToFileBuffer(msg->payload, msg->len);

  // Store the checksum
  writeToFileBuffer(&msg->checksumA, 1);
  writeToFileBuffer(&msg->checksumB, 1);

  return (true);
}




bool SFE_UBLOX_GNSS::initStorageNMEA()
{
  if (_storageNMEA != nullptr) // Check if storage already exists
    return true;

  _storageNMEA = new NMEA_STORAGE_t; // Allocate RAM for the main struct
  if (_storageNMEA == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initStorageNMEA: RAM alloc failed!"));
#endif
    return (false);
  }
  _storageNMEA->data = nullptr;

  _storageNMEA->data = new uint8_t[maxNMEAByteCount];
  if (_storageNMEA->data == nullptr)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initStorageNMEA: RAM alloc failed!"));
#endif
    return (false);
  }

  return (true);
}

// PRIVATE: Check how much space is available in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceAvailable(void)
{
  return (fileBufferSize - fileBufferSpaceUsed());
}

// PRIVATE: Check how much space is used in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceUsed(void)
{
  if (fileBufferHead >= fileBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (fileBufferHead - fileBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the fileBufferSize
    return ((uint16_t)(((uint32_t)fileBufferHead + (uint32_t)fileBufferSize) - (uint32_t)fileBufferTail));
  }
}


// PRIVATE: Write theBytes to the file buffer
void SFE_UBLOX_GNSS::writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&ubxFileBuffer[fileBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&ubxFileBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    fileBufferHead = bytesLeftToCopy;                                             // Update Head. The next byte written will be written here.
  }
  else
  {
    fileBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }

  // Update fileBufferMaxAvail if required
  uint16_t bytesInBuffer = fileBufferSpaceUsed();
  if (bytesInBuffer > fileBufferMaxAvail)
    fileBufferMaxAvail = bytesInBuffer;
}




// PRIVATE: Check how much space is used in the buffer
uint16_t SFE_UBLOX_GNSS::rtcmBufferSpaceUsed(void)
{
  if (rtcmBufferHead >= rtcmBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (rtcmBufferHead - rtcmBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the rtcmBufferSize
    return ((uint16_t)(((uint32_t)rtcmBufferHead + (uint32_t)rtcmBufferSize) - (uint32_t)rtcmBufferTail));
  }
}

// PRIVATE: Write theBytes to the RTCM buffer
void SFE_UBLOX_GNSS::writeToRTCMBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = rtcmBufferSize - rtcmBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&rtcmBuffer[rtcmBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&rtcmBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    rtcmBufferHead = bytesLeftToCopy;                                          // Update Head. The next byte written will be written here.
  }
  else
  {
    rtcmBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }
}


// Given a pointer, extract an signed integer with width bits, starting at bit start
int64_t SFE_UBLOX_GNSS::extractSignedBits(uint8_t *ptr, uint16_t start, uint16_t width)
{

  unsignedSigned64 result;
  result.unsigned64 = 0;

  uint64_t twosComplement = 0xFFFFFFFFFFFFFFFF;

  bool isNegative;

  uint16_t count = 0;
  uint8_t bitMask = 0x80;

  // Skip whole bytes (8 bits)
  ptr += start / 8;
  count += (start / 8) * 8;

  // Loop until we reach the start bit
  while (count < start)
  {
    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }
  }

  isNegative = *ptr & bitMask; // Record the first bit - indicates in the number is negative

  // We have reached the start bit and ptr is pointing at the correct byte
  // Now extract width bits, incrementing ptr and shifting bitMask as we go
  while (count < (start + width))
  {
    if (*ptr & bitMask)       // Is the bit set?
      result.unsigned64 |= 1; // Set the corresponding bit in result

    bitMask >>= 1;        // Shift the bit mask
    count++;              // Increment the count
    twosComplement <<= 1; // Shift the two's complement mask (clear LSB)

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }

    if (count < (start + width)) // Do we need to shift result?
      result.unsigned64 <<= 1;   // Shift the result
  }

  // Handle negative number
  if (isNegative)
    result.unsigned64 |= twosComplement; // OR in the two's complement mask

  return result.signed64;
}

void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
  (void)incoming;
}


// Given a pointer, extract an unsigned integer with width bits, starting at bit start
uint64_t SFE_UBLOX_GNSS::extractUnsignedBits(uint8_t *ptr, uint16_t start, uint16_t width)
{
  uint64_t result = 0;
  uint16_t count = 0;
  uint8_t bitMask = 0x80;

  // Skip whole bytes (8 bits)
  ptr += start / 8;
  count += (start / 8) * 8;

  // Loop until we reach the start bit
  while (count < start)
  {
    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }
  }

  // We have reached the start bit and ptr is pointing at the correct byte
  // Now extract width bits, incrementing ptr and shifting bitMask as we go
  while (count < (start + width))
  {
    if (*ptr & bitMask) // Is the bit set?
      result |= 1;      // Set the corresponding bit in result

    bitMask >>= 1; // Shift the bit mask
    count++;       // Increment the count

    if (bitMask == 0) // Have we counted 8 bits?
    {
      ptr++;          // Point to the next byte
      bitMask = 0x80; // Reset the bit mask
    }

    if (count < (start + width)) // Do we need to shift result?
      result <<= 1;              // Shift the result
  }

  return result;
}

// Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t SFE_UBLOX_GNSS::extractSignedLong(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned32 converter;
  converter.unsigned32 = extractLong(msg, spotToStart);
  return (converter.signed32);
}

// Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t SFE_UBLOX_GNSS::extractSignedInt(ubxPacket *msg, uint16_t spotToStart)
{
  unsignedSigned16 converter;
  converter.unsigned16 = extractInt(msg, spotToStart);
  return (converter.signed16);
}


















//done
bool SFE_UBLOX_GNSS::checkUblox(uint8_t requestedClass, uint8_t requestedID)
{
  return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}


















// Check if any callbacks are waiting to be processed
void SFE_UBLOX_GNSS::checkCallbacks(void)
{
  if (checkCallbacksReentrant == true) // Check for reentry (i.e. checkCallbacks has been called from inside a callback)
    return;

  checkCallbacksReentrant = true;
  if (packetUBXNAVPVT != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXNAVPVT->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXNAVPVT->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXNAVPVT->callbackPointerPtr(packetUBXNAVPVT->callbackData); // Call the callback
        }
        packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
  if (packetUBXRXMCOR != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMCOR->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMCOR->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMCOR->callbackPointerPtr(packetUBXRXMCOR->callbackData); // Call the callback
        }
        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }
      
  if (packetUBXRXMPMPmessage != nullptr)                                               // If RAM has been allocated for message storage
    if (packetUBXRXMPMPmessage->callbackData != nullptr)                               // If RAM has been allocated for the copy of the data
      if (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == true) // If the copy of the data is valid
      {
        if (packetUBXRXMPMPmessage->callbackPointerPtr != nullptr) // If the pointer to the callback has been defined
        {
          packetUBXRXMPMPmessage->callbackPointerPtr(packetUBXRXMPMPmessage->callbackData); // Call the callback
        }
        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
      }


  checkCallbacksReentrant = false;
}





































//done
// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool SFE_UBLOX_GNSS::setRXMPMPmessageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *))
{
  if (packetUBXRXMPMPmessage == nullptr)
    initPacketUBXRXMPMPmessage();        // Check that RAM has been allocated for the data
  if (packetUBXRXMPMPmessage == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMPMPmessage->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMPMPmessage->callbackData = new UBX_RXM_PMP_message_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMPMPmessage->callbackData == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("setAutoRXMPMPmessagecallbackPtr: RAM alloc failed!"));
    return (false);
  }

  packetUBXRXMPMPmessage->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

//done
// PRIVATE: Allocate RAM for packetUBXRXMPMPmessage and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMPMPmessage()
{
  packetUBXRXMPMPmessage = new UBX_RXM_PMP_message_t; // Allocate RAM for the main struct
  if (packetUBXRXMPMPmessage == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initPacketUBXRXMPMPmessage: RAM alloc failed!"));
    return (false);
  }
  packetUBXRXMPMPmessage->automaticFlags.flags.all = 0;
  packetUBXRXMPMPmessage->callbackPointerPtr = nullptr;
  packetUBXRXMPMPmessage->callbackData = nullptr;
  return (true);
}

















//done
void SFE_UBLOX_GNSS::softwareResetGNSSOnly()
{
  // Issue controlled software reset (GNSS only)
  uint8_t softwareResetGNSS[4] = {0, 0, 0x02, 0};
  cfgRst(softwareResetGNSS, 4);
}

//done
void SFE_UBLOX_GNSS::cfgRst(uint8_t *data, uint8_t len)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RST;
  packetCfg.len = len;
  packetCfg.startingSpot = 0;
  for (uint8_t i = 0; i < len; i++)
    payloadCfg[i] = *data++;
  sendCommand(&packetCfg, 0); // don't expect ACK
}





//done
void SFE_UBLOX_GNSS::setCommunicationBus(GNSSDeviceBus &theBus)
{
  _sfeBus = &theBus;
}

//done
bool SFE_UBLOX_GNSS::init(uint16_t maxWait, bool assumeSuccess)
{
  createLock(); // Create the lock semaphore - if needed

  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Create storage for RTCM data - only useful on systems where the GNSS is interfaced via SPI and you may want to write
  // data to another SPI device (e.g. Ethernet) in a safe way, avoiding processRTCM (which would be called _during_ checkUblox).
  createRTCMBuffer();

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("begin: isConnected - second attempt"));
    }
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("begin: isConnected - third attempt"));
    }
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("begin: third attempt failed. Assuming success..."));
    }
    return (true);
  }
  return (connected);
}

//done
void SFE_UBLOX_GNSS::setFileBufferSize(uint16_t bufferSize)
{
  fileBufferSize = bufferSize;
}

//done
bool SFE_UBLOX_GNSS::createFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize
  {
    if (_printDebug == true)
    {
      Serial.println(F("createFileBuffer: Warning. fileBufferSize is zero. Data logging is not possible."));
    }
    return (false);
  }

  if (ubxFileBuffer != nullptr) // Bail if RAM has already been allocated for the file buffer
  {                             // This will happen if you call .begin more than once - without calling .end first
    if (_printDebug == true)
    {
      Serial.println(F("createFileBuffer: Warning. File buffer already exists. Skipping..."));
    }
    return (false);
  }

  ubxFileBuffer = new uint8_t[fileBufferSize]; // Allocate RAM for the buffer

  if (ubxFileBuffer == nullptr) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("createFileBuffer: RAM alloc failed!"));
    }
    fileBufferSize = 0; // Set file buffer size so user can check with getFileBufferSize (ubxFileBuffer is protected)
    return (false);
  }
  if (_printDebug == true)
  {
    Serial.print(F("createFileBuffer: fileBufferSize is: "));
    Serial.println(fileBufferSize);
  }

  fileBufferHead = 0; // Initialize head and tail
  fileBufferTail = 0;

  return (true);
}

//done
bool SFE_UBLOX_GNSS::setPacketCfgPayloadSize(size_t payloadSize)
{
  bool success = true;

  if ((payloadSize == 0) && (payloadCfg != nullptr))
  {
    // Zero payloadSize? Dangerous! But we'll free the memory anyway...
    delete[] payloadCfg; // Created with new[]
    payloadCfg = nullptr;
    packetCfg.payload = payloadCfg;
    packetCfgPayloadSize = payloadSize;
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("setPacketCfgPayloadSize: Zero payloadSize!"));
  }

  else if (payloadCfg == nullptr) // Memory has not yet been allocated - so use new
  {
    payloadCfg = new uint8_t[payloadSize];
    packetCfg.payload = payloadCfg;

    if (payloadCfg == nullptr)
    {
      success = false;
      packetCfgPayloadSize = 0;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        Serial.println(F("setPacketCfgPayloadSize: RAM alloc failed!"));
    }
    else
      packetCfgPayloadSize = payloadSize;

    if ((packetCfgPayloadSize + 8) > spiBufferSize) // Warn the user if spiBuffer is now smaller than the packetCfg payload. Could result in lost data
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        Serial.println(F("setPacketCfgPayloadSize: packetCfgPayloadSize > spiBufferSize!"));
    }
  }

  else // Memory has already been allocated - so resize
  {
    uint8_t *newPayload = new uint8_t[payloadSize];

    if (newPayload == nullptr) // Check if the alloc was successful
    {
      success = false;                                           // Report failure. Don't change payloadCfg, packetCfg.payload or packetCfgPayloadSize
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        Serial.println(F("setPacketCfgPayloadSize: RAM resize failed!"));
    }
    else
    {
      memcpy(newPayload, payloadCfg, payloadSize <= packetCfgPayloadSize ? payloadSize : packetCfgPayloadSize); // Copy as much existing data as we can
      delete[] payloadCfg;                                                                                      // Free payloadCfg. Created with new[]
      payloadCfg = newPayload;                                                                                  // Point to the newPayload
      packetCfg.payload = payloadCfg;                                                                           // Update the packet pointer
      packetCfgPayloadSize = payloadSize;                                                                       // Update the packet payload size
    }

    if ((packetCfgPayloadSize + 8) > spiBufferSize) // Warn the user if spiBuffer is now smaller than the packetCfg payload. Could result in lost data
    {
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("setPacketCfgPayloadSize: packetCfgPayloadSize > spiBufferSize!"));
    }
  }

  return (success);
}

//done
bool SFE_UBLOX_GNSS::createRTCMBuffer(void)
{
  if (rtcmBufferSize == 0) // Bail if the user has not called setRTCMBufferSize
  {
    return (false);
  }

  if (rtcmBuffer != nullptr) // Bail if RAM has already been allocated for the buffer
  {                          // This will happen if you call .begin more than once - without calling .end first
    return (false);
  }

  rtcmBuffer = new uint8_t[rtcmBufferSize]; // Allocate RAM for the buffer

  if (rtcmBuffer == nullptr) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      Serial.println(F("createRTCMBuffer: RAM alloc failed!"));
    }
    rtcmBufferSize = 0; // Set buffer size so user can check with getRTCMBufferSize (rtcmBuffer is protected)
    return (false);
  }

  rtcmBufferHead = 0; // Initialize head and tail
  rtcmBufferTail = 0;

  return (true);
}

//done
bool SFE_UBLOX_GNSS::isConnected(uint16_t maxWait)
{
  if (_commType == COMM_TYPE_I2C)
  {
    if (!ping())
      return false; // Sensor did not ack
  }

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles, let's request the config for most appropriate port
  uint8_t en;
  if (_commType == COMM_TYPE_I2C)
    return getVal8(UBLOX_CFG_I2CINPROT_UBX, &en, VAL_LAYER_RAM, maxWait);
  else
    return false;
}

//done
// Given a spot, extract a byte from the payload
uint8_t SFE_UBLOX_GNSS::extractByte(ubxPacket *msg, uint16_t spotToStart)
{
  return (msg->payload[spotToStart]);
}

//done
bool SFE_UBLOX_GNSS::getVal8(uint32_t key, uint8_t *val, uint8_t layer, uint16_t maxWait)
{
  bool result = getVal(key, layer, maxWait) == SFE_UBLOX_STATUS_DATA_RECEIVED;
  if (result)
    *val = extractByte(&packetCfg, 8);
  return result;
}
//done
uint8_t SFE_UBLOX_GNSS::getVal8(uint32_t key, uint8_t layer, uint16_t maxWait) // Unsafe overload
{
  uint8_t result = 0;
  getVal8(key, &result, layer, maxWait);
  return result;
}

//done
sfe_ublox_status_e SFE_UBLOX_GNSS::getVal(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALGET;
  packetCfg.len = 4 + 4 * 1; // While multiple keys are allowed, we will send only one key at a time
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  // VALGET uses different memory layer definitions to VALSET
  // because it can only return the value for one layer.
  // So we need to fiddle the layer here.
  // And just to complicate things further, the ZED-F9P only responds
  // correctly to layer 0 (RAM) and layer 7 (Default)!
  uint8_t getLayer = VAL_LAYER_DEFAULT; // 7 is the "Default Layer"
  if (layer == VAL_LAYER_RAM)           // Did the user request the RAM layer?
  {
    getLayer = 0; // Layer 0 is RAM
  }
  else if (layer == VAL_LAYER_BBR) // Did the user request the BBR layer?
  {
    getLayer = 1; // Layer 1 is BBR
  }
  else if (layer == VAL_LAYER_FLASH) // Did the user request the Flash layer?
  {
    getLayer = 2; // Layer 2 is Flash
  }

  payloadCfg[0] = 0;        // Message Version - set to 0
  payloadCfg[1] = getLayer; // Layer

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK;    // Mask off the size identifer bits
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;
  if (_printDebug == true)
  {
    Serial.print(F("getVal key: 0x"));
    Serial.print(key, HEX);
    Serial.println();
  }

  // Send VALGET command with this key

  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);
  if (_printDebug == true)
  {
    Serial.print(F("getVal: sendCommand returned: "));
    Serial.println(statusString(retVal));
  }
  // Verify the response is the correct length as compared to what the user called (did the module respond with 8-bits but the user called getVal32?)
  // Response is 8 bytes plus cfg data
  // if(packet->len > 8+1)

  // The response is now sitting in payload, ready for extraction
  return (retVal);
}

//done
sfe_ublox_status_e SFE_UBLOX_GNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{

  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet
  if (_printDebug == true)
  {
    Serial.print(F("\nSending: "));
    printPacket(outgoingUBX, true); // Always print payload
  }

  if (_commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
      if (_printDebug == true)
      {
        Serial.println(F("Send I2C Command failed"));
      }
      return retVal;
    }
  }

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
      if (_printDebug == true)
      {
        Serial.println(F("sendCommand: Waiting for ACK response"));
      }
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
      if (_printDebug == true)
      {
        Serial.println(F("sendCommand: Waiting for No ACK response"));
      }
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }

  return retVal;
}
//done
void SFE_UBLOX_GNSS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

//done
// Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e SFE_UBLOX_GNSS::sendI2cCommand(ubxPacket *outgoingUBX)
{
  // From the integration guide:
  // "The receiver does not provide any write access except for writing UBX and NMEA messages to the
  //  receiver, such as configuration or aiding data. Therefore, the register set mentioned in section Read
  //  Access is not writeable. Following the start condition from the master, the 7-bit device address and
  //  the RW bit (which is a logic low for write access) are clocked onto the bus by the master transmitter.
  //  The receiver answers with an acknowledge (logic low) to indicate that it is responsible for the given
  //  address. Now, the master can write 2 to N bytes to the receiver, generating a stop condition after the
  //  last byte being written. The number of data bytes must be at least 2 to properly distinguish from
  //  the write access to set the address counter in random read accesses."
  // I take two things from this:
  // 1) We do not need to write 0xFF to point at register 0xFF. We're already pointing at it.
  // 2) We must always write at least 2 bytes, otherwise it looks like we are starting to do a read.
  // Point 2 is important. It means:
  // * In this function:
  //     if we do multiple writes (because we're trying to write more than i2cTransactionSize),
  //     we may need to write one byte less in the penultimate write to ensure we always have two bytes left for the final write.
  // * In pushRawData:
  //     if there is one byte to write, or one byte left to write, we need to do the same thing and may need to store a single
  //     byte until pushRawData is called again.

  // The total number of bytes to be written is: payload len + 8
  // UBX_SYNCH_1
  // UBX_SYNCH_2
  // cls
  // id
  // len (MSB)
  // len (LSB)
  // < payload >
  // checksumA
  // checksumB

  // i2cTransactionSize will be at least 8. We don't need to check for smaller values than that.

  uint16_t bytesLeftToSend = outgoingUBX->len; // How many bytes remain to be sent
  uint16_t startSpot = 0;                      // Payload pointer

  // Check if we can send all the data in one transfer?
  if (bytesLeftToSend + 8 <= i2cTransactionSize)
  {
    uint8_t buf[i2cTransactionSize];
    buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
    buf[1] = UBX_SYNCH_2; // b
    buf[2] = outgoingUBX->cls;
    buf[3] = outgoingUBX->id;
    buf[4] = outgoingUBX->len & 0xFF; // LSB
    buf[5] = outgoingUBX->len >> 8;   // MSB
    uint16_t i = 0;
    for (; i < outgoingUBX->len; i++)
      buf[i + 6] = outgoingUBX->payload[startSpot + i];
    buf[i + 6] = outgoingUBX->checksumA;
    buf[i + 7] = outgoingUBX->checksumB;

    if (writeBytes(buf, bytesLeftToSend + 8) != bytesLeftToSend + 8)
      return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
  }

  else
  {
    uint8_t buf[6];
    buf[0] = UBX_SYNCH_1; // μ - oh ublox, you're funny. I will call you micro-blox from now on.
    buf[1] = UBX_SYNCH_2; // b
    buf[2] = outgoingUBX->cls;
    buf[3] = outgoingUBX->id;
    buf[4] = outgoingUBX->len & 0xFF; // LSB
    buf[5] = outgoingUBX->len >> 8;   // MSB

    if (writeBytes(buf, 6) != 6)
      return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

    // If bytesLeftToSend is zero, that's OK.
    // If bytesLeftToSend is >= 2, that's OK.
    // But if bytesLeftToSend is 1, we need to carry that byte over and send it with the checksum bytes
    while (bytesLeftToSend > 1)
    {
      uint16_t len = bytesLeftToSend; // How many bytes should we actually write?
      if (len > i2cTransactionSize)   // Limit len to i2cTransactionSize
        len = i2cTransactionSize;

      bytesLeftToSend -= len; // Calculate how many bytes will be left after we do this write

      // Write a portion of the payload to the bus.
      // Keep going until we've sent as many bytes as we can in this transmission (x == len)
      // or until we reach the end of the payload ((startSpot + x) == (outgoingUBX->len))
      uint16_t x = len;
      if ((startSpot + x) >= (outgoingUBX->len))
        x = outgoingUBX->len - startSpot;

      if (writeBytes(&outgoingUBX->payload[startSpot], x) != x)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

      startSpot += x;
    }

    // Finally, write any left-over bytes plus the checksum
    if (bytesLeftToSend == 1)
    {
      buf[0] = outgoingUBX->payload[startSpot];
      buf[1] = outgoingUBX->checksumA;
      buf[2] = outgoingUBX->checksumB;

      if (writeBytes(buf, 3) != 3)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
    }
    else
    {
      buf[0] = outgoingUBX->checksumA;
      buf[1] = outgoingUBX->checksumB;

      if (writeBytes(buf, 2) != 2)
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK
    }
  }

  return (SFE_UBLOX_STATUS_SUCCESS);
}

//done
uint8_t SFE_UBLOX_GNSS::writeBytes(uint8_t *data, uint8_t length)
{
  return _sfeBus->writeBytes(data, length);
}

//done
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() < (startTime + (unsigned long)maxTime))
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: valid data and valid ACK received after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by an automatic packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: no data and valid ACK after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: data being OVERWRITTEN after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: CRC failed after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
        if (_printDebug == true)
        {
          Serial.print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        // if (_printDebug == true)
        // {
        //   Serial.print(F("waitForACKResponse: valid data after "));
        //   Serial.print(millis() - startTime);
        //   Serial.println(F(" msec. Waiting for ACK."));
        // }
      }

    } // checkUbloxInternal == true

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }           // while (millis() < (startTime + (unsigned long)maxTime))

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
    if (_printDebug == true)
    {
      Serial.print(F("waitForACKResponse: TIMEOUT with valid data after "));
      Serial.print(millis() - startTime);
      Serial.println(F(" msec. "));
    }
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
  }

  if (_printDebug == true)
  {
    Serial.print(F("waitForACKResponse: TIMEOUT after "));
    Serial.print(millis() - startTime);
    Serial.println(F(" msec."));
  }

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// mafroud done
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {

      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          Serial.print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          Serial.print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   Serial.print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   Serial.print(millis() - startTime);
        //   Serial.print(F(" msec. Class: 0x"));
        //   Serial.print(outgoingUBX->cls, HEX);
        //   Serial.print(F(" ID: 0x"));
        //   Serial.print(outgoingUBX->id, HEX);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          Serial.print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
          Serial.print(millis() - startTime);
          Serial.println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
      }
    }

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    Serial.print(F("waitForNoACKResponse: TIMEOUT after "));
    Serial.print(millis() - startTime);
    Serial.println(F(" msec. No packet received."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}


//done
bool SFE_UBLOX_GNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{

  bool ok = false;
  if (_commType == COMM_TYPE_I2C)
    ok = (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  return ok;
}

//done
// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool SFE_UBLOX_GNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    // From the u-blox integration manual:
    // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
    //  address and thus allows any register to be read. The second "current address" form omits the
    //  register address. If this second form is used, then an address pointer in the receiver is used to
    //  determine which register to read. This address pointer will increment after each read unless it
    //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
    //  unaltered."
    uint16_t bytesAvailable = available();

    if (bytesAvailable == 0)
    {

      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);
    }

    if (_printDebug == true)
    {
      Serial.print(F("checkUbloxI2C: "));
      Serial.print(bytesAvailable);
      Serial.println(F(" bytes available"));
    }

    while (bytesAvailable)
    {
      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t buf[i2cTransactionSize];
      uint8_t bytesReturned = readBytes(buf, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          process(buf[x], incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
      {
        // Something has gone very wrong. Sensor did not respond - or a bus error happened...
        if (_resetCurrentSentenceOnBusError)
          currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          Serial.println(F("checkUbloxI2C: bus error? bytesReturned != bytesToRead"));
        }

        return (false);
      }

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()


//doone
uint16_t SFE_UBLOX_GNSS::available()
{
  return _sfeBus->available();
}


//done
uint8_t SFE_UBLOX_GNSS::readBytes(uint8_t *data, uint8_t length)
{
  return _sfeBus->readBytes(data, length);
}


//done
bool SFE_UBLOX_GNSS::getModuleInfo(uint16_t maxWait)
{
  if (moduleSWVersion == nullptr)
    initModuleSWVersion();        // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
    return (false);

  // Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;

  packetCfg.len = 0;
  packetCfg.startingSpot = 40; // Start at first "extended software information" string

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
    return (false);                                                       // If command send fails then bail

  // Payload should now contain ~220 characters (depends on module type)

  // We will step through the payload looking at each extension field of 30 bytes
  char *ptr;
  uint8_t fwProtMod = 0; // Flags to show if we extracted the FWVER, PROTVER and MOD data
  for (uint8_t extensionNumber = 0; extensionNumber < ((packetCfg.len - 40) / 30); extensionNumber++)
  {
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "FWVER="); // Check for FWVER (should be in extension 1)
    if (ptr != nullptr)
    {
      ptr += strlen("FWVER="); // Point to the firmware type (HPG etc.)
      int i = 0;
      while ((i < firmwareTypeLen) && (*ptr != '\0') && (*ptr != ' ')) // Extract the firmware type (3-7 chars)
        moduleSWVersion->firmwareType[i++] = *ptr++;
      moduleSWVersion->firmwareType[i] = '\0'; // NULL-terminate

      if (*ptr == ' ')
        ptr++; // Skip the space

      int firmwareHi = 0;
      int firmwareLo = 0;
      int scanned = sscanf(ptr, "%d.%d", &firmwareHi, &firmwareLo);
      if (scanned == 2) // Check we extracted the firmware version successfully
      {
        moduleSWVersion->firmwareVersionHigh = firmwareHi;
        moduleSWVersion->firmwareVersionLow = firmwareLo;
        fwProtMod |= 0x01; // Record that we got the FWVER
      }
    }
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "PROTVER="); // Check for PROTVER (should be in extension 2)
    if (ptr != nullptr)
    {
      ptr += strlen("PROTVER="); // Point to the protocol version
      int protHi = 0;
      int protLo = 0;
      int scanned = sscanf(ptr, "%d.%d", &protHi, &protLo);
      if (scanned == 2) // Check we extracted the firmware version successfully
      {
        moduleSWVersion->protocolVersionHigh = protHi;
        moduleSWVersion->protocolVersionLow = protLo;
        fwProtMod |= 0x02; // Record that we got the PROTVER
      }
    }
    ptr = strstr((const char *)&payloadCfg[(30 * extensionNumber)], "MOD="); // Check for MOD (should be in extension 3)
    if (ptr != nullptr)
    {
      ptr += strlen("MOD="); // Point to the module name
      int i = 0;
      while ((i < moduleNameMaxLen) && (*ptr != '\0') && (*ptr != ' ')) // Copy the module name
        moduleSWVersion->moduleName[i++] = *ptr++;
      moduleSWVersion->moduleName[i] = '\0'; // NULL-terminate
      fwProtMod |= 0x04;                     // Record that we got the MOD
    }
  }

  if (fwProtMod == 0x07) // Did we extract all three?
  {
    if (_printDebug == true)
    {
      Serial.print(F("getModuleInfo: FWVER: "));
      Serial.print(moduleSWVersion->firmwareVersionHigh);
      Serial.print(F("."));
      Serial.println(moduleSWVersion->firmwareVersionLow);
      Serial.print(F("getModuleInfo: PROTVER: "));
      Serial.print(moduleSWVersion->protocolVersionHigh);
      Serial.print(F("."));
      Serial.println(moduleSWVersion->protocolVersionLow);
      Serial.print(F("getModuleInfo: MOD: "));
      Serial.println(moduleSWVersion->moduleName);
    }

    moduleSWVersion->moduleQueried = true; // Mark this data as new

    return (true);
  }

  return (false); // We failed
}


//done
// PRIVATE: Allocate RAM for moduleSWVersion and initialize it
bool SFE_UBLOX_GNSS::initModuleSWVersion()
{
  moduleSWVersion = new moduleSWVersion_t; // Allocate RAM for the main struct
  if (moduleSWVersion == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initModuleSWVersion: RAM alloc failed!"));
    return (false);
  }
  moduleSWVersion->protocolVersionHigh = 0; // Clear the contents
  moduleSWVersion->protocolVersionLow = 0;
  moduleSWVersion->firmwareVersionHigh = 0;
  moduleSWVersion->firmwareVersionLow = 0;
  moduleSWVersion->firmwareType[0] = 0;
  moduleSWVersion->moduleName[0] = 0;
  moduleSWVersion->moduleQueried = false;
  return (true);
}


//done
// Get the firmware version of the u-blox module we're communicating with
uint8_t SFE_UBLOX_GNSS::getFirmwareVersionHigh(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->firmwareVersionHigh);
}
uint8_t SFE_UBLOX_GNSS::getFirmwareVersionLow(uint16_t maxWait)
{
  if (!prepareModuleInfo(maxWait))
    return 0;
  return (moduleSWVersion->firmwareVersionLow);
}

//done
bool SFE_UBLOX_GNSS::prepareModuleInfo(uint16_t maxWait)
{
  if (moduleSWVersion == nullptr)
    initModuleSWVersion();        // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == nullptr) // Bail if the RAM allocation failed
    return (false);

  if (moduleSWVersion->moduleQueried == false)
    getModuleInfo(maxWait);

  return moduleSWVersion->moduleQueried;
}
//done
const char *SFE_UBLOX_GNSS::getFirmwareType(uint16_t maxWait)
{
  static const char unknownFirmware[4] = {'T', 'B', 'D', '\0'};
  if (!prepareModuleInfo(maxWait))
    return unknownFirmware;
  return ((const char *)moduleSWVersion->firmwareType);
}


//done
bool SFE_UBLOX_GNSS::setVal8(uint32_t key, uint8_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[1] = {value};
  return (setValN(key, val, 1, layer, maxWait));
}

//done
bool SFE_UBLOX_GNSS::setNavigationFrequency(uint8_t navFreq, uint8_t layer, uint16_t maxWait)
{
  if (navFreq == 0) // Return now if navFreq is zero
    return (false);

  if (navFreq > 40)
    navFreq = 40; // Limit navFreq to 40Hz so i2cPollingWait is set correctly

  // Adjust the I2C polling timeout based on update rate
  // Do this even if the sendCommand fails
  i2cPollingWaitNAV = 1000 / (((int)navFreq) * 4);                                                // This is the number of ms to wait between checks for new I2C data. Max is 250. Min is 6.
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  uint16_t measurementRate = 1000 / navFreq;

  return setVal16(UBLOX_CFG_RATE_MEAS, measurementRate, layer, maxWait);
}

//done
// Set the DGNSS differential mode
bool SFE_UBLOX_GNSS::setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode, uint8_t layer, uint16_t maxWait)
{
  return setVal8(UBLOX_CFG_NAVHPG_DGNSSMODE, (uint8_t)dgnssMode, layer, maxWait);
}

//done
bool SFE_UBLOX_GNSS::setUART2Input(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_UART2INPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_UART2INPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_UART2INPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_UART2INPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}

//done
bool SFE_UBLOX_GNSS::addCfgValset8(uint32_t key, uint8_t value)
{
  uint8_t val[1] = {value};
  return (addCfgValsetN(key, val, 1));
}

//done
bool SFE_UBLOX_GNSS::sendCfgValset(uint16_t maxWait)
{
  if (_numCfgKeys == 0)
    return true; // Nothing to send...

  // Send VALSET command with this key and value
  bool success = sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK

  if (success)
    _numCfgKeys = 0;

  return success;
}

//done
// Configure a port to input UBX, NMEA, RTCM3, SPARTN or a combination thereof
bool SFE_UBLOX_GNSS::setI2CInput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_I2CINPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_I2CINPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_I2CINPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait);  // This will be NACK'd if the module does not support RTCM3
  result |= setVal8(UBLOX_CFG_I2CINPROT_SPARTN, (comSettings & COM_TYPE_SPARTN) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support SPARTN
  return result;
}
//done
bool SFE_UBLOX_GNSS::setI2COutput(uint8_t comSettings, uint8_t layer, uint16_t maxWait)
{
  bool result = newCfgValset(layer);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_UBX, (comSettings & COM_TYPE_UBX) == 0 ? 0 : 1);
  result &= addCfgValset8(UBLOX_CFG_I2COUTPROT_NMEA, (comSettings & COM_TYPE_NMEA) == 0 ? 0 : 1);
  result &= sendCfgValset(maxWait);
  result |= setVal8(UBLOX_CFG_I2COUTPROT_RTCM3X, (comSettings & COM_TYPE_RTCM3) == 0 ? 0 : 1, layer, maxWait); // This will be NACK'd if the module does not support RTCM3
  return result;
}

//done
bool SFE_UBLOX_GNSS::setValN(uint32_t key, uint8_t *value, uint8_t N, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + N; // 4 byte header, 4 byte key ID, N bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[i + 4] = key >> (8 * i); // Key

  // Load user's value
  for (uint8_t i = 0; i < N; i++)
    payloadCfg[i + 8] = *value++;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

//done
bool SFE_UBLOX_GNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  uint8_t val[2] = {(uint8_t)(value >> 0), (uint8_t)(value >> 8)};
  return (setValN(key, val, 2, layer, maxWait));
}
//done
bool SFE_UBLOX_GNSS::setValSigned16(uint32_t key, int16_t value, uint8_t layer, uint16_t maxWait)
{
  unsignedSigned16 converter;
  converter.signed16 = value;
  return (setVal16(key, converter.unsigned16, layer, maxWait));
}

//done
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
bool SFE_UBLOX_GNSS::newCfgValset(uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4; // 4 byte header
  packetCfg.startingSpot = 0;

  _numCfgKeys = 0;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // All done
  return (true);
}

//done
bool SFE_UBLOX_GNSS::addCfgValsetN(uint32_t key, uint8_t *value, uint8_t N)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("addCfgValsetN: autosend"));
    if (sendCommand(&packetCfg) != SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
      return false;
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeys = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - (4 + N)))
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("addCfgValsetN: packetCfgPayloadSize reached!"));

    return false;
  }

  if (_numCfgKeys == CFG_VALSET_MAX_KEYS)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("addCfgValsetN: key limit reached!"));
    return false;
  }

  // Load key into outgoing payload
  key &= ~UBX_CFG_SIZE_MASK; // Mask off the size identifer bits
  for (uint8_t i = 0; i < 4; i++)
    payloadCfg[packetCfg.len + i] = key >> (8 * i); // Key

  // Load user's value
  for (uint8_t i = 0; i < N; i++)
    payloadCfg[packetCfg.len + i + 4] = *value++; // Value

  // Update packet length: 4 byte key ID, 8 bytes of value
  packetCfg.len = packetCfg.len + 4 + N;

  _numCfgKeys++;

  // All done
  return (true);
}

//done
// SPARTN dynamic keys
//"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
//"Every time the 'current' key is expired, 'next' takes its place."
//"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
// The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
bool SFE_UBLOX_GNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key)
{
  uint8_t *binaryKey = new uint8_t[keyLengthBytes]; // Allocate memory to store the binaryKey

  if (binaryKey == nullptr)
  {
    if (_printDebug == true)
      Serial.println(F("setDynamicSPARTNKey: binaryKey RAM allocation failed!"));
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes * 2); i += 2)
  {
    if ((key[i] >= '0') && (key[i] <= '9'))
    {
      binaryKey[i >> 1] = (key[i] - '0') << 4;
    }
    else if ((key[i] >= 'a') && (key[i] <= 'f'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'a') << 4;
    }
    else if ((key[i] >= 'A') && (key[i] <= 'F'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key[i + 1] >= '0') && (key[i + 1] <= '9'))
    {
      binaryKey[i >> 1] |= key[i + 1] - '0';
    }
    else if ((key[i + 1] >= 'a') && (key[i + 1] <= 'f'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'a';
    }
    else if ((key[i + 1] >= 'A') && (key[i + 1] <= 'F'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKey(keyLengthBytes, validFromWno, validFromTow, (const uint8_t *)binaryKey);

  delete[] binaryKey; // Free the memory allocated for binaryKey

  return (ok);
}

//done
bool SFE_UBLOX_GNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes + 12;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x01; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes;
  payloadCfg[6] = validFromWno & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno >> 8;
  payloadCfg[8] = validFromTow & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow >> 24) & 0xFF;

  memcpy(&payloadCfg[12], key, keyLengthBytes);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}
//done
bool SFE_UBLOX_GNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                                        uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2)
{
  uint8_t *binaryKey1 = new uint8_t[keyLengthBytes1]; // Allocate memory to store binaryKey1

  if (binaryKey1 == nullptr)
  {
    if (_printDebug == true)
      Serial.println(F("setDynamicSPARTNKeys: binaryKey1 RAM allocation failed!"));
    return (false);
  }

  uint8_t *binaryKey2 = new uint8_t[keyLengthBytes2]; // Allocate memory to store binaryKey2

  if (binaryKey2 == nullptr)
  {
    if (_printDebug == true)
      Serial.println(F("setDynamicSPARTNKeys: binaryKey2 RAM allocation failed!"));
    delete[] binaryKey1;
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes1 * 2); i += 2)
  {
    if ((key1[i] >= '0') && (key1[i] <= '9'))
    {
      binaryKey1[i >> 1] = (key1[i] - '0') << 4;
    }
    else if ((key1[i] >= 'a') && (key1[i] <= 'f'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'a') << 4;
    }
    else if ((key1[i] >= 'A') && (key1[i] <= 'F'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key1[i + 1] >= '0') && (key1[i + 1] <= '9'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] - '0';
    }
    else if ((key1[i + 1] >= 'a') && (key1[i + 1] <= 'f'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'a';
    }
    else if ((key1[i + 1] >= 'A') && (key1[i + 1] <= 'F'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes2 * 2); i += 2)
  {
    if ((key2[i] >= '0') && (key2[i] <= '9'))
    {
      binaryKey2[i >> 1] = (key2[i] - '0') << 4;
    }
    else if ((key2[i] >= 'a') && (key2[i] <= 'f'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'a') << 4;
    }
    else if ((key2[i] >= 'A') && (key2[i] <= 'F'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key2[i + 1] >= '0') && (key2[i + 1] <= '9'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] - '0';
    }
    else if ((key2[i + 1] >= 'a') && (key2[i + 1] <= 'f'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'a';
    }
    else if ((key2[i + 1] >= 'A') && (key2[i + 1] <= 'F'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKeys(keyLengthBytes1, validFromWno1, validFromTow1, (const uint8_t *)binaryKey1,
                              keyLengthBytes2, validFromWno2, validFromTow2, (const uint8_t *)binaryKey2);

  delete[] binaryKey1; // Free the memory allocated for binaryKey1
  delete[] binaryKey2; // Free the memory allocated for binaryKey2

  return (ok);
}
//done
bool SFE_UBLOX_GNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                                        uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes1 + (size_t)keyLengthBytes2 + 20;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x02; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes1;
  payloadCfg[6] = validFromWno1 & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno1 >> 8;
  payloadCfg[8] = validFromTow1 & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow1 >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow1 >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow1 >> 24) & 0xFF;
  payloadCfg[12] = 0x00; // reserved1
  payloadCfg[13] = keyLengthBytes2;
  payloadCfg[14] = validFromWno2 & 0xFF; // validFromWno little-endian
  payloadCfg[15] = validFromWno2 >> 8;
  payloadCfg[16] = validFromTow2 & 0xFF; // validFromTow little-endian
  payloadCfg[17] = (validFromTow2 >> 8) & 0xFF;
  payloadCfg[18] = (validFromTow2 >> 16) & 0xFF;
  payloadCfg[19] = (validFromTow2 >> 24) & 0xFF;

  memcpy(&payloadCfg[20], key1, keyLengthBytes1);
  memcpy(&payloadCfg[20 + keyLengthBytes1], key2, keyLengthBytes2);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}

//done
// Enable automatic navigation message generation by the GNSS. This changes the way getPVT works.
bool SFE_UBLOX_GNSS::setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint8_t layer, uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoPVT(true, false, layer, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVT failed

  if (packetUBXNAVPVT->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVT->callbackData = new UBX_NAV_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVT->callbackData == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("setAutoPVTcallbackPtr: RAM alloc failed!"));
    return (false);
  }

  packetUBXNAVPVT->callbackPointerPtr = callbackPointerPtr; // RAM has been allocated so now update the pointer

  return (true);
}
//done
// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVT(bool enable, uint8_t layer, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, true, layer, maxWait);
}
//done
// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVT(bool enable, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, implicitUpdate, layer, maxWait);
}
//done
// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVTrate(uint8_t rate, bool implicitUpdate, uint8_t layer, uint16_t maxWait)
{
  if (packetUBXNAVPVT == nullptr)
    initPacketUBXNAVPVT();        // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  uint32_t key = UBLOX_CFG_MSGOUT_UBX_NAV_PVT_I2C;
  bool ok = setVal8(key, rate, layer, maxWait);
  if (ok)
  {
    packetUBXNAVPVT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ok;
}

//done
bool SFE_UBLOX_GNSS::setRXMCORcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_COR_data_t *))
{
  if (packetUBXRXMCOR == nullptr)
    initPacketUBXRXMCOR();        // Check that RAM has been allocated for the data
  if (packetUBXRXMCOR == nullptr) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMCOR->callbackData == nullptr) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMCOR->callbackData = new UBX_RXM_COR_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMCOR->callbackData == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("setAutoRXMCORcallbackPtr: RAM alloc failed!"));
    return (false);
  }

  packetUBXRXMCOR->callbackPointerPtr = callbackPointerPtr;
  return (true);
}


//done
// PRIVATE: Allocate RAM for packetUBXRXMCOR and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMCOR()
{
  packetUBXRXMCOR = new UBX_RXM_COR_t; // Allocate RAM for the main struct
  if (packetUBXRXMCOR == nullptr)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      Serial.println(F("initPacketUBXRXMCOR: RAM alloc failed!"));
    return (false);
  }
  packetUBXRXMCOR->automaticFlags.flags.all = 0;
  packetUBXRXMCOR->callbackPointerPtr = nullptr;
  packetUBXRXMCOR->callbackData = nullptr;
  return (true);
}


















SfeI2C::SfeI2C(void) : _i2cPort{nullptr}, _address{0}
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////
// I2C init()
//
// Methods to init/setup this device.
// The caller can provide a Wire Port, or this class will use the default.
// Always update the address in case the user has changed the I2C address - see Example9
bool SfeI2C::init(I2C_INTERFACE &wirePort, uint8_t address, bool bInit)
{
    // if we don't have a wire port already
    if (!_i2cPort)
    {
      _i2cPort = &wirePort;

      if (bInit)
        _i2cPort->begin();
    }

    _address = address;

    return true;
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // I2C init()
  //
  // Methods to init/setup this device.
  // The caller can provide a Wire Port, or this class will use the default.
bool SfeI2C::init(uint8_t address)
{
    return init(Wire_r, address);
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // ping()
  //
  // Is a device connected?
bool SfeI2C::ping()
{

    if (!_i2cPort)
      return false;

    _i2cPort->beginTransmission(_address);
    return _i2cPort->endTransmission() == 0;
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // available()
  //
  // Checks how many bytes are waiting in the GNSS's I2C buffer
  // It does this by reading registers 0xFD and 0xFE
  //
  // From the u-blox integration manual:
  // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
  //  address and thus allows any register to be read. The second "current address" form omits the
  //  register address. If this second form is used, then an address pointer in the receiver is used to
  //  determine which register to read. This address pointer will increment after each read unless it
  //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
  //  unaltered."

uint16_t SfeI2C::available()
{

    if (!_i2cPort)
      return false;

    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
      return (0); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint16_t bytesReturned = _i2cPort->requestFrom(_address, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
      return (0); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    return (bytesAvailable);
}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // writeBytes()

uint8_t SfeI2C::writeBytes(uint8_t *dataToWrite, uint8_t length)
{
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    _i2cPort->beginTransmission(_address);
    uint8_t written = _i2cPort->write((const uint8_t *)dataToWrite, length);
    if (_i2cPort->endTransmission() == 0)
      return written;

    return 0;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // readBytes()

uint8_t SfeI2C::readBytes(uint8_t *data, uint8_t length)
{
    if (!_i2cPort)
      return 0;

    if (length == 0)
      return 0;

    uint8_t bytesReturned = _i2cPort->requestFrom(_address, length);

    for (uint8_t i = 0; i < bytesReturned; i++)
      *data++ = _i2cPort->read();

    return bytesReturned;
}







