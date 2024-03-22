#ifndef MAXON_H
#define MAXON_H


/*
  The interface for the EPOS 4 controller, containing the functions called by the functions of the MAXON class.
*/
class EPOS4_Interface {

    /*
      VCS_OpenDevice opens the port to send and receive commands. Ports can be RS232, USB, and CANopen interfaces.

      Parameters:
       - DeviceName (char*): name of connected device (EPOS, EPOS2 or EPOS4)
       - ProtocolStackName (char*): name of used communication protocol (MAXON_RS232, MAXON SERIAL V2 or CANopen)
       - InterfaceName (char*): name of interface (RS232, USB,  IXXAT_<<BoardName>> <<DeviceNumber>>, 
          Kvaser_<<BoardName>> <<DeviceNumber>>, NI_<<BoardName>> <<DeviceNumber>> or Vector_<<BoardName>> <<DeviceNumber>>)
       - PortName (char*): name of port (COM1, COM2, USB0, USB1, CAN0, CAN1, ...)

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (HANDLE): handle for communication port access, nonzero if successful, otherwise "0"
    */
    HANDLE VCS_OpenDevice(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, DWORD* pErrorCode);

    /*
      VCS_CloseDevice closes the port and releases it for other applications. If no opened ports are available,
      the function returns “0”

      Parameters:
       - KeyHandle (HANDLE): handle for port access

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
    */
    BOOL VCS_CloseDevice(HANDLE KeyHandle, DWORD* pErrorCode);

    /*
      VCS_CloseAllDevices closes all opened ports and releases them for other applications. If no opened 
      ports are available, the function returns “0”.

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_CloseAllDevices(DWORD* pErrorCode);

    /*
      VCS_SetObject writes an object value at the given index and subindex.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - ObjectIndex (WORD): object index
       - ObjectSubIndex (BYTE): object subindex
       - pData (void*): object data
       - NbOfBytesToWrite (DWORD): object length to write (number of bytes)

      Return parameters:
       - pNbOfBytesWritten (DWORD*): object length written (number of bytes)
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_SetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToWrite,
                        DWORD* pNbOfBytesWritten, DWORD* pErrorCode);

    /*
      VCS_GetObject reads an object value at the given index and subindex.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - ObjectIndex (WORD): object index
       - ObjectSubIndex (BYTE): object subindex
       - NbOfBytesToRead (DWORD): object length to read (number of bytes)

      Return parameters:
       - pData (void*): object data
       - pNbOfBytesRead (DWORD*): object length read (number of bytes)
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzeron if successful, otherwise "0"
    */
    BOOL VCS_GetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToRead,
                        DWORD* pNbOfBytesRead, DWORD* pErrorCode);

    /*
      VCS_Restore restores all default parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_Restore(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode);

    /*
      VCS_Store stores all parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_Store(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode);

    /*
      VCS_GetDcMotorParameter reads all DC motor parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pNominalCurrent (WORD*): maximal continuous current
       - pMaxOutputCurrent (WORD*): maximal peak current
       - pThermalTimeConstant (WORD*): thermal time constant winding
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_GetDcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent,
                                    WORD* pThermalTimeConstant, DWORD* pErrorCode);

    /*
      VCS_GetEcMotorParameter reads all EC motor parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pNominalCurrent (WORD*): maximal continuous current
       - pMaxOutputCurrent (WORD*): maximal peak current
       - pThermalTimeConstant (WORD*): thermal time constant winding
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_GetEcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent,
                                    WORD* pThermalTimeConstant, BYTE* pNbOfPolePairs, DWORD* pErrorCode);

    /*
      VCS_SetOperationMode sets the operation mode.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - Mode (__int8): operation mode

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_SetOperationMode(HANDLE KeyHandle, WORD NodeId, __int8 Mode, DWORD* pErrorCode);

    /*
      VCS_SetState reads the actual state machine state.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - State (WORD): value of the state machine

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    BOOL VCS_SetState(HANDLE KeyHandle, WORD NodeId, WORD State, DWORD* pErrorCode);

};


/*
  
*/
class MAXON {

    public:

        /*
          Constructor
        */
        MAXON();




    private:



};






#endif