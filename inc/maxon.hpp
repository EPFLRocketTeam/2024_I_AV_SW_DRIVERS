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
       - ProtocolStackName (char*): name of used communication protocol
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
    BOOL VCS_CloseAllDevices(DWORD* pErrorCode)

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