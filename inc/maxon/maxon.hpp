#ifndef MAXON_H
#define MAXON_H

#include <cstdint>


/***************************************************************************************************************************
*                                                     TYPE DEFINITIONS                                                     *
****************************************************************************************************************************/

// 8 bits unsigned integer
typedef uint8_t BYTE;

// 16 bits unsigned integer
typedef uint16_t WORD;

// 32 bits unsigned integer
typedef uint32_t DWORD;

// 32 bits signed integer
typedef int32_t BOOL;

// TODO: void pointer?
typedef void* HANDLE;


/***************************************************************************************************************************
*                                                         CONSTANTS                                                         *
****************************************************************************************************************************/

// The device name given to the VCS_OpenDevice function.
#define DEVICE_NAME "EPOS4"

// The maximum length for the protocol stack name given to the VCS_OpenDevice function.
#define MAX_PROTOCOL_STACK_NAME_LENGTH 100

// The maximum length for the interface name given to the VCS_OpenDevice function.
#define MAX_INTERFACE_NAME_LENGTH 100

// The maximum length for the port name given to the VCS_OpenDevice function.
#define MAX_PORT_NAME_LENGTH 100

// The value corresponding to the Profile Position Mode.
#define OMD_PROFILE_POSITION_MODE 1

// The value corresponding to the Profile Velocity Mode.
#define OMD_PROFILE_VELOCITY_MODE 3

// The value corresponding to the Homing Mode.
#define OMD_HOMING_MODE 6


/***************************************************************************************************************************
*                                                      CLASS DEFINITIONS                                                   *
****************************************************************************************************************************/


/*
  The interface for the EPOS 4 controller, containing the functions called by the functions of the MAXON class.
*/
class EPOS4_Interface {

  public:

    /*
      VCS_OpenDevice opens the port to send and receive commands. Ports can be RS232, USB, and CANopen interfaces.

      Parameters:
       - DeviceName (char*): name of connected device (EPOS, EPOS2 or EPOS4)
       - ProtocolStackName (char*): name of used communication protocol (MAXON_RS232, MAXON SERIAL V2 or CANopen)
       - InterfaceName (char*): name of interface (RS232, USB, IXXAT_<<BoardName>> <<DeviceNumber>>, 
          Kvaser_<<BoardName>> <<DeviceNumber>>, NI_<<BoardName>> <<DeviceNumber>> or Vector_<<BoardName>> <<DeviceNumber>>)
       - PortName (char*): name of port (COM1, COM2, USB0, USB1, CAN0, CAN1, ...)

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (HANDLE): handle for communication port access, nonzero if successful, otherwise "0"
    */
    virtual HANDLE VCS_OpenDevice(char* DeviceName, char* ProtocolStackName, char* InterfaceName, char* PortName, DWORD* pErrorCode) = 0;

    /*
      VCS_GetProtocolStackNameSelection returns all available protocol stack names.

      Parameters:
       - DeviceName (char*): device name
       - StartOfSelection (BOOL): true to get the first selection string and false to get the next selection string
       - MaxStrSize (WORD): reserved memory size for the name

      Return parameters:
       - pProtocolStackNameSel (char*): pointer to available protocol stack name
       - pEndOfSelection (BOOL): true if no more string available and false otherwise
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetProtocolStackNameSelection(char* DeviceName, BOOL StartOfSelection, char* pProtocolStackNameSel, WORD MaxStrSize,
                                              BOOL* pEndOfSelection, DWORD* pErrorCode) = 0;

    /*
      VCS_GetInterfaceNameSelection returns all available interface names.

      Parameters:
       - DeviceName (char*): device name
       - ProtocolStackName (char*): protocol stack name
       - StartOfSelection (BOOL): true to get the first selection string, false to get the next selection string
       - MaxStrSize (WORD): reserved memory size for the interface name

      Return parameters:
       - pInterfaceNameSel (char*): name of interface
       - pEndOfSelection (BOOL*): true if no more string available, false otherwise
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetInterfaceNameSelection(char* DeviceName, char* ProtocolStackName, BOOL StartOfSelection, char* pInterfaceNameSel,
                                              WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode) = 0;

    /*
      VCS_GetPortNameSelection returns all available port names.

      Parameters:
       - DeviceName (char*): device name
       - ProtocolStackName (char*): protocol stack name
       - InterfaceName (char*): interface name
       - StartOfSelection (BOOL): true to get the first selection string, false to get the next selection string
       - MaxStrSize (WORD): reserved memory size for the port name

      Return parameters:
       - pPortSel (char*): pointer to port name
       - pEndOfSelection (BOOL*): true if no more string available, false otherwise
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetPortNameSelection(char* DeviceName, char* ProtocolStackName, char* InterfaceName, BOOL StartOfSelection,
                                    char* pPortSel, WORD MaxStrSize, BOOL* pEndOfSelection, DWORD* pErrorCode) = 0;

    /*
      VCS_CloseDevice closes the port and releases it for other applications. If no opened ports are available,
      the function returns “0”

      Parameters:
       - KeyHandle (HANDLE): handle for port access

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
    */
    virtual BOOL VCS_CloseDevice(HANDLE KeyHandle, DWORD* pErrorCode) = 0;

    /*
      VCS_CloseAllDevices closes all opened ports and releases them for other applications. If no opened 
      ports are available, the function returns “0”.

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_CloseAllDevices(DWORD* pErrorCode) = 0;

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
    virtual BOOL VCS_SetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToWrite,
                        DWORD* pNbOfBytesWritten, DWORD* pErrorCode) = 0;

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
    virtual BOOL VCS_GetObject(HANDLE KeyHandle, WORD NodeId, WORD ObjectIndex, BYTE ObjectSubIndex, void* pData, DWORD NbOfBytesToRead,
                        DWORD* pNbOfBytesRead, DWORD* pErrorCode) = 0;

    /*
      VCS_Restore restores all default parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_Restore(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode) = 0;

    /*
      VCS_Store stores all parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_Store(HANDLE KeyHandle, WORD NodeId, DWORD* pErrorCode) = 0;

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
    virtual BOOL VCS_GetDcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent,
                                    WORD* pThermalTimeConstant, DWORD* pErrorCode) = 0;

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
    virtual BOOL VCS_GetEcMotorParameter(HANDLE KeyHandle, WORD NodeId, WORD* pNominalCurrent, WORD* pMaxOutputCurrent,
                                    WORD* pThermalTimeConstant, BYTE* pNbOfPolePairs, DWORD* pErrorCode) = 0;

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
    virtual BOOL VCS_SetOperationMode(HANDLE KeyHandle, WORD NodeId, __int8 Mode, DWORD* pErrorCode) = 0;

    /*
      VCS_SetState sets the actual state machine state.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - State (WORD): value of the state machine

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_SetState(HANDLE KeyHandle, WORD NodeId, WORD State, DWORD* pErrorCode) = 0;

    /*
      VCS_SetPositionProfile sets the position profile parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - ProfileVelocity (DWORD): position profile velocity
       - ProfileAcceleration (DWORD): position profile acceleration
       - ProfileDeceleration (DWORD): position profile deceleration

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_SetPositionProfile(HANDLE KeyHandle, WORD NodeId, DWORD ProfileVelocity, DWORD ProfileAcceleration, 
                                DWORD ProfileDeceleration, DWORD* pErrorCode) = 0;

    /*
      VCS_GetPositionProfile returns the position profile parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pProfileVelocity (DWORD*): position profile velocity
       - pProfileAcceleration (DWORD*): position profile acceleration
       - pProfileDeceleration (DWORD*): position profile deceleration
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetPositionProfile(HANDLE KeyHandle, WORD NodeId, DWORD* pProfileVelocity, DWORD* pProfileAcceleration, 
                                DWORD* pProfileDeceleration, DWORD* pErrorCode) = 0;

    /*
      VCS_SetVelocityProfile sets the velocity profile parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node Id of the addressed device
       - ProfileAcceleration (DWORD): velocity profile acceleration
       - ProfileDeceleration (DWORD): velocity profile deceleration

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_SetVelocityProfile(HANDLE KeyHandle, WORD NodeId, DWORD ProfileAcceleration, 
                                    DWORD ProfileDeceleration, DWORD* pErrorCode) = 0;

    /*
      VCS_GetVelocityProfile returns the velocity profile parameters.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters;
       - pProfileAcceleration (DWORD*): velocity profile acceleration
       - pProfileDeceleration (DWORD*): velocity profile deceleration
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetVelocityProfile(HANDLE KeyHandle, WORD NodeId, DWORD* pProfileAcceleration, 
                                    DWORD* pProfileDeceleration, DWORD* pErrorCode) = 0;

    /*
      VCS_MoveToPosition starts movement with position profile to target position.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - TargetPosition (long): target position
       - Absolute (BOOL): true to start an absolute and false to start a relative movement
       - Immediately (BOOL): true to start immediately and false to wait for the end of the last positioning

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_MoveToPosition(HANDLE KeyHandle, WORD NodeId, long TargetPosition, BOOL Absolute,
                                BOOL Immediately, DWORD* pErrorCode) = 0;

    /*
      VCS_MoveWithVelocity starts the movement with velocity profile to target velocity.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device
       - TargetVelocity (long): target velocity

      Return parameters:
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_MoveWithVelocity(HANDLE KeyHandle, WORD NodeId, long TargetVelocity, DWORD* pErrorCode) = 0;

    /*
      VCS_GetPositionIs returns the position actual value.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pPositionIs (long*): position actual value
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetPositionIs(HANDLE KeyHandle, WORD NodeId, long* pPositionIs, DWORD* pErrorCode) = 0;

    /*
      VCS_GetVelocityIs reads the velocity actual value.

      Parameters:
       - KeyHandle (HANDLE): handle for port access
       - NodeId (WORD): node ID of the addressed device

      Return parameters:
       - pVelocityIs (long*): velocity actual value
       - pErrorCode (DWORD*): error information on the executed function
       - return value (BOOL): nonzero if successful, otherwise "0"
    */
    virtual BOOL VCS_GetVelocityIs(HANDLE KeyHandle, WORD NodeId, long* pVelocityIs, DWORD* pErrorCode) = 0;

};


/*
  The Maxon class, which will be used to program the Maxon motor.
*/
class Maxon {

    public:

      /*
        The constructor.

        Parameters:
         - Interface (EPOS4_Interface*): the interface used to control the board
      */
      Maxon(EPOS4_Interface* Interface);

      /*
        Rotates the maxon motor by a given angle. If the target is more than a 15° angle from the
        reference, the motor will only rotate to the 15° angle. And if the target is less than the
        -15° angle from the reference, it will only rotate to the -15° angle.

        Parameters:
         - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
      */
      void Rotate(double Angle);

      /*
        Rotates the maxon motor by a given angle in a given time. If the target is more than a 15°
        angle from the reference, the motor will only rotate to the 15° angle. And if the target is
        less than the -15° angle from the reference, it will only rotate to the -15° angle. If the
        time given is negative or null, nothing will happen.

        Parameters:
         - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
         - Time (double): the time the motor will take to rotate to the target, in seconds
      */
      void Rotate(double Angle, double Time);

      /*
        Rotates the maxon motor to a given angle from the reference. If the angle is more than 15°,
        the motor will only rotate to 15°. And if it is less than -15°, the motor will only rotate
        to the -15° angle.

        Parameters:
         - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
      */
      void RotateFromRef(double Angle);

      /*
        Rotates the maxon motor to a given angle from the reference in a given time. If the angle
        is more than 15°, the motor will only rotate to 15°. And if it is less than -15°, the motor
        will only rotate to the -15° angle. If the time given is negative or null, nothing will happen.

        Parameters:
         - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
         - Time (double): the time the motor will take to rotate to the target, in seconds
      */
      void RotateFromRef(double Angle, double Time);

      /*
        Rotates the maxon motor with a given angular velocity. If the given angular velocity is negative or
        null, nothing will happen.

        Parameters:
         - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
      */
      void RotateWithVelocity(double Velocity);

      /*
        Rotates the maxon motor with a given angular velocity and with a given time to reach the target. If
        the given angular velocity is negative or null, nothing will happen. If the time given is negative
        or null, nothing will happen.

        Parameters:
         - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
         - Time (double): the time the motor will take to reach the target, in seconds
      */
      void RotateWithVelocity(double Velocity, double Time);

      /*
        Ends the communication with the maxon motor. The functions called on the Maxon object will not work
        anymore. This function must be called everytime we are done using the Maxon object.
      */
      void EndCommunication(void);

      /*
        Gets the current angle of the maxon motor.

        Return parameters:
         - return value (long): the current angle, in degrees
      */
      long GetCurrentAngle(void);

      /*
        Gets the current angular velocity of the maxon motor.

        Return parameters:
         - return value (long): the current angular velocity, in revolutions per minute
      */
      long GetCurrentVelocity(void);


    private:

      // the EPOS 4 interface used to control the maxon motor
      EPOS4_Interface* Interface;

      // the handle for port access
      HANDLE KeyHandle;

      // the node ID of the addressed device
      WORD NodeId;

};

#endif