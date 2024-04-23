#include "inc/maxon/maxon.hpp"
#include <cstdlib>


/*
  A constructor.

  Parameters:
   - Interface (EPOS4_Interface*): the interface used to control the board

  Return parameters:
   - pError (bool*): false if successful, otherwise true
*/
Maxon::Maxon(EPOS4_Interface* InterfaceParam, bool* pError) {

    // initialize the attributes
    Interface = InterfaceParam;
    KeyHandle = NULL;
    NodeId = 0;
    *pError = true;

    // determine the protocol stack name
    BOOL endOfSelection = false;
    DWORD errorCode = 0;
    char protocolStackName[MAX_PROTOCOL_STACK_NAME_LENGTH];
    if (!Interface->VCS_GetProtocolStackNameSelection(DEVICE_NAME, true, protocolStackName, MAX_PROTOCOL_STACK_NAME_LENGTH, &endOfSelection, &errorCode)) return;
    
    // determine the interface name
    char interfaceName[MAX_INTERFACE_NAME_LENGTH];
    if (!Interface->VCS_GetInterfaceNameSelection(DEVICE_NAME, protocolStackName, true, interfaceName, MAX_INTERFACE_NAME_LENGTH, &endOfSelection, &errorCode)) return;

    // determine the port name
    char portName[MAX_PORT_NAME_LENGTH];
    if (!Interface->VCS_GetPortNameSelection(DEVICE_NAME, protocolStackName, interfaceName, true, portName, MAX_PORT_NAME_LENGTH, &endOfSelection, &errorCode)) return;

    // open the port to send and receive commands and save the handle
    KeyHandle = Interface->VCS_OpenDevice(DEVICE_NAME, protocolStackName, interfaceName, portName, &errorCode);
    if (KeyHandle <= 0) {
        KeyHandle = NULL;
        return;
    }

    // TODO: set default values

    *pError = false;
    NodeId = 1;
    RefAngle = DEFAULT_REF_ANGLE;
    Resolution = DEFAULT_RESOLUTION;
    MaxAngle = DEFAULT_MAX_ANGLE;
};

/*
  A constructor.

  Parameters:
   - Interface (EPOS4_Interface*): the interface used to control the board
   - ProtocolStackName (char*): name of used communication protocol (MAXON_RS232, MAXON SERIAL V2 or CANopen)
   - InterfaceName (char*): name of interface (RS232, USB, IXXAT_<<BoardName>> <<DeviceNumber>>, 
        Kvaser_<<BoardName>> <<DeviceNumber>>, NI_<<BoardName>> <<DeviceNumber>> or Vector_<<BoardName>> <<DeviceNumber>>)
   - PortName (char*): name of port (COM1, COM2, USB0, USB1, CAN0, CAN1, ...)

  Return parameters:
   - pError (bool*): false if successful, otherwise true
*/
Maxon::Maxon(EPOS4_Interface* InterfaceParam, char* ProtocolStackName, char* InterfaceName, char* PortName, bool* pError) {

    // initialize the attributes
    Interface = InterfaceParam;
    KeyHandle = NULL;
    NodeId = 0;
    DWORD errorCode = 0;
    *pError = true;

    // check the arguments
    if (!ProtocolStackName || !InterfaceName || !PortName) return;

    // open the port to send and receive commands and save the handle
    KeyHandle = Interface->VCS_OpenDevice(DEVICE_NAME, ProtocolStackName, InterfaceName, PortName, &errorCode);
    if (KeyHandle <= 0) {
        KeyHandle = NULL;
        return;
    }

    // TODO: set default values

    *pError = false;
    NodeId = 1;
    RefAngle = DEFAULT_REF_ANGLE;
    Resolution = DEFAULT_RESOLUTION;
    MaxAngle = DEFAULT_MAX_ANGLE;
};

/*
  Rotates the maxon motor by a given angle. If the target is more than the maximum angle away from the
  reference, the motor will only rotate to the maximum angle. And if the target is less than the opposite
  of the maximum angle from the reference, it will only rotate to the opposite of the maximum angle.

  Parameters:
   - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
*/
void Maxon::Rotate(double Angle) {
    if (KeyHandle == NULL) return;
    long currAngle = GetCurrentAngle();
    RotateFromRef(currAngle + Angle);
};

/*
  Rotates the maxon motor by a given angle in a given time. If the target is more than the maximum
  angle away from the reference, the motor will only rotate to the maximum angle. And if the target is
  less than the opposite of the maximum angle away from the reference, it will only rotate to the opposite
  of the maximum angle. If the time given is negative or null, nothing will happen.

  Parameters:
   - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
   - Time (double): the time the motor will take to rotate to the target, in seconds
*/
void Maxon::Rotate(double Angle, double Time) {
    if (KeyHandle == NULL) return;
    long currAngle = GetCurrentAngle();
    RotateFromRef(currAngle + Angle);
};

/*
  Rotates the maxon motor to a given angle from the reference. If the angle is more than the maximum angle,
  the motor will only rotate to the maximum angle. And if it is less than the opposite of the maximum angle, 
  the motor will only rotate to the opposite of the maximum angle.

  Parameters:
   - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
*/
void Maxon::RotateFromRef(double Angle) {
    if (KeyHandle == NULL) return;
    if (Angle > MaxAngle) {
        Angle = MaxAngle;
    } else if (Angle < -MaxAngle) {
        Angle = -MaxAngle;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // compute the target position
    long targetPos = Angle * REDUCTION * ((double) Resolution) / TURN;
    Interface->VCS_MoveToPosition(KeyHandle, NodeId, targetPos, false, true, &errorCode);
};

/*
  Rotates the maxon motor to a given angle from the reference in a given time. If the angle
  is more than 15°, the motor will only rotate to 15°. And if it is less than -15°, the motor
  will only rotate to the -15° angle. If the time given is negative or null, nothing will happen.

  Parameters:
   - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
   - Time (double): the time the motor will take to rotate to the target, in seconds
*/
void Maxon::RotateFromRef(double Angle, double Time) {
    if (KeyHandle == NULL || Time <= 0) return;
    if (Angle > MaxAngle) {
        Angle = MaxAngle;
    } else if (Angle < -MaxAngle) {
        Angle = -MaxAngle;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // get the current position profile parameters
    DWORD velocity = 0;
    DWORD acceleration = 0;
    DWORD deceleration = 0;
    if (!Interface->VCS_GetPositionProfile(KeyHandle, NodeId, &velocity, &acceleration, &deceleration, &errorCode)) return;

    // update the parameters for the target to be reached in time Time (the velocity is in revolutions per minute)
    long currAngle = GetCurrentAngle();
    velocity = abs(currAngle - Angle) * REDUCTION / (TURN * Time * SEC_TO_MIN);
    if (!Interface->VCS_SetPositionProfile(KeyHandle, NodeId, velocity, acceleration, deceleration, &errorCode)) return;

    // compute the target position
    long targetPos = Angle * REDUCTION * ((double) Resolution) / TURN;
    Interface->VCS_MoveToPosition(KeyHandle, NodeId, targetPos, false, true, &errorCode);
};

/*
  Rotates the maxon motor with a given angular velocity. If the given angular velocity is negative or
  null, nothing will happen.

  Parameters:
   - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
*/
void Maxon::RotateWithVelocity(double Velocity) {
    if (KeyHandle == NULL || Velocity <= 0) return;

    // TODO: limit the angle

    // set the operation mode to the profile velocity mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_VELOCITY_MODE, &errorCode)) return;
    
    // update the target velocity to the given velocity
    long targetVelocity = Velocity * REDUCTION;
    Interface->VCS_MoveWithVelocity(KeyHandle, NodeId, targetVelocity, &errorCode);
};

/*
  Rotates the maxon motor with a given angular velocity and with a given time to reach the target. If
  the given angular velocity is negative or null, nothing will happen. If the time given is negative
  or null, nothing will happen.

  Parameters:
   - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
   - Time (double): the time the motor will take to reach the target, in seconds
*/
void Maxon::RotateWithVelocity(double Velocity, double Time) {
    if (KeyHandle == NULL || Velocity <= 0 || Time <= 0) return;

    // TODO: limit the angle

    // set the operation mode to the profile velocity mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_VELOCITY_MODE, &errorCode)) return;

    // get the current velocity profile parameters
    DWORD acceleration = 0;
    DWORD deceleration = 0;
    if (!Interface->VCS_GetVelocityProfile(KeyHandle, NodeId, &acceleration, &deceleration, &errorCode)) return;

    // update the acceleration/deceleration for the target to be reached in time Time
    long currVelocity = GetCurrentVelocity();
    if (currVelocity < Velocity) {
        acceleration = (Velocity - currVelocity) / Time;
    } else if (currVelocity > Velocity) {
        deceleration = (currVelocity - Velocity) / Time;
    }
    if (!Interface->VCS_SetVelocityProfile(KeyHandle, NodeId, acceleration, deceleration, &errorCode)) return;
    
    // update the target velocity to the given velocity
    long targetVelocity = Velocity * REDUCTION;
    Interface->VCS_MoveWithVelocity(KeyHandle, NodeId, targetVelocity, &errorCode);
};

/*
  Ends the communication with the maxon motor. The functions called on the Maxon object will not work
  anymore. This function must be called everytime we are done using the Maxon object.
*/
void Maxon::EndCommunication(void) {
    if (KeyHandle == NULL) return;

    DWORD errorCode = 0;
    Interface->VCS_CloseDevice(KeyHandle, &errorCode);
    KeyHandle = NULL;
    NodeId = 0;
};

/*
  Gets the current angle of the maxon motor.

  Return parameters:
   - return value (long): the current angle, in degrees
*/
long Maxon::GetCurrentAngle(void) {
    if (KeyHandle == NULL) return 0.;

    // get the current position
    DWORD errorCode = 0;
    long position = 0;
    if (!Interface->VCS_GetPositionIs(KeyHandle, NodeId, &position, &errorCode)) return 0.;

    // compute the corresponding angle in degrees
    position = position * TURN / (REDUCTION * ((double) Resolution));
    return position;
};

/*
  Gets the current angular velocity of the maxon motor.

  Return parameters:
   - return value (long): the current angular velocity, in revolutions per minute
*/
long Maxon::GetCurrentVelocity(void) {
    if (KeyHandle == NULL) return 0.;

    // get the current position
    DWORD errorCode = 0;
    long velocity = 0;
    if (!Interface->VCS_GetVelocityIs(KeyHandle, NodeId, &velocity, &errorCode)) return 0.;

    // compute the actual angular velocity
    velocity = velocity / REDUCTION;
    return velocity;
};

/*
  Get the current reference angle (home/neutral position).

  Return parameters:
   - return value (double): the current reference angle, in degrees
*/
double Maxon::GetReferenceAngle(void) {
    if (KeyHandle == NULL) return 0.;
    return RefAngle;
};

/*
  Set the current reference angle to a new value (home/neutral position).

  Parameters:
   - Angle (double): the new reference angle, in degrees
*/
void Maxon::SetReferenceAngle(double Angle) {
    if (KeyHandle == NULL) return;

    // set the operational mode to "homing mode"
    DWORD errorCode = 0;
    if (!Interface->VCS_ActivateHomingMode(KeyHandle, NodeId, &errorCode)) return;

    // get the current homing mode parameters
    DWORD homingAcceleration = 0;
    DWORD speedSwitch = 0;
    DWORD speedIndex = 0;
    long homeOffset = 0;
    WORD currentThreshold = 0;
    long homePosition = 0;
    if (!Interface->VCS_GetHomingParameter( KeyHandle, NodeId, &homingAcceleration, &speedSwitch, &speedIndex, &homeOffset, 
                                            &currentThreshold, &homePosition, &errorCode)) return;

    // update the homing position (reference angle)
    homePosition = Angle * REDUCTION * ((double) Resolution) / TURN;
    if (!Interface->VCS_SetHomingParameter( KeyHandle, NodeId, homingAcceleration, speedSwitch, speedIndex, homeOffset, 
                                            currentThreshold, homePosition, &errorCode)) return;

    RefAngle = Angle;
};

/*
  Get the current resolution in angle.

  Return parameters:
   - return value (int): the current resolution in angle, in pulse per turn
*/
int Maxon::GetResolution(void) {
    if (KeyHandle == NULL) return 0.;
    return Resolution;
};

/*
  Set the current resolution in angle to a new value.

  Parameters:
   - NewResolution (int): the new resolution in angle, in pulse per turn
*/
void Maxon::SetResolution(int NewResolution) {
    if (KeyHandle == NULL) return;

    // get the incremental encoder parameters
    DWORD encoderResolution = 0;
    BOOL invertedPolarity = 0;
    DWORD errorCode = 0;
    if (!Interface->VCS_GetIncEncoderParameter(KeyHandle, NodeId, &encoderResolution, &invertedPolarity, &errorCode)) return;

    // set the new resolution
    encoderResolution = NewResolution;
    if (!Interface->VCS_SetIncEncoderParameter(KeyHandle, NodeId, encoderResolution, invertedPolarity, &errorCode)) return;

    Resolution = NewResolution;
};

/*
  Get the current maximum angle for the motors (with respect to the neutral position).

  Return parameters:
   - return value (double): current maximum angle, in degrees
*/
double Maxon::GetMaxAngle(void) {
    if (KeyHandle == NULL) return 0.;
    return MaxAngle;
};

/*
  Set the current maximum angle for the motors to a new value (with respect to the neutral position).

  Parameters:
   - Angle (double): the new maximum angle, in degrees
*/
void Maxon::SetMaxAngle(double Angle) {
    MaxAngle = Angle;
};