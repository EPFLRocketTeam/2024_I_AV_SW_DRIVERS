#include "inc/maxon/maxon.hpp"
#include <cstdlib>


/*
  The constructor.

  Parameters:
   - Interface (EPOS4_Interface*): the interface used to control the board
*/
Maxon::Maxon(EPOS4_Interface* InterfaceParam) {

    // initialize the attributes
    Interface = InterfaceParam;
    KeyHandle = NULL;
    NodeId = 0;

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

    // TODO: find the nodeID (VCS_FindDeviceCommunicationSettings?)
    NodeId = 0;
};

/*
  Rotates the maxon motor by a given angle. If the target is more than a 15° angle from the
  reference, the motor will only rotate to the 15° angle. And if the target is less than the
  -15° angle from the reference, it will only rotate to the -15° angle.

  Parameters:
   - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
*/
void Maxon::Rotate(double Angle) {
    if (KeyHandle == NULL) return;
    long currAngle = GetCurrentAngle();
    if (currAngle + Angle > 15) {
        Angle = 15 - currAngle;
    } else if (currAngle + Angle < -15) {
        Angle = -15 - currAngle;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // TODO: compute the target position
    long targetPos = 0;
    Interface->VCS_MoveToPosition(KeyHandle, NodeId, targetPos, true, true, &errorCode);
};

/*
  Rotates the maxon motor by a given angle in a given time. If the target is more than a 15°
  angle from the reference, the motor will only rotate to the 15° angle. And if the target is
  less than the -15° angle from the reference, it will only rotate to the -15° angle. If the
  time given is negative or null, nothing will happen.

  Parameters:
   - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
   - Time (double): the time the motor will take to rotate to the target, in seconds
*/
void Maxon::Rotate(double Angle, double Time) {
    if (KeyHandle == NULL) return;
    long currAngle = GetCurrentAngle();
    if (currAngle + Angle > 15) {
        Angle = 15 - currAngle;
    } else if (currAngle + Angle < -15) {
        Angle = -15 - currAngle;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // get the current position profile parameters
    DWORD velocity = 0;
    DWORD acceleration = 0;
    DWORD deceleration = 0;
    if (!Interface->VCS_GetPositionProfile(KeyHandle, NodeId, &velocity, &acceleration, &deceleration, &errorCode)) return;

    // update the parameters for the target to be reached in time Time
    velocity = abs(Angle) / Time;
    if (!Interface->VCS_SetPositionProfile(KeyHandle, NodeId, velocity, acceleration, deceleration, &errorCode)) return;

    // TODO: compute the target position
    long targetPos = 0;
    Interface->VCS_MoveToPosition(KeyHandle, NodeId, targetPos, true, true, &errorCode);
};

/*
  Rotates the maxon motor to a given angle from the reference. If the angle is more than 15°,
  the motor will only rotate to 15°. And if it is less than -15°, the motor will only rotate
  to the -15° angle.

  Parameters:
   - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
*/
void Maxon::RotateFromRef(double Angle) {
    if (KeyHandle == NULL) return;
    if (Angle > 15) {
        Angle = 15;
    } else if (Angle < -15) {
        Angle = -15;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // TODO: compute the target position
    long targetPos = 0;
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
    if (KeyHandle == NULL) return;
    if (Angle > 15) {
        Angle = 15;
    } else if (Angle < -15) {
        Angle = -15;
    }

    // set the operation mode to the profile position mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_POSITION_MODE, &errorCode)) return;

    // get the current position profile parameters
    DWORD velocity = 0;
    DWORD acceleration = 0;
    DWORD deceleration = 0;
    if (!Interface->VCS_GetPositionProfile(KeyHandle, NodeId, &velocity, &acceleration, &deceleration, &errorCode)) return;

    // update the parameters for the target to be reached in time Time
    long currAngle = GetCurrentAngle();
    velocity = abs(currAngle - Angle) / Time;
    if (!Interface->VCS_SetPositionProfile(KeyHandle, NodeId, velocity, acceleration, deceleration, &errorCode)) return;

    // TODO: compute the target position
    long targetPos = 0;
    Interface->VCS_MoveToPosition(KeyHandle, NodeId, targetPos, false, true, &errorCode);
};

/*
  Rotates the maxon motor with a given angular velocity. If the given angular velocity is negative or
  null, nothing will happen.

  Parameters:
   - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
*/
void Maxon::RotateWithVelocity(double Velocity) {
    if (KeyHandle == NULL) return;

    // TODO: limit the angle

    // set the operation mode to the profile velocity mode
    DWORD errorCode = 0;
    if (!Interface->VCS_SetOperationMode(KeyHandle, NodeId, OMD_PROFILE_VELOCITY_MODE, &errorCode)) return;
    
    // update the target velocity to the given velocity (reduction rate is 130:1)
    long targetVelocity = Velocity * 130.0;
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
    if (KeyHandle == NULL) return;

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
    
    // update the target velocity to the given velocity (reduction rate is 130:1)
    long targetVelocity = Velocity * 130.0;
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

    // TODO: compute the corresponding angle in degrees

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

    // compute the actual angular velocity (the reduction rate is 130:1)
    velocity = velocity / 130.0;

    return velocity;
};