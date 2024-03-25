#include "inc/maxon/maxon.hpp"


/*
  The constructor.

  Parameters:
   - Interface (EPOS4_Interface*): the interface used to control the board
*/
Maxon::Maxon(EPOS4_Interface* Interface) {


};

/*
  Rotates the maxon motor by a given angle. If the target is more than a 15° angle from the
  reference, the motor will only rotate to the 15° angle. And if the target is less than the
  -15° angle from the reference, it will only rotate to the -15° angle.

  Parameters:
   - Angle (double): the angle to rotate the motor, in degrees (can be positive or negative)
*/
void Maxon::Rotate(double Angle) {
    

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


};

/*
  Rotates the maxon motor to a given angle from the reference. If the angle is more than 15°,
  the motor will only rotate to 15°. And if it is less than -15°, the motor will only rotate
  to the -15° angle.

  Parameters:
   - Angle (double): the angle to which the motor will rotate, in degrees (can be positive or negative)
*/
void Maxon::RotateFromRef(double Angle) {


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


};

/*
  Rotates the maxon motor with a given angular velocity. If the given angular velocity is negative or
  null, nothing will happen.

  Parameters:
   - Velocity (double): the angular velocity used to rotate the motor, in revolutions per minute
*/
void Maxon::RotateWithVelocity(double Velocity) {


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


};

/*
  Ends the communication with the maxon motor. The functions called on the Maxon object will not work
  anymore. This function must be called everytime we are done using the Maxon object.
*/
void Maxon::EndCommunication(void) {


};

/*
  Gets the current angle of the maxon motor.

  Return parameters:
   - return value (double): the current angle, in degrees
*/
double Maxon::GetCurrentAngle(void) {

    return 0.;
};

/*
  Gets the current angular velocity of the maxon motor.

  Return parameters:
   - return value (double): the current angular velocity, in revolutions per minute
*/
double Maxon::GetCurrentVelocity(void) {

    return 0.;
};