//---------------------------------------------------------------------------------
// Source Filename:  DriveSystem.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#include <math.h>

#include "DriveSystem.h"

//---------------------------------------------------------------------------------
//                           Local Defines
//---------------------------------------------------------------------------------

#define ANGLE_CHANGE_WAIT_TIME       0.1
#define STEP                         5.0

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    DriveSystem::DriveSystem(UINT32 frontLeftMotorChannel,
//                             UINT32 rearLeftMotorChannel,
//                             UINT32 frontRightMotorChannel,
//                             UINT32 rearRightMotorChannel, Gyro *gyro)
//
//        frontLeftMotorChannel -
//        rearLeftMotorChannel  -
//        frontRightMotorChannel-
//        rearRightMotorChannel -
//        gyro                  -
//
// Description:
//
//    Constructor for the Drive System class.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
DriveSystem::DriveSystem(UINT32 frontLeftMotorChannel,
                         UINT32 rearLeftMotorChannel,
                         UINT32 frontRightMotorChannel,
                         UINT32 rearRightMotorChannel, Gyro *gyro) :
   RobotDrive(frontLeftMotorChannel,  rearLeftMotorChannel,
              frontRightMotorChannel, rearRightMotorChannel)
{
   // store the gyro object
   m_gyro = gyro;

   // PID controller for the gyro angles
   m_gyroPID = new HVA_PIDController(0.03000,    // P gain
                                     0.00300,    // I gain
                                     0.02000,    // D gain
                                     m_gyro,     // source
                                     this,       // output
                                     0.05);      // period

   // Set up the turn controller
   m_gyroPID->SetInputRange(0.0, 360.0);         // set gyro input range
   m_gyroPID->SetOutputRange(-0.3, 0.3);         // rotation output range
   m_gyroPID->SetTolerance(0.5);                 // degrees error tolerance
   m_gyroPID->SetInputWrap(true);                // gryo wraps at 0 and 360
   m_gyroPID->Disable();                         // disable the PID

   // clear the target rotation angle
   m_targetGyro = 0.0;
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void DriveSystem::PIDWrite(float PID_output);
//
//       PID_output - The rotation value to turn the robot (-1 to 1).
//
// Description:
//
//    Routine called by the gyro PID to turn the robot the desired direction and
// power.
//
// Return:
//
//    None.
//
// History:
//
//    ARY  12.10.2011 - First version of the routine.
//
//---------------------------------------------------------------------------------

void DriveSystem::PIDWrite(float PID_output)
{
   // turn the robot based on the gyro PID controller
   //HolonomicDrive(0.0, 0.0, PID_output);
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void DriveSystem::DriveToAngle(float angle)
//
//       angle - The angle to drive to.
//
// Description:
//
//    Method to rotate the robot the a certain angle.
//
// Return:
//
//    None.
//
// History:
//
//    ARY  12.10.2011 - First version of the routine.
//
//---------------------------------------------------------------------------------
void DriveSystem::DriveToAngle(float angle)
{
   // remember the target PID angle
   m_targetGyro = angle;

   // set the gyro set point
   m_gyroPID->SetSetpoint(m_targetGyro);
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void DriveSystem::DriveToRelativeAngle(float angle);
//
//       angle - The angle to drive to.
//
// Description:
//
//    Method to rotate the robot to the angle from the current angle.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void DriveSystem::DriveToRelativeAngle(float angle)
{
   float presentAngle;
   
   // calculate the new target angle
   // Note: The PID Get method returns the gyro angle in a range between 0 and 360
   presentAngle = m_gyro->PIDGet();
   m_targetGyro = presentAngle + angle;

	while (m_targetGyro > 360.0)
		m_targetGyro -= 360.0;
	while (m_targetGyro < 0.0)
		m_targetGyro += 360.0;
   
   printf("Present: %f  Target: %f\n", presentAngle, m_targetGyro);
   
   // set the gyro set point
   m_gyroPID->SetSetpoint(m_targetGyro);
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//
// Description:
//
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void DriveSystem::DisablePID()
{
   // determine if the PID is enabled
   if (m_gyroPID->IsEnabled() == true)
      m_gyroPID->Disable();
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//
// Description:
//
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void DriveSystem::EnablePID()
{
   // detemrine if the PID is disabled
   if (m_gyroPID->IsEnabled() == false)
   {
      // reset the PID and enable
      ResetPID();
      m_gyroPID->Enable();
   }
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//
// Description:
//
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void DriveSystem::ResetPID()
{
   // reset the PID
   m_gyroPID->Reset();

   // reset the gyro to the present angle
   // Note: The PID Get method returns the gyro angle in a range between 0 and 360
   m_targetGyro = m_gyro->PIDGet();
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void DriveSystem::Kinect(KinectStick *leftArm, KinectStick *rightArm)
//
//       leftArm  - Left arm from the kinect.
//       rightArm - Right arm from the kinect.
//
// Description:
//
//    Method to drive the robot off of the values provided by the kinect.
//
// Return Value:
//
//    None.
//
// History:
//
//    12/10/11 ARY - First version of the routine.
//---------------------------------------------------------------------------------

void DriveSystem::Kinect(KinectStick *leftArm, KinectStick *rightArm)
{
   float right;
   float left;
   float direction;

   // get values from kinect
   right = -rightArm->GetY();
   left  = -leftArm->GetY();

   // determine the direction
   if (right >= 0)
      direction = 0.0f;
   else
      direction = 180.0f;

   // set the deadband
   if (fabs(right) < 0.2)
      right = 0;
   if (fabs(left) < 0.2)
      left = 0;

   // scale the values
   right = fabs(right) * 0.5;
   left  = left        * 0.5;

   // drive the motors
   HolonomicDrive(right, direction, left);
}
