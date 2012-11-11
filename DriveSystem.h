//---------------------------------------------------------------------------------
// Source Filename:  DriveSystem.h
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#ifndef __DriveSystem_h__
#define __DriveSystem_h__

#include "WPILib.h"

#include "HVA_PIDController.h"

//---------------------------------------------------------------------------------
// Class Definition:
//
//    Class DriveSystem
//
// Drive System constructor based on the RobotDrive and PIDOutput classes.
//
//---------------------------------------------------------------------------------
class DriveSystem : public RobotDrive, public PIDOutput
{
public:
   HVA_PIDController *m_gyroPID;    // PID controller used the turn the robot to a given heading

   // Constructor for the Drive System
   DriveSystem(UINT32 frontLeftMotorChannel,  UINT32 rearLeftMotorChannel,
               UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel,
               Gyro *gyro);

   // Method to drive the robot off of the Kinect
   void Kinect(KinectStick *leftArm, KinectStick *rightArm);

   // Method to drive the base of the robot to a particular angle
   void DriveToAngle(float angle);

   // Method to drive the base of the robot to a particular angle from the current angle
   void DriveToRelativeAngle(float angle);

   // set the output of the PID = to global rotation
   void PIDWrite(float PID_output);

   // Disable the PID used for diving
   void DisablePID();
   void EnablePID();
   void ResetPID();
   
private:
   float  m_targetGyro;
   Gyro  *m_gyro;                   // Gyro to get heading
   Timer *m_timer;
};

#endif
