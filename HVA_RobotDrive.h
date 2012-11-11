#ifndef __HVA_RobotDrive_h__
#define __HVA_RobotDrive_h__

#include "WPILib.h"
#include "Math.h"

#define MAX_ACCELERATION_ARCADE  .25
#define MAX_ACCELERATION_DISTANCE  .25
#define TIME_THRESHOLD      1

class HVA_RobotDrive: public RobotDrive
{
private:
   
public:
   HVA_RobotDrive(UINT32 leftMotorChannel, UINT32 rightMotorChannel);
   HVA_RobotDrive(UINT32 frontLeftMotorChannel, UINT32 rearLeftMotorChannel,
            UINT32 frontRightMotorChannel, UINT32 rearRightMotorChannel);
   HVA_RobotDrive(SpeedController *leftMotor, SpeedController *rightMotor);
   HVA_RobotDrive(SpeedController &leftMotor, SpeedController &rightMotor);
   HVA_RobotDrive(SpeedController *frontLeftMotor, SpeedController *rearLeftMotor,
            SpeedController *frontRightMotor, SpeedController *rearRightMotor);
   HVA_RobotDrive(SpeedController &frontLeftMotor, SpeedController &rearLeftMotor,
            SpeedController &frontRightMotor, SpeedController &rearRightMotor);
   
   bool DriveDistanceUsingVelocity(float maxSpeed, float distance);
   void ArcadeVelocityDriveStepped(float moveValue, float rotateValue, bool squaredInputs = true);
   
};

#endif
