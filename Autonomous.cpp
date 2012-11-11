#include "MyRobot.h"

/**
 * Drive left & right motors for 2 seconds then stop
 */
void MyRobot::Autonomous()
{
   // Enable the jaguars
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
   m_shooterRotate->EnableControl();
   m_shooterWheel->EnableControl();

   // Combined the autonomous switchs into one value
   int autoSwitchValue = (m_driverStation->GetDigitalIn(AUTO_SWITCH_2) << 2)
         + (m_driverStation->GetDigitalIn(AUTO_SWITCH_1) << 1)
         + m_driverStation->GetDigitalIn(AUTO_SWITCH_0);

   printf("Auto mode: %i\n", autoSwitchValue);

   switch (autoSwitchValue)
   {
   case 1:
      // If auto 1 is selected then drive for distance
      driveStraightForDistance();
      break;
   case 2:
      // If auto 2 is selected then shoot 2 balls
      shootTwoBalls();
      break;
   case 3:
      // If auto 3 is selected, drive and shoot balls
      driveAndShootTwo();
      break;
   default:
      // Do nothing if no auto mode is selected
      break;
   }

   shootTwoBalls();
}

void MyRobot::driveStraightForDistance()
{
   enum
   {
      DRIVING, REVERSE, STOPPED
   } autoState = DRIVING;

   if (driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous())
   {
      /************************ Run Stability Wheels *****************************/
      SetStabilityWheelState();
      runStabilityWheels();

      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(.05, 10.0,
               MAX_ACCELERATION_DISTANCE))
         {
            autoState = STOPPED;
         }
         break;
      case REVERSE:
         if (m_robotDrive->DriveDistanceUsingVelocity(.05, -10.0,
               MAX_ACCELERATION_DISTANCE))
         {
            autoState = STOPPED;
         }
         break;
      case STOPPED:
         m_robotDrive->ArcadeVelocityDriveStepped(0, 0, MAX_ACCELERATION_ARCADE);
         break;
      }
   }
}

void MyRobot::shootTwoBalls()
{
   enum
   {
      AIMING, SHOOT_FIRST_BALL, ROTATE_FERRIS, SHOOT_SECOND_BALL, DONE
   } autoState = AIMING;

   double time;
   bool previousFerris = m_ferrisWheelStop->Get();

   while (IsAutonomous())
   {
      switch (autoState)
      {
      case AIMING:
         if (cameraControl())
         {
            autoState = SHOOT_FIRST_BALL;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;
      case SHOOT_FIRST_BALL:
         cameraControl();
         if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            autoState = ROTATE_FERRIS;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
            m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
         }
         break;
      case ROTATE_FERRIS:
         cameraControl();
         if (m_ferrisWheelStop->Get() == true && previousFerris == false)
         {
            m_ferrisWheel->Set(0.0);
            autoState = SHOOT_SECOND_BALL;
            time = Timer::GetFPGATimestamp();
         }
         previousFerris = m_ferrisWheelStop->Get();
         break;
      case SHOOT_SECOND_BALL:
         cameraControl();
         if ((Timer::GetFPGATimestamp() - time) > 2)
         {
            autoState = DONE;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
         }
         else if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
         }
         break;
      case DONE:
         break;
      }
   }
}
void MyRobot::driveAndShootTwo()
{
   enum
   {
      kBackward, kStop
   } driveState = kBackward;
   enum
   {
      AIMING, SHOOT_FIRST_BALL, ROTATE_FERRIS, SHOOT_SECOND_BALL, DONE
   } shootState = AIMING;

   if (driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   double time;
   bool previousFerris = m_ferrisWheelStop->Get();

   while (IsAutonomous())
   {
      switch (driveState)
      {
      case kBackward:
         if (m_robotDrive->DriveDistanceUsingVelocity(.05, -10.0,
               MAX_ACCELERATION_DISTANCE))
         {
            driveState = kStop;
         }
         break;
      case kStop:
         m_robotDrive->ArcadeVelocityDriveStepped(0, 0, MAX_ACCELERATION_ARCADE);
         break;
      }
      switch (shootState)
      {
      case AIMING:
         if (cameraControl())
         {
            shootState = SHOOT_FIRST_BALL;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;
      case SHOOT_FIRST_BALL:
         cameraControl();
         if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            shootState = ROTATE_FERRIS;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
            m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
         }
         break;
      case ROTATE_FERRIS:
         cameraControl();
         if (m_ferrisWheelStop->Get() == true && previousFerris == false)
         {
            m_ferrisWheel->Set(0.0);
            shootState = SHOOT_SECOND_BALL;
            time = Timer::GetFPGATimestamp();
         }
         previousFerris = m_ferrisWheelStop->Get();
         break;
      case SHOOT_SECOND_BALL:
         cameraControl();
         if ((Timer::GetFPGATimestamp() - time) > 2)
         {
            shootState = DONE;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
         }
         else if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
         }
         break;
      case DONE:
         break;
      }
   }
}
