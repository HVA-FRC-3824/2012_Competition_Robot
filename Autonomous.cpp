#include "MyRobot.h"

/*************************** Local Defines ***************************************/
#define MIN_SPINUP_TIME                     3.0

#define DISTANCE_TO_BRIDGE                  10.0

#define TIME_FOR_PISTON_TO_MOVE             1.0

#define VELOCITY_FOR_TOP_GOAL_AT_KEY        2100.0
#define VOLTAGE_FOR_TOP_GOAL_AT_KEY         6.7

#define DISTANCE_TO_SHOOTING_POSITION_HIGH  56.0
#define VELOCITY_TO_SHOOTING_POSITION_HIGH  2000.0
#define VOLTAGE_TO_SHOOTING_POSITION_HIGH   6.3

#define DISTANCE_TO_SHOOTING_POSITION_MID   12.0
#define VELOCITY_TO_SHOOTING_POSITION_MID   1276.0
#define VOLTAGE_TO_SHOOTING_POSITION_MID    4.4
/*********************************************************************************/

/**
 * Run autonomous mode
 */
void MyRobot::Autonomous()
{
   double time;

   // Enable the jaguars and victor motor controllers
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
   m_shooterRotate->EnableControl();

   // determine the control panel shooter mode and enable the shooter.
   if (m_driverStationEnhancedIO->GetDigital(SHOOTER_VOLTAGE_MODE) == true)
   {
      // set the shooter to voltage mode
      if (m_shooterWheel->GetControlMode() != CANJaguar::kVoltage)
         setShooterModeToVoltage();
   }
   else
   {
      // set the shooter to PWM mode
      if (m_shooterWheel->GetControlMode() != CANJaguar::kSpeed)
         setShooterModeToSpeed();
   }

   // Combined the autonomous switches into one value
   int autoSwitchValue = (m_buttonBox->GetRawButton(AUTO_SWITCH_2) << 2)
         + (m_buttonBox->GetRawButton(AUTO_SWITCH_1) << 1)
         + m_buttonBox->GetRawButton(AUTO_SWITCH_0);

   // Could specify shooting mode
   //   setShooterModeToSpeed();

   // get the present time
   time = Timer::GetFPGATimestamp();

   switch (autoSwitchValue)
   {
   case 1:
   {
      // wait for shooter to reach speed
      while ((Timer::GetFPGATimestamp() - time) < MIN_SPINUP_TIME)
      {
         // use the camera for tracking, if enabled
         // Note: also keeps the shooter wheel running
         runShooterControl(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      }

      // Shoot Two Balls
      shootTwoBalls(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      break;
   }

   case 2:
   {
      // Wait 5 Sec Shoot Two Balls
      while ((Timer::GetFPGATimestamp() - time) < 5)
      {
         // use the camera for tracking, if enabled
         // Note: also keeps the shooter wheel running
         runShooterControl(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      }

      // shoot 2 balls
      shootTwoBalls(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      break;
   }

   case 3:
   {
      // Wait 10 Sec Shoot Two Balls
      while ((Timer::GetFPGATimestamp() - time) < 10)
      {
         // use the camera for tracking, if enabled
         // Note: also keeps the shooter wheel running
         runShooterControl(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      }

      // shoot 2 balls
      shootTwoBalls(VELOCITY_FOR_TOP_GOAL_AT_KEY, VOLTAGE_FOR_TOP_GOAL_AT_KEY);
      break;
   }

   case 4:
   {
      // Drive Dump Balls Center Bridge Drive Forward and Shoot Balls

      // Calculate the distance to the bridge
      // Drive to shooting position then shoot
      // Make distance negative so that the robot will drive backwards
      float distanceToDrive = -((m_backUltra->GetRangeInches()
            - DISTANCE_TO_BRIDGE) / 12.0);

      // Drive to the center bridge
      driveAndRunShooter(distanceToDrive);

      // Dump the bridge
      dumpBridge();

      //      // Drive back to the center
      //      driveAndRunShooter(distanceToDrive,
      //            VELOCITY_FOR_TOP_GOAL_AT_KEY);

      // Drive to shooting position then shoot
      distanceToDrive = (m_frontUltra->GetRangeInches()
            - DISTANCE_TO_SHOOTING_POSITION_HIGH) / 12.0;

      // drive to the shooting position
      // Note: calls runShooterControl for aiming and keeps the shooter running
      driveAndRunShooter(distanceToDrive, 0.4, VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);

      // shoot 2 balls
      shootTwoBalls(VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);

      break;
   }

   case 5:
   {
      // Drive and shoot to the top goal.

      // Drive to shooting position then shoot
      float distanceToDrive = (m_frontUltra->GetRangeInches()
            - DISTANCE_TO_SHOOTING_POSITION_HIGH) / 12.0;

      // drive to the shooting position
      // Note: calls runShooterControl for aiming and keeps the shooter running
      driveAndRunShooter(distanceToDrive, DEFAULT_AUTO_SPEED, VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);

      // get the pressent time
      time = Timer::GetFPGATimestamp();

//      // wait for two seconds - TODO - NEEDED?
//      while (time - Timer::GetFPGATimestamp() > 2)
//      {
//         // use the camera for tracking, if enabled
//         // Note: also keeps the shooter wheel running
//         runShooterControl(VELOCITY_TO_SHOOTING_POSITION_HIGH);
//      }

      // shoot 2 balls
      shootTwoBalls(VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);
      break;
   }

   case 6:
   {
      // Drive to shooting position then shoot
      float distanceToDrive = (m_frontUltra->GetRangeInches()
            - DISTANCE_TO_SHOOTING_POSITION_MID) / 12.0;

      // drive to the shooting position
      // Note: calls runShooterControl for aiming and keeps the shooter running
      driveAndRunShooter(distanceToDrive, DEFAULT_AUTO_SPEED, VELOCITY_TO_SHOOTING_POSITION_MID, VOLTAGE_TO_SHOOTING_POSITION_MID);

      // get the pressent time
      time = Timer::GetFPGATimestamp();

      // wait for two seconds - TODO - NEEDED?
//      while (time - Timer::GetFPGATimestamp() > 2)
//      {
//         // use the camera for tracking, if enabled
//         // Note: also keeps the shooter wheel running
//         runShooterControl(VELOCITY_TO_SHOOTING_POSITION_MID);
//      }

      // shoot 2 balls
      shootTwoBalls(VELOCITY_TO_SHOOTING_POSITION_MID, VOLTAGE_TO_SHOOTING_POSITION_MID);
      break;
   }

   case 7:
   {
      // Drive Dump Balls Center Bridge Drive Forward and Shoot Balls
      
      float distanceToDrive;

      // Drive to shooting position then shoot
      distanceToDrive = (m_frontUltra->GetRangeInches()
            - DISTANCE_TO_SHOOTING_POSITION_HIGH) / 12.0;

      // drive to the shooting position
      // Note: calls runShooterControl for aiming and keeps the shooter running
      driveAndRunShooter(distanceToDrive, 0.4, VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);

      // shoot 2 balls
      shootTwoBalls(VELOCITY_TO_SHOOTING_POSITION_HIGH, VOLTAGE_TO_SHOOTING_POSITION_HIGH);
      
      // Drive back to the original position.
      driveAndRunShooter(-(distanceToDrive + 1), 0.4);

      // Calculate the distance to the bridge
      // Drive to shooting position then shoot
      // Make distance negative so that the robot will drive backwards
      distanceToDrive = -((m_backUltra->GetRangeInches()
            - DISTANCE_TO_BRIDGE) / 12.0);

      // Drive to the center bridge
      driveAndRunShooter(distanceToDrive);

      // Dump the bridge
      dumpBridge();

      break;
   }

   default:
   {
      // Do nothing if no auto mode is selected
      break;
   }
   }
}

void MyRobot::driveAndRunShooter(float distance, float speed, float defaultVelocity, float defaultVoltage)
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   // use the right motor to get the drive motor state
   if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
   {
      // use velocity mode to drive
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && (autoState != DONE))
   {
      runShooterControl(defaultVelocity, defaultVoltage);

      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistance(speed, distance,
               MAX_ACCELERATION_DISTANCE))
         {
            autoState = DONE;
         }
         break;

      case DONE:
         m_robotDrive->ArcadeVelocityDriveStepped(0, 0, MAX_ACCELERATION_ARCADE);
         break;
      }
   }
}

void MyRobot::driveStraightForwardAndBackForDistance()
{
   static double testTime = Timer::GetFPGATimestamp();

   enum
   {
      DRIVING, REVERSE, STOPPED
   } autoState = DRIVING;

   if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && (autoState != STOPPED))
   {
      if (Timer::GetFPGATimestamp() - testTime > .2)
      {
         printf("Velocity Setpoint: %f\n", m_rightMotor->Get());
         printf("Current Velocity: %f\n", m_rightMotor->GetSpeed());
         printf("Error:%f\n", m_rightMotor->Get() - m_rightMotor->GetSpeed());
         printf("Output:%f\n\n", m_rightMotor->BaseVictorGet());
         testTime = Timer::GetFPGATimestamp();
      }

      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistance(.20, 20.0, MAX_ACCELERATION_DISTANCE))
         {
            autoState = REVERSE;
         }
         break;

      case REVERSE:
         if (m_robotDrive->DriveDistance(.4, -20.0, MAX_ACCELERATION_DISTANCE))
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

void MyRobot::shootOneBall(float defaultVelocity, float defaultVoltage)
{
   enum
   {
      AIMING, SHOOT_FIRST_BALL, DONE
   } autoState = AIMING;

   double time = Timer::GetFPGATimestamp();

   while (IsAutonomous() && autoState != DONE)
   {
      switch (autoState)
      {
      case AIMING:
         if (runShooterControl(defaultVelocity, defaultVoltage) || (Timer::GetFPGATimestamp()
               - time) > 2)
         {
            autoState = SHOOT_FIRST_BALL;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;

      case SHOOT_FIRST_BALL:
         runShooterControl(defaultVelocity, defaultVoltage);
         if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            autoState = DONE;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
         }
         break;

      case DONE:
         break;
      }
   }
}

void MyRobot::shootTwoBalls(float defaultVelocity, float defaultVoltage)
{
   enum
   {
      AIMING, SHOOT_FIRST_BALL, ROTATE_FERRIS, SHOOT_SECOND_BALL, DONE
   } autoState = AIMING;

   double time = Timer::GetFPGATimestamp();

   bool previousFerris = m_ferrisWheelStop->Get();

   while (IsAutonomous() && autoState != DONE)
   {
      switch (autoState)
      {
      case AIMING:
         if (runShooterControl(defaultVelocity, defaultVoltage) || (Timer::GetFPGATimestamp()
               - time) > 2)
         {
            autoState = SHOOT_FIRST_BALL;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;

      case SHOOT_FIRST_BALL:
         runShooterControl(defaultVelocity, defaultVoltage);
         if ((Timer::GetFPGATimestamp() - time) > 1)
         {
            autoState = ROTATE_FERRIS;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
            runFerrisWheel(kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;

      case ROTATE_FERRIS:
         runShooterControl(defaultVelocity, defaultVoltage);
         //if (m_ferrisWheelStop->Get() == true && previousFerris == false)
         if (Timer::GetFPGATimestamp() - time > 1)
         {
            runFerrisWheel(kStop);
            autoState = SHOOT_SECOND_BALL;
            time = Timer::GetFPGATimestamp();
         }
         previousFerris = m_ferrisWheelStop->Get();
         break;

      case SHOOT_SECOND_BALL:
         runShooterControl(defaultVelocity, defaultVoltage);
         if ((Timer::GetFPGATimestamp() - time) > 3)
         {
            autoState = DONE;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
         }
         else if ((Timer::GetFPGATimestamp() - time) > 2)
         {
            m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
         }
         break;

      case DONE:
         break;
      }
   }
}

/**
 * Dump the bridge
 */
void MyRobot::dumpBridge(float timeToHoldDownBridge,
      float defaultVelocity, float defaultVoltage)
{
   enum
   {
      kBaseMoving, kPusherDown, kPusherMoving, DONE
   } autoState = kBaseMoving;
   double time = Timer::GetFPGATimestamp();

   // While the autonomous loop is running
   while (IsAutonomous() && autoState != DONE)
   {
      // Run the shooter so that the robot will be ready to shoot
      runShooterControl(defaultVelocity, defaultVoltage);

      switch (autoState)
      {
      case kBaseMoving:
         // Extend the ramp extender
         if (m_rampExtender->Get() != DoubleSolenoid::kForward)
            m_rampExtender->Set(DoubleSolenoid::kForward);

         // Wait for the piston to finish moving before going to the next state
         if ((Timer::GetFPGATimestamp() - time) >= TIME_FOR_PISTON_TO_MOVE)
         {
            time = Timer::GetFPGATimestamp();
            autoState = kPusherDown;
         }
         break;

      case kPusherDown:
         // Extend the pusher
         if (m_rampPusher->Get() != DoubleSolenoid::kForward)
            m_rampPusher->Set(DoubleSolenoid::kForward);

         // Wait for the time that the bridge should be held for
         if ((Timer::GetFPGATimestamp() - time) >= timeToHoldDownBridge)
         {
            time = Timer::GetFPGATimestamp();
            autoState = kPusherMoving;
         }
         break;

      case kPusherMoving:
         // Retrack the pusher
         if (m_rampPusher->Get() != DoubleSolenoid::kReverse)
            m_rampPusher->Set(DoubleSolenoid::kReverse);

         // Wait for the piston to finish moving before going to the next state
         if ((Timer::GetFPGATimestamp() - time) >= TIME_FOR_PISTON_TO_MOVE)
         {
            // Retract the extender
            if (m_rampExtender->Get() != DoubleSolenoid::kReverse)
               m_rampExtender->Set(DoubleSolenoid::kReverse);

            // Finished with the pusher move
            autoState = DONE;
         }
         break;

      default:
         break;
      }

   }
}
