#include "MyRobot.h"

#define MIN_SPINUP_TIME             4
#define AUTO_MAX_SPEED             .2
// TODO - Determin distances
#define DISTANCE_TO_CENTER_BRIDGE   5
#define DISTANCE_TO_ALLIANCE_BRIDGE 5
#define TIME_TO_DUMP_BRIDGE         2
#define DISTANCE_TO_SHOOTING_POSITION 6.2

/**
 * Drive left & right motors for 2 seconds then stop
 */
void MyRobot::Autonomous()
{
   m_shooterRotationControlState = kAutonomous;

   // Enable the jaguars
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
   m_shooterRotate->EnableControl();
   m_shooterWheel->EnableControl();

   double time = Timer::GetFPGATimestamp();

   // Combined the autonomous switchs into one value
   int autoSwitchValue = (m_buttonBox->GetRawButton(AUTO_SWITCH_2) << 2)
         + (m_buttonBox->GetRawButton(AUTO_SWITCH_1) << 1)
         + m_buttonBox->GetRawButton(AUTO_SWITCH_0);

   printf("Auto mode: %i\n", autoSwitchValue);

   switch (autoSwitchValue)
   {
   case 1:
      // Shoot Two Balls
      while ((Timer::GetFPGATimestamp() - time) < MIN_SPINUP_TIME)
      {
         cameraControl();
      }
      shootTwoBalls();
      break;
   case 2:
      // Wait 5 Sec Shoot Two Balls
      time = Timer::GetFPGATimestamp();
      while ((Timer::GetFPGATimestamp() - time) < 5)
      {
         cameraControl();
      }
      shootTwoBalls();
      break;
   case 3:
      // Wait 10 Sec Shoot Two Balls
      time = Timer::GetFPGATimestamp();
      while ((Timer::GetFPGATimestamp() - time) < 10)
      {
         cameraControl();
      }
      shootTwoBalls();
      break;
   case 4:
      // Drive Dump Balls Center Bridge Drive Forward and Shoot Balls
      driveToCenterBridge();
      dumpBridge();
      driveFromCenterBridge();
      shootTwoBalls();
      break;
   case 5:
      // Drive Dump Balls Alliance Bridge Drive Forward and Shoot Balls
      driveToAllianceBridge();
      dumpBridge();
      driveFromAllianceBridge();
      shootTwoBalls();
   case 6:
      while ((Timer::GetFPGATimestamp() - time) < MIN_SPINUP_TIME)
      {
         cameraControl();
      }
      // Shoot Balls Drive Dump Balls on Alliance Bridge
      shootTwoBalls();
      driveToAllianceBridge();
      dumpBridge();
      break;
   case 7:
      //Drive to shooting position then shoot.
      // Drive Dump Balls Center Bridge Drive Forward and Shoot Balls
      //      driveToShootingPosition();
      //      time = Timer::GetFPGATimestamp();
      //      while (time - Timer::GetFPGATimestamp() > 2)
      //      {
      //         cameraControl();
      //      }
      //      shootTwoBalls();
      //      break;
      // Test Auto
      driveStraightForDistance();
      break;
   default:
      // Do nothing if no auto mode is selected
      break;
   }
}

void MyRobot::driveToShootingPosition()
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != DONE)
   {
      cameraControl();

      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(AUTO_MAX_SPEED,
               DISTANCE_TO_SHOOTING_POSITION, MAX_ACCELERATION_DISTANCE))
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

void MyRobot::driveStraightForDistance()
{
   static double testTime = Timer::GetFPGATimestamp();
   enum
   {
      DRIVING, REVERSE, STOPPED
   } autoState = DRIVING;

   if (m_driveSetting != kVelocity)
   {
      printf("Before Set mode to Velocity\n");
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != STOPPED)
   {
      if (Timer::GetFPGATimestamp() - testTime > .2)
      {
         printf("Velocity Setpoint: %f\n", m_rightMotor->Get());
         printf("Current Velocity: %f\n", m_rightMotor->GetSpeed());
         printf("Error:%f\n", m_rightMotor->Get() - m_rightMotor->GetSpeed());
         printf("Output:%f\n\n", m_rightMotor->BaseGet());
         testTime = Timer::GetFPGATimestamp();
      }

      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(.20, 20.0,
               MAX_ACCELERATION_DISTANCE))
         {
            autoState = REVERSE;
         }
         break;
      case REVERSE:
         if (m_robotDrive->DriveDistanceUsingVelocity(.4, -20.0,
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

void MyRobot::shootOneBall()
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
         if (cameraControl() || (Timer::GetFPGATimestamp() - time) > 2)
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
            autoState = DONE;
            m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
         }
         break;
      case DONE:
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

   double time = Timer::GetFPGATimestamp();
   ;
   bool previousFerris = m_ferrisWheelStop->Get();

   while (IsAutonomous() && autoState != DONE)
   {
      switch (autoState)
      {
      case AIMING:
         if (cameraControl() || (Timer::GetFPGATimestamp() - time) > 2)
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
            runFerrisWheel(kForward);
            time = Timer::GetFPGATimestamp();
         }
         break;
      case ROTATE_FERRIS:
         cameraControl();
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
         cameraControl();
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

   if (m_driveSetting == kPercentage)
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

/**
 * Dump the bridge
 */
void MyRobot::dumpBridge()
{
   double time = Timer::GetFPGATimestamp();

   // Lower Stability wheel
   //runStabilityWheels(kBackDeployed);

   while ((Timer::GetFPGATimestamp() - time) < TIME_TO_DUMP_BRIDGE)
   {
   }

   // Raise Stablility wheel
   //runStabilityWheels(kNeitherDeployed);
}

/**
 * Drive to the center bridge
 */
void MyRobot::driveToCenterBridge()
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != DONE)
   {
      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(AUTO_MAX_SPEED,
               -DISTANCE_TO_CENTER_BRIDGE, MAX_ACCELERATION_DISTANCE))
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

/**
 * Drive From the center bridge
 */
void MyRobot::driveFromCenterBridge()
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != DONE)
   {
      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(AUTO_MAX_SPEED,
               DISTANCE_TO_CENTER_BRIDGE, MAX_ACCELERATION_DISTANCE))
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

/**
 * Drive to the center bridge
 */
void MyRobot::driveToAllianceBridge()
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != DONE)
   {
      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(AUTO_MAX_SPEED,
               -DISTANCE_TO_ALLIANCE_BRIDGE, MAX_ACCELERATION_DISTANCE))
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

/**
 * Drive From the center bridge
 */
void MyRobot::driveFromAllianceBridge()
{
   enum
   {
      DRIVING, DONE
   } autoState = DRIVING;

   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   while (IsAutonomous() && autoState != DONE)
   {
      /************************ Run Stability Wheels *****************************/
      switch (autoState)
      {
      case DRIVING:
         if (m_robotDrive->DriveDistanceUsingVelocity(AUTO_MAX_SPEED,
               DISTANCE_TO_ALLIANCE_BRIDGE, MAX_ACCELERATION_DISTANCE))
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

/*****************************************************************************/
/*                           Control Defines                                 */
/*****************************************************************************/

// defines that need to be converted to time
#define DRIVE_CORRECTION_BALANCE_TIME          0.10

// state machine defines
#define MINIMUM_GRYO_ANGLE_FOR_ON_RAMP         14.0
#define RAMP_FALL_THRESHOLD                    1.0
#define RAMP_ALMOST_BALANCED_THRESHOLD         2.0
#define RAMP_BALANCED_THRESHOLD                6.0

// motor power drive defines
#define DRIVE_STOP_MOTOR_POWER                 0.0
#define DRIVE_TO_RAMP_MOTOR_POWER              0.1
#define DRIVE_UP_RAMP_MOTOR_POWER              0.075

#define DRIVE_CORRECT_MOMENTOM_MOTOR_POWER     0.015
#define DRIVE_CORRECT_RAMP_ANGLE_MOTOR_POWER   0.01

// angular velocity control parameters
#define DRIVE_MOMENTOM_OFFSET_MOTOR_POWER     -0.50
#define ANGULAR_PROPOTIONAL_GAIN               0.0
#define ANGULAR_VELOCITY_TARGET               -1.0

/*****************************************************************************/
// Routine Definition:
//
//    void RobotDemo::Balance()
//
// Description:
//
//    Routine to control the robot motor for balancing on the ramp based on
// the byro angle and state.
//
// State Machine (MP -> Motor Power):
//
//    Note: Assume the gyro has been reset to zero and robot is not on the
//          ramp.
//
//  MP = DRIVE_TO_RAMP_MOTOR_POWER              MP = DRIVE_UP_RAMP_MOTOR_POWER
//      +------+                                               +---------+
//      |      |                                               |         |
//      V      |                                               V         |
//   +-------------+  Gyro > MINIMUM_ANGLE_FOR_ON_RAMP      +---------------+
//   | BEFORE_RAMP |--------------------------------------->| DRIVE_UP_RAMP |
//   +-------------+                                        +---------------+
//                    Gyro < (maxGyro - RAMP_FALL_THRESHOLD)        |
// +----------------------------------------------------------------+
// |
// |  Control ramp angular velocity
// |     +------+
// |     |      |
// |     V      |
// |  +-------------+ Gyro < RAMP_ALMOST_BALANCED_THRESHOLD +----------------+
// +->| RAMP_MOVING |-------------------------------------->|RAMP_HORIZONTAL |
//    +-------------+                                       +----------------+
//
//    In the RAMP_MOVING state, the control system adjusts the robot position
// to control the ramp angular velocity. The control system is a simple P
// controller with an offset.
//
//       MP = Proportionsl Gain * (Setpoint - Present) + Motor Offset
//
//    In the RAMP_HORIZONTAL state,
//
// System Inputs:
//
//    Vertical Gryo angle.
//
// System Outputs:
//
//    Motor Power (-1.0 to 1.0).
//
// History:
//
//     1/14/12 JWY - First version of routine.
//    1/16/12 ARY - Ported code to the robot.
/*****************************************************************************/
void MyRobot::balance()
{
   //   if (driveSetting == kPercentage)
   //   {
   //      setDriveModeToVelocity();
   //   }
   //   
   //   printf("Time, Gyro Angle, Angular Velocity, Drive Power, State\n");
   //
   //   // Variables for balencing
   //   float maxGyro = 0.0f; // The max angle recorded by the gyro
   //   float previousGryo = 0.0f; // The previously recorded Gyro angle.
   //   double previousTime = 0.0;
   //   Timer driveTimer; // Timer to drive by.
   //   Timer balenceTimer; //Timer to see how long the robot has been balencing
   //   float printTime = 0.0;
   //
   //   balenceTimer.Reset();
   //   balenceTimer.Start();
   //
   //   // Enumeration to record the corrent state of the robot.
   //   enum
   //   {
   //      BEFORE_RAMP, DRIVE_UP_RAMP, RAMP_MOVING, RAMP_HORIZONTAL
   //   } rampState = BEFORE_RAMP;
   //
   //   // Inputs and Outputs
   //   float gyro;
   //   float angularVelocity;
   //   float drivePower = 0.0f;
   //
   //   // Reset the gyro position to 0
   //   m_gyroVertical->Reset();
   //
   //   while (IsAutonomous())
   //   {
   //      stabilityWheelState = (StabilityWheelState)getStabilityStateFromGyro();
   //      runStabilityWheels();
   //      
   //      // read the gyro
   //      gyro = -m_gyroVertical->GetAngle();
   //
   //      // assume the gyro will be increasing as the robot starts up the ramp
   //      // want to find the "maximum" value
   //      if (gyro > maxGyro)
   //         maxGyro = gyro;
   //
   //      // determine the angular velocity
   //      if (gyro != previousGryo)
   //      {
   //         angularVelocity = (gyro - previousGryo) / (balenceTimer.Get() - previousTime);
   //         previousTime = balenceTimer.Get();
   //      }
   //      
   //      //    angularVelocity = gyro - previousGryo;
   //
   //      // remember the gyro setting
   //      previousGryo = gyro;
   //
   //
   //      // determine the ramp balance state
   //      switch (rampState)
   //      {
   //      case BEFORE_RAMP:
   //         // ensure the robot is well on the ramp (gryo is greater than some ramp angle)
   //         if (maxGyro > MINIMUM_GRYO_ANGLE_FOR_ON_RAMP)
   //         {
   //            // driving up the ramp
   //            rampState = DRIVE_UP_RAMP;
   //
   //            // drive up the ramp (need to drive slowly)
   //            drivePower = DRIVE_UP_RAMP_MOTOR_POWER;
   //         }
   //         else
   //         {
   //            // keep driving up the ramp
   //            drivePower = DRIVE_TO_RAMP_MOTOR_POWER;
   //         }
   //         break;
   //
   //      case DRIVE_UP_RAMP:
   //         // determine if the ramp has started back towards horizontal
   //         if (gyro < (maxGyro - RAMP_FALL_THRESHOLD))
   //         {
   //            // ramp is starting to fall (to horizontal)
   //            rampState = RAMP_MOVING;
   //            drivePower = DRIVE_STOP_MOTOR_POWER;
   //         }
   //         else
   //         {
   //            // drive up the ramp (need to drive slowly)
   //            drivePower = DRIVE_UP_RAMP_MOTOR_POWER;
   //         }
   //         break;
   //
   //      case RAMP_MOVING:
   //         // determine if the ramp is almost back to horizontal
   //         if (gyro < RAMP_ALMOST_BALANCED_THRESHOLD)
   //         {
   //            // ramp is close to horizontal, so quite going backwards and drive forward
   //            drivePower = 0.0;
   //            rampState = RAMP_HORIZONTAL;
   //
   //            // start drive forward timer
   //            driveTimer.Reset();
   //            driveTimer.Start();
   //         }
   //
   //         // ramp is still falling towards horizontal
   //         else
   //         {
   //            // drive backwards starting when the ramp starts to fall based on the
   //            // angular velocity
   ////            drivePower = (ANGULAR_PROPOTIONAL_GAIN
   ////                  * (ANGULAR_VELOCITY_TARGET - angularVelocity))
   ////                  + DRIVE_MOMENTOM_OFFSET_MOTOR_POWER;
   //            drivePower = -.015;
   //         }
   //         break;
   //
   //      case RAMP_HORIZONTAL:
   //         // drive forwards to go back to the center of gravity
   //         if (driveTimer.Get() <= DRIVE_CORRECTION_BALANCE_TIME)
   //         {
   //            // drive forwards
   //            drivePower = DRIVE_CORRECT_MOMENTOM_MOTOR_POWER;
   //         }
   //
   //         // done driving forward to correct drive back offset
   //         else if (gyro > RAMP_BALANCED_THRESHOLD)
   //         {
   //            // drive robot back to slow down ramp fall
   //            drivePower = DRIVE_CORRECT_RAMP_ANGLE_MOTOR_POWER;
   //         }
   //
   //         else if ((gyro < -RAMP_BALANCED_THRESHOLD))
   //         {
   //            // drive robot back to slow down ramp fall
   //            drivePower = -DRIVE_CORRECT_RAMP_ANGLE_MOTOR_POWER;
   //         }
   //         break;
   //      }
   //
   //      // print the gyro value
   //      if (printTime + .10 < balenceTimer.Get())
   //      {
   //         printf("%3f, %6.2f, %6.2f, %6.2f, %d\n", balenceTimer.Get(), gyro,
   //               angularVelocity, drivePower, rampState);
   //         printTime = balenceTimer.Get();
   //      }
   //      m_rightMotor->Set(drivePower * MAX_SPEED_VELOCITY);
   //      m_leftMotor->Set(drivePower * MAX_SPEED_VELOCITY);
   //   }
   //   delete (&driveTimer);
   //   delete (&balenceTimer);
}

void MyRobot::newBalance()
{
   if (m_driveSetting == kPercentage)
   {
      setDriveModeToVelocity();
   }

   printf("Time, Gyro Angle, Angular Velocity, Drive Power, State\n");

   // Variables for balencing
   float maxGyro = 0.0f; // The max angle recorded by the gyro
   float previousGryo = 0.0f; // The previously recorded Gyro angle.
   double previousTime = 0.0;
   Timer driveTimer; // Timer to drive by.
   Timer balenceTimer; //Timer to see how long the robot has been balencing
   float printTime = 0.0;

   balenceTimer.Reset();
   balenceTimer.Start();

   // Enumeration to record the corrent state of the robot.
   enum
   {
      BEFORE_RAMP,
      DRIVE_UP_RAMP,
      RAMP_MOVING,
      RAMP_HORIZONTAL,
      DONE,
      DRIVE_DISTANCE
   } rampState = BEFORE_RAMP;

   // Inputs and Outputs
   float gyro;
   float angularVelocity;
   float drivePower = 0.0f;

   // Reset the gyro position to 0
   m_gyroVertical->Reset();

   while (IsAutonomous())
   {
      //runStabilityWheels((StabilityWheelState)getStabilityStateFromGyro());

      // read the gyro
      gyro = -m_gyroVertical->GetAngle();

      // assume the gyro will be increasing as the robot starts up the ramp
      // want to find the "maximum" value
      if (gyro > maxGyro)
         maxGyro = gyro;

      // determine the angular velocity
      if (gyro != previousGryo)
      {
         angularVelocity = (gyro - previousGryo) / (balenceTimer.Get()
               - previousTime);
         previousTime = balenceTimer.Get();
      }

      //    angularVelocity = gyro - previousGryo;

      // remember the gyro setting
      previousGryo = gyro;

      // determine the ramp balance state
      switch (rampState)
      {
      case BEFORE_RAMP:
         // ensure the robot is well on the ramp (gryo is greater than some ramp angle)
         if (maxGyro > MINIMUM_GRYO_ANGLE_FOR_ON_RAMP)
         {
            // driving up the ramp
            rampState = DRIVE_UP_RAMP;

            // drive up the ramp (need to drive slowly)
            drivePower = DRIVE_UP_RAMP_MOTOR_POWER;
         }
         else
         {
            // keep driving up the ramp
            drivePower = DRIVE_TO_RAMP_MOTOR_POWER;
         }
         break;

      case DONE:
         drivePower = DRIVE_STOP_MOTOR_POWER;
         break;

      case DRIVE_UP_RAMP:
         // determine if the ramp has started back towards horizontal
         if (gyro < (maxGyro - RAMP_FALL_THRESHOLD))
         {
            // ramp is starting to fall (to horizontal)
            rampState = RAMP_MOVING;
            drivePower = DRIVE_STOP_MOTOR_POWER;
         }
         else
         {
            // drive up the ramp (need to drive slowly)
            drivePower = DRIVE_UP_RAMP_MOTOR_POWER;
         }
         break;

      case RAMP_MOVING:
         // determine if the ramp is almost back to horizontal
         if (gyro < RAMP_ALMOST_BALANCED_THRESHOLD)
         {
            // ramp is close to horizontal, so quite going backwards and drive forward
            drivePower = 0.0;
            rampState = RAMP_HORIZONTAL;

            // start drive forward timer
            driveTimer.Reset();
            driveTimer.Start();
         }

         // ramp is still falling towards horizontal
         else
         {
            // drive backwards starting when the ramp starts to fall based on the
            // angular velocity
            //            drivePower = (ANGULAR_PROPOTIONAL_GAIN
            //                  * (ANGULAR_VELOCITY_TARGET - angularVelocity))
            //                  + DRIVE_MOMENTOM_OFFSET_MOTOR_POWER;
            drivePower = -.015;
         }
         break;

      case RAMP_HORIZONTAL:
         // drive forwards to go back to the center of gravity
         if (driveTimer.Get() <= DRIVE_CORRECTION_BALANCE_TIME)
         {
            // drive forwards
            drivePower = DRIVE_CORRECT_MOMENTOM_MOTOR_POWER;
         }

         // done driving forward to correct drive back offset
         else if (gyro > RAMP_BALANCED_THRESHOLD)
         {
            while (m_robotDrive->DriveDistanceUsingVelocity(.01, .057,
                  MAX_ACCELERATION_DISTANCE) == false)
            {
               //runStabilityWheels((StabilityWheelState) getStabilityStateFromGyro());
            }
            rampState = DONE;
         }

         else if ((gyro < -RAMP_BALANCED_THRESHOLD))
         {
            while (m_robotDrive->DriveDistanceUsingVelocity(.01, -.057,
                  MAX_ACCELERATION_DISTANCE) == false)
            {
               //runStabilityWheels((StabilityWheelState) getStabilityStateFromGyro());
            }
            rampState = DONE;

         }
         break;
      }

      // print the gyro value
      if (printTime + .10 < balenceTimer.Get())
      {
         printf("%3f, %6.2f, %6.2f, %6.2f, %d\n", balenceTimer.Get(), gyro,
               angularVelocity, drivePower, rampState);
         printTime = balenceTimer.Get();
      }
      m_rightMotor->Set(drivePower * MAX_VELOCITY);
      m_leftMotor->Set(drivePower * MAX_VELOCITY);
   }
   delete (&driveTimer);
   delete (&balenceTimer);
}
