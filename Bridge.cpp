#include "MyRobot.h"

/*************************** Local Defines ***************************************/
#define JOYSTICK_DEADBAND                   0.1

#define BALANCE_ON_BRIDGE_SPEED             1.0
#define BALANCE_DISTANCE_DELTA             (1.0 / 12.0)

#define DRIVE_SPEED                         0.8

#define MAX_DRIVE_ON_BRIDGE_MANUAL_SPEED    1.5

#define ON_BRIDGE_ANGLE_CHECK              14.0
#define BRIDGE_TIPPING_ANGLE_CHECK         13.0

#define RAMP_FALL_THRESHOLD                 3.0

#define BRIDGE_TILTED_ANGLE_CHECK           3.0

#define ONE_INCH                           (1.0 / 12.0)

#define BRIDGE_TILT_CORRECTION_DISTANCE    (ONE_INCH * 2.0)

/*********************************************************************************/

/**
 * Run the bridge deployment
 */
void MyRobot::runBridgeDeployment(void)
{
   // check for deploy extender
   if (m_driverStationEnhancedIO->GetDigital(RAMP_EXTENDER) == true)
   {
      // extend extender if extender is not extended
      if (m_rampExtender->Get() != DoubleSolenoid::kForward)
      {
         // initialize bridge balance state when extender is extended
         InitializeBalanceOnBridge();
         m_rampExtender->Set(DoubleSolenoid::kForward);
      }
   }
   else
   {
      // retract extender if extender is not retracted
      if (m_rampExtender->Get() != DoubleSolenoid::kReverse)
         m_rampExtender->Set(DoubleSolenoid::kReverse);
   }

   // check for deploy pusher
   // Note: do not deploy pusher if the pusher is not extended
   if ((m_driverStationEnhancedIO->GetDigital(RAMP_PUSHER) == true)
         && (m_driverStationEnhancedIO->GetDigital(RAMP_EXTENDER) == true))
   {
      // extend pusher if pusher is not extended
      if (m_rampPusher->Get() != DoubleSolenoid::kForward)
         m_rampPusher->Set(DoubleSolenoid::kForward);
   }
   else
   {
      // retract pusher if pusher is not retracted
      if (m_rampPusher->Get() != DoubleSolenoid::kReverse)
         m_rampPusher->Set(DoubleSolenoid::kReverse);
   }
}

/**
 * Run the bridge balance mode
 */
void MyRobot::runBridgeBalancingMode(bool previousTriggerState)
{
   static bool previousIncrementButtonState = m_joystick->GetRawButton(INCREMENT_POSITION);
   static bool previousDecrementButtonState = m_joystick->GetRawButton(DECREMENT_POSITION);
   static JoystickState joystickState;

   float balanceDistanceDelta;

   /************************ Trigger first pulled ********************************/
   // determine if the trigger has just be pulled
   if ((previousTriggerState == false) &&
       (m_joystick->GetTrigger() == true))
   {
      // ignore the joysick position until it passes through the deadband
      joystickState = kIgnore;

      // determine if autonomous mode should be executed
      if (m_runningAutonomous == true)
      {
         setDriveModeToVelocity(BALANCE_ON_BRIDGE_SPEED);
      }
      else
      {
         // get the present location
         m_rightPosition = m_rightMotor->GetPosition();
         m_leftPosition  = m_leftMotor->GetPosition();
      }
   }

   /************************ Check Joystick *************************************/
   // check the joystick state
   if ((fabs(m_joystick->GetY()) > JOYSTICK_DEADBAND) &&
       (joystickState == kInDeadband))
   {
      // joystick is out of the deadband and should be used for control
      joystickState = kOutDeadband;
      
      // Set the drive mode here so that the user gets the faster manual velocity setting
      setDriveModeToVelocity(MAX_DRIVE_ON_BRIDGE_MANUAL_SPEED);

      // out of deadband so no longer running autonomous
      m_runningAutonomous = false;
   }

   // check for joystick in deadband
   else if (fabs(m_joystick->GetY()) < JOYSTICK_DEADBAND)
   {
      // determine if the joystick is changing from velocity to position mode
      // so update the drive position
      if (joystickState == kOutDeadband)
      {  
         // get the present location
         m_rightPosition = m_rightMotor->GetPosition();
         m_leftPosition  = m_leftMotor->GetPosition();
      }

      // the joystick is in the deadband so ignore for velocity control
      joystickState = kInDeadband;
   }

   /************************ Run Velocity Mode **********************************/
   if (joystickState == kOutDeadband)
   {
      // ensure the robot is in velocity mode
      if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
      {
         // set the drive to velocity
         setDriveModeToVelocity(MAX_DRIVE_ON_BRIDGE_MANUAL_SPEED);
      }

      // control the robot drive
      m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
                                m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);
   }
   
   /************************ Run Position Mode **********************************/
   else
   {
      // perform the balance on bridge algorithm
      if (m_runningAutonomous == true)
         BalanceOnBridge();
      else
      {
         // ensure the balance distance offset is zero before reading the switched
         balanceDistanceDelta = 0.0;

         // check if increment distance
         if ((previousIncrementButtonState != m_joystick->GetRawButton(INCREMENT_POSITION)) &&
             (m_joystick->GetRawButton(INCREMENT_POSITION) == true))
         {
            balanceDistanceDelta = BALANCE_DISTANCE_DELTA;
         }

         // check if decrement distance
         else if ((previousDecrementButtonState != m_joystick->GetRawButton(DECREMENT_POSITION)) &&
                  (m_joystick->GetRawButton(DECREMENT_POSITION) == true))
         {
            balanceDistanceDelta = -BALANCE_DISTANCE_DELTA;
         }

         // ensure the robot is in position mode
         if (m_rightMotor->GetControlMode() != HVA_Victor::kPosition)
         {
            // set the drive to velocity
            setDriveModeToPosition();
         }

         // determine if the operator wants to change the bridge position
         if (balanceDistanceDelta != 0.0)
         {
            // change the desired bridge position
            m_rightPosition += balanceDistanceDelta;
            m_leftPosition  += balanceDistanceDelta;
         }
         
         // set the desired ending position
         m_rightMotor->Set(m_rightPosition);
         m_leftMotor->Set(m_leftPosition);
      }
   }

   // update the move absolute increment button states
   previousIncrementButtonState = m_joystick->GetRawButton(INCREMENT_POSITION);
   previousDecrementButtonState = m_joystick->GetRawButton(DECREMENT_POSITION);
}

/**
 * Initialize the balance on bridge variables
 */
void MyRobot::InitializeBalanceOnBridge(void)
{
   // set the balance state
   m_balancingState = INITIAL_BRIDGE_STATE;

   // balance autonomously
   m_runningAutonomous = true;

   // reset the gyro
   m_gyroVertical->Reset();
}

/**
 * Routine to balance on the bridge using velocity and position drive modes
 */
void MyRobot::BalanceOnBridge(void)
{
   static float  previousGryo;
   static double angularVelocityTime;
   static double startBalanceTime;
   static double debugTime;

   float angularVelocity;

   // get the gyro angle
   float gyroAngle = -m_gyroVertical->GetAngle();

   // determine if in the routine the first time after initialization to setup
   // the initial time values
   if (m_balancingState == INITIAL_BRIDGE_STATE)
   {
      // set the angular velocity, start balance and debug times
      angularVelocityTime = startBalanceTime = debugTime = Timer::GetFPGATimestamp();

      // show the debug header
      //printf("Time\tState\tGyro Angle\tAngular Velocity\tDrive Speed\tDrive Position\tSetpoint\n");
   }

   // not the first time in the routine since initialization so update the angular
   // velocity
   else
   {
      // determine if the gyro angle has changed
      if (gyroAngle != previousGryo)
      {
         // determine the angular velocity
         angularVelocity = (gyroAngle - previousGryo) / (Timer::GetFPGATimestamp() - angularVelocityTime);

         // remember the new gyro angular velocity calculation time
         angularVelocityTime = Timer::GetFPGATimestamp();
      }
   }

   // remember the gyro setting
   previousGryo = gyroAngle;

   // determine the bridge state
   switch (m_balancingState)
   {
      case INITIAL_BRIDGE_STATE: // setup for autonomous driving/balancing on bridge
      {
         // determine gyro angle to determine the drive direction
         if (gyroAngle > BRIDGE_TILTED_ANGLE_CHECK)
         {
            // change state to driving forward onto bridge
            m_balancingState = DRIVING_FORWARD_ONTO_BRIDGE;

            // drive forward on the bridge
            m_robotDrive->SetLeftRightMotorOutputs(-DRIVE_SPEED, -DRIVE_SPEED);
         }
         else if (gyroAngle < -BRIDGE_TILTED_ANGLE_CHECK)
         {
            // change state to driving forward onto bridge
            m_balancingState = DRIVING_BACKWARD_ONTO_BRIDGE;

            // drive forward on the bridge
            m_robotDrive->SetLeftRightMotorOutputs(DRIVE_SPEED, DRIVE_SPEED);
         }
         else
         {
            // bridge is not in the down position so don't know how to balance
         }
         break;
      }

      case DRIVING_FORWARD_ONTO_BRIDGE: // driving onto the bridge
      {
         // determine if the robot is on the bridge
         if (gyroAngle > ON_BRIDGE_ANGLE_CHECK)
         {
            // change state to driving forward on bridge
            m_balancingState = DRIVING_FORWARD_ON_BRIDGE;
         }

         break;
      }

      case DRIVING_FORWARD_ON_BRIDGE: // on bridge and driving up
      {
         // determine if the ramp has started back towards horizontal
         if (gyroAngle < BRIDGE_TIPPING_ANGLE_CHECK)
         {
            // get the present location
            m_rightPosition = m_rightMotor->GetPosition() - BRIDGE_TILT_CORRECTION_DISTANCE;
            m_leftPosition = m_leftMotor->GetPosition() - BRIDGE_TILT_CORRECTION_DISTANCE;

            // set drive to position mode
            m_rightMotor->ChangeControlMode(HVA_Victor::kPosition);
            m_leftMotor->ChangeControlMode(HVA_Victor::kPosition);

            // set the desired ending position
            m_rightMotor->Set(m_rightPosition);
            m_leftMotor->Set(m_leftPosition);

            // enable the position mode
            m_rightMotor->EnableControl();
            m_leftMotor->EnableControl();

            // done with autonomous balance
            m_runningAutonomous = false;
         }
         break;
      }

      case DRIVING_BACKWARD_ONTO_BRIDGE: // driving onto the bridge
      {
         // determine if the robot is on the bridge
         if (gyroAngle < -ON_BRIDGE_ANGLE_CHECK)
         {
            // change state to driving forward on bridge
            m_balancingState = DRIVING_BACKWARD_ON_BRIDGE;
         }
         break;
      }

      case DRIVING_BACKWARD_ON_BRIDGE: // on bridge and driving up
      {
         // determine if bridge starts to tip
         if (gyroAngle > -BRIDGE_TIPPING_ANGLE_CHECK)
         {
            // get the present location
            m_rightPosition = m_rightMotor->GetPosition() + BRIDGE_TILT_CORRECTION_DISTANCE;
            m_leftPosition  = m_leftMotor->GetPosition() + BRIDGE_TILT_CORRECTION_DISTANCE;

            // set drive to position mode
            m_rightMotor->ChangeControlMode(HVA_Victor::kPosition);
            m_leftMotor->ChangeControlMode(HVA_Victor::kPosition);

            // set the desired ending position
            m_rightMotor->Set(m_rightPosition);
            m_leftMotor->Set(m_leftPosition);

            // enable the position mode
            m_rightMotor->EnableControl();
            m_leftMotor->EnableControl();

            // done with autonomous balance
            m_runningAutonomous = false;
         }
         break;
      }
   }

//   // print out debug statement every quarter second
//   if (Timer::GetFPGATimestamp() - debugTime >= 0.0)
//   {
//      // reset the timer time
//      debugTime = Timer::GetFPGATimestamp();
//
//      // Time, State, Gyro Angle, Angular Velocity, Drive Speed, Drive Position
//      //printf("%f\t%d\t%f\t%f\t%f\t%f\t%f\n",
//            //            printf("%f, %d, %f, %f, %f, %f, %f\n",
//            //(float) (Timer::GetFPGATimestamp() - startBalanceTime),
//            //m_balancingState, gyroAngle, angularVelocity, m_rightMotor->GetSpeed(), m_rightMotor->GetPosition(), m_rightMotor->Get());
//   }
}
