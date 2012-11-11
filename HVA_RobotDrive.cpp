#include "WPILib.h"
#include "HVA_RobotDrive.h"

/*************************** Local Defines ***************************************/

#define RAMP_FALL_THRESHOLD                 3.0

#define BRIDGE_TILTED_ANGLE_CHECK          12.0

#define ONE_INCH                           (1.0 / 12.0)

#define BRIDGE_TILT_CORRECTION_DISTANCE    (ONE_INCH * 2)

/*********************************************************************************/

HVA_RobotDrive::HVA_RobotDrive(UINT32 leftMotorChannel,
      UINT32 rightMotorChannel) :
   RobotDrive(leftMotorChannel, rightMotorChannel)
{

}

HVA_RobotDrive::HVA_RobotDrive(UINT32 frontLeftMotor, UINT32 rearLeftMotor,
      UINT32 frontRightMotor, UINT32 rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

HVA_RobotDrive::HVA_RobotDrive(SpeedController *leftMotor,
      SpeedController *rightMotor) :
   RobotDrive(leftMotor, rightMotor)
{

}

HVA_RobotDrive::HVA_RobotDrive(SpeedController &leftMotor,
      SpeedController &rightMotor) :
   RobotDrive(leftMotor, rightMotor)
{

}

HVA_RobotDrive::HVA_RobotDrive(SpeedController *frontLeftMotor,
      SpeedController *rearLeftMotor, SpeedController *frontRightMotor,
      SpeedController *rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

HVA_RobotDrive::HVA_RobotDrive(SpeedController &frontLeftMotor,
      SpeedController &rearLeftMotor, SpeedController &frontRightMotor,
      SpeedController &rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

/**
 * Drive the robot a certain distance and then stop.
 * Return true when distance is reached.
 */
bool HVA_RobotDrive::DriveDistance(float maxSpeed, float distance,
      float maxAcceleration)
{
   static bool distanceRunning = false;
   static bool stopping = false;
   static double startDistanceRight = 0;
   static double motorOutputVelocity = 0.0;
   static double previousTime = Timer::GetFPGATimestamp();

   double changeInTime = Timer::GetFPGATimestamp() - previousTime;
   double stoppingDistance = 0;

   previousTime = Timer::GetFPGATimestamp();

   // get the absolute value of the maximum speed
   maxSpeed = fabs(maxSpeed);

   // Make sure the code is still being looped through
   // i.e. the drive to distance mode was stopped and then restarted
   if (changeInTime > TIME_THRESHOLD)
   {
      // set the new change in time and recalculate the distance
      changeInTime = Timer::GetFPGATimestamp() - previousTime;
      distanceRunning = false;
   }

   //  start running distance control
   if (distanceRunning == false)
   {
      // get the current speed so the robot does not stop
      motorOutputVelocity = m_rearRightMotor->Get() / m_maxOutput;

      // get the present position
      startDistanceRight = ((HVA_Victor*) m_rearRightMotor)->GetPosition();

      // setup to drive (not stopped).
      distanceRunning = true;
      stopping = false;
   }

   // determine if the desired distance is positive
   if (distance >= 0)
   {
      // Calculate the stopping distance based on kinematics
      stoppingDistance = pow(motorOutputVelocity * (m_maxOutput), 2) / (2
            * maxAcceleration * (m_maxOutput));

      // Start stopping if stopping distance is greater than distance left
      if (stoppingDistance >= (distance
            - (((HVA_Victor*) m_rearRightMotor)->GetPosition()
                  - startDistanceRight)))
      {
         stopping = true;
      }

      // Time
      printf("%f",Timer::GetFPGATimestamp());
      
      // Stopping distance
      printf("\t%f", stoppingDistance);

      //distance to target
      printf(
            "\t%f",
            (distance - (((HVA_Victor*) m_rearRightMotor)->GetPosition()
                  - startDistanceRight)));

      // depending on the stage find velocity differently
      if (stopping == false)
      {
         // determine if accelerating
         if ((motorOutputVelocity + maxAcceleration * changeInTime) <= maxSpeed)
         {
            // accelerate
            motorOutputVelocity += maxAcceleration * changeInTime;
         }
         else
         {
            // hold at maximum speed
            motorOutputVelocity = maxSpeed;
         }
      }

      // Stopping
      else
      {
         // if still moving then deaccelerate
         if ((motorOutputVelocity - maxAcceleration * changeInTime) >= 0)
         {
            // deaccelerate
            motorOutputVelocity -= maxAcceleration * changeInTime;
         }
         else
         {
            // stopped
            motorOutputVelocity = 0;
            distanceRunning = false;
         }
      }
      
      static float previousDistance = ((HVA_Victor*) m_rearRightMotor)->GetPosition();
      static float previousTime = Timer::GetFPGATimestamp();

      // new calc for velocity
      printf("\t%f", ((((HVA_Victor*) m_rearRightMotor)->GetPosition() - previousDistance)/(Timer::GetFPGATimestamp()-previousTime)));
      previousDistance = ((HVA_Victor*) m_rearRightMotor)->GetPosition();
      previousTime= Timer::GetFPGATimestamp();
      // Current velocity
      printf(
            "\t%f",
            ((HVA_Victor*) m_rearRightMotor)->GetSpeed());

      // Velocity setpoint
      printf("\t%f\n", motorOutputVelocity * m_maxOutput);

      // set the motor velocities
      SetLeftRightMotorOutputs(-motorOutputVelocity, -motorOutputVelocity);
   }

   // desired distance is negative
   else
   {
      // Calculate the stopping distance
      stoppingDistance = -(pow(motorOutputVelocity * (m_maxOutput / 60), 2)
            / (2 * maxAcceleration * (m_maxOutput / 60)));

      // Start stopping if stopping distance is greater than distance left
      if (stoppingDistance <= (distance
            - (((HVA_Victor*) m_rearRightMotor)->GetPosition()
                  - startDistanceRight)))
      {
         stopping = true;
      }

      // depending on the stage find velocity differently
      if (stopping == false)
      {
         if (motorOutputVelocity - maxAcceleration * changeInTime >= -maxSpeed)
         {
            motorOutputVelocity -= maxAcceleration * changeInTime;
         }
         else
         {
            motorOutputVelocity = -maxSpeed;
         }
      }

      // Stopping
      else
      {
         if (motorOutputVelocity + maxAcceleration * changeInTime <= 0)
         {
            motorOutputVelocity += maxAcceleration * changeInTime;
         }
         else
         {
            motorOutputVelocity = 0;
            distanceRunning = false;
         }
      }

      // run the motors in velocity mode
      SetLeftRightMotorOutputs(-motorOutputVelocity, -motorOutputVelocity);
   }

   // Check to see if the distance is reached.
   if (distanceRunning == true)
   {
      return false;
   }
   else
   {
      return true;
   }
}

/**
 * Initialize the balance on bridge variables
 */
void HVA_RobotDrive::InitializeBalanceOnBridge(Gyro *gyro)
{
   // set the balance state
   m_balancingState = INITIAL_BRIDGE_STATE;

   m_maxGyro = 0.0; // The max angle recorded by the gyro
   m_minGyro = 0.0; // The min angle recorded by the gyro
   m_previousGryo = 0.0; // The previously recorded Gyro angle

   // reset the gyro
   gyro->Reset();
}

/**
 * Routine to balance on the bridge using velocity and position drive modes
 */
void HVA_RobotDrive::BalanceOnBridge(float maxVelocity, Gyro *gyro,
      float positionDelta)
{
   static float rearRightPosition;
   static float rearLeftPosition;
   static float previousGryo;
   static double angularVelocityTime;
   static double startBalanceTime;
   static double debugTime;

   float angularVelocity;

   // get the gyro angle
   float gyroAngle = -gyro->GetAngle();

   // assume the gyro will be increasing as the robot starts up the ramp
   // want to find the "maximum" value
   if (gyroAngle > m_maxGyro)
      m_maxGyro = gyroAngle;
   if (gyroAngle < m_minGyro)
      m_minGyro = gyroAngle;

   // determine if in the routine the first time after initialization to setup
   // the initial time values
   if (m_balancingState == INITIAL_BRIDGE_STATE)
   {
      // set the angular velocity, start balance and debug times
      angularVelocityTime = startBalanceTime = debugTime
            = Timer::GetFPGATimestamp();

      // show the debug header
      printf(
            "Time\tState\tGyro Angle\tMin Angle\tMax Angle\tAngular Velocity\tDrive Speed\tDrive Position\tSetpoint\n");
   }
   else
   {
      // determine if the gyro angle has changed
      if (gyroAngle != previousGryo)
      {
         // determine the angular velocity
         angularVelocity = (gyroAngle - previousGryo)
               / (Timer::GetFPGATimestamp() - angularVelocityTime);

         // remember the new gyro angular velocity calculation time
         angularVelocityTime = Timer::GetFPGATimestamp();
      }
   }

   // remember the gyro setting
   previousGryo = gyroAngle;

   // determine the bridge state
   switch (m_balancingState)
   {
   case INITIAL_BRIDGE_STATE:
   {
      /***************** INITIAL TESTING *************************************/
      //         m_balancingState = POSITION_CONTROL_ON_BRIDGE;
      //
      //         // get the present location
      //         rearRightPosition = ((HVA_Victor*) m_rearRightMotor)->GetPosition();
      //         rearLeftPosition  = ((HVA_Victor*) m_rearLeftMotor)->GetPosition();
      //
      //         // set drive to position mode
      //         ((HVA_Victor*) m_rearRightMotor)->ChangeControlMode(HVA_Victor::kPosition);
      //         ((HVA_Victor*) m_rearLeftMotor)->ChangeControlMode(HVA_Victor::kPosition);
      //
      //         // set the desired ending position
      //         ((HVA_Victor*) m_rearRightMotor)->Set(rearRightPosition + BRIDGE_TILT_CORRECTION_DISTANCE);
      //         ((HVA_Victor*) m_rearLeftMotor)->Set(rearLeftPosition + BRIDGE_TILT_CORRECTION_DISTANCE);
      //
      //         // enable the position mode
      //         ((HVA_Victor*) m_rearRightMotor)->EnableControl();
      //         ((HVA_Victor*) m_rearLeftMotor)->EnableControl();

      /***********************************************************************/

      // determine gyro angle to determine the drive direction
      if (gyroAngle > BRIDGE_TILTED_ANGLE_CHECK)
      {
         // change state to driving forward on bridge
         m_balancingState = DRIVING_FORWARD_ON_BRIDGE;

         // drive forward on the bridge
         SetLeftRightMotorOutputs(-maxVelocity, -maxVelocity);
      }
      else if (gyroAngle < -BRIDGE_TILTED_ANGLE_CHECK)
      {
         // change state to driving forward on bridge
         m_balancingState = DRIVING_BACKWARD_ON_BRIDGE;

         // drive forward on the bridge
         SetLeftRightMotorOutputs(maxVelocity, maxVelocity);
      }
      else
      {
         // bridge is not in the down position so don't know how to balance
         printf("Error: Bridge angle: %f\n", gyroAngle);
      }

      break;
   }

   case DRIVING_FORWARD_ON_BRIDGE:
   {
      // determine if the ramp has started back towards horizontal
      if (gyroAngle < (m_maxGyro - RAMP_FALL_THRESHOLD))
      {
         // change state to driving forward on bridge
         m_balancingState = POSITION_CONTROL_ON_BRIDGE;

         // get the present location
         rearRightPosition = ((HVA_Victor*) m_rearRightMotor)->GetPosition();
         rearLeftPosition = ((HVA_Victor*) m_rearLeftMotor)->GetPosition();

         // set drive to position mode
         ((HVA_Victor*) m_rearRightMotor)->ChangeControlMode(
               HVA_Victor::kPosition);
         ((HVA_Victor*) m_rearLeftMotor)->ChangeControlMode(
               HVA_Victor::kPosition);

         // set the desired ending position
         ((HVA_Victor*) m_rearRightMotor)->Set(
               rearRightPosition - BRIDGE_TILT_CORRECTION_DISTANCE);
         ((HVA_Victor*) m_rearLeftMotor)->Set(
               rearLeftPosition - BRIDGE_TILT_CORRECTION_DISTANCE);

         // enable the position mode
         ((HVA_Victor*) m_rearRightMotor)->EnableControl();
         ((HVA_Victor*) m_rearLeftMotor)->EnableControl();
      }
      //         else
      //         {
      //            // drive forward on the bridge
      //            // Note: Need to continue to call to keep the watchdog feed
      //            SetLeftRightMotorOutputs(-maxVelocity, -maxVelocity);
      //         }

      break;
   }

   case DRIVING_BACKWARD_ON_BRIDGE:
   {
      // determine if bridge starts to tip
      if (gyroAngle > (m_minGyro + RAMP_FALL_THRESHOLD))
      {
         // change state to driving forward on bridge
         m_balancingState = POSITION_CONTROL_ON_BRIDGE;

         // get the present location
         rearRightPosition = ((HVA_Victor*) m_rearRightMotor)->GetPosition();
         rearLeftPosition = ((HVA_Victor*) m_rearLeftMotor)->GetPosition();

         // set drive to position mode
         ((HVA_Victor*) m_rearRightMotor)->ChangeControlMode(
               HVA_Victor::kPosition);
         ((HVA_Victor*) m_rearLeftMotor)->ChangeControlMode(
               HVA_Victor::kPosition);

         // set the desired ending position
         ((HVA_Victor*) m_rearRightMotor)->Set(
               rearRightPosition + BRIDGE_TILT_CORRECTION_DISTANCE);
         ((HVA_Victor*) m_rearLeftMotor)->Set(
               rearLeftPosition + BRIDGE_TILT_CORRECTION_DISTANCE);

         // enable the position mode
         ((HVA_Victor*) m_rearRightMotor)->EnableControl();
         ((HVA_Victor*) m_rearLeftMotor)->EnableControl();
      }
      //         else
      //         {
      //            // drive forward on the bridge
      //            // Note: Need to continue to call to keep the watchdog feed
      //            SetLeftRightMotorOutputs(maxVelocity, maxVelocity);
      //         }

      break;
   }

   case POSITION_CONTROL_ON_BRIDGE:
   {
      // Make sure the motors are in position mode
      if (((HVA_Victor*) m_rearRightMotor)->GetControlMode()
            != HVA_Victor::kPosition)
      {
         // set drive to position mode
         ((HVA_Victor*) m_rearRightMotor)->ChangeControlMode(
               HVA_Victor::kPosition);
         ((HVA_Victor*) m_rearLeftMotor)->ChangeControlMode(
               HVA_Victor::kPosition);

         // set the desired ending position
         ((HVA_Victor*) m_rearRightMotor)->Set(rearRightPosition);
         ((HVA_Victor*) m_rearLeftMotor)->Set(rearRightPosition);

         // enable the position mode
         ((HVA_Victor*) m_rearRightMotor)->EnableControl();
         ((HVA_Victor*) m_rearLeftMotor)->EnableControl();
      }

      // determine if the operator wants to change the bridge position
      if (positionDelta != 0.0)
      {
         // change the desired bridge position
         rearRightPosition += positionDelta;
         rearLeftPosition += positionDelta;

         // set the desired ending position
         ((HVA_Victor*) m_rearRightMotor)->Set(rearRightPosition);
         ((HVA_Victor*) m_rearLeftMotor)->Set(rearLeftPosition);
      }

      break;
   }
   }

   // print out debug statement every quarter second
   if (Timer::GetFPGATimestamp() - debugTime >= 0.0)
   {
      // reset the timer time
      debugTime = Timer::GetFPGATimestamp();

      // Time, State, Gyro Angle, Min Angle, Max Angle, Angular Velocity, Drive Speed, Drive Position
      printf(
            "%f\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
            //            printf("%f, %d, %f, %f, %f, %f, %f, %f, %f\n",
            (float) (Timer::GetFPGATimestamp() - startBalanceTime),
            m_balancingState, gyroAngle, m_minGyro, m_maxGyro, angularVelocity,
            ((HVA_Victor*) m_rearRightMotor)->GetSpeed(),
            ((HVA_Victor*) m_rearRightMotor)->GetPosition(),
            ((HVA_Victor*) m_rearRightMotor)->Get());
   }
}

/**
 * Arcade drive implements single stick driving.
 * This function lets you directly provide joystick values from any source.
 * This function limits the amount of acceleration allowed.
 * @param moveValue The value to use for towards/backwards
 * @param rotateValue The value to use for the rotate right/left
 * @param squaredInputs If set, increases the sensitivity at low speeds
 */
void HVA_RobotDrive::ArcadeVelocityDriveStepped(float moveValue,
      float rotateValue, float maxAcceleration, bool squaredInputs)
{
   static double previousTime = Timer::GetFPGATimestamp();

   // local variables to hold the computed PWM values for the motors
   float leftMotorDesiredOutput;
   float rightMotorDesiredOutput;

   static float leftMotorOutputVelocity;
   static float rightMotorOutputVelocity;

   moveValue = -Limit(moveValue);
   rotateValue = -Limit(rotateValue);

   if (squaredInputs)
   {
      // square the inputs (while preserving the sign) to increase fine control while permitting full power
      if (moveValue >= 0.0)
      {
         moveValue = (moveValue * moveValue);
      }
      else
      {
         moveValue = -(moveValue * moveValue);
      }
      if (rotateValue >= 0.0)
      {
         rotateValue = (rotateValue * rotateValue);
      }
      else
      {
         rotateValue = -(rotateValue * rotateValue);
      }
   }

   if (moveValue > 0.0)
   {
      if (rotateValue > 0.0)
      {
         leftMotorDesiredOutput = moveValue - rotateValue;
         rightMotorDesiredOutput = max(moveValue, rotateValue);
      }
      else
      {
         leftMotorDesiredOutput = max(moveValue, -rotateValue);
         rightMotorDesiredOutput = moveValue + rotateValue;
      }
   }
   else
   {
      if (rotateValue > 0.0)
      {
         leftMotorDesiredOutput = -max(-moveValue, rotateValue);
         rightMotorDesiredOutput = moveValue + rotateValue;
      }
      else
      {
         leftMotorDesiredOutput = moveValue - rotateValue;
         rightMotorDesiredOutput = -max(-moveValue, -rotateValue);
      }
   }

   double changeInTime = Timer::GetFPGATimestamp() - previousTime;
   previousTime = Timer::GetFPGATimestamp();

   if (changeInTime < TIME_THRESHOLD)
   {
      // Set the output for the left motor.
      if (leftMotorDesiredOutput > leftMotorOutputVelocity + maxAcceleration
            * changeInTime)
         leftMotorOutputVelocity += maxAcceleration * changeInTime;
      else if (leftMotorDesiredOutput < leftMotorOutputVelocity
            - maxAcceleration * changeInTime)
         leftMotorOutputVelocity -= maxAcceleration * changeInTime;
      else
         leftMotorOutputVelocity = leftMotorDesiredOutput;

      // Set the output for the right motor.
      if (rightMotorDesiredOutput > rightMotorOutputVelocity + maxAcceleration
            * changeInTime)
         rightMotorOutputVelocity += maxAcceleration * changeInTime;
      else if (rightMotorDesiredOutput < rightMotorOutputVelocity
            - maxAcceleration * changeInTime)
         rightMotorOutputVelocity -= maxAcceleration * changeInTime;
      else
         rightMotorOutputVelocity = rightMotorDesiredOutput;
   }
   else
   {
      leftMotorOutputVelocity = m_rearLeftMotor->Get() / m_maxOutput;
      rightMotorOutputVelocity = m_rearRightMotor->Get() / m_maxOutput;
   }

   SetLeftRightMotorOutputs(-leftMotorOutputVelocity, -rightMotorOutputVelocity);
}

void HVA_RobotDrive::ArcadeVelocityDriveStepped(float moveValue,
      float rotateValue, float maxForwardAcceleration,
      float maxRotationalAcceleration, bool squaredInputs)
{
   static double previousTime = Timer::GetFPGATimestamp();

   // local variables to hold the computed PWM values for the motors
   float leftMotorDesiredOutput;
   float rightMotorDesiredOutput;

   static float leftMotorOutputVelocity;
   static float rightMotorOutputVelocity;

   moveValue = -Limit(moveValue);
   rotateValue = -Limit(rotateValue);

   if (squaredInputs)
   {
      // square the inputs (while preserving the sign) to increase fine control while permitting full power
      if (moveValue >= 0.0)
      {
         moveValue = (moveValue * moveValue);
      }
      else
      {
         moveValue = -(moveValue * moveValue);
      }
      if (rotateValue >= 0.0)
      {
         rotateValue = (rotateValue * rotateValue);
      }
      else
      {
         rotateValue = -(rotateValue * rotateValue);
      }
   }

   if (moveValue > 0.0)
   {
      if (rotateValue > 0.0)
      {
         leftMotorDesiredOutput = moveValue - rotateValue;
         rightMotorDesiredOutput = max(moveValue, rotateValue);
      }
      else
      {
         leftMotorDesiredOutput = max(moveValue, -rotateValue);
         rightMotorDesiredOutput = moveValue + rotateValue;
      }
   }
   else
   {
      if (rotateValue > 0.0)
      {
         leftMotorDesiredOutput = -max(-moveValue, rotateValue);
         rightMotorDesiredOutput = moveValue + rotateValue;
      }
      else
      {
         leftMotorDesiredOutput = moveValue - rotateValue;
         rightMotorDesiredOutput = -max(-moveValue, -rotateValue);
      }
   }

   double changeInTime = Timer::GetFPGATimestamp() - previousTime;
   previousTime = Timer::GetFPGATimestamp();
   if (changeInTime < TIME_THRESHOLD)
   {
      if ((leftMotorDesiredOutput >= 0 && rightMotorDesiredOutput >= 0)
            || (leftMotorDesiredOutput <= 0 && rightMotorDesiredOutput <= 0))
      {
         // Set the output for the left motor.
         if (leftMotorDesiredOutput > leftMotorOutputVelocity
               + maxForwardAcceleration * changeInTime)
            leftMotorOutputVelocity += maxForwardAcceleration * changeInTime;
         else if (leftMotorDesiredOutput < leftMotorOutputVelocity
               - maxForwardAcceleration * changeInTime)
            leftMotorOutputVelocity -= maxForwardAcceleration * changeInTime;
         else
            leftMotorOutputVelocity = leftMotorDesiredOutput;

         // Set the output for the right motor.
         if (rightMotorDesiredOutput > rightMotorOutputVelocity
               + maxForwardAcceleration * changeInTime)
            rightMotorOutputVelocity += maxForwardAcceleration * changeInTime;
         else if (rightMotorDesiredOutput < rightMotorOutputVelocity
               - maxForwardAcceleration * changeInTime)
            rightMotorOutputVelocity -= maxForwardAcceleration * changeInTime;
         else
            rightMotorOutputVelocity = rightMotorDesiredOutput;
      }
      else
      {
         // Set the output for the left motor.
         if (leftMotorDesiredOutput > leftMotorOutputVelocity
               + maxRotationalAcceleration * changeInTime)
            leftMotorOutputVelocity += maxRotationalAcceleration * changeInTime;
         else if (leftMotorDesiredOutput < leftMotorOutputVelocity
               - maxRotationalAcceleration * changeInTime)
            leftMotorOutputVelocity -= maxRotationalAcceleration * changeInTime;
         else
            leftMotorOutputVelocity = leftMotorDesiredOutput;

         // Set the output for the right motor.
         if (rightMotorDesiredOutput > rightMotorOutputVelocity
               + maxRotationalAcceleration * changeInTime)
            rightMotorOutputVelocity += maxRotationalAcceleration
                  * changeInTime;
         else if (rightMotorDesiredOutput < rightMotorOutputVelocity
               - maxRotationalAcceleration * changeInTime)
            rightMotorOutputVelocity -= maxRotationalAcceleration
                  * changeInTime;
         else
            rightMotorOutputVelocity = rightMotorDesiredOutput;
      }
   }
   else
   {
      leftMotorOutputVelocity = m_rearLeftMotor->Get() / m_maxOutput;
      rightMotorOutputVelocity = m_rearRightMotor->Get() / m_maxOutput;
   }

   SetLeftRightMotorOutputs(-leftMotorOutputVelocity, -rightMotorOutputVelocity);
}
