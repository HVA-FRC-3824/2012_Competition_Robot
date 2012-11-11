#include "WPILib.h"
#include "HVA_RobotDrive.h"

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(UINT32 leftMotorChannel,
      UINT32 rightMotorChannel) :
   RobotDrive(leftMotorChannel, rightMotorChannel)
{

}

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(UINT32 frontLeftMotor, UINT32 rearLeftMotor,
      UINT32 frontRightMotor, UINT32 rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(SpeedController *leftMotor,
      SpeedController *rightMotor) :
   RobotDrive(leftMotor, rightMotor)
{

}

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(SpeedController &leftMotor,
      SpeedController &rightMotor) :
   RobotDrive(leftMotor, rightMotor)
{

}

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(SpeedController *frontLeftMotor,
      SpeedController *rearLeftMotor, SpeedController *frontRightMotor,
      SpeedController *rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

/**
 *
 */
HVA_RobotDrive::HVA_RobotDrive(SpeedController &frontLeftMotor,
      SpeedController &rearLeftMotor, SpeedController &frontRightMotor,
      SpeedController &rearRightMotor) :
   RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor)
{

}

float HVA_RobotDrive::GetMaxOutput()
{
   return m_maxOutput;
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

      // set the motor velocities
      SetLeftRightMotorOutputs(-motorOutputVelocity, -motorOutputVelocity);
   }

   // desired distance is negative
   else
   {
      // Calculate the stopping distance
      stoppingDistance = -(pow(motorOutputVelocity * (m_maxOutput), 2)
            / (2 * maxAcceleration * (m_maxOutput)));

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

/**
 *
 */
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
