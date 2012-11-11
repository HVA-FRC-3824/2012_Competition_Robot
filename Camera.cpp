#include "MyRobot.h"

/*************************** Camera / Shooter Defines *************************/
#define NO_TARGET                 -999999.0f
#define SHOOTER_SPEED_CHANGE              .2
#define MINIMUM_ROTATION_OF_SHOOTER      -.3
#define MAXIMUM_ROTATION_OF_SHOOTER       .3

/* Conversion Defines */
#define GYRO_CONVERSION               0.0044
#define PIXEL_CONVERSION             0.00009
#define PIXEL_OFFSET                      -8
/******************************************************************************/

#define PIXEL_OFF_THRESHOLD              5.0
#define I_INCRESS_THRESHOLD             20.0
#define VOLTAGE_THRESHOLD               0.01
/**
 * Camera control
 */
bool MyRobot::cameraControl(void)
{
   int targetStatus; // Is the target found by the robot
   double distanceToTarget; // The distance from the targets
   static double voltageToDriveShooter; // The voltage to drive the shooter
   double valueToRotate; // The value to rotate the shooter to relative to current position
   double previousGyro = m_gyroHorizontal->GetAngle();
   double pixelOff;

   // Process the camera images
   targetStatus = readCamera(heightOfTriangle, distanceToTarget,
         voltageToDriveShooter, valueToRotate, pixelOff);

   // Add to the rotate value based off the gyro
   valueToRotate += (previousGyro - m_gyroHorizontal->GetAngle())
         * GYRO_CONVERSION;
   previousGyro = m_gyroHorizontal->GetAngle();

   // Check to see if the shooter is to be controled by the robot or by the operators
   if ((m_driverStation->GetDigitalIn(AUTONOMOUSLY_RUN_SHOOTER) == 0) || (IsAutonomous()
         == true))
   {
      // change the modes of the jaguars
      if ((m_shooterWheel->GetControlMode() != CANJaguar::kVoltage)
            || (m_shooterRotate->GetControlMode() != CANJaguar::kPosition))
      {
         // Set up the Shooter Jaguar
         m_shooterWheel->ChangeControlMode(CANJaguar::kVoltage);
         m_shooterWheel->Set(0.0f);
         m_shooterWheel->EnableControl();

         // Set up the Rotate Jaguar
         m_shooterRotate->ChangeControlMode(CANJaguar::kPosition);
         m_shooterRotate->SetPID(4000, 0.2, 10.0);
         m_shooterRotate->Set(m_shooterRotate->GetPosition());
         m_shooterRotate->EnableControl();
      }

      // Control the Shooter Autonomously
      autonomouslyDriveShooter(targetStatus, distanceToTarget,
            voltageToDriveShooter, valueToRotate);
   }
   else
   {
      // Change the modes of the modes of the jaguars
      if ((m_shooterWheel->GetControlMode() != CANJaguar::kPercentVbus)
            || (m_shooterRotate->GetControlMode() != CANJaguar::kPercentVbus))
      {
         // Set up the Shooter Jaguar
         m_shooterWheel->ChangeControlMode(CANJaguar::kPercentVbus);
         m_shooterWheel->Set(0.0f);
         m_shooterWheel->EnableControl();

         // Set up the Rotate Jaguar
         m_shooterRotate->ChangeControlMode(CANJaguar::kPercentVbus);
         m_shooterRotate->Set(0.0f);
         m_shooterRotate->EnableControl();
      }
   }
   
   printf("Pixels off: %f\n", pixelOff);
   
   if ((pixelOff < PIXEL_OFF_THRESHOLD) && (fabs(
         m_shooterWheel->GetOutputVoltage() - voltageToDriveShooter)
         < VOLTAGE_THRESHOLD))
   {
      readyToFire = true;
   }
   else
   {
      readyToFire = false;
   }
   return readyToFire;
}

/**
 * Routine to process the camera images.
 *
 * Return of 0 means targets found
 * Return of 1 means only one top target found
 * Return of 2 means no targets found
 */
int MyRobot::readCamera(double &heightOfTriangle, double &distanceToTarget,
      double &voltageToDriveShooter, double &valueToRotate, double &pixelOff)
{
   int targetStatus = 0; // Is the target found by the robot

   // Get the camera instance.
   AxisCamera &camera = AxisCamera::GetInstance();

   if (camera.IsFreshImage())
   {
      // get the camera image
      HSLImage *HSLimage = camera.GetImage();

      // Clear the array
      for (int i = 0; i < NUMBER_OF_TARGETS; i++)
      {
         for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
         {
            results[i][j] = 0.0f;
            if (j == ASPECT_RATIO_SCORE_INDEX)
               results[i][j] = NO_TARGET;
         }
      }

      // Process the image and get the results
      IVA_ProcessImage(HSLimage->GetImaqImage(), results);

      // Check to see if the target was found
      if ((results[RIGHT_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET)
            || (results[LEFT_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET))
      {
         if (results[TOP_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET)
         {
            // No targets found
            targetStatus = 2;
         }
         else
         {
            // One target found
            targetStatus = 1;
         }
      }

      // printf("Received data.\n");
      // printf("Unsorted data.\n");
      // for (int i = 0; i < NUMBER_OF_TARGETS; i++)
      // {
      //    for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
      //    {
      //       printf("%f ", results[i][j]);
      //    }
      //    printf("\n");
      // }
      // printf("\n");

      // Sort the array
      if (targetStatus == 0)
      {
         sortTargetArray(results);
      }

      // printf("Sorted data.\n");
      // for (int i = 0; i < NUMBER_OF_TARGETS; i++)
      // {
      //    for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
      //    {
      //       printf("%f ", results[i][j]);
      //    }
      //    printf("\n");
      // }
      // printf("\n");

      // Print information for excel
      // for (int i = 0; i < NUMBER_OF_TARGETS; i++)
      // {
      //    if (i == 0)
      //       printf("%5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f",
      //               results[i][CENTER_OF_MASS_X_INDEX],
      //               results[i][CENTER_OF_MASS_Y_INDEX],
      //               results[i][BOUNDING_RECT_WIDTH_INDEX],
      //               results[i][BOUNDING_RECT_HEIGHT_INDEX],
      //               results[i][PARTICAL_AREA_INDEX],
      //               results[i][BOUNDING_BOX_AREA_INDEX],
      //               results[i][FIRST_PIX_X_INDEX],
      //               results[i][FIRST_PIX_Y_INDEX],
      //               results[i][ASPECT_RATIO_SCORE_INDEX],
      //               results[i][RECTANGLE_SCORE_INDEX],
      //               results[i][PARAMETER_INDEX]);
      //    else
      //       printf("\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f",
      //               results[i][CENTER_OF_MASS_X_INDEX],
      //               results[i][CENTER_OF_MASS_Y_INDEX],
      //               results[i][BOUNDING_RECT_WIDTH_INDEX],
      //               results[i][BOUNDING_RECT_HEIGHT_INDEX],
      //               results[i][PARTICAL_AREA_INDEX],
      //               results[i][BOUNDING_BOX_AREA_INDEX],
      //               results[i][FIRST_PIX_X_INDEX],
      //               results[i][FIRST_PIX_Y_INDEX],
      //               results[i][ASPECT_RATIO_SCORE_INDEX],
      //               results[i][RECTANGLE_SCORE_INDEX],
      //               results[i][PARAMETER_INDEX]);
      // }
      // printf("\n\n");
      delete HSLimage;
   }

   // Calculate the distance to the basket.
   // Calculate the heigth of the top triangle
   if (targetStatus == 0)
   {
      heightOfTriangle = sqrt(
            pow(
                  results[TOP_TARGET][CENTER_OF_MASS_X_INDEX]
                        - ((results[LEFT_TARGET][CENTER_OF_MASS_X_INDEX]
                              + results[RIGHT_TARGET][CENTER_OF_MASS_X_INDEX])
                              / 2.0), 2.0) + pow(
                  results[TOP_TARGET][CENTER_OF_MASS_Y_INDEX]
                        - ((results[LEFT_TARGET][CENTER_OF_MASS_Y_INDEX]
                              + results[RIGHT_TARGET][CENTER_OF_MASS_Y_INDEX])
                              / 2.0), 2.0));

      //double oldDistance = 43.867483765 * pow(0.986054120856, height);

      // Calculate Distance
      distanceToTarget = 2741.0998 * pow((1 / heightOfTriangle), 1.2082);

      // Calculate the voltage to drive the shooter
      // TODO - Recalculate the Voltage off of the height of the triangle
      if (m_buttonBox->GetRawButton(CALCULATE_VOLTAGE) == true)
      {
         voltageToDriveShooter = 5.275914116 * exp(
               0.0286983122 * distanceToTarget);
      }

   }
   // Calculate the value to rotate if a target was found
   /*
    Range of the Camera: -160 to 160     Combined: 320
    Field of view of the Camera: 47 deg
    Range of the Rotator: -.4 to .4      Combined: .8
    Field of the Rotator: 180 deg

    degree     encoder value      47      .8
    -----  * --------------   =  ----- * ------ = 0.0006528
    pixel      degree             320     180
    */

   // Calculate the value to rotate the shooter to.
   if (targetStatus == 0 || targetStatus == 1)
   {
      pixelOff = (results[TOP_TARGET][CENTER_OF_MASS_X_INDEX] - (320 / 2));
      valueToRotate = pixelOff * PIXEL_CONVERSION;
   }

   return targetStatus;
}

/**
 * Method to drive the shooter automonously
 */
void MyRobot::autonomouslyDriveShooter(int targetStatus,
      double distanceToTarget, double voltageToDriveShooter,
      double valueToRotate)
{
   static float wheelOffset = 0.0;
   static bool buttonPressedShooterCorrect = false;

   // Set the offset to zero
   if ((m_buttonBox->GetRawButton(RESET_BALL_OFFSET) == false)
         && (buttonPressedShooterCorrect == false))
   {
      wheelOffset = 0.0;
      buttonPressedShooterCorrect = true;
   }

   // Increment the offset
   else if ((m_buttonBox->GetRawButton(INCREMENT_BALL_OFFSET) == true)
         && (buttonPressedShooterCorrect == false))
   {
      wheelOffset += SHOOTER_SPEED_CHANGE;
      buttonPressedShooterCorrect = true;
   }

   // Decrement the offset
   else if ((m_buttonBox->GetRawButton(DECREMENT_BALL_OFFSET) == true)
         && (buttonPressedShooterCorrect == false))
   {
      wheelOffset -= SHOOTER_SPEED_CHANGE;
      buttonPressedShooterCorrect = true;
   }

   else if ((m_buttonBox->GetRawButton(RESET_BALL_OFFSET) != false)
         && (m_buttonBox->GetRawButton(INCREMENT_BALL_OFFSET) != true)
         && (m_buttonBox->GetRawButton(DECREMENT_BALL_OFFSET) != true))
   {
      buttonPressedShooterCorrect = false;
   }

   // Set the Voltage output for the wheel
   m_shooterWheel->Set(voltageToDriveShooter + wheelOffset);

   // Set the Rotational value for the shooter
   double rotationValue = m_shooterRotate->GetPosition() + valueToRotate;// + PIXEL_OFFSET;
   
   // <Test>
   // Incresse I if the goal is close
//   if(fabs(rotationValue - m_shooterRotate->GetPosition()) < .01)
//   {
//      if (m_shooterRotate->GetI() == TURN_CONTROLLER_I)
//      {
//         m_shooterRotate->SetPID(m_shooterRotate->GetP(), .01, m_shooterRotate->GetD());
//      }
//   }
//   else
//   {
//      if (m_shooterRotate->GetI() != TURN_CONTROLLER_I)
//      {
//         m_shooterRotate->SetPID(m_shooterRotate->GetP(), TURN_CONTROLLER_I, m_shooterRotate->GetD());
//      }
//   }
   // </Test>
   /************* Enable to track target *************************************/
   if ((rotationValue >= MINIMUM_ROTATION_OF_SHOOTER) && (rotationValue
         <= MAXIMUM_ROTATION_OF_SHOOTER))
   {
      m_shooterRotate->Set(rotationValue);
   }
   else
   {
      if (rotationValue > MAXIMUM_ROTATION_OF_SHOOTER)
      {
         if (targetStatus == 2)
         {
            m_shooterRotate->Set(0.0f);
            valueToRotate = 0.0f;
         }
         else
         {
            m_shooterRotate->Set(MAXIMUM_ROTATION_OF_SHOOTER);
         }
      }
      else
      {
         if (targetStatus == 2)
         {
            m_shooterRotate->Set(0.0f);
            valueToRotate = 0.0f;
         }
         else
         {
            m_shooterRotate->Set(MINIMUM_ROTATION_OF_SHOOTER);
         }
      }
   }
   /******************************************************************************/
}

/**
 * Sort the targets
 */
void MyRobot::sortTargetArray(
      double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS])
{
   double temp[NUMBER_OF_TARGET_PARAMETERS];

   // Check to see if the left and right are backwards
   if (targets[LEFT_TARGET][CENTER_OF_MASS_X_INDEX]
         > targets[RIGHT_TARGET][CENTER_OF_MASS_X_INDEX])
   {
      for (int i = 0; i < NUMBER_OF_TARGET_PARAMETERS; i++)
      {
         //printf("in switch target values\n");
         temp[i] = targets[LEFT_TARGET][i];
         targets[LEFT_TARGET][i] = targets[RIGHT_TARGET][i];
         targets[RIGHT_TARGET][i] = temp[i];
      }
   }
}
