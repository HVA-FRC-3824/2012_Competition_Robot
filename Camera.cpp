#include "MyRobot.h"

/*************************** Camera / Shooter Defines *************************/
#define NO_TARGET                 -999999.0f
#define SHOOTER_SPEED_CHANGE              .2

/* Conversion Defines */
#define GYRO_CONVERSION               0.0044
// TODO - tweak
//#define PIXEL_CONVERSION             0.00009
#define PIXEL_CONVERSION             0.00025

// Incress the value of the Voltage offset to incress the distance of the shooter.
// Calculated shooter voltage offset       29.32880164
#define SHOOTER_VOLTAGE_OFFSET           29.42880164

// Incress the value of the pixel offset in order to aim more to the right.
#define PIXEL_OFFSET                     10.0
/******************************************************************************/

#define PIXEL_OFF_THRESHOLD              5.0
#define I_INCRESS_THRESHOLD             20.0
#define VOLTAGE_THRESHOLD                0.1
/**
 * Camera control
 */
bool MyRobot::cameraControl(void)
{
   int    targetStatus; // Is the target found by the robot
   double distanceToTarget; // The distance from the targets
   static double voltageToDriveShooter = DEFAULT_AUTONOMOUS_VOLTAGE; // The voltage to drive the shooter
   double valueToRotate; // The value to rotate the shooter to relative to current position
   double previousGyro = m_gyroHorizontal->GetAngle();
   double pixelOff;

   // Check to see if the shooter is to be controled by the robot or by the operators
   if ((m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == true) ||
       (IsAutonomous() == true))
   {
      printf("In Camera Code\n");

      // Process the camera images
      targetStatus = readCamera(heightOfTriangle, distanceToTarget,
            voltageToDriveShooter, valueToRotate, pixelOff);

      // Add to the rotate value based off the gyro
      printf("Gyro: %f\n", m_gyroHorizontal->GetAngle());

      //      valueToRotate += (previousGyro - m_gyroHorizontal->GetAngle())
      //            * GYRO_CONVERSION;
      //      previousGyro = m_gyroHorizontal->GetAngle();

      // Control the Shooter Autonomously
      autonomouslyDriveShooter(targetStatus, distanceToTarget,
            voltageToDriveShooter, valueToRotate);
   }

   if ((fabs(pixelOff) < PIXEL_OFF_THRESHOLD) &&
       (fabs(m_shooterWheel->GetOutputVoltage() - m_shooterWheel->Get())
        < VOLTAGE_THRESHOLD))
   {
      readyToFire = true;
      m_driverStationEnhancedIO->SetDigitalOutput(READY_TO_FIRE_LED, true);
   }
   else
   {
      readyToFire = false;
      m_driverStationEnhancedIO->SetDigitalOutput(READY_TO_FIRE_LED, false);
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
                        double &voltageToDriveShooter, double &valueToRotate,
                        double &pixelOff)
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

      printf("Sorted data.\n");
      for (int i = 0; i < NUMBER_OF_TARGETS; i++)
      {
         for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
         {
            printf("%f ", results[i][j]);
         }
         printf("\n");
      }
      printf("\n");

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
   else // Not a new image
   {
      // no targets found
      targetStatus = 3;
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

      // Calculate the voltage to drive the shooter
      if (IsOperatorControl())
      {
         // Make sure there are targets before changing voltage
         if (targetStatus == 0)
         {
            voltageToDriveShooter = (-0.00001894 * pow(heightOfTriangle, 3))
                  + (0.00602003 * pow(heightOfTriangle, 2)) + (-0.64413759
                  * heightOfTriangle) + SHOOTER_VOLTAGE_OFFSET;
         }
      }
      else // In Autonomous
      {
         printf("Volate: %f", voltageToDriveShooter);
//         if (m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER)
//               == true)
//         {
//            //Todo - add in autnomous volt
//            double cameraValue = (-0.00001894 * pow(heightOfTriangle, 3))
//                  + (0.00602003 * pow(heightOfTriangle, 2)) + (-0.64413759
//                  * heightOfTriangle) + SHOOTER_VOLTAGE_OFFSET;
//            if ((targetStatus == 0) && (cameraValue
//                  >= MIN_SHOOTER_VOLTAGE_AUTONOMOUS) && (cameraValue
//                  <= MAX_SHOOTER_VOLTAGE_AUTONOMOUS))
//               voltageToDriveShooter = cameraValue;
//         }
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
   // ToDO -add in
   if ((targetStatus == 0 || targetStatus == 1) && (IsOperatorControl()
         || (m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER)
               == true)))
   {
      // Only Calculate pixel off if targets are found
      pixelOff = ((results[TOP_TARGET][CENTER_OF_MASS_X_INDEX] + PIXEL_OFFSET)
            - (320 / 2));
   }
   else
   {
      // No targets found so no pixel error
      pixelOff = 0;
   }

   valueToRotate = pixelOff * PIXEL_CONVERSION;
   printf("Pixel Off: %f\nValue to Rotate BG: %f\n\n", pixelOff, valueToRotate);

   return targetStatus;
}

/**
 * Method to drive the shooter automonously
 */
void MyRobot::autonomouslyDriveShooter(int targetStatus, double distanceToTarget,
                                       double voltageToDriveShooter, double valueToRotate)
{
   // Set the Voltage output for the wheel if the Override is off
   if ((m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE) == false) ||
       (IsAutonomous() == true))
   {
      m_shooterWheel->Set(voltageToDriveShooter);
   }

   // Use the pot to adjust the voltage
   else if ((m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE) == true) &&
            (m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE_MODE) == false))
   {
      m_shooterWheel->Set(
            voltageToDriveShooter + ((m_driverStationEnhancedIO->GetAnalogIn(
                  SHOOTER_VELOCITY_POT) - MID_POT_VALUE)
                  * ((MAX_SHOOTER_VOLTAGE_ADJUSTMENT) / (MAX_POT_VALUE
                        - MID_POT_VALUE))));
   }

   // Only run the rotation of the shooter if the mode is Autonomous
   if (m_shooterRotationControlState == kAutonomous)
   {
      // Set the Rotational value for the shooter
      double rotationValue = m_shooterRotate->GetPosition() + valueToRotate;// + PIXEL_OFFSET;

      // Incresse I if the goal is close
      static bool incressedI = false;
      if (fabs(rotationValue - m_shooterRotate->GetPosition()) < VALUE_TO_INCRESS_I)
      {
         if (incressedI == false)
         {
            m_shooterRotate->SetPID(m_shooterRotate->GetP(), INCRESSED_I_VALUE,
                  m_shooterRotate->GetD());
            incressedI = true;
         }
      }
      else
      {
         if (incressedI = true)
         {
            m_shooterRotate->SetPID(m_shooterRotate->GetP(),
                  ROTATE_CONTROLLER_I, m_shooterRotate->GetD());
            incressedI = false;
         }
      }

      /************* Enable to track target *************************************/
      if ((rotationValue >= MINIMUM_ROTATION_OF_SHOOTER) &&
          (rotationValue <= MAXIMUM_ROTATION_OF_SHOOTER))
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
}

/**
 * Sort the targets
 */
void MyRobot::sortTargetArray(
      double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS])
{
   double temp[NUMBER_OF_TARGET_PARAMETERS];

   // Check to see if the left and right are backwards
   if (targets[LEFT_TARGET][CENTER_OF_MASS_X_INDEX] >
         targets[RIGHT_TARGET][CENTER_OF_MASS_X_INDEX])
   {
      for (int i = 0; i < NUMBER_OF_TARGET_PARAMETERS; i++)
      {
         temp[i] = targets[LEFT_TARGET][i];
         targets[LEFT_TARGET][i] = targets[RIGHT_TARGET][i];
         targets[RIGHT_TARGET][i] = temp[i];
      }
   }
}
