#include "MyRobot.h"

/*************************** Camera / Shooter Defines *************************/
#define NO_TARGET                 -999999.0f
#define SHOOTER_SPEED_CHANGE              .2

/* Conversion Defines */
#define GYRO_CONVERSION               0.0044

#define PIXEL_CONVERSION             0.00025

// Incress the value of the Voltage offset to incress the distance of the shooter.
// Calculated shooter voltage offset       29.32880164
//#define SHOOTER_VOLTAGE_OFFSET           29.42880164  // Competition Original
#define SHOOTER_VOLTAGE_OFFSET             29.62880164

// Incress the value of the pixel offset in order to aim more to the right.
//#define PIXEL_OFFSET                    10.0
#define PIXEL_OFFSET                     15.0

#define PIXEL_OFF_THRESHOLD              5.0
#define ROTATION_OFF_THRESHOLD           .05

#define I_INCRESS_THRESHOLD             20.0
#define VOLTAGE_THRESHOLD                0.1
#define VELOCITY_THRESHOLD              20.0

/******************************************************************************/

/**
 * Ball Shooter control
 */
bool MyRobot::runShooterControl(float defaultVelocity, float defaultVoltage)
{
   int    targetStatus     = 2;   // Is the target found by the robot (default to no targets)
   float  voltageToDriveShooter = defaultVoltage; // The voltage to drive the shooter
   float  velocityToDriveShooter = defaultVelocity; // The RPM to drive the shooter
   double distanceToTarget = 0.0; // The distance from the targets
   double valueToRotate    = 0.0; // The value to rotate the shooter to relative to current position
   double pixelOff;
   
   /************************ Check Shooting Wheel Mode **************************/
   // determine the control panel shooter mode
   if (m_driverStationEnhancedIO->GetDigital(SHOOTER_VOLTAGE_MODE) == true)
   {
      // set the shooter to voltage mode
      if (m_shooterWheel->GetControlMode() != CANJaguar::kVoltage)
         setShooterModeToVoltage();
   }
   else if (m_driverStationEnhancedIO->GetDigital(SHOOTER_VELOCITY_MODE) == true)
   {
      // set the shooter to PWM mode
      if (m_shooterWheel->GetControlMode() != CANJaguar::kSpeed)
         setShooterModeToSpeed();
   }
   else
   {
      // set the shooter to PWM mode
      if (m_shooterWheel->GetControlMode() != CANJaguar::kSpeed)
         setShooterModeToSpeed();
   }

   /************************ Check if Image data is needed **********************/
   // Check to see if the shooter is to be controled by the robot or by the operators
   if (m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == true && m_ferrisState == kStop)
   {
      // Process the camera images
      // Note: All method parameters are outputs
      targetStatus = readCamera(heightOfTriangle, distanceToTarget,
                                voltageToDriveShooter, velocityToDriveShooter,
                                valueToRotate, pixelOff);
   }

   /************************ Check for rotation override ************************/
   // Check for rotation override and autonomous mode
   if ((m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE)        == true) ||
       (m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == false))
   {
      // If the rotation is being driven by the operator then set the value equal to the pot
      if (m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == true)
      {
         m_shooterRotate->Set(
               (m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_ROTATION_POT)
                     - MID_POT_VALUE) * ((MAXIMUM_ROTATION_OF_SHOOTER)
                     / (MAX_POT_VALUE - MID_POT_VALUE)));
      }

      // automounously run shooter is disabled and not in operator override,
      // so move to home position
      else
      {
         // move shooter to home position
         m_shooterRotate->Set(0.0f);
      }
   }

   // Set rotation from image. i.e: autonomouly run shooter and rotation override
   // ride is not enabled
   else
   {
      // use targets to set rotation
      setRotationBasedOnTargets(targetStatus, valueToRotate);
   }

   /************************ RUN ShootingWheel ********************************/
   // drive the shooter manually if the cammera tracking is disabled or the full override is called
   if ((m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE_MODE) == true) &&
       (m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE)      == true))
   {
      // determine the shooter mode
      if (m_shooterWheel->GetControlMode() == CANJaguar::kVoltage)
      {
         // voltage mode
         m_shooterWheel->Set(
               (((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT)
                     / MAX_POT_VALUE) * (MAX_SHOOTER_SPEED_PERCENT
                     - MIN_SHOOTER_SPEED_PERCENT)) + MIN_SHOOTER_SPEED_PERCENT)
                     * MAX_ROBOT_VOLTAGE);
      }
      else
      {
         m_shooterWheel->Set(
               ((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT)
                     / MAX_POT_VALUE) *
                      (MAX_SHOOTER_SPEED_RPM - MIN_SHOOTER_SPEED_RPM))
                       + MIN_SHOOTER_SPEED_RPM);
      }
   }

   // check to see if the shooter wheel is not overriden or controlled by the targets
   // if not then run the shooter wheel the defaut velocity or voltage
   else if ((m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == false))
   {
      setWheelBasedOnTargets(defaultVoltage, defaultVelocity);
   }

   // shooter wheel controlled by targets
   else
   {
      // set the wheel velocity based on targets
      setWheelBasedOnTargets(voltageToDriveShooter, velocityToDriveShooter);
   }

   /*********************** Determine if ready to shoot **************************/
   // criteria for being ready to shoot
   // Pixels off from targets are within range and shooter wheel is at correct speed
   if ((((m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == true) && 
         (m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == false) && 
         (fabs(pixelOff) < PIXEL_OFF_THRESHOLD)) || 
       (((m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER) == false) || 
         (m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == true)) && 
         (fabs(m_shooterRotate->GetPosition() - m_shooterRotate->Get()) < ROTATION_OFF_THRESHOLD))) && 
       (((m_shooterWheel->GetControlMode() == CANJaguar::kVoltage) &&
         (fabs(m_shooterWheel->GetOutputVoltage() - m_shooterWheel->Get()) < VOLTAGE_THRESHOLD)) ||
        ((m_shooterWheel->GetControlMode() == CANJaguar::kSpeed) &&
         (fabs(m_shooterWheel->GetSpeed() - m_shooterWheel->Get()) < VELOCITY_THRESHOLD))))
   {
      readyToFire = true;
      m_driverStationEnhancedIO->SetDigitalOutput(READY_TO_FIRE_LED, true);
   }

   // not ready to shoot (wheel is not at proper velocity
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
                        float &voltageToDriveShooter, float &velocityToDriveShooter,
                        double &valueToRotate, double &pixelOff)
{
   int targetStatus = 0; // Is the target found by the robot

   // Get the camera instance.
   AxisCamera &camera = AxisCamera::GetInstance();

   // determine is a camera image is available
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

   // Calculate the distance to the basket based on the heigth of the top triangle
   if (targetStatus == 0)
   {
      // find the height of the triangle
      heightOfTriangle = sqrt(pow(
                  results[TOP_TARGET][CENTER_OF_MASS_X_INDEX]
                        - ((results[LEFT_TARGET][CENTER_OF_MASS_X_INDEX]
                              + results[RIGHT_TARGET][CENTER_OF_MASS_X_INDEX])
                              / 2.0), 2.0) + pow(
                  results[TOP_TARGET][CENTER_OF_MASS_Y_INDEX]
                        - ((results[LEFT_TARGET][CENTER_OF_MASS_Y_INDEX]
                              + results[RIGHT_TARGET][CENTER_OF_MASS_Y_INDEX])
                              / 2.0), 2.0));

      // Make sure there are targets before changing voltage
      // Do not calculate the velocity to drive the shooter if the override mode
      // switch is up
      // TODO - add code to drive shooter off of the ultrasonic sensor.
      if (targetStatus == 0 && m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE_MODE) == false)
      {
         // calculate the voltage of the shooter
         voltageToDriveShooter = (-0.00001894 * pow(heightOfTriangle, 3))
               + (0.00602003 * pow(heightOfTriangle, 2)) + (-0.64413759
               * heightOfTriangle) + SHOOTER_VOLTAGE_OFFSET;

         // calculate the speed (RPM) of the shooter
         // TODO - Calculate the equation for the velocity to drive the shooter
         //velocityToDriveShooter =
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
   // TODO - Possibly add in tracking
   if (targetStatus == 0 || targetStatus == 1)
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

   // set the value to rotate the shooter
   valueToRotate = pixelOff * PIXEL_CONVERSION;
   printf("Pixel Off: %f\nValue to Rotate BG: %f\n\n", pixelOff, valueToRotate);

   return targetStatus;
}

/**
 * Set the shooter rotation based on the targets
 */
void MyRobot::setRotationBasedOnTargets(int targetStatus, double valueToRotate)
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
}

/**
 * Method to drive the shooter automonously
 */
void MyRobot::setWheelBasedOnTargets(float voltageToDriveShooter, float velocityToDriveShooter)
{
   // check the wheel shooter mode for voltage mode
   if (m_shooterWheel->GetControlMode() == CANJaguar::kVoltage)
   {
      // Set the Voltage output for the wheel if the Override is off
      if (m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE) == false)
      {
         // direct set from targets
         m_shooterWheel->Set(voltageToDriveShooter);
      }

      // Use the pot to offset the voltage
      else
      {
         m_shooterWheel->Set(
               voltageToDriveShooter + ((m_driverStationEnhancedIO->GetAnalogIn(
                     SHOOTER_VELOCITY_POT) - MID_POT_VALUE)
                     * ((MAX_SHOOTER_VOLTAGE_ADJUSTMENT) / (MAX_POT_VALUE
                           - MID_POT_VALUE))));
      }
   }

   // must be in velocity (RPM) mode
   else
   {
      // Set the Voltage output for the wheel if the Override is off
      if (m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE) == false)
      {
         // direct set from targets
         printf("Setting Shooter SetPoint\n");
         m_shooterWheel->Set(velocityToDriveShooter);
      }

      // Use the pot to offset the voltage
      else
      {
         m_shooterWheel->Set(
               velocityToDriveShooter + ((m_driverStationEnhancedIO->GetAnalogIn(
                     SHOOTER_VELOCITY_POT) - MID_POT_VALUE)
                     * ((MAX_SHOOTER_VELOCITY_ADJUSTMENT) / (MAX_POT_VALUE
                           - MID_POT_VALUE))));
      }
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
