#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR             1.0
#define NUMBER_OF_ENCODER_LINES         1440
#define NUMBER_OF_ENCODER_LINES_MOTORS   720
#define PID_CHANGE_VALUE                   2

/************************** Power Defines *************************************/
#define SHOOTER_ROTATE_SPEED             0.4f
#define FERRIS_ROTATE_SPEED              0.3f
#define MAX_SPEED                        1.0f
/******************************************************************************/

/************************** Turn Controller Defines ***************************/
#define TURN_CONTROLLER_P                0.02
#define TURN_CONTROLLER_I                0.0001
#define TURN_CONTROLLER_D                0.0
#define TURN_CONTROLLER_PERIOD           0.005
/******************************************************************************/

static void cameraChecker(MyRobot *robot)
{
   printf("In camera tracker\n");
   robot->CameraControl();
}

MyRobot::MyRobot(void)
{
   m_dashboardDataFormat = new DashboardDataFormat();
   m_rightMotor          = new CANJaguar(RIGHT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
   m_leftMotor           = new CANJaguar(LEFT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
   m_shooterWheel        = new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kPercentVbus);
   m_shooterRotate       = new CANJaguar(SHOOTER_ROTATE_MOTOR, CANJaguar::kPercentVbus);
   m_ferrisWheel         = new Victor(FERRIS_WHEEL_MOTOR);
   m_frontBallPickup     = new Victor(FRONT_BALL_PICKUP_MOTOR);
   m_backBallPickup      = new Victor(BACK_BALL_PICKUP_MOTOR);
   m_robotDrive          = new RobotDrive(m_leftMotor, m_rightMotor); // create a robot drive system with four moters
   m_compressor          = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
   m_ballLiftSolenoid    = new DoubleSolenoid(BALL_LIFT_FORWARD, BALL_LIFT_REVERSED);
   m_frontBridgeWheel    = new DoubleSolenoid(FRONT_BRIDGE_WHEEL_FORWARD, FRONT_BRIDGE_WHEEL_REVERSE);
   m_frontBridgeWheel    = new DoubleSolenoid(BACK_BRIDGE_WHEEL_FORWARD, BACK_BRIDGE_WHEEL_REVERSE);
   m_joystick            = new Joystick(JOYSTICK_PORT); // as they are declared above.
   m_buttonBox           = new Joystick(BUTTON_BOX_PORT);
   m_ferrisWheelStop     = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
   m_bottomSlot          = new DigitalInput(BOTTOM_SLOT_DETECTOR_PORT);
   m_gyroHorizontal      = new Gyro(GYRO_HORIZONTAL_PORT);
   m_gyroVerticall       = new Gyro(GYRO_VERTICAL_PORT);
   m_cameraChecker       = new Task("Camera", (FUNCPTR)cameraChecker, 9000);

   m_driverStation       = DriverStation::GetInstance();
   AxisCamera &camera    = AxisCamera::GetInstance();

   // reset the gyro PID and start with zero rotation
   m_rotation = 0.0f;

   // PID controller for the gyro angles
   m_pidOutput      = new HVA_PIDOutput(&m_rotation);
   m_turnController = new HVA_PIDController(TURN_CONTROLLER_P,         // P
                                            TURN_CONTROLLER_I,         // I
                                            TURN_CONTROLLER_D,         // D
                                            m_gyroHorizontal,          // source
                                            m_pidOutput,               // output
                                            TURN_CONTROLLER_PERIOD);   // period

   /******************************* Setup ********************************/
   // Set up the right drive Jaguar
   m_rightMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_rightMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_rightMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
   m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

   // Set up the left drive Jaguar
   m_leftMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_leftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_leftMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
   m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

   // Set up the Shooter Jaguar
   m_shooterWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_shooterWheel->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_shooterWheel->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   // Set up the Rotate Jaguar
   m_shooterRotate->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_shooterRotate->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_shooterRotate->ConfigEncoderCodesPerRev(1440);
   m_shooterRotate->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   // Reverse neither drive motor
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

   // Set the max speed for the robot
   m_robotDrive->SetMaxOutput(MAX_SPEED);

   // Setup the camera
   camera.WriteResolution(AxisCamera::kResolution_320x240);
   camera.WriteCompression(20);
   camera.WriteBrightness(50);

   // Set up the turn controller
   m_turnController->SetInputRange(-360.0, 360.0);
   m_turnController->SetOutputRange(-0.6, 0.6);
   m_turnController->SetTolerance(1.0 / 90.0 * 100);
   m_turnController->Disable();

   // Set the expiration for the watchdog
   m_robotDrive->SetExpiration(10.0);
   m_robotDrive->SetSafetyEnabled(false);
   GetWatchdog().SetExpiration(10.0);
   GetWatchdog().SetEnabled(false);
   /**************************************************************************/

   /******************************* Run **************************************/
   m_compressor->Start();
   /**************************************************************************/
}

/**
 * Drive left & right motors for 2 seconds then stop
 */
void MyRobot::Autonomous(void)
{
   // Enable the jaguars
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
   m_shooterRotate->EnableControl();
   m_shooterWheel->EnableControl();

   startTracking();
   
   while (IsAutonomous())
   {
      printf("Ready to Fire:%i\n", readyToFire);
   }
}

void MyRobot::shootTwoBalls()
{  
      bool readyToFire;
      
      do 
      {
        readyToFire = CameraControl();
   
      }while(readyToFire == false);
}

/**
 * Runs the motors with arcade steering.
 */
void MyRobot::OperatorControl(void)
{
   // Enable the jaguars
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
   m_shooterRotate->EnableControl();
   m_shooterWheel->EnableControl();

   //m_robotDrive->SetSafetyEnabled(true);
   while (IsOperatorControl())
   {
      // Read and process image and control shooter
      CameraControl();

      // Read the operator controls
      readOperatorControls();

      /************Debug Printouts **********************************************/
      printf("Height of the Triagle: %f\n", heightOfTriangle);
      printf("Voltage: %f\n\n", m_shooterWheel->Get());
      // printf("Right Motor: %f, Left Motor: %f\n", m_rightMotor->Get(),
      //         m_leftMotor->Get());
      // printf("Shooter Wheel: %f, Shooter Rotate: %f\n",
      //         m_shooterWheel->Get(), m_shooterRotate->Get());
      // printf("Target Status: %i\n", targetStatus);
      /**************************************************************************/

      // Send data to the dashboard
      sendDashboardData();

      // Wait for a motor update time
      Wait(0.005);
   }
}

/**
 * Read the operator controls and drive the robot accordinly.
 */
void MyRobot::readOperatorControls()
{
   static bool ferrisRotate        = false; // Is the ferris wheel rotating by itself?
   static bool previousSwitchValue = false; // The previous value of the ferris wheel switch
   
   /******************** Adjust the turn PID *********************************/
   static float buttonPressedPID = false;

   // Increment the value of P
   if ((m_buttonBox->GetRawButton(P_INCREMENT) == false) &&
       (buttonPressedPID == false))
   {
      buttonPressedPID = true;
      m_turnController->SetPID(m_turnController->GetP() * PID_CHANGE_VALUE,
                               m_turnController->GetI(), m_turnController->GetD());
   }

   // Decrement the value of P
   else if ((m_buttonBox->GetRawButton(P_DECREMENT) == false) &&
            (buttonPressedPID == false))
   {
      buttonPressedPID = true;
      m_turnController->SetPID(m_turnController->GetP() / PID_CHANGE_VALUE,
                        m_turnController->GetI(), m_turnController->GetD());
   }

   // Increment the value of I
   else if ((m_buttonBox->GetRawButton(I_INCREMENT) == true) &&
            (buttonPressedPID == true))
   {
      buttonPressedPID = false;
      m_turnController->SetPID(m_turnController->GetP(),
                               m_turnController ->GetI() * PID_CHANGE_VALUE,
                               m_turnController->GetD());
   }

   // Decrement the value of I
   else if ((m_buttonBox->GetRawButton(I_DECREMENT) == true) &&
            (buttonPressedPID == true))
   {
      buttonPressedPID = false;
      m_turnController->SetPID(m_turnController->GetP(),
                               m_turnController ->GetI() / PID_CHANGE_VALUE,
                               m_turnController->GetD());
   }

   // Increment the value of D
   else if ((m_buttonBox->GetRawButton(D_INCREMENT) == true) &&
            (buttonPressedPID == true))
   {
      buttonPressedPID = false;
      m_turnController->SetPID(m_turnController->GetP(),
                               m_turnController ->GetI(),
                               m_turnController->GetD() * PID_CHANGE_VALUE);
   }

   // Decrement the value of D
   else if ((m_buttonBox->GetRawButton(D_DECREMENT) == true) &&
            (buttonPressedPID == true))
   {
      buttonPressedPID = false;
      m_turnController->SetPID(m_turnController->GetP(),
                               m_turnController ->GetI(),
                               m_turnController->GetD() / PID_CHANGE_VALUE);
   }
   // Check to see if no buttons are pushed
   else if ((m_buttonBox->GetRawButton(P_INCREMENT) != false) &&
            (m_buttonBox->GetRawButton(P_DECREMENT) != false) &&
            (m_buttonBox->GetRawButton(I_INCREMENT) != true)  &&
            (m_buttonBox->GetRawButton(I_DECREMENT) != true)  &&
            (m_buttonBox->GetRawButton(D_INCREMENT) != true)  &&
            (m_buttonBox->GetRawButton(D_DECREMENT) != true))
   {
      buttonPressedPID = false;
   }

   // If the auto straight is on then run the rotation off of the gyro
   if (m_driverStation->GetDigitalIn(AUTO_STRAIGHT) == true)
   {
      if (m_turnController->IsEnabled() == false)
      {
         m_turnController->SetSetpoint(m_gyroHorizontal->GetAngle());
         m_turnController->Enable();
      }

      // allow the joystick to adjust the angle
      m_turnController->SetSetpoint(m_turnController->GetSetpoint() + m_joystick->GetThrottle());

      // drive the robot using Arcade drive
      m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis), m_rotation);
   }

   // Run the rotation off the joystick
   else
   {
      if (m_turnController->IsEnabled() == true)
      {
         m_turnController->Disable();
      }

      // drive the robot using Arcade drive
      m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
                                m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);
   }

   /************ RUN ShootingWheel *******************************************/
   if (m_driverStation->GetDigitalIn(MINIBOT_PORT) == 1)
   {
      m_shooterWheel->Set((m_joystick->GetTwist() + 1) * 0.5f);
   }
   /**************************************************************************/

   /************ BALL PICK_UP ************************************************/
   // if button 6 is pushed then stop the ball pickerupper
   if (m_joystick->GetRawButton(STOP_BALL_PICKUP) == true)
   {
      m_frontBallPickup->Set(0.0f);
      m_backBallPickup->Set(0.0f);
   }

   // if button 7 is pushed then run the ball pickerupper to suck balls
   else if (m_joystick->GetRawButton(START_BALL_PICKUP) == true)
   {
      m_frontBallPickup->Set(-1.0f);
      m_backBallPickup->Set(-1.0f);
   }

   // if button 8 is pushed then run the ball pickerupper to spit balls
   else if (m_joystick->GetRawButton(REVERSE_BALL_PICKUP) == true)
   {
      m_frontBallPickup->Set(1.0f);
      m_backBallPickup->Set(1.0f);
   }
   /**************************************************************************/

   /************ Ferris Wheel ************************************************/
   // Check limit switch and only stop the wheel if the switch is newly pressed
   if (m_ferrisWheelStop->Get() == true && previousSwitchValue == false)
   {
      ferrisRotate = false;
      previousSwitchValue = true;
   }

   // check to see if the switch is released
   else if (m_ferrisWheelStop->Get() == false)
   {
      previousSwitchValue = false;
   }

   // If button 9 is pushed, the ferris wheel moves forward
   if (m_joystick->GetRawButton(START_FERRIS_WHEEL) == true)
   {
      m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
      ferrisRotate = true;
   }

   // If button 10 is pushed, the ferris wheel will stop and drive backwards
   else if (m_joystick->GetRawButton(REVERSE_FERRIS_WHEEL) == true)
   {
      m_ferrisWheel->Set(-FERRIS_ROTATE_SPEED);
      ferrisRotate = false;
   }
   else if (ferrisRotate == true)
   {
       m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
   }
   else
   {
       m_ferrisWheel->Set(0.0f);
   }
   /**************************************************************************/

   /************ RUN Shooter Turn ********************************************/
   if (m_driverStation->GetDigitalIn(MINIBOT_PORT) == 1)
   {
      if (m_joystick->GetRawButton(ROTATE_SHOOTER_COUNTER_CLOCK) == true)
         m_shooterRotate->Set(-SHOOTER_ROTATE_SPEED);
      else if (m_joystick->GetRawButton(ROTATE_SHOOTER_CLOCKWISE) == true)
         m_shooterRotate->Set(SHOOTER_ROTATE_SPEED);
      else
         m_shooterRotate->Set(0.0f);
   }
   /**************************************************************************/

   /************ RUN THE SOLENOIDS *******************************************/
   if (m_joystick->GetRawButton(BALL_LIFT_SHOOT) == true)
   {
      m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
   }
   else
   {
      m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
   }
   /**************************************************************************/

   // if (m_joystick->GetRawButton(DRIVE_WITH_VELOCITY) == true)
   // {
   //    m_rightMotor->ChangeControlMode(CANJaguar::kSpeed);
   //    m_leftMotor->ChangeControlMode(CANJaguar::kSpeed);
   //    m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   //    m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   //    m_rightMotor->SetPID(0.01, 0.01, 0.0);
   //    m_leftMotor->SetPID(0.01, 0.01, 0.0);
   //    m_robotDrive->SetMaxOutput(300);
   //    m_rightMotor->EnableControl();
   //    m_leftMotor->EnableControl();
   // }

   if (m_joystick->GetRawButton(DRIVE_WITH_VOLTAGE) == true)
   {
      m_rightMotor->ChangeControlMode(CANJaguar::kPercentVbus);
      m_leftMotor->ChangeControlMode(CANJaguar::kPercentVbus);
      m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
      m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
      m_robotDrive->SetMaxOutput(MAX_SPEED);
      m_rightMotor->EnableControl();
      m_leftMotor->EnableControl();
   }

   // if (m_rightMotor->GetFaults() != 0)
   //    printf("Right Fault: %i", m_rightMotor->GetFaults());
   //
   // if (m_leftMotor->GetFaults() != 0)
   //    printf("Left Fault: %i", m_leftMotor->GetFaults());
}

/**
 * Routine sends data to the dashboard.
 */
void MyRobot::sendDashboardData()
{
   // send IO data to the DriverStation
   m_dashboardDataFormat->SendLCDData(heightOfTriangle,
           m_rightMotor->GetSpeed(), m_leftMotor->GetSpeed(),
           m_rightMotor->Get(), m_leftMotor->Get(), m_turnController->GetP(),
           m_turnController->GetI(), m_turnController->GetD(),
           m_shooterWheel->Get());
   m_dashboardDataFormat->SendIOPortData();
   m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS(MyRobot);


