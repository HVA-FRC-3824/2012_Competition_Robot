#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

MyRobot::MyRobot(void)
{
   m_dashboardDataFormat = new DashboardDataFormat();
   m_rightMotor = new HVA_Victor(RIGHT_DRIVE_MOTOR,
         new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B),
         HVA_Victor::kPercentVbus);
   m_leftMotor = new HVA_Victor(LEFT_DRIVE_MOTOR,
         new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B), HVA_Victor::kPercentVbus);
   m_shooterWheel = new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kVoltage);
   m_shooterRotate = new CANJaguar(SHOOTER_ROTATE_MOTOR, CANJaguar::kPosition);
   m_ferrisWheel = new Victor(FERRIS_WHEEL_MOTOR);
   m_frontBallPickup = new Victor(FRONT_BALL_PICKUP_MOTOR);
   m_backBallPickup = new Victor(BACK_BALL_PICKUP_MOTOR);
   m_frontUltra = new AnalogSonar(ANALOG_ULTRASONIC);
   m_backUltra = new Ultrasonic(ULTRASONIC_PING_OUTPUT, ULTRASONIC_ECHO_INPUT);
   m_robotDrive = new HVA_RobotDrive(m_leftMotor, m_rightMotor);
   m_compressor = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
   m_ballLiftSolenoid = new DoubleSolenoid(BALL_LIFT_FORWARD,
         BALL_LIFT_REVERSED);
   m_rampExtender = new DoubleSolenoid(RAMP_EXTENDER_FORWARD,
         RAMP_EXTENDER_REVERSE);
   m_rampPusher = new DoubleSolenoid(RAMP_PUSHER_FORWARD, RAMP_PUSHER_REVERSE);
   m_joystick = new Joystick(JOYSTICK_PORT);
   m_buttonBox = new Joystick(BUTTON_BOX_PORT);
   //m_buttonBox2              = new Joystick(BUTTON_BOX_PORT2);
   m_bottomBallSensor = new AnalogTrigger(BOTTOM_BALL_SENSOR);
   m_ferrisWheelStop = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
   m_gyroHorizontal = new Gyro(GYRO_HORIZONTAL_PORT);
   m_gyroVertical = new Gyro(GYRO_VERTICAL_PORT);
   m_driverStation = DriverStation::GetInstance();
   m_driverStationEnhancedIO = &DriverStation::GetInstance()->GetEnhancedIO();
   AxisCamera &camera = AxisCamera::GetInstance();
   m_ferrisInterruptHandler = new Task("FerrisInterruptHandler", (FUNCPTR)ferrisHandler, 0);

   /******************************* Setup ********************************/
   // Set the State Enums for the robot
   m_ferrisState = kStop;
   m_runningAutonomous = false;

   // Set up the bottom analog trigger
   m_bottomBallSensor->SetLimitsVoltage(4, 8);

   // Set up the Shooter Jaguar
   m_shooterWheel->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
   m_shooterWheel->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
   m_shooterWheel->ConfigEncoderCodesPerRev(6);
   m_shooterWheel->ConfigFaultTime(0.5);

   // Set up the Rotate Jaguar
   m_shooterRotate->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_shooterRotate->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_shooterRotate->ConfigEncoderCodesPerRev(1440);
   m_shooterRotate->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_shooterRotate->SetPID(ROTATE_CONTROLLER_P, ROTATE_CONTROLLER_I,
         ROTATE_CONTROLLER_D);
   m_shooterRotate->ConfigFaultTime(0.5);

   // Reverse the correct drive motor
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

   // Set the max speed for the robot
   m_robotDrive->SetMaxOutput(MAX_VOLTAGE_PERCENT);

   // Setup the camera
   camera.WriteResolution(AxisCamera::kResolution_320x240);
   camera.WriteCompression(20);
   camera.WriteBrightness(50);

   // Set the expiration for the watchdog
   m_robotDrive->SetExpiration(10.0);
   m_robotDrive->SetSafetyEnabled(false);
   /**************************************************************************/

   /******************************* Run **************************************/
   m_compressor->Start();
   m_backUltra->SetAutomaticMode(true);
   m_backUltra->SetEnabled(true);
   /**************************************************************************/
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
      // control the ball shooter
      runShooterControl();

      // run the drive system
      runDriveControl();

      // control the ball pickup
      runBallPickup();

      // control the bridge control
      runBridgeDeployment();

      // control the ferris wheel
      runFerrisWheelFromControls();

      // control the ball shooter
      runBallShooter();

      /************Debug Printouts **********************************************/
      //printf("Distance: %f\n", m_rightMotor->GetPosition());
      //printf("Position: %f\n", m_shooterRotate->GetPosition());
      //printf("Front Sonar: %f\n", m_frontUltra->GetRangeInches());
      //printf("Back Sonar: %f\n", m_backUltra->GetRangeInches());
      //printf("Right Postition:%f\n", m_rightMotor->GetPosition());
      //printf("Left Postition:%f\n", m_leftMotor->GetPosition());
      //printf("Botom Ball Sensor:%i\n", !m_bottomBallSensor->GetTriggerState());
      //printf("Shooter Velocity: %f\n\n", m_shooterWheel->GetSpeed());
      //printf("FerrisVolt:%f\n", m_ferrisWheel->Get());
      //printf("Range:%f\n", m_ultrasonic->GetRangeInches());
      //printf("Current right:%f\n", m_rightMotor->GetOutputCurrent());
      //printf("Current left:%f\n\n", m_leftMotor->GetOutputCurrent());
      //printf("ShooterVoltage: %f\n",  ((((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT)/MAX_POT_VALUE)*(MAX_SHOOTER_SPEED_PERCENT - MIN_SHOOTER_SPEED_PERCENT)) + MIN_SHOOTER_SPEED_PERCENT) * MAX_ROBOT_VOLTAGE));
      //printf("Position: %f\n", ((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_ROTATION_POT) - MID_POT_VALUE) * ((MAXIMUM_ROTATION_OF_SHOOTER)/(MAX_POT_VALUE-MID_POT_VALUE))));
      //printf("WheelOffSet: %f\n", ((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT) - MID_POT_VALUE)*((MAX_SHOOTER_VOLTAGE_ADJUSTMENT)/(MAX_POT_VALUE - MID_POT_VALUE))));

      //printf("Height of the Triagle: %f\n", heightOfTriangle);
      //printf("Voltage: %f\n", m_shooterWheel->Get());
      //printf("Motor Value: %f\n", m_rightMotor->Get());
      //printf("Motor Velocity: %f\n", m_rightMotor->GetSpeed());
      //printf("Right Position: %f\n", m_rightMotor->GetPosition());
      //printf("Left Position: %f\n", m_leftMotor->GetPosition());
      //printf("Joystick: %f\n", m_joystick->GetY());
      //printf("Vert Gyro: %f\n", m_gyroVertical->GetAngle());
      //printf("FerrisLimit: %i\n", m_ferrisWheelStop->Get());
      //printf("Stability State: %i\n\n", stabilityWheelState);
      // printf("Right Motor: %f, Left Motor: %f\n", m_rightMotor->Get(),
      //         m_leftMotor->Get());
      // printf("Shooter Wheel: %f, Shooter Rotate: %f\n",
      //         m_shooterWheel->Get(), m_shooterRotate->Get());
      // printf("Target Status: %i\n", targetStatus);
      //printf("IO: %i\n\n", m_ferrisWheelStop->Get());
      //printf("FerrisWheelState: %i\n", m_ferrisState);
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
void MyRobot::runDriveControl()
{
   static bool drivingToRamp = false;
   static bool previousTriggerState = m_joystick->GetTrigger();
   static float distanceToDrive;

   /************************ Control the Drive Mode ***************************/
   if (m_joystick->GetRawButton(DRIVE_WITH_VELOCITY) == true)
   {
      // use the right motor to get the drive motor state
      if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
      {
         setDriveModeToVelocity();
      }
   }
   else if (m_joystick->GetRawButton(DRIVE_WITH_VOLTAGE) == true)
   {
      // use the right motor to get the drive motor state
      if (m_rightMotor->GetControlMode() != HVA_Victor::kPercentVbus)
      {
         setDriveModeToVoltagePercent();
      }
   }

   /************************ Drive the Robot **********************************/
   // if the trigger is pulled then drive up the ramp and balance (while the trigger is pulled)
   if (m_joystick->GetTrigger() == true)
   {
      printf("Previous triggerstate:%i\n", previousTriggerState);

      // bridge balancing mode
      runBridgeBalancingMode(previousTriggerState);

      previousTriggerState = true;
   }

   // if the joystick top button is pushed then drive to the ramp (while the button is pushed)
   else if (m_joystick->GetTop() == true)
   {
      // determine if the joystick button is pressed
      if ((previousTriggerState != m_joystick->GetTrigger())
            && (m_joystick->GetTrigger() == true))
      {
         // ensure the drive is in velocity mode
         if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
         {
            // set the drive to velocity
            setDriveModeToVelocity();
         }

         // determine the distance to the ramp
         distanceToDrive = -((m_backUltra->GetRangeInches() - DISTANCE_TO_RAMP)
               / 12);
         drivingToRamp = true;
      }

      // determine if the robot is moving (not arrived at the destination)
      // Note: Must keep calling the drive routine to keep the robot moving, but
      //       do not call the routing when the robot has reached the destination
      if (drivingToRamp == true)
      {
         // determine if the robot has reached the ramp
         if (m_robotDrive->DriveDistance(0.2, distanceToDrive,
               MAX_ACCELERATION_DISTANCE))
         {
            // no longer driving to the ramp
            drivingToRamp = false;
         }
      }
   }

   // Drive the robot using Arcade drive
   else
   {
      // inidcate that the robot is not driving to the ramp
      drivingToRamp = false;

      // determine if the trigger is released
      if ((previousTriggerState == true) && (m_joystick->GetTrigger() == false))
      {
         // not running autonomous
         m_runningAutonomous = false;

         // ensure the drive mode is back to voltage
         if (m_rightMotor->GetControlMode() != HVA_Victor::kPercentVbus)
            setDriveModeToVoltagePercent();
      }

      // drive the robot in voltage or velocity mode
      if ((m_rightMotor->GetControlMode() == HVA_Victor::kPercentVbus)
            || (m_rightMotor->GetControlMode() == HVA_Victor::kVelocity))
      {
         m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
               m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);
      }
      else if (m_rightMotor->GetControlMode() == HVA_Victor::kVelocity)
      {
         //      m_robotDrive->ArcadeVelocityDriveStepped(
         //            m_joystick->GetAxis(Joystick::kYAxis),
         //            (float) (m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR),
         //            (float) MAX_ACCELERATION_ARCADE, 2.0f);
      }
   }

   // remember the user input control states
   if (m_joystick->GetTrigger() == false)
      previousTriggerState = false;
}

/**
 * Run the ball pickup
 */
void MyRobot::runBallPickup(void)
{
   // if switch 7 is flipped then run the ball pickerupper to suck balls
   if (m_driverStationEnhancedIO->GetDigital(START_BALL_PICKUP) == true)
   {
      m_frontBallPickup->Set(-BALL_PICKUP_SPEED);
      m_backBallPickup->Set(-BALL_PICKUP_SPEED);
   }

   // if switch 8 is flipped then run the ball pickerupper to spit balls
   else if (m_driverStationEnhancedIO->GetDigital(REVERSE_BALL_PICKUP) == true)
   {
      m_frontBallPickup->Set(BALL_PICKUP_SPEED);
      m_backBallPickup->Set(BALL_PICKUP_SPEED);
   }

   // if the switch is not flipped then do not run the ball pickup
   else
   {
      m_frontBallPickup->Set(0.0f);
      m_backBallPickup->Set(0.0f);
   }
}

/**
 * Run the ferrisWheel off of the robot state
 */
void MyRobot::runFerrisWheel(FerrisState state)
{
   static FerrisState previousState = kStop;
   static double time = Timer::GetFPGATimestamp();

   m_ferrisState = state;

   if ((state != previousState) && (state != kStop))
   {
      time = Timer::GetFPGATimestamp();
   }

   if ((state != kStop) && ((Timer::GetFPGATimestamp() - time)
         >= TIME_TILL_INTERRUPT_ENABLE))
   {
      printf("Stated Task");
      m_ferrisWheelStop->EnableInterrupts();
      //m_ferrisWheelStop->RequestInterrupts();
      m_ferrisInterruptHandler->Start((UINT32)this);
   }

   switch (state)
   {
   case kForward:
      m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
      break;

   case kBackward:
      m_ferrisWheel->Set(-FERRIS_ROTATE_SPEED);
      break;

   case kStop:
      m_ferrisWheel->Set(0.0f);
      break;
   }
}

/**
 * Determine the state the ferris wheel should be in.
 */
void MyRobot::runFerrisWheelFromControls(void)
{
   static bool previousSwitchValue = false; // The previous value of the ferris wheel switch

   FerrisState ferrisStateToSet = m_ferrisState;

   if (m_ferrisWheelStop->Get() == true && previousSwitchValue == false)
   {
      ferrisStateToSet = kStop;
      previousSwitchValue = true;
   }

   // check to see if the switch is released
   else if (m_ferrisWheelStop->Get() == false)
   {
      previousSwitchValue = false;
   }

   // Stop the wheel is both buttons are pressed
   if ((m_driverStationEnhancedIO->GetDigital(START_FERRIS_WHEEL) == true)
         && (m_driverStationEnhancedIO->GetDigital(REVERSE_FERRIS_WHEEL)
               == true))
   {
      ferrisStateToSet = kStop;
   }

   // If button 9 is pushed, the ferris wheel moves forward
   else if (m_driverStationEnhancedIO->GetDigital(START_FERRIS_WHEEL) == true)
   {
      ferrisStateToSet = kForward;
   }

   // If button 10 is pushed, the ferris wheel will stop and drive backwards
   else if (m_driverStationEnhancedIO->GetDigital(REVERSE_FERRIS_WHEEL) == true)
   {
      ferrisStateToSet = kBackward;
   }

   runFerrisWheel(ferrisStateToSet);
}

void MyRobot::runBallShooter(void)
{
   if (m_driverStationEnhancedIO->GetDigital(BALL_LIFT_SHOOT) == true)
   {
      m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
   }
   else
   {
      m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
   }
}

/**
 * Set the drive mode to run off velocity
 *    Motor Values: -1 to 1
 */
void MyRobot::setDriveModeToVelocity(double maxVelocity)
{
   m_rightMotor->ChangeControlMode(HVA_Victor::kVelocity);
   m_leftMotor->ChangeControlMode(HVA_Victor::kVelocity);

   m_robotDrive->SetMaxOutput(maxVelocity);

   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
}

/**
 * Set the drive mode to run off voltage percentage
 *    Motor Values:
 */
void MyRobot::setDriveModeToVoltagePercent()
{
   m_rightMotor->ChangeControlMode(HVA_Victor::kPercentVbus);
   m_leftMotor->ChangeControlMode(HVA_Victor::kPercentVbus);

   m_robotDrive->SetMaxOutput(MAX_VOLTAGE_PERCENT);

   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
}

/**
 * Set the drive mode to run off position PID
 */
void MyRobot::setDriveModeToPosition()
{
   m_rightMotor->ChangeControlMode(HVA_Victor::kPosition);
   m_leftMotor->ChangeControlMode(HVA_Victor::kPosition);

   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
}

/**
 * Set the shooter mode to run off voltage
 *    Motor Values:
 */
void MyRobot::setShooterModeToVoltage(void)
{
   m_shooterWheel->ChangeControlMode(CANJaguar::kVoltage);
   m_shooterWheel->EnableControl();
}

/**
 * Set the shooter mode to run off RPM
 */
void MyRobot::setShooterModeToSpeed(void)
{
   m_shooterWheel->ChangeControlMode(CANJaguar::kSpeed);
   m_shooterWheel->SetPID(SHOOTER_VELOCITY_P, SHOOTER_VELOCITY_I,
         SHOOTER_VELOCITY_D);
   m_shooterWheel->EnableControl();
}

void MyRobot::sendDashboardData()
{
   // send IO data to the DriverStation
   m_dashboardDataFormat->SendLCDData(
         heightOfTriangle,
         m_rightMotor->GetSpeed(),
         m_leftMotor->GetSpeed(),
         m_rightMotor->Get(),
         m_leftMotor->Get(),
         m_shooterWheel->Get(),
         m_shooterWheel->GetSpeed(),
         ((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT)
               - MID_POT_VALUE) * ((MAX_SHOOTER_VOLTAGE_ADJUSTMENT)
               / (MAX_POT_VALUE - MID_POT_VALUE))));
   m_dashboardDataFormat->SendIOPortData(
         !m_bottomBallSensor->GetTriggerState(),
         m_shooterRotate->GetPosition(), m_frontUltra->GetRangeInches(),
         m_backUltra->GetRangeInches(), m_gyroHorizontal->GetAngle(),
         -m_gyroVertical->GetAngle(), m_shooterWheel->GetOutputVoltage());
   m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS(MyRobot)
;
