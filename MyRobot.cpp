#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR             1.0

#define BALANCE_ON_BRIDGE_SPEED          0.10
#define BALANCE_DISTANCE_DELTA          (1.0/12.0)

MyRobot::MyRobot(void)
{
   m_dashboardDataFormat     = new DashboardDataFormat();
   m_rightMotor              = new HVA_Victor(RIGHT_DRIVE_MOTOR,
                               new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B), HVA_Victor::kPercentVbus);
   m_leftMotor               = new HVA_Victor(LEFT_DRIVE_MOTOR,
                               new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B), HVA_Victor::kPercentVbus);
   m_shooterWheel            = new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kVoltage);
   m_shooterRotate           = new CANJaguar(SHOOTER_ROTATE_MOTOR, CANJaguar::kPosition);
   m_ferrisWheel             = new Victor(FERRIS_WHEEL_MOTOR);
   m_frontBallPickup         = new Victor(FRONT_BALL_PICKUP_MOTOR);
   m_backBallPickup          = new Victor(BACK_BALL_PICKUP_MOTOR);
   m_frontUltra              = new AnalogSonar(ANALOG_ULTRASONIC);
   m_backUltra               = new Ultrasonic(ULTRASONIC_PING_OUTPUT, ULTRASONIC_ECHO_INPUT);
   m_robotDrive              = new HVA_RobotDrive(m_leftMotor, m_rightMotor);
   m_compressor              = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
   m_ballLiftSolenoid        = new DoubleSolenoid(BALL_LIFT_FORWARD, BALL_LIFT_REVERSED);
   m_rampExtender            = new DoubleSolenoid(RAMP_EXTENDER_FORWARD, RAMP_EXTENDER_REVERSE);
   m_rampPusher              = new DoubleSolenoid(RAMP_PUSHER_FORWARD, RAMP_PUSHER_REVERSE);
   m_joystick                = new Joystick(JOYSTICK_PORT);
   m_buttonBox               = new Joystick(BUTTON_BOX_PORT);
   //m_buttonBox2              = new Joystick(BUTTON_BOX_PORT2);
   m_bottomBallSensor        = new AnalogTrigger(BOTTOM_BALL_SENSOR);
   m_ferrisWheelStop         = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
   m_gyroHorizontal          = new Gyro(GYRO_HORIZONTAL_PORT);
   m_gyroVertical            = new Gyro(GYRO_VERTICAL_PORT);
   m_driverStation           = DriverStation::GetInstance();
   m_driverStationEnhancedIO = &DriverStation::GetInstance()->GetEnhancedIO();
   AxisCamera &camera        = AxisCamera::GetInstance();

   /******************************* Setup ********************************/
   // Set the State Enums for the robot
   m_rampState    = kPusherRetracted;
   m_ferrisState  = kStop;
   m_shooterRotationControlState = kTeleoperated;

   // setup the ferris wheel location switch interrupt
   m_ferrisWheelStop->RequestInterrupts(MyRobot::ferrisHandler, this);

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
      // Read and process image and control shooter
      cameraControl();

      // Read the operator controls
      readOperatorControls();

      /************Debug Printouts **********************************************/
      printf("Distance: %f\n", m_rightMotor->GetPosition());
//      printf("Position: %f\n", m_shooterRotate->GetPosition());
//      printf("Front Sonar: %f\n", m_frontUltra->GetRangeInches());
//      printf("Back Sonar: %f\n", m_backUltra->GetRangeInches());
//      printf("Right Postition:%f\n", m_rightMotor->GetPosition());
//      printf("Left Postition:%f\n", m_leftMotor->GetPosition());
//      printf("Botom Ball Sensor:%i\n", !m_bottomBallSensor->GetTriggerState());
//      printf("Shooter Velocity: %f\n\n", m_shooterWheel->GetSpeed());
      //      printf("FerrisVolt:%f\n", m_ferrisWheel->Get());
      //      printf("Range:%f\n", m_ultrasonic->GetRangeInches());
      //      printf("Current right:%f\n", m_rightMotor->GetOutputCurrent());
      //      printf("Current left:%f\n\n", m_leftMotor->GetOutputCurrent());
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
void MyRobot::readOperatorControls()
{
   static bool drivingToRamp = false;
   static bool previousTriggerState = m_joystick->GetTrigger();
   static bool previousIncrementButtonState = m_joystick->GetRawButton(INCREMENT_POSITION);
   static bool previousDecrementButtonState = m_joystick->GetRawButton(DECREMENT_POSITION);
   static float distanceToDrive;
   static HVA_Victor::ControlMode previousDriveSetting = m_rightMotor->GetControlMode();

   float balanceDistanceDelta;

   /************************ Control the Drive Mode ***************************/
   if (m_joystick->GetRawButton(DRIVE_WITH_VELOCITY) == true)
   {
      setDriveModeToVelocity();
   }

   else if (m_joystick->GetRawButton(DRIVE_WITH_VOLTAGE) == true)
   {
      setDriveModeToVoltagePercent();
   }

   /************************ Drive the Robot **********************************/
   // if the trigger is pulled then drive up the ramp and balance (while the trigger is pulled)
   if (m_joystick->GetTrigger() == true)
   {
      // determine if the trigger has just be pulled
      if ((previousTriggerState != m_joystick->GetTrigger()) &&
          (m_joystick->GetTrigger() == true))
      {
         // ensure the drive is in velocity mode
         if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
         {
            // remember the present drive setting to allow setting it back
            // when the trigger is released
            previousDriveSetting = m_rightMotor->GetControlMode();

            // set the drive to velocity
            setDriveModeToVelocity();
         }
      }

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

      // perform the balance on bridge algorithm
      m_robotDrive->BalanceOnBridge(BALANCE_ON_BRIDGE_SPEED, m_gyroVertical, balanceDistanceDelta);
   }

   // if the joysitck top button is pushed then drive to the ramp (while the button is pushed)
   else if (m_joystick->GetTop() == true)
      {
      // determine if the trigger has just be pulled
      if ((previousTriggerState != m_joystick->GetTrigger()) &&
          (m_joystick->GetTrigger() == true))
      {
         // ensure the drive is in velocity mode
         if (m_rightMotor->GetControlMode() != HVA_Victor::kVelocity)
         {
            // remember the present drive setting to allow setting it back
            // when the trigger is released
            previousDriveSetting = m_rightMotor->GetControlMode();

            // set the drive to velocity
            setDriveModeToVelocity();
         }

         // determine the distance to the ramp
         distanceToDrive = -((m_backUltra->GetRangeInches() - DISTANCE_TO_RAMP) / 12);
         drivingToRamp = true;
      }

      // determine if the robot is moving (not arrived at the destination)
      // Note: Must keep calling the drive routine to keep the robot moving, but
      //       do not call the routing when the robot has reached the destination
      if (drivingToRamp == true)
      {
         // determine if the robot has reached the ramp
         if (m_robotDrive->DriveDistance(0.2, distanceToDrive, MAX_ACCELERATION_DISTANCE))
         {

            // no longer driving to the ramp
            drivingToRamp = false;
         }
      }
   }

   // Drive the robot using Arcade drive
   else
   {
      // determine if the drive mode should be return to the initial setting
      if (m_rightMotor->GetControlMode() != previousDriveSetting)
      {
      // Note: only allow voltage and velocity modes (default to voltage)
      switch(previousDriveSetting)
         {
         case HVA_Victor::kVelocity:
            setDriveModeToVelocity();
            break;

         default:
            setDriveModeToVoltagePercent();
         }
      }

      drivingToRamp = false;

      if ((m_rightMotor->GetControlMode() == HVA_Victor::kPercentVbus) ||
          (m_rightMotor->GetControlMode() == HVA_Victor::kVelocity))
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
   previousTriggerState         = m_joystick->GetTrigger();
   previousIncrementButtonState = m_joystick->GetRawButton(INCREMENT_POSITION);
   previousDecrementButtonState = m_joystick->GetRawButton(DECREMENT_POSITION);

   /************************ Check Rotation Override Switch *******************/
   // Check the rotation override switches, Check to see if operator control
   if (((m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == true)
         || (m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER)
               == false)) && (IsOperatorControl() == true))
   {
      m_shooterRotationControlState = kTeleoperated;
   }
   else
   {
      m_shooterRotationControlState = kAutonomous;
   }

   /************************ RUN Bridge Pusher ********************************/
   if (m_driverStationEnhancedIO->GetDigital(STABILITY_SWITCH_BACKWARD_OVERRIDE)
         == true)
   {
      if (m_rampExtender->Get() != DoubleSolenoid::kForward)
      {
         m_robotDrive->InitializeBalanceOnBridge(m_gyroVertical);
         m_rampExtender->Set(DoubleSolenoid::kForward);
      }
   }
   else
   {
      if (m_rampExtender->Get() != DoubleSolenoid::kReverse)
         m_rampExtender->Set(DoubleSolenoid::kReverse);
   }

   if ((m_driverStationEnhancedIO->GetDigital(STABILITY_SWITCH_FORWARD_OVERRIDE)
         == true) && (m_driverStationEnhancedIO->GetDigital(
         STABILITY_SWITCH_BACKWARD_OVERRIDE) == true))
   {
      if (m_rampPusher->Get() != DoubleSolenoid::kForward)
         m_rampPusher->Set(DoubleSolenoid::kForward);
   }
   else
   {
      if (m_rampPusher->Get() != DoubleSolenoid::kReverse)
         m_rampPusher->Set(DoubleSolenoid::kReverse);
   }

   /************************ RUN ShootingWheel ********************************/
   // drive the shooter manually if the cammera tracking is disabled or the full override is called
   if (((m_driverStationEnhancedIO->GetDigital(SHOOTER_WHEEL_OVERRIDE_MODE)
         == true) && (m_driverStationEnhancedIO->GetDigital(
         SHOOTER_WHEEL_OVERRIDE) == true)))
   {
      m_shooterWheel->Set(
            (((m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_VELOCITY_POT)
                  / MAX_POT_VALUE) * (MAX_SHOOTER_SPEED_PERCENT
                  - MIN_SHOOTER_SPEED_PERCENT)) + MIN_SHOOTER_SPEED_PERCENT)
                  * MAX_ROBOT_VOLTAGE);
   }
   else if ((m_driverStationEnhancedIO->GetDigital(AUTONOMOUSLY_RUN_SHOOTER)
         == false))
   {
      m_shooterWheel->Set(0.0f);
   }

   /************************ BALL PICK_UP *************************************/
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

   /************************ Ferris Wheel *************************************/
   runFerrisWheelFromControls();

   /************************ RUN Shooter Turn *********************************/
   // If the rotation is being driven by the operator then set the value equall to the pot
   if (m_shooterRotationControlState == kTeleoperated
         && m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == true)
   {
      m_shooterRotate->Set(
            (m_driverStationEnhancedIO->GetAnalogIn(SHOOTER_ROTATION_POT)
                  - MID_POT_VALUE) * ((MAXIMUM_ROTATION_OF_SHOOTER)
                  / (MAX_POT_VALUE - MID_POT_VALUE)));
   }
   else if (m_shooterRotationControlState == kTeleoperated
         && m_driverStationEnhancedIO->GetDigital(ROTATION_OVERRIDE) == false)
   {
      m_shooterRotate->Set(0.0f);
   }

   /************************ RUN Ball Shooter ********************************/
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

   if ((state != kStop) && ((Timer::GetFPGATimestamp() - time) >= TIME_TILL_INTERRUPT_ENABLE))
   {
      m_ferrisWheelStop->EnableInterrupts();
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

/**
 * Set the drive mode to run off velocity
 *    Motor Values: -1 to 1
 */
void MyRobot::setDriveModeToVelocity(void)
{
   m_rightMotor->ChangeControlMode(HVA_Victor::kVelocity);
   m_leftMotor->ChangeControlMode(HVA_Victor::kVelocity);

   m_robotDrive->SetMaxOutput(MAX_VELOCITY);

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
 * Routine sends data to the dashboard.
 */
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
         m_backUltra->GetRangeInches(),
         m_gyroHorizontal->GetAngle(),
         -m_gyroVertical->GetAngle());
   m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS(MyRobot)
;
