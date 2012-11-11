#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR             1.0
#define NUMBER_OF_ENCODER_LINES         1440
#define NUMBER_OF_ENCODER_LINES_MOTORS   720
#define PID_CHANGE_VALUE                   2
#define TIPPED_THRESHOLD                   2

MyRobot::MyRobot(void)
{
   m_dashboardDataFormat = new DashboardDataFormat();
   m_rightMotor = new CANJaguar(RIGHT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
   m_leftMotor = new CANJaguar(LEFT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
   m_shooterWheel = new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kPercentVbus);
   m_shooterRotate = new CANJaguar(SHOOTER_ROTATE_MOTOR,
         CANJaguar::kPercentVbus);
   m_ferrisWheel = new Victor(FERRIS_WHEEL_MOTOR);
   m_frontBallPickup = new Victor(FRONT_BALL_PICKUP_MOTOR);
   m_backBallPickup = new Victor(BACK_BALL_PICKUP_MOTOR);
   m_robotDrive = new HVA_RobotDrive(m_leftMotor, m_rightMotor); // create a robot drive system with four moters
   m_compressor = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
   m_ballLiftSolenoid = new DoubleSolenoid(BALL_LIFT_FORWARD,
         BALL_LIFT_REVERSED);
   m_frontBridgeWheel = new DoubleSolenoid(FRONT_BRIDGE_WHEEL_FORWARD,
         FRONT_BRIDGE_WHEEL_REVERSE);
   m_backBridgeWheel = new DoubleSolenoid(BACK_BRIDGE_WHEEL_FORWARD,
         BACK_BRIDGE_WHEEL_REVERSE);
   m_joystick = new Joystick(JOYSTICK_PORT); // as they are declared above.
   m_buttonBox = new Joystick(BUTTON_BOX_PORT);
   m_ferrisWheelStop = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
   m_bottomSlot = new DigitalInput(BOTTOM_SLOT_DETECTOR_PORT);
   m_frontBridgeWheelLimit = new DigitalInput(FRONT_BRIDGE_WHEEL_LIMIT);
   m_backBridgeWheelLimit = new DigitalInput(BACK_BRIDGE_WHEEL_LIMIT);
   m_gyroHorizontal = new Gyro(GYRO_HORIZONTAL_PORT);
   m_gyroVertical = new Gyro(GYRO_VERTICAL_PORT);
   m_driverStation = DriverStation::GetInstance();
   AxisCamera &camera = AxisCamera::GetInstance();

   // reset the gyro PID and start with zero rotation
   m_rotation = 0.0f;

   // PID controller for the gyro angles
   m_pidOutput = new HVA_PIDOutput(&m_rotation);
   m_turnController = new HVA_PIDController(TURN_CONTROLLER_P, // P
         TURN_CONTROLLER_I, // I
         TURN_CONTROLLER_D, // D
         m_gyroHorizontal, // source
         m_pidOutput, // output
         TURN_CONTROLLER_PERIOD); // period

   m_rightJaguarSource = new HVA_PIDJaguarVelocity(m_rightMotor);
   m_leftJaguarSource = new HVA_PIDJaguarVelocity(m_leftMotor);

   m_currentVelocityRight = new PIDController(MOTOR_VELOCTIY_TO_CURRENT_P,
         MOTOR_VELOCTIY_TO_CURRENT_I, MOTOR_VELOCTIY_TO_CURRENT_D,
         m_rightJaguarSource, m_rightMotor);

   m_currentVelocityLeft = new PIDController(MOTOR_VELOCTIY_TO_CURRENT_P,
         MOTOR_VELOCTIY_TO_CURRENT_I, MOTOR_VELOCTIY_TO_CURRENT_D,
         m_rightJaguarSource, m_leftMotor);

   m_rightMotorVelocityPID = new CurrentVelocityController(
         m_currentVelocityRight);
   m_leftMotorVelocityPID
         = new CurrentVelocityController(m_currentVelocityLeft);

   m_robotDriveVelocityPID = new HVA_RobotDrive(m_leftMotorVelocityPID,
         m_rightMotorVelocityPID);

   /******************************* Setup ********************************/
   // Set the State Enums for the robot
   driveSetting = kPercentage;
   stabilityWheelState = kNeitherDeployed;
   ferrisState = kStop;

   // Set up the right drive Jaguar
   m_rightMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_rightMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_rightMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
   m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
   m_rightMotor->ConfigFaultTime(.5);

   // Set up the left drive Jaguar
   m_leftMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_leftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_leftMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
   m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
   m_leftMotor->ConfigFaultTime(.5);

   // Set up the Shooter Jaguar
   m_shooterWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_shooterWheel->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_shooterWheel->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_shooterWheel->ConfigFaultTime(.5);

   // Set up the Rotate Jaguar
   m_shooterRotate->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
   m_shooterRotate->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
   m_shooterRotate->ConfigEncoderCodesPerRev(1440);
   m_shooterRotate->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_shooterRotate->ConfigFaultTime(.5);

   // Reverse the correct drive motor
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
   m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

   // Reverse the correct drive motor
   m_robotDriveVelocityPID->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
   m_robotDriveVelocityPID->SetInvertedMotor(RobotDrive::kRearRightMotor, false);

   // Set the max speed for the robot
   m_robotDrive->SetMaxOutput(MAX_VOLTAGE_PERCENT);

   // Setup the camera
   camera.WriteResolution(AxisCamera::kResolution_320x240);
   camera.WriteCompression(20);
   camera.WriteBrightness(50);

   // Set up the turn controller
   m_turnController->SetInputRange(-360.0, 360.0);
   m_turnController->SetOutputRange(-0.6, 0.6);
   m_turnController->SetTolerance(1.0 / 90.0 * 100);
   m_turnController->Disable();

   // Set up the current controllers
   m_currentVelocityRight->SetInputRange(-40, 40);
   m_currentVelocityRight->Disable();
   m_currentVelocityLeft->SetInputRange(-40, 40);
   m_currentVelocityLeft->Disable();

   // Set the expiration for the watchdog
   m_robotDrive->SetExpiration(10.0);
   m_robotDrive->SetSafetyEnabled(false);
   m_robotDriveVelocityPID->SetExpiration(10.0);
   m_robotDriveVelocityPID->SetSafetyEnabled(false);
   /**************************************************************************/

   /******************************* Run **************************************/
   m_compressor->Start();
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
      printf("Position: %f\n", m_shooterRotate->GetPosition());
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
      /**************************************************************************/

      printf("Faults (r,l) %x, %x\n", m_rightMotor->GetFaults(),
            m_leftMotor->GetFaults());
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
   /******************** Adjust the turn PID *********************************/
   //   static float buttonPressedPID = false;
   //
   //   // Increment the value of P
   //   if ((m_buttonBox->GetRawButton(P_INCREMENT) == false) && (buttonPressedPID
   //         == false))
   //   {
   //      buttonPressedPID = true;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP() * PID_CHANGE_VALUE,
   //            m_shooterRotate->GetI(), m_shooterRotate->GetD());
   //   }
   //
   //   // Decrement the value of P
   //   else if ((m_buttonBox->GetRawButton(P_DECREMENT) == false)
   //         && (buttonPressedPID == false))
   //   {
   //      buttonPressedPID = true;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP() / PID_CHANGE_VALUE,
   //            m_shooterRotate->GetI(), m_shooterRotate->GetD());
   //   }
   //
   //   // Increment the value of I
   //   else if ((m_buttonBox->GetRawButton(I_INCREMENT) == true)
   //         && (buttonPressedPID == true))
   //   {
   //      buttonPressedPID = false;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP(),
   //            m_shooterRotate ->GetI() * PID_CHANGE_VALUE,
   //            m_shooterRotate->GetD());
   //   }
   //
   //   // Decrement the value of I
   //   else if ((m_buttonBox->GetRawButton(I_DECREMENT) == true)
   //         && (buttonPressedPID == true))
   //   {
   //      buttonPressedPID = false;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP(),
   //            m_shooterRotate ->GetI() / PID_CHANGE_VALUE,
   //            m_shooterRotate->GetD());
   //   }
   //
   //   // Increment the value of D
   //   else if ((m_buttonBox->GetRawButton(D_INCREMENT) == true)
   //         && (buttonPressedPID == true))
   //   {
   //      buttonPressedPID = false;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP(),
   //            m_shooterRotate ->GetI(),
   //            m_shooterRotate->GetD() * PID_CHANGE_VALUE);
   //   }
   //
   //   // Decrement the value of D
   //   else if ((m_buttonBox->GetRawButton(D_DECREMENT) == true)
   //         && (buttonPressedPID == true))
   //   {
   //      buttonPressedPID = false;
   //      m_shooterRotate->SetPID(m_shooterRotate->GetP(),
   //            m_shooterRotate ->GetI(),
   //            m_shooterRotate->GetD() / PID_CHANGE_VALUE);
   //   }
   //   // Check to see if no buttons are pushed
   //   else if ((m_buttonBox->GetRawButton(P_INCREMENT) != false)
   //         && (m_buttonBox->GetRawButton(P_DECREMENT) != false)
   //         && (m_buttonBox->GetRawButton(I_INCREMENT) != true)
   //         && (m_buttonBox->GetRawButton(I_DECREMENT) != true)
   //         && (m_buttonBox->GetRawButton(D_INCREMENT) != true)
   //         && (m_buttonBox->GetRawButton(D_DECREMENT) != true))
   //   {
   //      buttonPressedPID = false;
   //   }

   /************************ Control the Drive Mode ***************************/
   if (m_joystick->GetRawButton(DRIVE_WITH_VELOCITY) == true)
   {
      setDriveModeToVelocity();
   }

   else if (m_joystick->GetRawButton(DRIVE_WITH_VOLTAGE) == true)
   {
      setDriveModeToVoltagePercent();
   }

   else if (m_joystick->GetRawButton(REVERSE_BALL_PICKUP) == true)
   {
      setDriveModeToCurrent();
   }

   /************************ Drive the Robot **********************************/
   // If the auto straight is on then run the rotation off of the gyro
   if (m_driverStation->GetDigitalIn(AUTO_STRAIGHT) == true)
   {
      if (m_turnController->IsEnabled() == false)
      {
         m_turnController->SetSetpoint(m_gyroHorizontal->GetAngle());
         m_turnController->Enable();
      }

      // allow the joystick to adjust the angle
      m_turnController->SetSetpoint(
            m_turnController->GetSetpoint() + m_joystick->GetThrottle());

      // drive the robot using Arcade drive
      if (driveSetting == kPercentage || driveSetting == kCurrent)
      {
         m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
               m_rotation);
      }
      else
      {
         m_robotDrive->ArcadeVelocityDriveStepped(
               m_joystick->GetAxis(Joystick::kYAxis), m_rotation,
               MAX_ACCELERATION_ARCADE);
      }
   }
   // Run the rotation off the joystick
   else
   {
      if (m_turnController->IsEnabled() == true)
      {
         m_turnController->Disable();
      }

      // drive the robot using Arcade drive
      if (driveSetting == kPercentage || driveSetting == kCurrent)
      {
         m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
               m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);
      }
      else
      {
         m_robotDrive->ArcadeVelocityDriveStepped(
               m_joystick->GetAxis(Joystick::kYAxis),
               (float) (m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR),
               (float) MAX_ACCELERATION_ARCADE, 2.0f);
      }
   }

   /************************ Run Stability Wheels *****************************/
   setStabilityWheelState();
   runStabilityWheels();

   /************************ RUN ShootingWheel ********************************/
   if (m_driverStation->GetDigitalIn(AUTONOMOUSLY_RUN_SHOOTER) == 1)
   {
      m_shooterWheel->Set((m_joystick->GetTwist() + 1) * 0.5f);
   }

   /************************ BALL PICK_UP *************************************/
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
   //   else if (m_joystick->GetRawButton(REVERSE_BALL_PICKUP) == true)
   //   {
   //      m_frontBallPickup->Set(1.0f);
   //      m_backBallPickup->Set(1.0f);
   //   }

   /************************ Ferris Wheel *************************************/
   // Check limit switch and only stop the wheel if the switch is newly pressed
   // <Test>
   //   if (m_joystick->GetRawButton(START_FERRIS_WHEEL) == true)
   //      m_ferrisWheel->Set(FERRIS_ROTATE_SPEED);
   //   else if (m_joystick->GetRawButton(REVERSE_FERRIS_WHEEL) == true)
   //      m_ferrisWheel->Set(-FERRIS_ROTATE_SPEED);
   //   else
   //      m_ferrisWheel->Set(0.0f);

   // </Test>
   setFerrisWheelState();
   runFerrisWheel();

   /************************ RUN Shooter Turn *********************************/
   if (m_driverStation->GetDigitalIn(AUTONOMOUSLY_RUN_SHOOTER) == 1)
   {
      if (m_joystick->GetRawButton(ROTATE_SHOOTER_COUNTER_CLOCK) == true)
         m_shooterRotate->Set(-SHOOTER_ROTATE_SPEED);
      else if (m_joystick->GetRawButton(ROTATE_SHOOTER_CLOCKWISE) == true)
         m_shooterRotate->Set(SHOOTER_ROTATE_SPEED);
      else
         m_shooterRotate->Set(0.0f);
   }

   /************************ RUN Ball Shooter ********************************/
   if (m_joystick->GetRawButton(BALL_LIFT_SHOOT) == true)
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
void MyRobot::runFerrisWheel(void)
{
   switch (ferrisState)
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
 * Determin the state the ferris wheel should be in.
 */
void MyRobot::setFerrisWheelState(void)
{
   static bool previousSwitchValue = false; // The previous value of the ferris wheel switch

   if (m_ferrisWheelStop->Get() == true && previousSwitchValue == false)
   {
      ferrisState = kStop;
      previousSwitchValue = true;
   }

   // check to see if the switch is released
   else if (m_ferrisWheelStop->Get() == false)
   {
      previousSwitchValue = false;
   }

   // Stop the wheel is both buttons are pressed
   if ((m_joystick->GetRawButton(START_FERRIS_WHEEL) == true) &&
         (m_joystick->GetRawButton(REVERSE_FERRIS_WHEEL) == true))
   {
      ferrisState = kStop;
   }
   
   // If button 9 is pushed, the ferris wheel moves forward
   else if (m_joystick->GetRawButton(START_FERRIS_WHEEL) == true)
   {
      ferrisState = kForward;
   }

   // If button 10 is pushed, the ferris wheel will stop and drive backwards
   else if (m_joystick->GetRawButton(REVERSE_FERRIS_WHEEL) == true)
   {
      ferrisState = kBackward;
   }

}

/**
 * Control Drive the Stability Wheels to match the state
 *    Posible States: kFrontDeployed, kBackDeployed, kNeitherDeployed
 */
void MyRobot::runStabilityWheels()
{
   printf("Front Switch: %i\n", m_frontBridgeWheelLimit->Get());
   printf("Back Switch: %i\n", m_backBridgeWheelLimit->Get());
   switch (stabilityWheelState)
   {
   case kFrontDeployed:
      if (m_backBridgeWheelLimit->Get() == true)
      {
         m_frontBridgeWheel->Set(DoubleSolenoid::kForward);
      }
      m_backBridgeWheel->Set(DoubleSolenoid::kReverse);
      break;
   case kBackDeployed:
      m_frontBridgeWheel->Set(DoubleSolenoid::kReverse);
      if (m_frontBridgeWheelLimit->Get() == true)
      {
         m_backBridgeWheel->Set(DoubleSolenoid::kForward);
      }
      break;
   case kNeitherDeployed:
      m_frontBridgeWheel->Set(DoubleSolenoid::kReverse);
      m_backBridgeWheel->Set(DoubleSolenoid::kReverse);
      break;
   }
}

/**
 * Determin the state the wheels need to be in
 */
void MyRobot::setStabilityWheelState()
{
   // Previous switch state
   static ThreeWaySwitchState previousStabilitySwitchState = kMiddle;

   // Is the switch newly Changed
   static bool newChange = false;

   // Current switch state
   ThreeWaySwitchState currentStabilitySwitchState;

   // Determin the state of the switch
   if (m_driverStation->GetDigitalIn(STABILlTY_SWITCH_UP) == false)
   {
      currentStabilitySwitchState = kUp;
   }
   else if (m_driverStation->GetDigitalIn(STABILITY_SWITCH_DOWN) == false)
   {
      currentStabilitySwitchState = kDown;
   }
   else
   {
      currentStabilitySwitchState = kMiddle;
   }

   // Determin if change is new
   if (currentStabilitySwitchState != previousStabilitySwitchState)
   {
      m_gyroVertical->Reset();
      newChange = true;
   }

   // Set the previous switch state equall to the current switch state
   previousStabilitySwitchState = currentStabilitySwitchState;

   /************************ Check the switchs and buttons ********************/
   // Front override button
   if (m_joystick->GetRawButton(STABILITY_SWITCH_FORWARD_OVERRIDE) == TRUE)
   {
      stabilityWheelState = kFrontDeployed;
      newChange = false;
   }

   // Back override button
   else if (m_joystick->GetRawButton(STABILITY_SWITCH_BACKWARD_OVERRIDE)
         == TRUE)
   {
      stabilityWheelState = kBackDeployed;
      newChange = false;
   }

   // Switch Up new change
   else if ((currentStabilitySwitchState == kUp) && (newChange == true))
   {
      stabilityWheelState = kFrontDeployed;

      // See if it is time to change Positions
      if (getStabilityStateFromGyro() != kNeitherDeployed)
      {
         newChange = false;
      }
   }

   // Switch Down new change
   else if ((currentStabilitySwitchState == kDown) && (newChange == true))
   {
      stabilityWheelState = kBackDeployed;

      // See if it is time to change Positions
      if (getStabilityStateFromGyro() != kNeitherDeployed)
      {
         newChange = false;
      }
   }

   // additional changes
   else if ((currentStabilitySwitchState != kMiddle) && (newChange == false))
   {
      stabilityWheelState = (StabilityWheelState) getStabilityStateFromGyro();
   }

   // Stability is raised
   else
   {
      stabilityWheelState = kNeitherDeployed;
   }
}
/**
 * Determine the state according to the gyro
 *    Returns: int
 *       0 = kNeitherDeployed
 *       1 = kFrontDeployed
 *       2 = kBackDeployed
 */
int MyRobot::getStabilityStateFromGyro(void)
{
   int state;
   // Run the tip avoidence off of the gyro
   if (m_gyroVertical->GetAngle() > TIPPED_THRESHOLD)
   {
      state = kFrontDeployed;
   }
   else if (m_gyroVertical->GetAngle() < -TIPPED_THRESHOLD)
   {
      state = kBackDeployed;
   }
   else
   {
      state = kNeitherDeployed;
   }
   return state;
}

/**
 * Set the drive mode to run of velocity 
 *    Motor Values: -1 to 1
 */
void MyRobot::setDriveModeToVelocity(void)
{
   driveSetting = kVelocity;
   m_rightMotor->ChangeControlMode(CANJaguar::kSpeed);
   m_leftMotor->ChangeControlMode(CANJaguar::kSpeed);
   m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_rightMotor->SetPID(MOTOR_VELOCITY_P, MOTOR_VELOCITY_I, MOTOR_VELOCITY_D);
   m_leftMotor->SetPID(MOTOR_VELOCITY_P, MOTOR_VELOCITY_I, MOTOR_VELOCITY_D);
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
   driveSetting = kPercentage;
   m_rightMotor->ChangeControlMode(CANJaguar::kPercentVbus);
   m_leftMotor->ChangeControlMode(CANJaguar::kPercentVbus);
   m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
   m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
   m_robotDrive->SetMaxOutput(MAX_VOLTAGE_PERCENT);
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
}

/**
 * Set the drive mode to current mode
 */
void MyRobot::setDriveModeToCurrent()
{
   driveSetting = kCurrent;
   m_rightMotor->ChangeControlMode(CANJaguar::kCurrent);
   m_leftMotor->ChangeControlMode(CANJaguar::kCurrent);
   m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
   m_rightMotor->SetPID(MOTOR_CURRENT_P, MOTOR_CURRENT_I, MOTOR_CURRENT_D);
   m_leftMotor->SetPID(MOTOR_CURRENT_P, MOTOR_CURRENT_I, MOTOR_CURRENT_D);
   m_robotDrive->SetMaxOutput(MAX_CURRENT);
   m_rightMotor->EnableControl();
   m_leftMotor->EnableControl();
}

void MyRobot::runVelocityPID(float setpointRight, float setpointLeft)
{
   if (m_currentVelocityRight->IsEnabled() == false
         || m_currentVelocityLeft->IsEnabled() == false)
   {
      m_currentVelocityRight->Reset();
      m_currentVelocityRight->Enable();
      m_currentVelocityLeft->Reset();
      m_currentVelocityLeft->Enable();
   }
   m_currentVelocityRight->SetSetpoint(setpointRight);
   m_currentVelocityLeft->SetSetpoint(setpointLeft);
}

void MyRobot::disableVelocityPID()
{
   m_currentVelocityRight->Disable();
   m_currentVelocityLeft->Disable();
}

/**
 * Routine sends data to the dashboard.
 */
void MyRobot::sendDashboardData()
{
   // send IO data to the DriverStation
   m_dashboardDataFormat->SendLCDData(heightOfTriangle,
         m_rightMotor->GetSpeed(), m_leftMotor->GetSpeed(),
         m_rightMotor->Get(), m_leftMotor->Get(), 0.0, 0.0, 0.0,
         m_shooterWheel->Get());
   m_dashboardDataFormat->SendIOPortData();
   m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS(MyRobot)
;
