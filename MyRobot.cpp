#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR     		0.50
#define NUMBER_OF_ENCODER_LINES	 		1440
#define NUMBER_OF_ENCODER_LINES_MOTORS 	720

/************************** Power Defines *************************************/
#define SHOOTER_ROTATE_SPEED  			20.0f
#define FARRIS_ROTATE_SPEED    			0.2f
#define MAX_SPEED 			   			80.0
/******************************************************************************/

RobotDemo::RobotDemo(void)
{
	m_dashboardDataFormat = new DashboardDataFormat();
	m_rightMotor = new CANJaguar(RIGHT_DRIVE_MOTOR, CANJaguar::kVoltage);
	m_leftMotor = new CANJaguar(LEFT_DRIVE_MOTOR, CANJaguar::kVoltage);
	m_shooterWheel = new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kVoltage);
	m_shooterWheelRedundant = new CANJaguar(SHOOTER_WHEEL_MOTOR_REDUNDANT, CANJaguar::kVoltage);
	m_shooterRotate = new CANJaguar(SHOOTER_ROTATE_MOTOR, CANJaguar::kVoltage);
	m_ferrisWheel = new Victor(FERRIS_WHEEL_MOTOR);
	m_frontBallPickup = new Victor(FRONT_BALL_PICKUP_MOTOR);
	m_backBallPickup = new Victor(BACK_BALL_PICKUP_MOTOR);
	m_robotDrive = new RobotDrive(m_leftMotor, m_rightMotor); // create a robot drive system with four moters

	m_compressor = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
	m_ballLiftSolenoid = new DoubleSolenoid(BALL_LIFT_FORWARD, BALL_LIFT_REVERSED);
	m_joystick = new Joystick(JOYSTICK_PORT); // as they are declared above.
	m_driverStation = DriverStation::GetInstance();
	m_ferrisWheelStop = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
	m_bottomSlot = new DigitalInput(BOTTOM_SLOT_DETECTOR_PORT);

	/******************************* Setup ********************************/
	// Set up the Jaguars
	// Set up the right Jaguar
	m_rightMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	m_rightMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	m_rightMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
	m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

	// Set up the left Jaguar
	m_leftMotor->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	m_leftMotor->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	m_leftMotor->ConfigEncoderCodesPerRev(NUMBER_OF_ENCODER_LINES_MOTORS);
	m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

	// Set up the Shooter Jaguar
	m_shooterWheel->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	m_shooterWheel->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	m_shooterWheelRedundant->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
	m_shooterWheelRedundant->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
	m_shooterWheel->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	m_shooterWheelRedundant->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
	
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

	// Set the expiration for the watchdog
	m_robotDrive->SetExpiration(10.0);
	/**********************************************************************/

	/******************************* Run **********************************/
	m_compressor->Start();
	/**********************************************************************/
}

/**
 * Drive left & right motors for 2 seconds then stop
 */
void RobotDemo::Autonomous(void)
{
	while (IsAutonomous())
	{
	}
}

/**
 * Runs the motors with arcade steering. 
 */
void RobotDemo::OperatorControl(void)
{
	// Enable the jaguars
	m_rightMotor->EnableControl();
	m_leftMotor->EnableControl();
	m_shooterRotate->EnableControl();
	m_shooterWheel->EnableControl();
	m_shooterWheelRedundant->EnableControl();

	bool ferrisRotate = false;
	bool previousSwitchValue = false;

	//m_robotDrive->SetSafetyEnabled(true);
	while (IsOperatorControl())
	{
		//printf("In operator control\n");

		range = ultra->GetRangeInches();
		printf("Rotate Encoder: %f\n", m_shooterRotate->GetPosition());
		// drive the robot using Arcade drive
		m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
				m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);

		/************ RUN ShootingWheel ***************************************/
		m_shooterWheel->Set((m_joystick->GetTwist() - 1) * 50.0f);
		m_shooterWheelRedundant->Set((m_joystick->GetTwist() - 1) * 50.0f);
		/**********************************************************************/

		/************ BALL PICK_UP ********************************************/
		// if button 6 is pushed then stop the ball pickerupper
		if (m_joystick->GetRawButton(STOP_BALL_PICKUP) == true)
		{
			m_frontBallPickup->Set(0.0f);
			m_backBallPickup->Set(0.0f);
		}

		// if button 7 is pushed then run the ball pickerupper to suck balls
		else if (m_joystick->GetRawButton(START_BALL_PICKUP) == true)
		{
			m_frontBallPickup->Set(1.0f);
			m_backBallPickup->Set(-1.0f);
		}

		// if button 8 is pushed then run the ball pickerupper to spit balls
		else if (m_joystick->GetRawButton(REVERSE_BALL_PICKUP) == true)
		{
			m_frontBallPickup->Set(-1.0f);
			m_backBallPickup->Set(1.0f);
		}
		/**********************************************************************/
		
		/************Debug Printouts ******************************************/
		printf("WheelStop: %i\n", m_ferrisWheelStop->Get());
		printf("Bottom Slot: %i\n\n", m_bottomSlot->Get());
		
		/************ Ferris Wheel ********************************************/
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
			m_ferrisWheel->Set(FARRIS_ROTATE_SPEED);
			ferrisRotate = true;
		}
		// If button 10 is pushed, the ferris wheel will stop and drive backwards
		else if (m_joystick->GetRawButton(REVERSE_FERRIS_WHEEL) == true)
		{
			m_ferrisWheel->Set(-FARRIS_ROTATE_SPEED);
			ferrisRotate = false;
		}
		else if (ferrisRotate == true)
		{
			m_ferrisWheel->Set(FARRIS_ROTATE_SPEED);
		}
		else
		{
			m_ferrisWheel->Set(0.0f);
		}
		/**********************************************************************/

		/************ RUN Shooter Turn ****************************************/
		if (m_joystick->GetRawButton(ROTATE_SHOOTER_COUNTER_CLOCK) == true)
			m_shooterRotate->Set(-SHOOTER_ROTATE_SPEED);
		else if (m_joystick->GetRawButton(ROTATE_SHOOTER_CLOCKWISE) == true)
			m_shooterRotate->Set(SHOOTER_ROTATE_SPEED);
		else
			m_shooterRotate->Set(0.0f);
		/**********************************************************************/

		/************ RUN THE SOLENOIDS ***************************************/
		if (m_joystick->GetRawButton(BALL_LIFT_SHOOT) == true)
		{
			m_ballLiftSolenoid->Set(DoubleSolenoid::kForward);
		}
		else
		{
			m_ballLiftSolenoid->Set(DoubleSolenoid::kReverse);
		}
		/********************************************/

//				if (m_joystick->GetRawButton(DRIVE_WITH_VELOCITY) == true)
//				{
//					m_rightMotor->ChangeControlMode(CANJaguar::kSpeed);
//					m_leftMotor->ChangeControlMode(CANJaguar::kSpeed);
//					m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
//					m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
//					m_rightMotor->SetPID(0.01, 0.01, 0.0);
//					m_leftMotor->SetPID(0.01, 0.01, 0.0);
//					m_robotDrive->SetMaxOutput(300);
//					m_rightMotor->EnableControl();
//					m_leftMotor->EnableControl();
//				}

				if (m_joystick->GetRawButton(DRIVE_WITH_VOLTAGE) == true)
				{
					m_rightMotor->ChangeControlMode(CANJaguar::kVoltage);
					m_leftMotor->ChangeControlMode(CANJaguar::kVoltage);
					m_rightMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
					m_leftMotor->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
					m_robotDrive->SetMaxOutput(MAX_SPEED);
					m_rightMotor->EnableControl();
					m_leftMotor->EnableControl();
				}

//				if (m_rightMotor->GetFaults() != 0)
//					printf("Right Fault: %i", m_rightMotor->GetFaults());
//				
//				if (m_leftMotor->GetFaults() != 0)
//					printf("Left Fault: %i", m_leftMotor->GetFaults());

		SendDashboardData();
		Wait(0.005); // wait for a motor update time
	}
}

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void Robot::SendDashboardData(ADXL345_I2C::AllAxes allAxes);
//
//       allAxes - 
//
// Description:
//
//    Routine sends data to the dashboard.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void RobotDemo::SendDashboardData()
{
	// send IO data to the DriverStation
	m_dashboardDataFormat->SendLCDData(range, m_rightMotor->GetSpeed(),
			m_leftMotor->GetSpeed(), m_rightMotor->Get(), m_leftMotor->Get(),
			m_rightMotor->GetPosition(), m_leftMotor->GetPosition(),
			m_shooterWheel->Get());
	m_dashboardDataFormat->SendIOPortData();
	m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS( RobotDemo)
;

