#include "MyRobot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR     		 1.0
#define NUMBER_OF_ENCODER_LINES	 		1440
#define NUMBER_OF_ENCODER_LINES_MOTORS 	 720
#define MINIMUM_ROTATION_OF_SHOOTER		 -.3
#define MAXIMUM_ROTATION_OF_SHOOTER		  .3
#define SHOOTER_SPEED_CHANGE			  .2

/************************** Power Defines *************************************/
#define SHOOTER_ROTATE_SPEED  			0.4f
#define FARRIS_ROTATE_SPEED    			0.3f
#define MAX_SPEED 			   			1.0f
/******************************************************************************/

/************************** Conversion Defines ********************************/
#define GYRO_CONVERSION				  0.0044
#define PIXEL_CONVERSION		     0.00009
/******************************************************************************/

#define NO_TARGET				  -999999.0f

MyRobot::MyRobot(void)
{
	m_dashboardDataFormat = new DashboardDataFormat();
	m_rightMotor = new CANJaguar(RIGHT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
	m_leftMotor = new CANJaguar(LEFT_DRIVE_MOTOR, CANJaguar::kPercentVbus);
	m_shooterWheel
			= new CANJaguar(SHOOTER_WHEEL_MOTOR, CANJaguar::kPercentVbus);
	m_shooterRotate = new CANJaguar(SHOOTER_ROTATE_MOTOR,
			CANJaguar::kPercentVbus);
	m_ferrisWheel = new Victor(FERRIS_WHEEL_MOTOR);
	m_frontBallPickup = new Victor(FRONT_BALL_PICKUP_MOTOR);
	m_backBallPickup = new Victor(BACK_BALL_PICKUP_MOTOR);
	m_robotDrive = new RobotDrive(m_leftMotor, m_rightMotor); // create a robot drive system with four moters

	m_compressor = new Compressor(COMPRESSOR_SENSOR, COMPRESSOR_RELAY);
	m_ballLiftSolenoid = new DoubleSolenoid(BALL_LIFT_FORWARD,
			BALL_LIFT_REVERSED);
	m_joystick = new Joystick(JOYSTICK_PORT); // as they are declared above.
	m_buttonBox = new Joystick(BUTTON_BOX_PORT);
	m_driverStation = DriverStation::GetInstance();
	m_ferrisWheelStop = new DigitalInput(FERRIS_WHEEL_STOP_PORT);
	m_bottomSlot = new DigitalInput(BOTTOM_SLOT_DETECTOR_PORT);
	m_gyro = new Gyro(GYRO_PORT);

	AxisCamera &camera = AxisCamera::GetInstance();

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

	// Set the expiration for the watchdog
	m_robotDrive->SetExpiration(10.0);
	m_robotDrive->SetSafetyEnabled(false);
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
	while (IsAutonomous())
	{
	}
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

	bool ferrisRotate = false; // Is the ferris wheel rotating by itself?
	bool previousSwitchValue = false; // The previous value of the ferris wheel switch

	int targetStatus; // Is the target found by the robot
	double distanceToTarget; // The distance from the targets
	double voltageToDriveShooter; // The voltage to drive the shooter
	double valueToRotate; // The value to rotate the shooter to relative to current position
	double previousGyro = m_gyro->GetAngle();

	//m_robotDrive->SetSafetyEnabled(true);
	while (IsOperatorControl())
	{

		// Process the camera images
		targetStatus = ReadCamera(heightOfTriangle, distanceToTarget, voltageToDriveShooter,
				valueToRotate);

		// Add to the rotate value based off the gyro
		valueToRotate += (previousGyro - m_gyro->GetAngle()) * GYRO_CONVERSION;
		previousGyro = m_gyro->GetAngle();

		// Check to see if the shooter is to be controled by the robot or by the operators
		if (m_driverStation->GetDigitalIn(MINIBOT_PORT) == 0)
		{
			// change the modes of the jaguars
			if (m_shooterWheel->GetControlMode() != CANJaguar::kVoltage
					|| m_shooterRotate->GetControlMode()
							!= CANJaguar::kPosition)
			{
				// Set up the Shooter Jaguar
				m_shooterWheel->ChangeControlMode(CANJaguar::kVoltage);
				m_shooterWheel->Set(0.0f);
				m_shooterWheel->EnableControl();

				// Set up the Rotate Jaguar
				m_shooterRotate->ChangeControlMode(CANJaguar::kPosition);
				m_shooterRotate->SetPID(4000, .2, 10.0);
				m_shooterRotate->Set(m_shooterRotate->GetPosition());
				m_shooterRotate->EnableControl();
			}
			// Control the Shooter Autonomously
			AutonomouslyDriveShooter(targetStatus, distanceToTarget,
					voltageToDriveShooter, valueToRotate);
		}
		else
		{
			// Change the modes of the modes of the jaguars
			if (m_shooterWheel->GetControlMode() != CANJaguar::kPercentVbus
					|| m_shooterRotate->GetControlMode()
							!= CANJaguar::kPercentVbus)
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

		// Read the Controls
		ReadControls(ferrisRotate, previousSwitchValue);

		/************Debug Printouts **********************************************/
		printf("Height of the Triagle: %f\n", heightOfTriangle);
		printf("Voltage: %f\n\n", m_shooterWheel->Get());
//		printf("Right Motor: %f, Left Motor: %f\n", m_rightMotor->Get(),
//				m_leftMotor->Get());
//		printf("Shooter Wheel: %f, Shooter Rotate: %f\n",
//				m_shooterWheel->Get(), m_shooterRotate->Get());
//		printf("Target Status: %i\n", targetStatus);
		/**************************************************************************/

		// Send the data to the dashboard
		SendDashboardData();

		// wait for a motor update time
		Wait(0.005);
	}
}

/**
 * Read the operator controls and drive the robot accordinly.
 */
void MyRobot::ReadControls(bool ferrisRotate, bool previousSwitchValue)
{
	// drive the robot using Arcade drive
	m_robotDrive->ArcadeDrive(m_joystick->GetAxis(Joystick::kYAxis),
			m_joystick->GetThrottle() * ROTATE_REDUCE_FACTOR);

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
		m_rightMotor->ChangeControlMode(CANJaguar::kPercentVbus);
		m_leftMotor->ChangeControlMode(CANJaguar::kPercentVbus);
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
}

/**
 * Routine to process the camera images.
 * 
 * Return of 0 means targets found
 * Return of 1 means only one top target found
 * Return of 2 means no targets found
 */
int MyRobot::ReadCamera(double &heightOfTriangle, double &distanceToTarget,
		double &voltageToDriveShooter, double &valueToRotate)
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
		if (results[RIGHT_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET
				|| results[LEFT_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET)
		{
			if (results[TOP_TARGET][ASPECT_RATIO_SCORE_INDEX] == NO_TARGET)
			{
				targetStatus = 2;
			}
			else
			{
				targetStatus = 1;
			}
		}

		//				printf("Received data.\n");
		//				printf("Unsorted data.\n");
		//				for (int i = 0; i < NUMBER_OF_TARGETS; i++)
		//				{
		//					for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
		//					{
		//						printf("%f ", results[i][j]);
		//					}
		//					printf("\n");
		//				}
		//				printf("\n");

		// Sort the array
		if (targetStatus == 0)
		{
			SortTargetArray(results);
		}

		//		printf("Sorted data.\n");
		//		for (int i = 0; i < NUMBER_OF_TARGETS; i++)
		//		{
		//			for (int j = 0; j < NUMBER_OF_TARGET_PARAMETERS; j++)
		//			{
		//				printf("%f ", results[i][j]);
		//			}
		//			printf("\n");
		//		}
		//		printf("\n");

		// Print information for excel
		//		for (int i = 0; i < NUMBER_OF_TARGETS; i++)
		//		{
		//			if (i == 0)
		//				printf(
		//						"%5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f",
		//						results[i][CENTER_OF_MASS_X_INDEX],
		//						results[i][CENTER_OF_MASS_Y_INDEX],
		//						results[i][BOUNDING_RECT_WIDTH_INDEX],
		//						results[i][BOUNDING_RECT_HEIGHT_INDEX],
		//						results[i][PARTICAL_AREA_INDEX],
		//						results[i][BOUNDING_BOX_AREA_INDEX],
		//						results[i][FIRST_PIX_X_INDEX],
		//						results[i][FIRST_PIX_Y_INDEX],
		//						results[i][ASPECT_RATIO_SCORE_INDEX],
		//						results[i][RECTANGLE_SCORE_INDEX],
		//						results[i][PARAMETER_INDEX]);
		//			else
		//				printf(
		//						"\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f\t %5.2f",
		//						results[i][CENTER_OF_MASS_X_INDEX],
		//						results[i][CENTER_OF_MASS_Y_INDEX],
		//						results[i][BOUNDING_RECT_WIDTH_INDEX],
		//						results[i][BOUNDING_RECT_HEIGHT_INDEX],
		//						results[i][PARTICAL_AREA_INDEX],
		//						results[i][BOUNDING_BOX_AREA_INDEX],
		//						results[i][FIRST_PIX_X_INDEX],
		//						results[i][FIRST_PIX_Y_INDEX],
		//						results[i][ASPECT_RATIO_SCORE_INDEX],
		//						results[i][RECTANGLE_SCORE_INDEX],
		//						results[i][PARAMETER_INDEX]);
		//		}
		//		printf("\n\n");
		delete HSLimage;
	}
	// Calculate the distance to the basket.
	// Calculate the heigth of the top triangle
	if (targetStatus == 0)
	{
		heightOfTriangle =
						sqrt(
								pow(
										results[TOP_TARGET][CENTER_OF_MASS_X_INDEX]
												- ((results[LEFT_TARGET][CENTER_OF_MASS_X_INDEX]
														+ results[RIGHT_TARGET][CENTER_OF_MASS_X_INDEX])
														/ 2.0), 2.0)
										+ pow(
												results[TOP_TARGET][CENTER_OF_MASS_Y_INDEX]
														- ((results[LEFT_TARGET][CENTER_OF_MASS_Y_INDEX]
																+ results[RIGHT_TARGET][CENTER_OF_MASS_Y_INDEX])
																/ 2.0), 2.0));

		//double oldDistance = 43.867483765 * pow(0.986054120856, height);

		// Calculate Distance
		// TODO: Calculate the distance equation
		distanceToTarget = 2741.0998 * pow((1 / heightOfTriangle), 1.2082);

		// Calculate the voltage to drive the shooter
		// TODO: Calculate the voltage equation
		if(m_buttonBox->GetRawButton(4) == true)
		{
		voltageToDriveShooter = 5.275914116 * exp(
				0.0286983122 * distanceToTarget);
		}

	}
	// Calculate the value to rotate if a target was found
	/*
	 Range of the Camera: -160 to 160     Combined: 320
	 Field of view of the Camera: 47 deg
	 Range of the Rotator: -.4 to .4	  Combined: .8
	 Field of the Rotator: 180 deg
	 
	 degree    	encoder value      47      .8
	 -----  * --------------   =  ----- * ------ = 0.0006528
	 pixel		degree             320     180
	 */
	// Calculate the value to rotate the shooter to.
	// TODO: Check the ratio constant
	if (targetStatus == 0 || targetStatus == 1)
	{
//		printf("pixeloff: %f\n",
//				(results[TOP_TARGET][CENTER_OF_MASS_X_INDEX] - (320 / 2)));
		valueToRotate = (results[TOP_TARGET][CENTER_OF_MASS_X_INDEX]
				- (320 / 2)) * PIXEL_CONVERSION;
	}

	return targetStatus;
}

/**
 * Method to drive the shooter automonously
 */
void MyRobot::AutonomouslyDriveShooter(int targetStatus,
		double distanceToTarget, double voltageToDriveShooter,
		double valueToRotate)
{
	static float wheelOffset = 0.0;
	static bool buttonPressed = false;
	
	// Set the offset to zero
	if (m_buttonBox->GetRawButton(RESET_BALL_OFFSET) == false && buttonPressed == false)
	{
		wheelOffset = 0.0;
		buttonPressed = true;
	}
	// Increment the offset
	else if (m_buttonBox->GetRawButton(INCREMENT_BALL_OFFSET) == true && buttonPressed == false)
	{
		wheelOffset += SHOOTER_SPEED_CHANGE;
		buttonPressed = true;
	}
	// Decrement the offset
	else if(m_buttonBox->GetRawButton(DECREMENT_BALL_OFFSET) == true && buttonPressed == false)
	{
		wheelOffset -= SHOOTER_SPEED_CHANGE;
		buttonPressed = true;
	}
	else if (m_buttonBox->GetRawButton(RESET_BALL_OFFSET) != false && m_buttonBox->GetRawButton(INCREMENT_BALL_OFFSET) != true && m_buttonBox->GetRawButton(DECREMENT_BALL_OFFSET) != true)
	{
		buttonPressed = false;
	}
	
	// Set the Voltage output for the wheel
	m_shooterWheel->Set(voltageToDriveShooter + wheelOffset);
	
	// Set the Rotational value for the shooter
	double rotationValue = m_shooterRotate->GetPosition() + valueToRotate;
	/************* Enable to track target *************************************/
	if (rotationValue >= MINIMUM_ROTATION_OF_SHOOTER && rotationValue
			<= MAXIMUM_ROTATION_OF_SHOOTER)
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
void MyRobot::SortTargetArray(
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

/**
 * Routine sends data to the dashboard.
 */
void MyRobot::SendDashboardData()
{
	// send IO data to the DriverStation
	m_dashboardDataFormat->SendLCDData(heightOfTriangle, m_rightMotor->GetSpeed(),
			m_leftMotor->GetSpeed(), m_rightMotor->Get(), m_leftMotor->Get(),
			m_rightMotor->GetPosition(), m_leftMotor->GetPosition(),
			m_shooterWheel->Get());
	m_dashboardDataFormat->SendIOPortData();
	m_dashboardDataFormat->SendVisionData();
}

START_ROBOT_CLASS(MyRobot)
;

