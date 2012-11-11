//------------------------------------------------------------------------------
// Source Filename:  Robot.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//------------------------------------------------------------------------------

#include <vector>
#include <algorithm>

#include "Robot.h"

#include "nivision.h"
#include "Vision/MonoImage.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

// percent of rotation power (in decimal)
#define ROTATE_REDUCE_FACTOR               0.50

//------------------------------------------------------------------------------
// Routine Summary:
//
//    Robot::Robot(void)
//
// Description:
//
//    Constructor for the Robot class. The constructor instantiates the robot
// controls and systems, which consists of the following:
//
//    Joystick 1    - Standard joystick with twist.
//    Joystick 2    -
//    Gyro          -
//    Accelerometer -
//    Kinects       -
//    Drive System  -
//    Dashboard     -
//    Compressor    -
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//------------------------------------------------------------------------------
Robot::Robot(void)
{
	// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, 4
	m_joystick = new Joystick(1); // joystick in  USB port 1
	m_armBox = new Joystick(2); // arm control box in USE port 2
	m_gyro = new Gyro(1); // gyro in analog port 1
	m_accelerometer = new ADXL345_I2C(1); // accelerometer
	m_driveSystem = new DriveSystem(1, 2, 3, 4, m_gyro); // create a robot drive system with four motors
	m_dashboardDataSender = new DashboardDataSender();
	m_compressor = new Compressor(1, 1); // compresser relay in relay 1
	//  and pressure switch in digital in 1
	m_leftArm = new KinectStick(1);
	m_rightArm = new KinectStick(2);
	m_triggerPressed = false; // trigger by default is not pressed

	// get driver station components setup
	m_driverStation = DriverStation::GetInstance();

	/******************************* Setup ********************************/
	// Invert the drive motors
	m_driveSystem->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	m_driveSystem->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);

	// set MotorSafety expiration
	m_driveSystem->SetExpiration(1.0);
	m_driveSystem->SetSafetyEnabled(false);

	// set sensitivity for the 2010 kit gyro
	m_gyro->SetSensitivity(0.007);

	/******************************* Run **********************************/
	// run compresser when robot has power to charge cylinder
	m_compressor->Start();
}

//------------------------------------------------------------------------------
// Routine Summary:
//
// Description:
//
//    RobotInit virtual method.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//------------------------------------------------------------------------------
void Robot::RobotInit()
{

}

//------------------------------------------------------------------------------
// Routine Summary:
//
// Description:
//
//    Disable virtual method.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void Robot::Disabled()
{

}

//------------------------------------------------------------------------------
// Routine Summary:
//
//    Robot::OperatorControl(void)
//
// Description:
//
//    Routine for the operator portion of the game.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//------------------------------------------------------------------------------
void Robot::OperatorControl(void)
{
	CalcAccelerometerOffset();
	//CameraTracking();
	//GyroTest();
	while (IsOperatorControl())
	{
		DriveControl();
		RunAccelerometer();
		
		// send the dashboard data
		SendDashboardData();
	}
}

//------------------------------------------------------------------------------
// Routine Summary:
//
//    void Robot::CalcAccelerometerOffset()
//
// Description:
//
//    Routine to calculate the offset for the Accelerometer and set up the variables.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//------------------------------------------------------------------------------
void Robot::CalcAccelerometerOffset()
{
	m_XaccelerationOffset = 0.0f;
	m_YaccelerationOffset = 0.0f;
	m_ZaccelerationOffset = 0.0f;

	m_Xacceleration = 0.0f;
	m_Yacceleration = 0.0f;
	m_Zacceleration = 0.0f;

	m_Xvelocity = 0.0f;
	m_Yvelocity = 0.0f;
	m_Zvelocity = 0.0f;

	m_Xdistance = 0.0f;
	m_Ydistance = 0.0f;
	m_Zdistance = 0.0f;

	for (int i = 0; i < 100; i++)
	{
		// Get the Acceleration
		allAxes = m_accelerometer->GetAccelerations();

		m_XaccelerationOffset += allAxes.XAxis;
		m_YaccelerationOffset += allAxes.YAxis;
		m_ZaccelerationOffset += allAxes.ZAxis;
	}

	m_XaccelerationOffset /= 100;
	m_YaccelerationOffset /= 100;
	m_ZaccelerationOffset /= 100;
	
	printf("X Offset:%f Y Offset:%f Z Offset:%f\n", m_XaccelerationOffset, m_YaccelerationOffset,
			m_ZaccelerationOffset);
}

//------------------------------------------------------------------------------
// Routine Summary:
//
//    void Robot::RunAccelerometer()
//
// Description:
//
//    Routine that is called in a loop to sum up the acceleration values from the robot.
//    The routine also sends the data to the dashboard.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//------------------------------------------------------------------------------
void Robot::RunAccelerometer()
{
	allAxes = m_accelerometer->GetAccelerations();

	// Get the Accelerations
	m_Xacceleration = allAxes.XAxis - m_XaccelerationOffset;
	m_Yacceleration = allAxes.YAxis - m_YaccelerationOffset;
	m_Zacceleration = allAxes.ZAxis - m_ZaccelerationOffset;

	// Calculate the Velocities
	m_Xvelocity += m_Xacceleration;
	m_Yvelocity += m_Yacceleration;
	m_Zvelocity += m_Zacceleration;

	// Calculate the Positions
	m_Xdistance += m_Xvelocity;
	m_Ydistance += m_Yvelocity;
	m_Zdistance += m_Zvelocity;
}

//------------------------------------------------------------------------------
// Routine Summary:
//
// Description:
//
//    Drive the robot using the joystick in port 1
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void Robot::DriveControl(void)
{
	// drive the robot using holonomic drive
	m_driveSystem->HolonomicDrive(m_joystick->GetMagnitude(),
			m_joystick->GetDirectionDegrees(),
			m_joystick->GetZ() * ROTATE_REDUCE_FACTOR);

	/******************** Kinect Code ********************/
	//m_driveSystem->Kinect(m_leftArm, m_rightArm);
	/*****************************************************/
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
void Robot::SendDashboardData()
{
	// send IO data to the DriverStation
	m_dashboardDataSender->SendIOPortData((int) m_gyro->GetAngle(),
			m_Xacceleration, m_Yacceleration, m_Zacceleration, m_Xvelocity,
			m_Yvelocity, m_Zvelocity, m_Xdistance, m_Ydistance, m_Zdistance,
			m_xDistanceFromCenter, 0.0f);
	m_dashboardDataSender->SendLCDData(m_gyro->GetAngle(),
			m_joystick->GetTwist() * 360, P, I, D);
}
START_ROBOT_CLASS(Robot)
;

