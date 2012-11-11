#ifndef __MyRobot_h__
#define __MyRobot_h__

#include "WPILib.h"
#include "DashboardDataFormat.h"
#include "ImageProcessing.h"

/*************************** Jaguar ID Defines ********************************/
#define RIGHT_DRIVE_MOTOR		 		2
#define LEFT_DRIVE_MOTOR		 		3
#define SHOOTER_WHEEL_MOTOR		 		4
#define SHOOTER_ROTATE_MOTOR			5
/******************************************************************************/

/**************************** PWM PORT Defines ********************************/
#define BACK_BALL_PICKUP_MOTOR			1
#define FRONT_BALL_PICKUP_MOTOR			2
#define FERRIS_WHEEL_MOTOR				3
/******************************************************************************/

/**************************** Analog I/O PORT Defines *************************/
#define GYRO_PORT						1
/******************************************************************************/

/**************************** Digital I/O PORT Defines ************************/
#define COMPRESSOR_SENSOR				1
#define FERRIS_WHEEL_STOP_PORT			2
#define BOTTOM_SLOT_DETECTOR_PORT		4

/**************************** Relay PORT Defines ******************************/
#define COMPRESSOR_RELAY				1
/******************************************************************************/

/**************************** Solenoid PORT Defines ***************************/
#define BALL_LIFT_FORWARD				1
#define BALL_LIFT_REVERSED				2
/******************************************************************************/

/**************************** USB PORT Defines ********************************/
#define JOYSTICK_PORT					1
#define BUTTON_BOX_PORT					2
/******************************************************************************/

/***************************** Joystick 1 Button Defines **********************/
#define BALL_LIFT_SHOOT					1
#define	DRIVE_WITH_VOLTAGE				3
#define DRIVE_WITH_VELOCITY				4
#define STOP_BALL_PICKUP				6
#define START_BALL_PICKUP				7
#define REVERSE_BALL_PICKUP				8
#define START_FERRIS_WHEEL				9
#define REVERSE_FERRIS_WHEEL			10
#define	ROTATE_SHOOTER_COUNTER_CLOCK	11
#define ROTATE_SHOOTER_CLOCKWISE		12
/******************************************************************************/

/***************************** Button Box Button Defines **********************/
#define RESET_BALL_OFFSET				11
#define INCREMENT_BALL_OFFSET			1
#define DECREMENT_BALL_OFFSET			2
/******************************************************************************/

/***************************** Cypris Button Defines **************************/
#define MINIBOT_PORT        			4
/******************************************************************************/

class MyRobot: public SimpleRobot
{
private:
	DashboardDataFormat *m_dashboardDataFormat; // object to send data to the Driver station
	CANJaguar *m_rightMotor;// Jaguar motor controllers
	CANJaguar *m_leftMotor;
	CANJaguar *m_shooterWheel;
	CANJaguar *m_shooterRotate;
	Victor *m_frontBallPickup;
	Victor *m_backBallPickup;
	Victor *m_ferrisWheel;
	RobotDrive *m_robotDrive; // robot drive system
	Compressor *m_compressor;
	DoubleSolenoid *m_ballLiftSolenoid;
	Joystick *m_joystick; // only joystick
	Joystick *m_buttonBox;
	DigitalInput *m_ferrisWheelStop;
	DigitalInput *m_bottomSlot;
	Gyro *m_gyro;
	Ultrasonic *ultra; // The ultra sonic sensor
	DriverStation *m_driverStation;
	double heightOfTriangle;
	
	// Array to hold the targets from the camera
	double results[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS];

public:
	MyRobot(void);

	//Drive left & right motors for 2 seconds then stop
	void Autonomous(void);

	//Runs the motors with arcade steering. 
	void OperatorControl(void);
	
	// Read the buttons of the controllers and respond accordenlly
	void ReadControls(bool ferrisRotate, bool previousSwitchValue);
	
	// Process the camera images
	int ReadCamera(double &heightOfTriangle, double &distanceToTarget,
			double &voltageToDriveShooter, double &valueToRotate);
	
	void AutonomouslyDriveShooter(int targetStatus, double distanceToTarget, double voltageToDriveShooter, double valueToRotate);
	
	// Sort the target array
	void SortTargetArray(double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS]);

	// Send data to the dashboard
	void SendDashboardData();
};

#endif
