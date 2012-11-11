//---------------------------------------------------------------------------------
// Source Filename:  GyroTest.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#include "WPILib.h"

#include "Robot.h"

//---------------------------------------------------------------------------------
//                           Local Defines
//---------------------------------------------------------------------------------

#define PIXIAL_SCALE          0.15

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    GyroTest(void);
//
// Description:
//
//    Routine to test the gyro for rotation PID control. The routine is called from
// the OperatorControl routine in teleoperated mode. The routine allows setting the
// P, I and D gains through the joystick.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
// Function to run the robot during autonomous period
//---------------------------------------------------------------------------------
void Robot::GyroTest(void)
{
	bool buttonPressed = false;

	// loop getting images from the camera and finding targets
	printf("Starting operator control loop\n");

	/********************* DEBUG **********************************************/
	P = m_driveSystem->m_gyroPID->GetP();
	I = m_driveSystem->m_gyroPID->GetI();
	D = m_driveSystem->m_gyroPID->GetD();
	/**************************************************************************/

	// enable the PID to drive the robot
	m_driveSystem->EnablePID();

	printf("**** Drive Enabled\n");

	// stay in operator control while enabled
	while (IsOperatorControl())
	{
		// Run the acaccelerometer
		RunAccelerometer();

		/********************* DEBUG ******************************************/
		if (m_joystick->GetTrigger() == true)
		{
			m_driveSystem->DriveToAngle(m_joystick->GetTwist() * 360);
			buttonPressed = true;
		}

		if ((m_joystick->GetRawButton(7) == true) && (buttonPressed == false))
		{
			P *= 1.5;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if ((m_joystick->GetRawButton(8) == true) && (buttonPressed
				== false))
		{
			P /= 1.5;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if ((m_joystick->GetRawButton(9) == true) && (buttonPressed
				== false))
		{
			I *= 1.5;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if ((m_joystick->GetRawButton(10) == true) && (buttonPressed
				== false))
		{
			I /= 1.5;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if ((m_joystick->GetRawButton(11) == true) && (buttonPressed
				== false))
		{
			D *= 2;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if ((m_joystick->GetRawButton(12) == true) && (buttonPressed
				== false))
		{
			D /= 2;
			m_driveSystem->m_gyroPID->SetPID(P, I, D);
			buttonPressed = true;
		}

		else if (m_joystick->GetTop() == true)
		{
			buttonPressed = false;
		}
		/**********************************************************************/

		// send the dashboard data
		SendDashboardData();
	}
}
