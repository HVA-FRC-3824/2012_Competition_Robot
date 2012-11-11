//---------------------------------------------------------------------------------
// Source Filename:  DashboardDataSender.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#include "DashboardDataSender.h"

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    DashboardDataSender();
//
// Description:
//
//    Send data to the dashboard. This class sends two types of data to the
// dashboard program:
//
//    1. Data representing all the ports on the robot
//    2. Camera tracking data so the dashboard can annotate the video stream with
//       target information.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
DashboardDataSender::DashboardDataSender()
{
	m_driverStationLCD = DriverStationLCD::GetInstance();

	// these timers make sure that the data is not sent to the dashboard more
	// than 10 times per second for efficiency.
	m_LCDTimer = new Timer();
	m_IOTimer = new Timer();
	m_visionTimer = new Timer();

	m_LCDTimer->Start();
	m_IOTimer->Start();
	m_visionTimer->Start();
}

//---------------------------------------------------------------------------------
// Send the text box data.
//---------------------------------------------------------------------------------
void DashboardDataSender::SendLCDData(double gyro, float rotation, float P,
		float I, float D)
{
	// ensure the timer has expired to only send data 10 times per second
	if (m_LCDTimer->Get() < 0.1)
		return;

	// reset the LCD timer
	m_LCDTimer->Reset();

	// print gyro angle to text box on driver station
	m_driverStationLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Gyro: %7.4f",
			gyro);
	m_driverStationLCD->Printf(DriverStationLCD::kUser_Line2, 1,
			"Rotation: %7.4f", rotation);
	m_driverStationLCD->Printf(DriverStationLCD::kUser_Line3, 1, "P: %7.4f", P);
	m_driverStationLCD->Printf(DriverStationLCD::kUser_Line4, 1, "I: %f", I);
	m_driverStationLCD->Printf(DriverStationLCD::kUser_Line5, 1, "D: %f", D);
	m_driverStationLCD->UpdateLCD();
}

//---------------------------------------------------------------------------------
// Send the vision tracking data.
//
// Sends the vision information to the dashboard so that the images will be
// annotated and the graphs will operate.
//---------------------------------------------------------------------------------
void DashboardDataSender::SendVisionData(float score, float position_x,
		float position_y, float minorRadius, float majorRadius)
{
	// ensure the timer has expired to only send data 10 times per second
	if (m_visionTimer->Get() < 0.1)
		return;

	// reset the vision timer
	m_visionTimer->Reset();

	// get the high priority dashboard packer
	Dashboard &dash =
			DriverStation::GetInstance()->GetHighPriorityDashboardPacker();

	dash.AddCluster();
	{
		dash.AddFloat(score);
		dash.AddFloat(position_x);
		dash.AddFloat(position_y);
		dash.AddFloat(minorRadius);
		dash.AddFloat(majorRadius);
	}

	dash.FinalizeCluster();
	dash.Finalize();
}

//---------------------------------------------------------------------------------
// Send IO port data to the dashboard.
//
// Send data representing the output of all the IO ports on the cRIO to the
// dashboard.
//---------------------------------------------------------------------------------

void DashboardDataSender::SendIOPortData(int gyro, float x_accel,
		float y_accel, float z_accel, float x_velocity, float y_velocity,
		float z_velocity, float x_dist, float y_dist, float z_dist,
		float xDistanceFromCenter, float rotationValue)
{
	// ensure the timer has expired to only send data 10 times per second
	if (m_IOTimer->Get() < 0.1)
		return;

	// reset the I/O timer
	m_IOTimer->Reset();

	// get the low priority dashboard packer
	Dashboard &dash =
			DriverStation::GetInstance()->GetLowPriorityDashboardPacker();

	//******** <GOOD STUFF> **********//
	dash.AddCluster();
	{
		dash.AddCluster(); // analog modules
		{
			dash.AddCluster();
			{
				for (int i = 1; i <= 8; i++)
					dash.AddFloat(
							(float) AnalogModule::GetInstance(1)->GetAverageVoltage(
									i));
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{
				for (int i = 1; i <= 8; i++)
					dash.AddFloat(0.0f);
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		dash.AddCluster(); // digital modules
		{
			dash.AddCluster();
			{
				dash.AddCluster();
				{
					int module = 1;
					dash.AddU8(
							DigitalModule::GetInstance(module)->GetRelayForward());
					dash.AddU8(
							DigitalModule::GetInstance(module)->GetRelayReverse());
					dash.AddU16(
							(short) DigitalModule::GetInstance(module)->GetDIO());
					dash.AddU16(
							(short) DigitalModule::GetInstance(module)->GetDIODirection());

					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++)
							dash.AddU8(
									(unsigned char) DigitalModule::GetInstance(
											module)->GetPWM(i));
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{
				dash.AddCluster();
				{
					dash.AddU8(0);
					dash.AddU8(0);
					dash.AddU16(0);
					dash.AddU16(0);

					dash.AddCluster();
					{
						for (int i = 1; i <= 10; i++)
							dash.AddU8(0);
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		dash.AddU16(gyro);
		dash.AddCluster();
		{
			dash.AddCluster();
			{
				dash.AddFloat(x_accel);
				dash.AddFloat(y_accel);
				dash.AddFloat(z_accel);
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{
				dash.AddFloat(x_velocity);
				dash.AddFloat(y_velocity);
				dash.AddFloat(z_velocity);
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{
				dash.AddFloat(x_dist);
				dash.AddFloat(y_dist);
				dash.AddFloat(z_dist);
			}
			dash.FinalizeCluster();
		}
		dash.FinalizeCluster();

		dash.AddFloat(xDistanceFromCenter);
		dash.AddFloat(rotationValue);
	}
	dash.FinalizeCluster();
	dash.Finalize();
	//******** </GOOD STUFF> **********//

	//******** <BAD STUFF> *********//
//	dash.AddCluster();
//	{
//		dash.AddCluster();
//		{
//			dash.AddFloat(x_accel);
//			dash.AddFloat(y_accel);
//			dash.AddFloat(z_accel);
//		}
//		dash.FinalizeCluster();
//
//		dash.AddCluster();
//		{
//			dash.AddFloat(x_velocity);
//			dash.AddFloat(y_velocity);
//			dash.AddFloat(z_velocity);
//		}
//		dash.FinalizeCluster();
//
//		dash.AddCluster();
//		{
//			dash.AddFloat(x_dist);
//			dash.AddFloat(y_dist);
//			dash.AddFloat(z_dist);
//		}
//		dash.FinalizeCluster();
//	}
//
//	dash.FinalizeCluster();
//	dash.Finalize();
	//******** </BAD STUFF> *********//

}
