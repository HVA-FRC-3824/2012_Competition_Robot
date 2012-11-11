//---------------------------------------------------------------------------------
// Source Filename:  DashboardSender.h
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#ifndef __DashboardDataSender_h__
#define __DashboardDataSender_h__

#include "WPILib.h"

#include <Timer.h>

//---------------------------------------------------------------------------------
// This class is one way to organize the data to send to the dashboard.
//---------------------------------------------------------------------------------

class DashboardDataSender
{
public:
	DashboardDataSender();

	void SendLCDData(double gyro, float rotation, float P, float I, float D);
	void SendIOPortData(int gyro, float x_accel, float y_accel, float z_accel,
			float x_velocity, float y_velocity, float z_velocity,
			float x_dist, float y_dist, float z_dist,
			float xDistanceFromCenter, float rotationValue);
	void SendVisionData(float score, float position_x, float position_y,
			float minorRadius, float majorRadius);

private:
	DriverStationLCD *m_driverStationLCD; // Text box on driver station
	Timer *m_LCDTimer;
	Timer *m_visionTimer;
	Timer *m_IOTimer;
};

#endif // __DashboardDataFormat_h__
