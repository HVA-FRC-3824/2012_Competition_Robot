//---------------------------------------------------------------------------------
// Source Filename:  CameraTracking.cpp
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

#include <Timer.h>

#include "Robot.h"

//------------------------------------------------------------------------------
//                           Local Defines
//------------------------------------------------------------------------------

#define PID_ENABLER              7

#define PIXIAL_SCALE          0.08

//------------------------------------------------------------------------------
//                           Local Variables
//------------------------------------------------------------------------------

// These parameters set ellipse finding in the NI imaq (Image Aquisition) library.
// Refer to the CVI Function Reference PDF document installed with LabVIEW for
// additional information.
static EllipseDescriptor ellipseDescriptor =
{ 20, // minMajorRadius
		200, // maxMajorRadius
		20, // minMinorRadius
		200 // maxMinorRadius
		};

static CurveOptions curveOptions =
{ IMAQ_NORMAL_IMAGE, // extractionMode
		40, // threshold
		IMAQ_NORMAL, // filterSize
		25, // minLength
		15, // rowStepSize
		15, // columnStepSize
		10, // maxEndPointGap
		1, // onlyClosed
		0 // subpixelAccuracy
		};

static ShapeDetectionOptions shapeOptions =
{ IMAQ_GEOMETRIC_MATCH_SHIFT_INVARIANT, // mode
		NULL, // angle ranges
		0, // num angle ranges
		{ 75, 125 }, // scale range
		500 // minMatchScore
		};

//---------------------------------------------------------------------------------
// Routine Summary:
//
//
// Description:
//
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void Robot::CameraTracking()
{
	Timer *CamerTimer = new Timer();

	// Create and set up a camera instance. first wait for the camera to start
	// if the robot was just powered on. This gives the camera time to boot.
	printf("Getting camera instance\n");
	AxisCamera &camera = AxisCamera::GetInstance();

	printf("Setting camera parameters\n");
	camera.WriteResolution(AxisCamera::kResolution_320x240);
	camera.WriteCompression(20);
	camera.WriteBrightness(50);

	// set MotorSafety expiration
	m_driveSystem->SetExpiration(1.0);
	m_driveSystem->ResetPID();

	// reset the gyro PID and start with zero rotation
	m_xDistanceFromCenter = 0;

	// start the camera timer
	CamerTimer->Start();

	// if there's a fresh and we're at the previous target heading then
	// get a camera image and process it
	while (IsOperatorControl())
	{
		// Run the acaccelerometer
		RunAccelerometer();
		
		if (m_driverStation->GetDigitalIn(PID_ENABLER) == 0)
			m_driveSystem->DisablePID();
		else
			m_driveSystem->EnablePID();

		if (camera.IsFreshImage())
		{
			// get the camera image
			HSLImage *image = camera.GetImage();

			// get the luminance plane only for the image to make the code
			// insensitive to lighting conditions.
			MonoImage *luminancePlane = image->GetLuminancePlane();
			vector<EllipseMatch> *results = luminancePlane->DetectEllipses(
					&ellipseDescriptor, &curveOptions, &shapeOptions, NULL);
			delete luminancePlane;
			delete image;

			if (results->size() == 0)
			{
				//printf("No target found\n");
			} else
			{
				// ensure the timer has expired to only send data 10 times per second
				if (CamerTimer->Get() > 1.5)
				{
					// reset the LCD timer
					CamerTimer->Reset();

					// create a list of targets corresponding to each ellipse found
					// in the image.
					//for (unsigned i = 0; i < results->size(); i++)
					for (unsigned i = 0; i < 1; i++)
					{
						EllipseMatch e = results->at(i);
						if (i == 0)
						{
							m_xDistanceFromCenter = e.position.x - (320 / 2);

							printf("Score: %f  Distance: %f  Angle: %f\n",
									(float) e.score, m_xDistanceFromCenter,
									m_xDistanceFromCenter * PIXIAL_SCALE);

							m_driveSystem->DriveToRelativeAngle(
									m_xDistanceFromCenter * PIXIAL_SCALE);

							m_dashboardDataSender->SendVisionData(
									(float) e.score, e.position.x,
									e.position.y, (float) e.minorRadius,
									(float) e.majorRadius);
						}
					}
				}
			}

			delete results;

			// send the dashboard data
			SendDashboardData();
		}
	}

	m_driveSystem->DisablePID();
	delete CamerTimer;
}
