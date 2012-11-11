//---------------------------------------------------------------------------------
// Source Filename:  Robot.h
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Class definition for the Custom Robot Code.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#ifndef __MyRobot_h__
#define __MyRobot_h__

#include "WPILib.h"
#include "ADXL345_I2C.h"
#include "Error.h"

#include "DashboardDataSender.h"
#include "DriveSystem.h"

//---------------------------------------------------------------------------------
// Class Definition:
//
//    Class Robot
//
// Robot constructor based on the SimpleRobot class.
//
//    ***** Module 1: Analog Breakout *****
//    Analog Input 01: Gyro
//    Analog Input 02: (not used)
//    Analog Input 03: (not used)
//    Analog Input 04: (not used)
//    Analog Input 05: (not used)
//    Analog Input 06: (not used)
//    Analog Input 07: (not used)
//    Analog Input 08: (not used)
//
//    ***** Module 2: Digital I/O *****
//    PWM 01: Rear Left Motor
//    PWM 02: Rear Right Motor
//    PWM 03: Front Left Motor
//    PWM 04: Front Right Motor
//    PWM 05: (not used)
//    PWM 06: (not used)
//    PWM 07: (not used)
//    PWM 08: (not used)
//    PWM 09: (not used)
//    PWM 10: (not used)
//
//    ***** Module 3: Solonoid *****
//    Digital I/O 01: Conpressor sensor
//    Digital I/O 02: (not used)
//    Digital I/O 03: (not used)
//    Digital I/O 04: (not used)
//    Digital I/O 05: (not used)
//    Digital I/O 06: (not used)
//    Digital I/O 07: (not used)
//    Digital I/O 08: (not used)
//
//------------------------------------------------------------------------------
class Robot : public SimpleRobot
{
public:
   Robot(void);

   void RobotInit();
   void Disabled();

   // override virtual routines for the game periods
   void Autonomous(void);
   void OperatorControl(void);

   // Find the offset for the accelerometer
   void CalcAccelerometerOffset();
   
   // Run the summation of the gyro
   void RunAccelerometer();
   
   // drive based on camera tracking
   void CameraTracking();

   // test the gyro control of robot rotation
   void GyroTest(void);

   void PneumaticControl();

   void DriveControl(void);
   void SendDashboardData();

   /****** DEBUG *****/
   float P;
   float I;
   float D;
   /******************/
   
private:
   DriveSystem         *m_driveSystem;         // drive system for the robot
   Joystick            *m_joystick;            // drive control joystick
   Joystick            *m_armBox;              // arm control box
   Gyro                *m_gyro;                // gyro
   DriverStation       *m_driverStation;       // driver station instance for driver station input
   Compressor          *m_compressor;          // compressor
   ADXL345_I2C         *m_accelerometer;       // accelerometer
   DashboardDataSender *m_dashboardDataSender; // object to send data to the Driver station
   KinectStick         *m_leftArm;             // Kinect left arm
   KinectStick         *m_rightArm;            // Kinect right arm

   bool                 m_triggerPressed;      // state of the trigger
   float                m_xDistanceFromCenter; //
   
   /*      <PRIVATES FOR ACCCELERATION>      */
   ADXL345_I2C::AllAxes allAxes;
   float                m_XaccelerationOffset;
   float  				m_YaccelerationOffset;
   float  				m_ZaccelerationOffset;
   
   float                m_Xacceleration;
   float  				m_Yacceleration;
   float  				m_Zacceleration;
   
   float				m_Xvelocity;
   float				m_Yvelocity;
   float				m_Zvelocity;
   
   float				m_Xdistance;
   float				m_Ydistance;
   float				m_Zdistance;
   /*      </PRIVATES FOR ACCCELERATION>     */ 
};
#endif
