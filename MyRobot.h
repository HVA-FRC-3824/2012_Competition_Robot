#ifndef __MyRobot_h__
#define __MyRobot_h__

#include "WPILib.h"
#include "DashboardDataFormat.h"
#include "ImageProcessing.h"
#include "HVA_PIDController.h"
#include "HVA_PIDOutput.cpp"
#include "HVA_RobotDrive.h"

/**
 * History:
 *    Custom 1 - First custom made before season.
 *    Custom 2 - First custom for the season. Contorls componates manually
 *    Custom 3 - Added camera tracking and camera shooter control
 *    Custom 4 - Added camera task and automomous modes
 *    Custom 5 - Added ramp control and velocity controls to the library
 */

/*************************** Robot Defines ************************************/
/* Jaguar ID Defines */
#define RIGHT_DRIVE_MOTOR                2
#define LEFT_DRIVE_MOTOR                 3
#define SHOOTER_WHEEL_MOTOR              4
#define SHOOTER_ROTATE_MOTOR             5

/* PWM PORT Defines */
#define BACK_BALL_PICKUP_MOTOR           1
#define FRONT_BALL_PICKUP_MOTOR          2
#define FERRIS_WHEEL_MOTOR               3

/* Analog I/O PORT Defines */
#define GYRO_HORIZONTAL_PORT             1
#define GYRO_VERTICAL_PORT               2

/* Digital I/O PORT Defines */
#define COMPRESSOR_SENSOR                1
#define FERRIS_WHEEL_STOP_PORT           2
#define BOTTOM_SLOT_DETECTOR_PORT        4

/* Relay PORT Defines */
#define COMPRESSOR_RELAY                 1

/* Solenoid PORT Defines */
#define BALL_LIFT_FORWARD                1
#define BALL_LIFT_REVERSED               2
#define FRONT_BRIDGE_WHEEL_FORWARD       3
#define FRONT_BRIDGE_WHEEL_REVERSE       4
#define BACK_BRIDGE_WHEEL_FORWARD        5
#define BACK_BRIDGE_WHEEL_REVERSE        6

/*************************** Drive Station Defines ****************************/
/* USB PORT Defines */
#define JOYSTICK_PORT                    1
#define BUTTON_BOX_PORT                  2

/* Joystick 1 Button Defines */
#define BALL_LIFT_SHOOT                  1
#define DRIVE_WITH_VOLTAGE               3
#define DRIVE_WITH_VELOCITY              4
#define STOP_BALL_PICKUP                 6
#define START_BALL_PICKUP                7
#define REVERSE_BALL_PICKUP              8
#define START_FERRIS_WHEEL               9
#define REVERSE_FERRIS_WHEEL            10
#define ROTATE_SHOOTER_COUNTER_CLOCK    11
#define ROTATE_SHOOTER_CLOCKWISE        12

/* Button Box Button Defines */
#define RESET_BALL_OFFSET               11
#define INCREMENT_BALL_OFFSET            1
#define DECREMENT_BALL_OFFSET            2
#define P_INCREMENT                      8
#define P_DECREMENT                      9
#define I_INCREMENT                      6
#define I_DECREMENT                      7
#define D_INCREMENT                      3
#define D_DECREMENT                      5

/* Cypris Button Defines */
#define MINIBOT_PORT                     4
#define AUTO_STRAIGHT                    7
#define BRIDGE_ASSIST                    5
#define DEPLOY_WHEELS                    2

/******************************************************************************/
class MyRobot: public SimpleRobot
{
private:
   enum {kVelocity, kPercentage} driveSetting;
   
   DashboardDataFormat *m_dashboardDataFormat; // object to send data to the Driver station
   CANJaguar           *m_rightMotor;          // Jaguar motor controllers
   CANJaguar           *m_leftMotor;
   CANJaguar           *m_shooterWheel;
   CANJaguar           *m_shooterRotate;
   Victor              *m_frontBallPickup;
   Victor              *m_backBallPickup;
   Victor              *m_ferrisWheel;
   HVA_RobotDrive      *m_robotDrive;          // robot drive system
   Compressor          *m_compressor;
   DoubleSolenoid      *m_ballLiftSolenoid;
   DoubleSolenoid      *m_frontBridgeWheel;
   DoubleSolenoid      *m_backBridgeWheel;
   Joystick            *m_joystick;            // only joystick
   Joystick            *m_buttonBox;
   DigitalInput        *m_ferrisWheelStop;
   DigitalInput        *m_bottomSlot;
   Gyro                *m_gyroHorizontal;
   Gyro                *m_gyroVerticall;
   Ultrasonic          *ultra;                 // The ultra sonic sensor
   DriverStation       *m_driverStation;
   HVA_PIDOutput       *m_pidOutput;           // object that handles the output of the PID controller
   HVA_PIDController   *m_turnController;      // PID controller used the turn the robot to a given heading
   double               heightOfTriangle;
   float                m_rotation;            // rotation to drive
   bool                 readyToFire;

   // Array to hold the targets from the camera
   double results[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS];

   // Read and process image and control shooter
   bool cameraControl(void);

   // Control the anti tip wheels
   void runTipAvoidence(void);
   
   // Read the buttons of the controllers and respond accordenlly
   void readOperatorControls(void);

   // Process the camera images
   int readCamera(double &heightOfTriangle, double &distanceToTarget,
         double &voltageToDriveShooter, double &valueToRotate, double &pixelOff);

   void autonomouslyDriveShooter(int targetStatus, double distanceToTarget,
                                 double voltageToDriveShooter, double valueToRotate);

   // Sort the target array
   void sortTargetArray(double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS]);

   // Send data to the dashboard
   void sendDashboardData();

public:
   MyRobot(void);

   // Drive left & right motors for 2 seconds then stop
   void Autonomous(void);

   // Runs the motors with arcade steering.
   void OperatorControl(void);
};

#endif
