#ifndef __MyRobot_h__
#define __MyRobot_h__

#include "WPILib.h"
#include "DashboardDataFormat.h"
#include "ImageProcessing.h"
#include "HVA_PIDController.h"
#include "HVA_PIDOutput.h"
#include "HVA_PIDSource.h"
#include "HVA_PIDJaguarVelocitySource.h"
#include "HVA_RobotDrive.h"
#include "HVA_CANJaguar.h"
#include "CurrentVelocityController.cpp"
#include "HVA_Victor.h"
#include "ShooterEncoder.h"
#include "AnalogSonar.h"

/**
 * History:
 *    Custom 1 - First custom made before season. (Outdated)
 *    Custom 2 - First custom for the season. Contorls componates manually. (Outdated)
 *    Custom 3 - Added camera tracking and camera shooter control. (Outdated)
 *    Custom 4 - Added camera task and automomous modes. (Outdated)
 *    Custom 5 - Added ramp control and added velocity controls to the library. (Outdated)
 *    Custom 6 - Added autonomous code and revised the ramp control. (Outdated)
 *    Custom 7 - Added new equation for voltage. (Stable)
 *    Custom 8 - Added balencing code to the autonomous. (Testing/Stable)
 *
 *    note:
 *       174 inch for 10 rev
 */

/*************************** Robot Defines ************************************/
/* Jaguar ID Defines */
#define SHOOTER_WHEEL_MOTOR                        4
#define SHOOTER_ROTATE_MOTOR                       5

/* PWM PORT Defines */
#define BACK_BALL_PICKUP_MOTOR                     1
#define FRONT_BALL_PICKUP_MOTOR                    2
#define FERRIS_WHEEL_MOTOR                         3
#define RIGHT_DRIVE_MOTOR                          4
#define LEFT_DRIVE_MOTOR                           5

/* Analog I/O PORT Defines */
#define GYRO_HORIZONTAL_PORT                       1
#define GYRO_VERTICAL_PORT                         2
#define BOTTOM_BALL_SENSOR                         3
#define ANALOG_ULTRASONIC                          4

/* Digital I/O PORT Defines */
#define COMPRESSOR_SENSOR                          1
#define FERRIS_WHEEL_STOP_PORT                     2
#define LEFT_ENCODER_A                             3
#define LEFT_ENCODER_B                             4
#define RIGHT_ENCODER_A                            5
#define RIGHT_ENCODER_B                            6
#define SHOOTER_PHOTO_GATE                         7
#define ULTRASONIC_PING_OUTPUT                     8
#define ULTRASONIC_ECHO_INPUT                      9

/* Relay PORT Defines */
#define COMPRESSOR_RELAY                           1

/* Solenoid PORT Defines */
#define BALL_LIFT_FORWARD                          1
#define BALL_LIFT_REVERSED                         2
#define RAMP_EXTENDER_FORWARD                      3
#define RAMP_EXTENDER_REVERSE                      4
#define RAMP_PUSHER_FORWARD                        6
#define RAMP_PUSHER_REVERSE                        5

/*************************** Drive Station Defines ****************************/
/* USB PORT Defines */
#define JOYSTICK_PORT                              1
#define BUTTON_BOX_PORT                            2
#define BUTTON_BOX_PORT2                           3

/* Joystick 1 */
#define DRIVE_WITH_VOLTAGE                         3
#define DRIVE_WITH_VELOCITY                        4
#define INCREMENT_POSITION                         7
#define DECREMENT_POSITION                         8

/* Button Box Button Defines */
#define AUTO_SWITCH_0                              1
#define AUTO_SWITCH_1                              2
#define AUTO_SWITCH_2                              3

/* Cypress Digital I/O Defines */
#define STABILITY_SWITCH_UP                        1
#define STABILITY_SWITCH_DOWN                      2
#define STABILITY_SWITCH_FORWARD_OVERRIDE          3
#define STABILITY_SWITCH_BACKWARD_OVERRIDE         4
#define SHOOTER_WHEEL_OVERRIDE_MODE                5
#define SHOOTER_WHEEL_OVERRIDE                     6
#define ROTATION_OVERRIDE                          7
#define START_BALL_PICKUP                          8
#define REVERSE_BALL_PICKUP                        9
#define AUTONOMOUSLY_RUN_SHOOTER                  10
#define BALL_LIFT_SHOOT                           11
#define START_FERRIS_WHEEL                        12
#define REVERSE_FERRIS_WHEEL                      13
#define READY_TO_FIRE_LED                         14

/* Cypress Analog Defines */
#define SHOOTER_VELOCITY_POT                       1
#define SHOOTER_ROTATION_POT                       2

/*************************** Max Accelerations ********************************/
#define MAX_ACCELERATION_ARCADE                  .40
#define MAX_ACCELERATION_DISTANCE                .25

/************************** Power Defines *************************************/
#define SHOOTER_ROTATE_SPEED                     0.4f
#define FERRIS_ROTATE_SPEED                      0.5f
#define MAX_VOLTAGE_PERCENT                      1.0
#define MAX_VELOCITY                            12.0
#define BALL_PICKUP_SPEED                        1.0f
#define MIN_SHOOTER_SPEED_PERCENT                0.0f
#define MAX_SHOOTER_SPEED_PERCENT                1.0f
#define MAX_ROBOT_VOLTAGE                       12.0
#define DEFAULT_AUTONOMOUS_VOLTAGE               7.0
#define MIN_SHOOTER_VOLTAGE_AUTONOMOUS           6.0
#define MAX_SHOOTER_VOLTAGE_AUTONOMOUS           7.75

/************************** PID Defines ***************************************/
#define TURN_CONTROLLER_P                        0.02
#define TURN_CONTROLLER_I                        0.0001
#define TURN_CONTROLLER_D                        0.0
#define TURN_CONTROLLER_PERIOD                   0.005

#define ROTATE_CONTROLLER_P                     4000
#define ROTATE_CONTROLLER_I                      0.2
#define VALUE_TO_INCRESS_I                       0.01
#define INCRESSED_I_VALUE                        1.0
#define ROTATE_CONTROLLER_D                     10.0

#define SHOOTER_VELOCITY_P                       1.0
#define SHOOTER_VELOCITY_I                       0.01
#define SHOOTER_VELOCITY_D                       0.0

/*************************** Autonomous Defines ************************************/
#define DISTANCE_TO_CENTER_BRIDGE           5
#define DISTANCE_TO_ALLIANCE_BRIDGE         5
#define TIME_TO_DUMP_BRIDGE                 2
#define DISTANCE_TO_SHOOTING_POSITION       6.2

/*************************** Other Defines ************************************/
#define MAXIMUM_ROTATION_OF_SHOOTER              0.3
#define MINIMUM_ROTATION_OF_SHOOTER         -MAXIMUM_ROTATION_OF_SHOOTER

#define MIN_POT_VALUE                            0.0
#define MAX_POT_VALUE                            3.3
#define MID_POT_VALUE                       ((MAX_POT_VALUE + MIN_POT_VALUE) / 2.0)

#define MAX_SHOOTER_VOLTAGE_ADJUSTMENT           1.0

#define TIME_TILL_INTERRUPT_ENABLE               0.5

#define RAMP_PUSHER_DELAY                        1.0

#define DISTANCE_TO_RAMP                         8

/******************************************************************************/

typedef enum

{
   kUp, kMiddle, kDown
} ThreeWaySwitchState;

typedef enum {
   kHome, kPusherExtended, kPusherRetracted
} RampState;

enum ControlState
{
   kAutonomous, kTeleoperated
};

class MyRobot: public SimpleRobot
{
public:
   static void ferrisHandler(uint32_t mask, void *param)
   {
      MyRobot *robot = (MyRobot*)param;
      robot->runFerrisWheel(kStop);
      robot->m_ferrisWheelStop->DisableInterrupts();
   }

   MyRobot(void);

   // Drive left & right motors for 2 seconds then stop
   void Autonomous(void);

   // Runs the motors with arcade steering.
   void OperatorControl(void);

private:
   enum FerrisState
   {
      kForward, kBackward, kStop
   } m_ferrisState;

   RampState    m_rampState;
   ControlState m_shooterRotationControlState;

   DashboardDataFormat       *m_dashboardDataFormat; // object to send data to the Driver station
   HVA_Victor                *m_rightMotor; // Right drive motor
   HVA_Victor                *m_leftMotor; // Left drive motor
   CurrentVelocityController *m_rightMotorVelocityPID; // Right Motor controlled by PID
   CurrentVelocityController *m_leftMotorVelocityPID; // left Motor controlled by PID
   CANJaguar                 *m_shooterWheel; // Shooter fly wheel
   CANJaguar                 *m_shooterRotate; // Motor used to rotate shooter
   Victor                    *m_frontBallPickup; // Front ball pick-up motor
   Victor                    *m_backBallPickup; // Rear ball pick-up moter
   Victor                    *m_ferrisWheel; // Ball storage wheel
   HVA_RobotDrive            *m_robotDrive; // robot drive system
   Compressor                *m_compressor; // Compressor
   DoubleSolenoid            *m_ballLiftSolenoid; // Ball lift
   DoubleSolenoid            *m_rampExtender;  // Horizontal ramp pusherel
   DoubleSolenoid            *m_rampPusher;    // Vertical ramp pusher
   Joystick                  *m_joystick; // Only joystick
   Joystick                  *m_buttonBox; // Box of Buttons
   //Joystick                  *m_buttonBox2; // Box of Buttons
   AnalogTrigger             *m_bottomBallSensor;
   DigitalInput              *m_ferrisWheelStop; // Ferris wheel limit switch
   tInterruptHandler         *handler;
   Gyro                      *m_gyroHorizontal; // Horizontally mounted gyro
   Gyro                      *m_gyroVertical; // Vertically mounted gyro
   AnalogSonar               *m_frontUltra; // The front sonic sensor
   Ultrasonic                *m_backUltra; // The back sonic sensor
   DriverStation             *m_driverStation; // Driver Station
   DriverStationEnhancedIO   *m_driverStationEnhancedIO; // Enhanced Driver Station
   HVA_PIDOutput             *m_pidOutput; // object that handles the output of the PID controller

   double heightOfTriangle; // Height used for controling shooter velocity
   float m_rotation;        // rotation to drive
   bool  readyToFire;       // Is the robot ready to fire

   // Array to hold the targets from the camera
   double results[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS];

   /************************ Basic Robot Control ******************************/
   // Read the buttons of the controllers and respond accordenlly
   void readOperatorControls(void);

   // Run the ferrisWheel off of the robot state
   void runFerrisWheel(FerrisState state);

   // Determin the state the ferris wheel should be in.
   void runFerrisWheelFromControls(void);

   /************************ Mode Changing Methods ****************************/
   // Set the drive mode to run of velocity
   void setDriveModeToVelocity(void);

   // Set the drive mode to run off voltage percentage
   void setDriveModeToVoltagePercent(void);

   // Set the drive mode to run off position
   void setDriveModeToPosition(void);

   /************************ PID Control **************************************/
   // Run the velocity pid
   void runVelocityPID(float setpointRight, float setpointLeft);
   void disableVelocityPID(void);

   /************************ Camera Control ***********************************/
   // Read and process image and control shooter
   bool cameraControl(void);

   // Process the camera images
   int readCamera(double &heightOfTriangle, double &distanceToTarget,
                  double &voltageToDriveShooter, double &valueToRotate,
                  double &pixelOff);

   // Sort the target array
   void sortTargetArray(
         double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS]);

   // Drive the shooter autonomously
   void autonomouslyDriveShooter(int targetStatus, double distanceToTarget,
         double voltageToDriveShooter, double valueToRotate);

   /************************ Autonomous Moves *********************************/
   void driveToShootingPosition(float distance = DISTANCE_TO_SHOOTING_POSITION);
   void driveStraightForwardAndBackForDistance();
   void shootOneBall();
   void shootTwoBalls();
   void driveAndShootTwo();
   void balance();
   void newBalance();
   void driveToCenterBridge();
   void driveToAllianceBridge();
   void driveFromCenterBridge();
   void driveFromAllianceBridge();
   void dumpBridge();

   /************************ Data Sending *************************************/
   // Send data to the dashboard
   void sendDashboardData();
};

#endif
