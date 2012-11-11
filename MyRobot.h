#ifndef __MyRobot_h__
#define __MyRobot_h__

#include "WPILib.h"
#include "DashboardDataFormat.h"
#include "ImageProcessing.h"
#include "HVA_PIDController.h"
#include "HVA_PIDOutput.cpp"
#include "HVA_PIDSource.cpp"
#include "HVA_PIDJaguarVelocitySource.cpp"
#include "HVA_RobotDrive.h"
#include "CurrentVelocityController.cpp"

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
#define RIGHT_DRIVE_MOTOR                  			3
#define LEFT_DRIVE_MOTOR                   			2
#define SHOOTER_WHEEL_MOTOR                			4
#define SHOOTER_ROTATE_MOTOR               			5
													
/* PWM PORT Defines */                     			
#define BACK_BALL_PICKUP_MOTOR             			1
#define FRONT_BALL_PICKUP_MOTOR            			2
#define FERRIS_WHEEL_MOTOR                 			3
													
/* Analog I/O PORT Defines */              			
#define GYRO_HORIZONTAL_PORT               			1
#define GYRO_VERTICAL_PORT                 			2
													
/* Digital I/O PORT Defines */             			
#define COMPRESSOR_SENSOR                  			1
#define FERRIS_WHEEL_STOP_PORT             			2
#define BOTTOM_SLOT_DETECTOR_PORT          			4
#define FRONT_BRIDGE_WHEEL_LIMIT           			5
#define BACK_BRIDGE_WHEEL_LIMIT            			7
                                           
/* Relay PORT Defines */                   
#define COMPRESSOR_RELAY                   			1
			
/* Solenoid PORT Defines */			
#define BALL_LIFT_FORWARD                  			1
#define BALL_LIFT_REVERSED                 			2
#define BACK_BRIDGE_WHEEL_FORWARD          			3
#define BACK_BRIDGE_WHEEL_REVERSE          			4
#define FRONT_BRIDGE_WHEEL_FORWARD         			6
#define FRONT_BRIDGE_WHEEL_REVERSE         			5

/*************************** Drive Station Defines ****************************/
/* USB PORT Defines */
#define JOYSTICK_PORT                      			1
#define BUTTON_BOX_PORT                    			2
			
/* Joystick 1 Button Defines */			
#define BALL_LIFT_SHOOT                    			1
#define STOP_BALL_PICKUP                   			2
#define DRIVE_WITH_VOLTAGE                 			3
#define DRIVE_WITH_VELOCITY                			4
#define STABILITY_SWITCH_FORWARD_OVERRIDE  			5
#define STABILITY_SWITCH_BACKWARD_OVERRIDE 			6
#define START_BALL_PICKUP                  			7
#define REVERSE_BALL_PICKUP                			8
#define START_FERRIS_WHEEL                 			9
#define REVERSE_FERRIS_WHEEL              			10
#define ROTATE_SHOOTER_COUNTER_CLOCK      			11
#define ROTATE_SHOOTER_CLOCKWISE          			12

/* Button Box Button Defines */
#define RESET_BALL_OFFSET                 			11
#define INCREMENT_BALL_OFFSET              			1
#define DECREMENT_BALL_OFFSET              			2
#define CALCULATE_VOLTAGE                  			4
#define P_INCREMENT                        			8
#define P_DECREMENT                        			9
#define I_INCREMENT                        			6
#define I_DECREMENT                        			7
#define D_INCREMENT                        			3
#define D_DECREMENT                        			5

/* Cypris Button Defines */
#define AUTO_SWITCH_0                      			5
#define AUTO_SWITCH_1                      			3
#define AUTO_SWITCH_2                      			2
#define AUTONOMOUSLY_RUN_SHOOTER           			4
#define AUTO_STRAIGHT                      			7
#define BRIDGE_ASSIST                      			5
#define STABILlTY_SWITCH_UP                			6
#define STABILITY_SWITCH_DOWN              			8

/*************************** Max Accelerations ********************************/
#define MAX_ACCELERATION_ARCADE          			.40
#define MAX_ACCELERATION_DISTANCE        			.40

/************************** Power Defines *************************************/
#define SHOOTER_ROTATE_SPEED             		  0.4f
#define FERRIS_ROTATE_SPEED              		  0.5f
#define MAX_VOLTAGE_PERCENT              		  1.0
#define MAX_VELOCITY                     		  600.0
#define MAX_CURRENT                      		  40

//#define MAX_SPEED_VELOCITY                      60.0

/************************** PID Defines ***************************************/
#define TURN_CONTROLLER_P                		  0.02
#define TURN_CONTROLLER_I                		  0.0001
#define TURN_CONTROLLER_D                		  0.0
#define TURN_CONTROLLER_PERIOD           		  0.005
		                                          
#define ROTATE_CONTROLLER_P              		  4000
#define ROTATE_CONTROLLER_I              		  0.2
#define VALUE_TO_INCRESS_I               		  0.01
#define INCRESSED_I_VALUE                		  1.0
#define ROTATE_CONTROLLER_D              		  10.0
		                                          
#define MOTOR_VELOCITY_P                 		  2.0
#define MOTOR_VELOCITY_I                 		  0.1
#define MOTOR_VELOCITY_D                 		  0.5
		                                          
#define MOTOR_CURRENT_P                  		  0.04
#define MOTOR_CURRENT_I                  		  0.01
#define MOTOR_CURRENT_D                  		  0.0

// TODO - Find the values to use for the velocity to current PID.
#define MOTOR_VELOCTIY_TO_CURRENT_P               0.01
#define MOTOR_VELOCTIY_TO_CURRENT_I               0.0
#define MOTOR_VELOCTIY_TO_CURRENT_D               0.0

/******************************************************************************/

enum ThreeWaySwitchState{kUp, kMiddle, kDown};
enum StabilityWheelState{kFrontDeployed = 1, kBackDeployed = 2, kNeitherDeployed = 0};

class MyRobot: public SimpleRobot
{
private:
   enum {kVelocity, kPercentage, kCurrent} driveSetting;
   enum StabilityWheelState stabilityWheelState;
   enum {kForward, kBackward, kStop} ferrisState;
   
   DashboardDataFormat 			*m_dashboardDataFormat;  // object to send data to the Driver station
   CANJaguar           			*m_rightMotor;           // Right drive motor
   CANJaguar           			*m_leftMotor;            // Left drive motor
   CurrentVelocityController 	*m_rightMotorVelocityPID; // Right Motor controlled by PID
   CurrentVelocityController 	*m_leftMotorVelocityPID;  // left Motor controlled by PID
   CANJaguar           			*m_shooterWheel;         // Shooter fly wheel
   CANJaguar           			*m_shooterRotate;        // Motor used to rotate shooter
   Victor              			*m_frontBallPickup;      // Front ball pick-up motor
   Victor              			*m_backBallPickup;       // Rear ball pick-up moter
   Victor              			*m_ferrisWheel;          // Ball storage wheel
   HVA_RobotDrive      			*m_robotDrive;           // robot drive system
   HVA_RobotDrive      			*m_robotDriveVelocityPID; // robot drive system controlled by PID
   Compressor          			*m_compressor;           // Compressor
   DoubleSolenoid      			*m_ballLiftSolenoid;     // Ball lift
   DoubleSolenoid      			*m_frontBridgeWheel;     // Front stability wheel
   DoubleSolenoid      			*m_backBridgeWheel;      // Back stability wheel
   Joystick            			*m_joystick;             // Only joystick
   Joystick            			*m_buttonBox;            // Box of Buttons
   DigitalInput        			*m_ferrisWheelStop;      // Ferris wheel limit switch
   DigitalInput        			*m_bottomSlot;           // Bottom slot photo gate
   DigitalInput        			*m_frontBridgeWheelLimit;// Limit switch used to see if the arm is up
   DigitalInput        			*m_backBridgeWheelLimit; // Limit switch used to see if the arm is up
   Gyro                			*m_gyroHorizontal;       // Horizontally mounted gyro
   Gyro                			*m_gyroVertical;         // Vertically mounted gyro
   Ultrasonic          			*ultra;                  // The ultra sonic sensor
   DriverStation       			*m_driverStation;        // Driver Station
   HVA_PIDOutput       			*m_pidOutput;            // object that handles the output of the PID controller
   HVA_PIDController   			*m_turnController;       // PID controller used the turn the robot to a given heading
   HVA_PIDJaguarVelocity 		*m_rightJaguarSource;  // Velocity source for jaguar
   HVA_PIDJaguarVelocity 		*m_leftJaguarSource;   // Velocity source for jaguar
   PIDController       			*m_currentVelocityRight; // PID controller to drive a certain velocity using current mode    
   PIDController       			*m_currentVelocityLeft;  // PID controller to drive a certain velocity using current mode           
   double              			 heightOfTriangle;       // Height used for controling shooter velocity
   float               			 m_rotation;             // rotation to drive
   bool                			 readyToFire;            // Is the robot ready to fire

   // Array to hold the targets from the camera
   double results[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS];

   /************************ Basic Robot Control ******************************/
   // Read the buttons of the controllers and respond accordenlly
   void readOperatorControls(void);
   
   // Run the ferrisWheel off of the robot state
   void runFerrisWheel(void);
   
   // Determin the state the ferris wheel should be in.
   void setFerrisWheelState(void);
   
   // Control Drive the Stability Wheels to match the state
   void runStabilityWheels(void);
   
   // Determine the state the wheels need to be in
   void setStabilityWheelState(void);
   
   // Determine the state according to the gyro
   int getStabilityStateFromGyro(void);
   
   /************************ Mode Changing Methods ****************************/
   // Set the drive mode to run of velocity
   void setDriveModeToVelocity(void);
   
   // Set the drive mode to run off voltage percentage
   void setDriveModeToVoltagePercent(void);
   
   // Set the drive mode to current mode
   void setDriveModeToCurrent(void);
   
   /************************ PID Control **************************************/
   // Run the velocity pid
   void runVelocityPID(float setpointRight, float setpointLeft);
   void disableVelocityPID(void);

   /************************ Camera Control ***********************************/
   // Read and process image and control shooter
   bool cameraControl(void);
   
   // Process the camera images
   int readCamera(double &heightOfTriangle, double &distanceToTarget,
         double &voltageToDriveShooter, double &valueToRotate, double &pixelOff);
   
   // Sort the target array
   void sortTargetArray(double targets[NUMBER_OF_TARGETS][NUMBER_OF_TARGET_PARAMETERS]);
   
   // Drive the shooter autonomously
   void autonomouslyDriveShooter(int targetStatus, double distanceToTarget,
                                 double voltageToDriveShooter, double valueToRotate);
   
   /************************ Autonomous Moves *********************************/
   void driveStraightForDistance();
   void driveStraightForDistanceCurrent();
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

public:
   MyRobot(void);

   // Drive left & right motors for 2 seconds then stop
   void Autonomous(void);

   // Runs the motors with arcade steering.
   void OperatorControl(void);
};

#endif
