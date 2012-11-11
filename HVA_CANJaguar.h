#ifndef HVA_CANJaguar_H_
#define HVA_CANJaguar_H_

#include "WPILib.h"
#include "HVA_PIDController.h"
#include "HVA_PIDOutput.h"
#include "HVA_PIDSource.h"
#include "HVA_PIDJaguarVelocitySource.h"

#define MOTOR_VELOCTIY_TO_CURRENT_P            0.15
#define MOTOR_VELOCTIY_TO_CURRENT_I            0.01
#define MOTOR_VELOCTIY_TO_CURRENT_D            0.0

//------------------------------------------------------------------------------
// Class to handle the custom HVA CAN Jaguar speed controller
//------------------------------------------------------------------------------
class HVA_CANJaguar: public CANJaguar
{
public:
   typedef enum { kPercentVbus = 0, kCurrent = 1, kSpeed = 2, kPosition = 3,
                  kVoltage = 4, kCurrentSpeed = 5 } HVA_ControlMode;

   HVA_ControlMode GetControlMode();

   HVA_CANJaguar(UINT8 deviceNumber, HVA_ControlMode controlMode = kPercentVbus);

   void ChangeControlMode(HVA_ControlMode controlMode);

   void EnableControl(double encoderInitialPosition = 0.0);
   void DisableControl();

   virtual void Set(float value, UINT8 syncGroup = 0);

private:
   float                        m_outputFromPID;
   PIDController               *m_velocityController;

   HVA_PIDJaguarVelocitySource *m_velocitySource;
   HVA_ControlMode              m_HVA_controlMode;
   HVA_PIDOutput               *m_PIDOutput;

   void enableVelocityPID();
   void disableVelocityPID();
};

#endif
