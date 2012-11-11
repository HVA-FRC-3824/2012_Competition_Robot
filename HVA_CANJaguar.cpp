#include "HVA_CANJaguar.h"

HVA_CANJaguar::HVA_CANJaguar(UINT8 deviceNumber, HVA_ControlMode controlMode) :
   CANJaguar(deviceNumber, (ControlMode)kVoltage)
{
   m_HVA_controlMode = controlMode;
   
   if (controlMode != kVoltage)
   {
      ChangeControlMode(controlMode);
   }

   // Create the PID
   m_velocitySource = new HVA_PIDJaguarVelocitySource(this);
   m_PIDOutput = new HVA_PIDOutput(&m_outputFromPID);
   m_velocityController = new PIDController(MOTOR_VELOCTIY_TO_CURRENT_P,
         MOTOR_VELOCTIY_TO_CURRENT_I, MOTOR_VELOCTIY_TO_CURRENT_D,
         m_velocitySource, m_PIDOutput);

   // Setup the PID Controller
   m_velocityController->SetOutputRange(-40, 40);
   m_velocityController->Disable();
}

void HVA_CANJaguar::ChangeControlMode(HVA_ControlMode controlMode)
{
   m_HVA_controlMode = controlMode;
   printf("In Change hva_controlmode:%i", m_HVA_controlMode);
   if(controlMode == kCurrentSpeed)
   {
      controlMode = kCurrent;
   }
   CANJaguar::ChangeControlMode((ControlMode) controlMode);
}

HVA_CANJaguar::HVA_ControlMode HVA_CANJaguar::GetControlMode()
{
   return m_HVA_controlMode;
}

void HVA_CANJaguar::Set(float value, UINT8 syncGroup)
{
   if (m_HVA_controlMode == kCurrentSpeed)
   {  
      static double time = Timer::GetFPGATimestamp();
      static int i = 1;
//      printf("Actual Set:%f\n", this->Get());
//      printf("Jag Mode: %i\n", CANJaguar::GetControlMode());
//      printf("Speed Reference:%i\n", this->GetSpeedReference());
//      printf("Position:%f\n", this->GetPosition());
//      printf("Current Velocity: %f\n", this->GetSpeed());
//      printf("Velocity Value: %f\n", value);
      m_velocityController->SetSetpoint(value);
//      printf("Current Set: %f\n", m_outputFromPID);
      if (Timer::GetFPGATimestamp()-time > .2 && i == 1)
      {
      printf("Error: %f\n", value - this->GetSpeed());
      printf("Current Set: %f\n\n", m_outputFromPID);
      time = Timer::GetFPGATimestamp();
      i = 0;
      }
      else if (i == 0)
      {
         //printf("Error: %f\n", value - this->GetSpeed());
         i = 1;
      }
      CANJaguar::Set(m_outputFromPID);
//      printf("Actual Set:%f\n\n", this->Get());
   }
   else
   {
      //printf("You are lost\n");
      CANJaguar::Set(value, syncGroup);
   }
}

void HVA_CANJaguar::enableVelocityPID()
{
   m_velocityController->Enable();
}

void HVA_CANJaguar::disableVelocityPID()
{
   m_velocityController->Disable();
}

void HVA_CANJaguar::EnableControl(double encoderInitialPosition)
{
   if (m_HVA_controlMode == kCurrentSpeed)
   {
      m_velocityController->Reset();
      enableVelocityPID();
   }
   CANJaguar::EnableControl(encoderInitialPosition);
}

void HVA_CANJaguar::DisableControl()
{
   if (m_HVA_controlMode == kCurrentSpeed)
   {
      disableVelocityPID();
   }
   CANJaguar::DisableControl();
}
