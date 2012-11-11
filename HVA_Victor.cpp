#include "HVA_Victor.h"

HVA_Victor::HVA_Victor(UINT32 channel, Encoder* encoder,
      ControlMode controlMode) :
   Victor(channel)
{
   m_encoder = encoder;
   m_encoder->SetDistancePerPulse(FEET_PER_PULSE);
   m_encoder->Start();
   m_PIDController = new PIDController(DEFAULT_P, DEFAULT_I, DEFAULT_D,
       new EncoderVelocitySource(m_encoder), new EncoderVelocityOutput(this));
   
   ChangeControlMode(controlMode);

   
   /************************ Setup ********************************************/
   m_PIDController->SetOutputRange(-1, 1);
   m_PIDController->Disable();
}

/**
 * Change the control mode the victors are running in
 */
void HVA_Victor::ChangeControlMode(ControlMode controlMode)
{
   DisableControl();
   m_controlMode = controlMode;
}

/**
 * Get the current control mode
 */
HVA_Victor::ControlMode HVA_Victor::GetControlMode()
{
   return m_controlMode;
}

/**
 * Set the value of the motor
 */
void HVA_Victor::Set(float value, UINT8 syncGroup)
{
   if (m_controlMode == kPercentVbus)
   {
      if (m_controlEnabled == true)
      {
         Victor::Set(value, syncGroup);
      }
      else // Control not enabled
      {
         Victor::Set(0.0f, syncGroup);
      }
   }
   else if (m_controlMode == kVelocity)
   {
      if (m_controlEnabled == true)
      {
         m_PIDController->SetSetpoint(value);
      }
      else // Control not enabled
      {
         m_PIDController->SetSetpoint(0.0f);
      }
   }
}

/**
 * Get the value of the motor
 */
float HVA_Victor::Get()
{
   if (m_controlMode == kPercentVbus)
   {
      return Victor::GetSpeed();
   }
   else if (m_controlMode == kVelocity)
   {
      return m_PIDController->GetSetpoint();
   }
   return 0;
}

float HVA_Victor::BaseGet()
{
   return Victor::GetSpeed();
}

/**
 * Set the PID
 */
void HVA_Victor::SetPID(double p, double i, double d)
{
   m_PIDController->SetPID(p, i, d);
}

/**
 * Get the P value of the PID controller
 */
double HVA_Victor::GetP()
{
   return m_PIDController->GetP();
}

/**
 * Get the I value of the PID controller
 */
double HVA_Victor::GetI()
{
   return m_PIDController->GetI();
}

/**
 * Get the D value of the PID controller
 */
double HVA_Victor::GetD()
{
   return m_PIDController->GetD();
}

/**
 * Enable the control of the motor
 */
void HVA_Victor::EnableControl()
{
   if (m_controlMode == kPercentVbus)
   {
      m_controlEnabled = true;
   }
   else if (m_controlMode == kVelocity)
   {
      m_controlEnabled = true;
      printf("Before Enabling PID\n");
      m_PIDController->Enable();
      printf("After Enabling PID\n");
   }
}

/**
 * Disable the control of the motor
 */
void HVA_Victor::DisableControl()
{
   if (m_controlMode == kPercentVbus)
   {
      m_controlEnabled = false;
   }
   else if (m_controlMode == kVelocity)
   {
      m_controlEnabled = false;
      m_PIDController->Disable();
      m_PIDController->Reset();
   }
}

/**
 * Get the Speed
 */
float HVA_Victor::GetSpeed()
{
   return m_encoder->GetRate();
}

/**
 * Get the Position
 */
float HVA_Victor::GetPosition()
{
   return m_encoder->GetDistance();
}
