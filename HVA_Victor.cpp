#include "HVA_Victor.h"

HVA_Victor::HVA_Victor(UINT32 channel, Encoder* encoder, ControlMode controlMode) :
                       Victor(channel)
{
   // remember the encoder object
   m_encoder = encoder;

   // set the encoder distance per pulse and start
   m_encoder->SetDistancePerPulse(FEET_PER_PULSE);
   m_encoder->Start();

   // instantiate the velocity and position PID controllers
   m_velocityPIDController = new PIDController(DEFAULT_VELOCITY_P, DEFAULT_VELOCITY_I, DEFAULT_VELOCITY_D,
                                               new EncoderVelocitySource(m_encoder),
                                               new VictorOutput(this));
   m_positionPIDController = new PIDController(DEFAULT_POSITION_P, DEFAULT_POSITION_I, DEFAULT_POSITION_D,
                                               new EncoderPositionSource(m_encoder),
                                               new VictorOutput(this));

   // the control mode from the constructor parameters
   ChangeControlMode(controlMode);

   /************************ Setup the PID Controllers ***************************/
   m_velocityPIDController->SetOutputRange(-1, 1);  // Victor set class range
   m_velocityPIDController->Disable();

   m_positionPIDController->SetOutputRange(-1, 1);  // Victor set class range
   m_positionPIDController->Disable();
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
   if (m_controlMode == HVA_Victor::kPercentVbus)
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
         m_velocityPIDController->SetSetpoint(value);
      }
      else  // Control not enabled
      {
         m_velocityPIDController->SetSetpoint(0.0f);
      }
   }

   else if (m_controlMode == kPosition)
   {
      if (m_controlEnabled == true)
      {
         m_positionPIDController->SetSetpoint(value);
      }
      else  // Control not enabled
      {
         // set the position to the present position
         m_positionPIDController->SetSetpoint(GetPosition());
      }
   }
}

/**
 * Get the value of the motor
 */
float HVA_Victor::Get()
{
   if (m_controlMode == HVA_Victor::kPercentVbus)
   {
      return Victor::GetSpeed();
   }
   else if (m_controlMode == kVelocity)
   {
      return m_velocityPIDController->GetSetpoint();
   }
   else if (m_controlMode == kPosition)
   {
      return m_positionPIDController->GetSetpoint();
   }
   return 0;
}

float HVA_Victor::BaseVictorGet()
{
   return Victor::GetSpeed();
}

/**
 * Set the velocity PID
 */
void HVA_Victor::SetVelocityPID(double p, double i, double d)
{
   m_velocityPIDController->SetPID(p, i, d);
}

/**
 * Get the P value of the velocity PID controller
 */
double HVA_Victor::GetVelocityP()
{
   return m_velocityPIDController->GetP();
}

/**
 * Get the I value of the velocity PID controller
 */
double HVA_Victor::GetVelocityI()
{
   return m_velocityPIDController->GetI();
}

/**
 * Get the D value of the velocity PID controller
 */
double HVA_Victor::GetVelocityD()
{
   return m_velocityPIDController->GetD();
}

/**
 * Set the position PID
 */
void HVA_Victor::SetPositionPID(double p, double i, double d)
{
   m_positionPIDController->SetPID(p, i, d);
}

/**
 * Get the P value of the position PID controller
 */
double HVA_Victor::GetPositionP()
{
   return m_positionPIDController->GetP();
}

/**
 * Get the I value of the position PID controller
 */
double HVA_Victor::GetPositionI()
{
   return m_positionPIDController->GetI();
}

/**
 * Get the D value of the position PID controller
 */
double HVA_Victor::GetPositionD()
{
   return m_positionPIDController->GetD();
}

/**
 * Enable the control of the motor
 */
void HVA_Victor::EnableControl()
{
   if (m_controlMode == HVA_Victor::kPercentVbus)
   {
      m_controlEnabled = true;
   }
   else if (m_controlMode == kVelocity)
   {
      m_controlEnabled = true;
      m_velocityPIDController->Enable();
   }
   else if (m_controlMode == kPosition)
   {
      m_controlEnabled = true;
      m_positionPIDController->Enable();
   }
}

/**
 * Disable the control of the motor
 */
void HVA_Victor::DisableControl()
{
   if (m_controlMode == HVA_Victor::kPercentVbus)
   {
      m_controlEnabled = false;
   }
   else if (m_controlMode == kVelocity)
   {
      m_controlEnabled = false;
      m_velocityPIDController->Disable();
      m_velocityPIDController->Reset();
   }
   else if (m_controlMode == kPosition)
   {
      m_controlEnabled = false;
      m_positionPIDController->Disable();
      m_positionPIDController->Reset();
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
