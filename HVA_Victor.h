#ifndef __HVA_Victor_h__
#define __HVA_Victor_h__

#include "WPILib.h"

/*************************** Defines ******************************************/
#define DEFAULT_VELOCITY_P                0.2
#define DEFAULT_VELOCITY_I                0.04

#define DEFAULT_VELOCITY_D                0.0

#define DEFAULT_POSITION_P                5.0
#define DEFAULT_POSITION_I                0.0
#define DEFAULT_POSITION_D                2.0

#define FEET_PER_PULSE                    0.002009

/******************************************************************************/

class HVA_Victor: public Victor
{
public:
   typedef enum
   {
      kPercentVbus, kVelocity, kPosition
   } ControlMode;

public:
   HVA_Victor(UINT32 channel, Encoder *encoder, ControlMode controlMode = kPercentVbus);

   // Change the control mode the victors are running in
   void ChangeControlMode(ControlMode controlMode);

   // Get the current control mode
   ControlMode GetControlMode();

   // Set the value of the motor
   virtual void Set(float value, UINT8 syncGroup=0);

   // Get the value of the motor
   virtual float Get();

   float BaseVictorGet();

   // Set the velocity PID
   void SetVelocityPID(double p, double i, double d);

   // Get the values of the velocity PID
   double GetVelocityP();
   double GetVelocityI();
   double GetVelocityD();

   // Set the position PID
   void SetPositionPID(double p, double i, double d);

   // Get the values of the position PID
   double GetPositionP();
   double GetPositionI();
   double GetPositionD();

   // Enable the control
   void EnableControl();

   // Disable the Control
   void DisableControl();

   // Get the Speed
   virtual float GetSpeed();

   // Get the Position
   virtual float GetPosition();

private:
   class EncoderVelocitySource : public PIDSource
   {
   public:
      EncoderVelocitySource(Encoder *encoder)
      {
         m_encoder = encoder;
      }

      double PIDGet()
      {
         return m_encoder->GetRate();
      }

   private:
      Encoder *m_encoder;
   };

   class EncoderPositionSource : public PIDSource
   {
   public:
      EncoderPositionSource(Encoder *encoder)
      {
         m_encoder = encoder;
      }

      double PIDGet()
      {
         return m_encoder->GetDistance();
      }

   private:
      Encoder *m_encoder;
   };

   class VictorOutput : public PIDOutput
   {
   public:
      VictorOutput(Victor* victor)
      {
         m_victor = victor;
      }

      void PIDWrite(float output)
      {
         m_victor->Victor::Set(output);
      }

   private:
      Victor *m_victor;
   };

   ControlMode    m_controlMode;
   bool           m_controlEnabled;
   Encoder       *m_encoder;
   PIDController *m_velocityPIDController;
   PIDController *m_positionPIDController;
};

#endif
