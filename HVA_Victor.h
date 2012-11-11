#ifndef __HVA_Victor_h__
#define __HVA_Victor_h__

#include "WPILib.h"

/*************************** Defines ******************************************/
#define DEFAULT_P                0.2
#define DEFAULT_I                0.04
#define DEFAULT_D                0.0
#define FEET_PER_PULSE           0.002009
/******************************************************************************/

class HVA_Victor: public Victor
{
public:
   typedef enum
   {
      kPercentVbus, kVelocity
   } ControlMode;
   
private:
   class EncoderVelocitySource : public PIDSource
   {
   private:
      Encoder *m_encoder;

   public:
      EncoderVelocitySource(Encoder *encoder)
      {
         m_encoder = encoder;
      }

      double PIDGet()
      {
         return m_encoder->GetRate();
      }
   };
   
   class EncoderVelocityOutput : public PIDOutput
   {
   private:
      Victor *m_victor;
      
   public:
      EncoderVelocityOutput(Victor* victor)
      {
         m_victor = victor;
      }

      void PIDWrite(float output)
      {
         m_victor->Victor::Set(output);
      }
   };
   
   ControlMode m_controlMode;
   PIDController *m_PIDController;
   Encoder *m_encoder;
   double m_p, m_i, m_d;
   bool m_controlEnabled;

public:
   HVA_Victor(UINT32 channel, Encoder* encoder, ControlMode controlMode = kPercentVbus);
   
   // Change the control mode the victors are running in
   void ChangeControlMode(ControlMode controlMode);
   
   // Get the current control mode
   ControlMode GetControlMode();
   
   // Set the value of the motor
   virtual void Set(float value, UINT8 syncGroup=0);
   
   // Get the value of the motor
   virtual float Get();
   
   float BaseGet();
   
   // Set the PID
   void SetPID (double p, double i, double d);
   
   // Get the values of the PID
   double GetP();
   double GetI();
   double GetD();
   
   // Enable the control
   void EnableControl();
   
   // Disable the Control
   void DisableControl();
   
   // Get the Speed
   virtual float GetSpeed();
   
   // Get the Position
   virtual float GetPosition();
};

#endif
