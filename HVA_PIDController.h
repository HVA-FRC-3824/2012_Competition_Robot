//---------------------------------------------------------------------------------
// Source Filename:  HVA_PIDController.h
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//                   The class was copied from the FIRST WPI PID library method.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#ifndef HVA_PID_H_
#define HVA_PID_H_

#include "Base.h"
#include "semLib.h"

class PIDOutput;
class PIDSource;
class Notifier;

/**
 * Class implements a PID Control Loop.
 *
 * Creates a separate thread which reads the given PIDSource and takes
 * care of the integral calculations, as well as writing the given
 * PIDOutput
 */
class HVA_PIDController
{
public:
   HVA_PIDController(float p, float i, float d,
                     PIDSource *source, PIDOutput *output,
                     float period = 0.05);
   ~HVA_PIDController();

   float Get();
   void  SetInputWrap(bool continuous = true);
   void  SetInputRange(float minimumInput, float maximumInput);
   void  SetOutputRange(float mimimumOutput, float maximumOutput);
   void  SetPID(float p, float i, float d);

   float GetP();
   float GetI();
   float GetD();

   void  SetSetpoint(float setpoint);
   float GetSetpoint();

   float GetError();

   void  SetTolerance(float percent);
   bool  OnTarget();

   void  Enable();
   void  Disable();
   bool  IsEnabled();

   void  Reset();

private:
   bool   m_continuous;        // do the endpoints wrap around? eg. Absolute encoder
   bool   m_enabled;           // is the pid controller enabled

   float  m_P;                 // factor for "proportional" control
   float  m_I;                 // factor for "integral" control
   float  m_D;                 // factor for "derivative" control
   float  m_period;            // update period of the PID controller

   float  m_maximumInput;      // maximum input - limit setpoint to this
   float  m_minimumInput;      // minimum input - limit setpoint to this
   float  m_maximumOutput;     // |maximum output|
   float  m_minimumOutput;     // |minimum output|

   float  m_error;
   float  m_previousError;     // the prior sensor input (used to compute velocity)
   double m_integralError;     // the sum of the errors for use in the integral calc
   float  m_result;

   float  m_tolerance;         // the percetage error that is considered on target
   float  m_setpoint;

   SEM_ID m_semaphore;

   PIDSource *m_pidInput;
   PIDOutput *m_pidOutput;

   Notifier  *m_controlLoop;

   static void CallCalculatePID(void *controller);

   void CalculatePID();
   DISALLOW_COPY_AND_ASSIGN(HVA_PIDController);
};
#endif
