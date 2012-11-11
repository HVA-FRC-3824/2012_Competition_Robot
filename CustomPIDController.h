//---------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------

#ifndef __GenericPIDController_h__
#define __GenericPIDController_h__

//------------------------------------------------------------------------------
// Class to handle generic PID input and output
//------------------------------------------------------------------------------
class GenericPIDOutput : public PIDOutput
{
public:
   // set a variable to point to the value used for the generic PID output
   GenericPIDOutput(float *PID_output)
   {
      m_PID_output = PID_output;
   }

   // set the output of the PID = to global rotation
   void PIDWrite(float PID_output)
   {
      *m_PID_output = PID_output;
   }

private:
   float *m_PID_output;
};

//------------------------------------------------------------------------------
// Class to handle how the PID control its input
//------------------------------------------------------------------------------
class GenericPIDSource : public PIDSource
{
public:
   // set a variable to point to the value used for the generic PID output
   GenericPIDSource(float *PID_source)
   {
      m_PID_source = PID_source;
   }

   // set the output of the PID = to global rotation
   double PIDGet()
   {
      return *m_PID_source;
   }

private:
   float *m_PID_source;
};
#endif
