#ifndef __HVA_PIDOutput_h__
#define __HVA_PIDOutput_h__

#include "WPILib.h"

//------------------------------------------------------------------------------
// Class to define a custom HVA PID output
//------------------------------------------------------------------------------
class HVA_PIDOutput : public PIDOutput
{
public:
   HVA_PIDOutput(float *f)
   {
      value = f;
   }

   void PIDWrite(float output)
   {
      *value = output;
   }

private:
   float *value;
};

#endif
