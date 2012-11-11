#ifndef __HVA_PIDSource_h__
#define __HVA_PIDSource_h__

#include "WPILib.h"

//------------------------------------------------------------------------------
// Class to define a custom HVA PID source
//------------------------------------------------------------------------------
     class HVA_PIDSource : public PIDSource
{
public:
   HVA_PIDSource(float *f)
   {
      value = f;
   }

   double PIDGet()
   {
      return *value;
   }

private:
   float *value;
};

#endif
