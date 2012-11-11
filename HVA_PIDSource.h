#ifndef __HVA_PIDSource_h__
#define __HVA_PIDSource_h__

#include "WPILib.h"

class HVA_PIDSource : public PIDSource
{
private:
   float *value;

public:
   HVA_PIDSource(float *f)
   {
      value = f;
   }

   double PIDGet()
   {
      return *value;
   }
};

#endif
