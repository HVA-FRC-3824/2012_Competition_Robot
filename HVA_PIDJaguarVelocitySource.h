#ifndef __HVA_PIDJaguarVelocitySource_h__
#define __HVA_PIDJaguarVelocitySource_h__

#include "WPILib.h"

class HVA_PIDJaguarVelocitySource : public PIDSource
{
private:
   float *value;
   CANJaguar *jaguar;

public:
   HVA_PIDJaguarVelocitySource(CANJaguar *jag)
   {
      jaguar = jag;
   }

   double PIDGet()
   {
      return jaguar->GetSpeed();
   }
};

#endif
