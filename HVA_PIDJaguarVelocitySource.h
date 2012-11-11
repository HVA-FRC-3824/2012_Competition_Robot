#ifndef __HVA_PIDJaguarVelocitySource_h__
#define __HVA_PIDJaguarVelocitySource_h__

#include "WPILib.h"

//------------------------------------------------------------------------------
// Class to handle the custom HVA PID Jaguar as a PID source
//------------------------------------------------------------------------------
class HVA_PIDJaguarVelocitySource : public PIDSource
{
public:
   HVA_PIDJaguarVelocitySource(CANJaguar *jag)
   {
      jaguar = jag;
   }

   double PIDGet()
   {
      return jaguar->GetSpeed();
   }

private:
   float *value;
   CANJaguar *jaguar;
};

#endif
