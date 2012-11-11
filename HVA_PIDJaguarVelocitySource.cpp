#include "WPILib.h"

class HVA_PIDJaguarVelocity : public PIDSource
{
private:
   float *value;
   CANJaguar *jaguar;

public:
   HVA_PIDJaguarVelocity(CANJaguar *jag)
   {
      jaguar = jag;
   }

   double PIDGet()
   {
      return jaguar->GetSpeed();
   }
};
