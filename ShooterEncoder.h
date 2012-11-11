#ifndef __ShooterEncoder_h__
#define __ShooterEncoder_h__

#include "WPILib.h"

class ShooterEncoder: public Counter
{
public:
   ShooterEncoder(UINT32 channel);
   ShooterEncoder(DigitalSource *source);
   ShooterEncoder(AnalogTrigger *trigger);

   // Set the distance for one pulse.
   void SetDistancePerPulse(double distancePerPulse);

   // Get the distance traveled
   double GetDistance();

   // Get the rate the shooter is travling
   double GetRate();

private:
   double m_distancePerPulse;
};

#endif
