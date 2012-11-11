#ifndef __ShooterEncoder_h__
#define __ShooterEncoder_h__

#include "WPILib.h"

class ShooterEncoder: public Counter
{
private:
   double m_distancePerPulse;
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
};

#endif
