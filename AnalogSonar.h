#ifndef __AnalogSonar_h__
#define __AnalogSonar_h__

#include "WPILib.h"

#define AVERAGE_BITS             3
#define SAMPLES_PER_SECOND       1000
//#define SCALING_FACTOR           0.0098
#define SCALING_FACTOR           1.0

class AnalogSonar: public AnalogChannel
{
private:
   void initSonar();
public:
   AnalogSonar(UINT8 moduleNumber, UINT32 channel);
   AnalogSonar(UINT32 channel);
   
   float GetRangeInches();
};

#endif
