#ifndef __AnalogSonar_h__
#define __AnalogSonar_h__

#include "WPILib.h"

#define AVERAGE_BITS             3
#define SAMPLES_PER_SECOND       1000
#define SCALING_FACTOR           103.1859

//------------------------------------------------------------------------------
// Class to handle the Analog Sonar position sensor
//------------------------------------------------------------------------------
class AnalogSonar: public AnalogChannel
{
public:
   AnalogSonar(UINT8 moduleNumber, UINT32 channel);
   AnalogSonar(UINT32 channel);

   float GetRangeInches();

private:
   void initSonar();
};

#endif
