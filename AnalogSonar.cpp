#include "AnalogSonar.h"

AnalogSonar::AnalogSonar(UINT8 moduleNumber, UINT32 channel) :
   AnalogChannel(moduleNumber, channel)
{
   initSonar();
}
AnalogSonar::AnalogSonar(UINT32 channel) :
   AnalogChannel(channel)
{
   initSonar();
}

void AnalogSonar::initSonar()
{
   SetAverageBits(AVERAGE_BITS);
   //GetModule()->SetSampleRate(SAMPLES_PER_SECOND);
}

float AnalogSonar::GetRangeInches()
{
   return GetAverageVoltage() * SCALING_FACTOR;
}
