#include "ShooterEncoder.h"

ShooterEncoder::ShooterEncoder(UINT32 channel) : Counter(channel)
{
   m_distancePerPulse = 1.0;
}

ShooterEncoder::ShooterEncoder(DigitalSource *source) : Counter(source)
{
   m_distancePerPulse = 1.0;
}

ShooterEncoder::ShooterEncoder(AnalogTrigger *trigger) : Counter(trigger)
{
   m_distancePerPulse = 1.0;
}

/**
 * Set the distance for one pulse.
 */
void ShooterEncoder::SetDistancePerPulse(double distancePerPulse)
{
   m_distancePerPulse = distancePerPulse;
}

/**
 * Get the distance traveled
 */
double ShooterEncoder::GetDistance()
{
   return Get() * m_distancePerPulse;
}

/**
 * Get the rate the shooter is travling
 */
double ShooterEncoder::GetRate()
{
   return (m_distancePerPulse / GetPeriod());
}
