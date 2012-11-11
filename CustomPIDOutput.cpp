#include "CustomPIDOutput.h"

CustomPIDOutput::CustomPIDOutput(float *rotation)
{
   m_pid_rotation = rotation;
}

void CustomPIDOutput::PIDWrite(float value)
{
   *m_pid_rotation = value;
}
