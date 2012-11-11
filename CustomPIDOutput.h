#ifndef __CustomPIDOutput_h__
#define __CustomPIDOutput_h__

#include "WPILib.h"

//------------------------------------------------------------------------------
// Class to handle how the PID control its output
//------------------------------------------------------------------------------
class CustomPIDOutput : public PIDOutput
{
public:
   // set a variable to point to the value used for the rotation of the robot,
   // so that pid write will change the rotation value for the Holonomic drive
   // during autonomous.
   CustomPIDOutput(float *rotation);

   // set the output of the pid = to global rotation
   void PIDWrite(float value);

private:
   float *m_pid_rotation;
};

#endif
