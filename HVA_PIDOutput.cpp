#include "WPILib.h"

class HVA_PIDOutput : public PIDOutput
{
private:
   RobotDrive *m_robotDrive;
   float *value;

public:
   HVA_PIDOutput(float *f)
   {
      value = f;
   }

   void PIDWrite(float output)
   {
      printf("Mag: %f\n", output);
      *value = output;
   }
};
