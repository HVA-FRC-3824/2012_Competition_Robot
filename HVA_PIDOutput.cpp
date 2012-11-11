#include "WPILib.h"

class HVA_PIDOutput : public PIDOutput
{
private:
   float *value;

public:
   HVA_PIDOutput(float *f)
   {
      value = f;
   }

   void PIDWrite(float output)
   {
      *value = output;
   }
};
