#ifndef __CurrentVelocityController_cpp__
#define __CurrentVelocityController_cpp__

#include "WPILib.h"

class CurrentVelocityController: public SpeedController
{
private:
   PIDController *controller;
public:
   CurrentVelocityController(PIDController *cont)
   {
      controller = cont;
   }
   void Set(float speed, UINT8 syncGroup=0)
   {
      controller->SetSetpoint(speed);
   }
   float Get()
   {
      return controller->GetSetpoint();
   }
   void Disable()
   {
      controller->Disable();
   }
};

#endif
