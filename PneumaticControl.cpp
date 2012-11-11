//---------------------------------------------------------------------------------
// Source Filename:  PneumaticControl.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#include "Robot.h"

//---------------------------------------------------------------------------------
// Routine Summary:
//
//    void Robot::PneumaticControl();
//
// Description:
//
//    Method to control the pneumatic components of the robot.
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
void Robot::PneumaticControl()
{
   // Fire when the trigger is pressed
   if ((m_joystick->GetTrigger() == true) &&
       (m_triggerPressed         == false))
   {
       printf("Fired!\n");

       // set the trigger to pressed
       m_triggerPressed = true;
   }

   // when the trigger is released set the trigger boolean to false
   else if ((m_joystick->GetTrigger() == false) &&
            (m_triggerPressed         == true))
   {
       // set the trigger to not pressed
       m_triggerPressed = false;
   }
}
