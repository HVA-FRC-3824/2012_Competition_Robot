#include "DashboardDataFormat.h"

DashboardDataFormat::DashboardDataFormat(void)
{
   analogModule1  = AnalogModule::GetInstance(1);
   analogModule2  = AnalogModule::GetInstance(2);
   digitalModule1 = DigitalModule::GetInstance(1);
   digitalModule2 = DigitalModule::GetInstance(2);

   dsLCD          = DriverStationLCD::GetInstance();
}

DashboardDataFormat::~DashboardDataFormat()
{

}

//---------------------------------------------------------------------------------
// Send the text box data.
//---------------------------------------------------------------------------------
void DashboardDataFormat::SendLCDData(double height, double rightSpeed,
      double leftSpeed, float rightSetValue, float leftSetValue,
      float throttleValue, float speed, float offset)
{
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line1, "R: %4.1f, %4.1f", rightSpeed,
                     rightSetValue);
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line2, "L: %4.1f, %4.1f", leftSpeed,
                     leftSetValue);
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line3, "H: %f", height);
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line4, "Offset: %f", offset);
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line5, "Shooter Value: %f",
                     throttleValue);
   dsLCD->PrintfLine(DriverStationLCD::kUser_Line6, "Speed: %f", speed);
   dsLCD->UpdateLCD();
}

void DashboardDataFormat::SendVisionData()
{
   Dashboard &dash = DriverStation::GetInstance()->GetHighPriorityDashboardPacker();

   dash.AddCluster(); // wire (2 elements)
   {
      dash.AddCluster(); // tracking data
      {
         dash.AddDouble(1.0);   // Joystick X
         dash.AddDouble(135.0); // angle
         dash.AddDouble(3.0);   // angular rate
         dash.AddDouble(5.0);   // other X
      }
      dash.FinalizeCluster();
      dash.AddCluster(); // target Info (2 elements)
      {
         dash.AddCluster(); // targets
         {
            dash.AddDouble(100.0); // target score
            dash.AddCluster();     // Circle Description (5 elements)
            {
               dash.AddCluster(); // Position (2 elements)
               {
                  dash.AddDouble(30.0); // X
                  dash.AddDouble(50.0); // Y
               }
               dash.FinalizeCluster();
            }
            dash.FinalizeCluster(); // Position
            dash.AddDouble(45.0);   // Angle
            dash.AddDouble(21.0);   // Major Radius
            dash.AddDouble(15.0);   // Minor Radius
            dash.AddDouble(324.0);  // Raw score
         }
         dash.FinalizeCluster(); // targets
      }
      dash.FinalizeCluster(); // target Info
   }
   dash.FinalizeCluster(); // wire
   dash.Finalize();
}

void DashboardDataFormat::SendIOPortData(bool bottomBallSensor, float position,
      float frontDistance, float backDistance, float horizontalGyro, float verticalGyro, float voltage, bool readyToFire, bool middleBallSensor, bool topBallSensor)
{
   Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();

   dash.AddCluster();
   {
      dash.AddCluster();
      { //analog modules
         dash.AddCluster();
         {
            if (analogModule1 != NULL)
            {
               for (int i = 1; i <= 8; i++)
               {
                  dash.AddFloat((float) analogModule1->GetAverageVoltage(i));
               }
            }
            else
            {
               for (int i = 1; i <= 8; i++)
               {
                  dash.AddFloat(0.0);
               }
            }
         }
         dash.FinalizeCluster();
         dash.AddCluster();
         {
            if (analogModule2 != NULL)
            {
               for (int i = 1; i <= 8; i++)
               {
                  dash.AddFloat((float) analogModule2->GetAverageVoltage(i));
               }
            }
            else
            {
               for (int i = 1; i <= 8; i++)
               {
                  dash.AddFloat(0.0);
               }
            }
         }
         dash.FinalizeCluster();
      }
      dash.FinalizeCluster();

      dash.AddCluster();
      { //digital modules
         dash.AddCluster();
         {
            dash.AddCluster();
            {
               if (digitalModule1 != NULL)
               {
                  dash.AddU8(digitalModule1->GetRelayForward());
                  dash.AddU8(digitalModule1->GetRelayReverse());
                  dash.AddU16((short) digitalModule1->GetDIO());
                  dash.AddU16((short) digitalModule1->GetDIODirection());
                  dash.AddCluster();
                  {
                     for (int i = 1; i <= 10; i++)
                     {
                        dash.AddU8((unsigned char) digitalModule1->GetPWM(i));
                     }
                  }
               }
               else
               {
                  dash.AddU8(0);
                  dash.AddU8(0);
                  dash.AddU16(0);
                  dash.AddU16(0);
                  dash.AddCluster();
                  {
                     for (int i = 1; i <= 10; i++)
                        dash.AddU8(0);
                  }
               }
               dash.FinalizeCluster();
            }
            dash.FinalizeCluster();
         }
         dash.FinalizeCluster();

         dash.AddCluster();
         {
            dash.AddCluster();
            {
               if (digitalModule2 != NULL)
               {
                  dash.AddU8(digitalModule2->GetRelayForward());
                  dash.AddU8(digitalModule2->GetRelayForward());
                  dash.AddU16((short) digitalModule2->GetDIO());
                  dash.AddU16(digitalModule2->GetDIODirection());
                  dash.AddCluster();
                  {
                     for (int i = 1; i <= 10; i++)
                     {
                        dash.AddU8((unsigned char) digitalModule2->GetPWM(i));
                     }
                  }
               }
               else
               {
                  dash.AddU8(0);
                  dash.AddU8(0);
                  dash.AddU16(0);
                  dash.AddU16(0);
                  dash.AddCluster();
                  {
                     for (int i = 1; i <= 10; i++)
                        dash.AddU8(0);
                  }
               }
               dash.FinalizeCluster();
            }
            dash.FinalizeCluster();
         }
         dash.FinalizeCluster();
      }
      dash.FinalizeCluster();

      // Can't read solenoids without an instance of the object
      dash.AddU8((char) 0);

      dash.AddBoolean(bottomBallSensor);
      dash.AddFloat(position);
      dash.AddFloat(frontDistance);
      dash.AddFloat(backDistance);
      dash.AddFloat(horizontalGyro);
      dash.AddFloat(verticalGyro);
      dash.AddFloat(voltage);
      dash.AddBoolean(readyToFire);
      dash.AddBoolean(middleBallSensor);
      dash.AddBoolean(topBallSensor);
   }
   dash.FinalizeCluster();
   dash.Finalize();
}
