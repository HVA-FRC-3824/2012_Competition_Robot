//---------------------------------------------------------------------------------
// Source Filename:  Robot.cpp
//
// Date Created:     12/19/11
//
// Programmer:       Aaron Young
//
// Description:      Custom Robot Code for the Hardin Valley Academy FRC Robot.
//                   The class was copied from the FIRST WPI PID library method.
//
// Version History:  1 - Starting code for the 2012 season.
//---------------------------------------------------------------------------------

#include <math.h>

#include "Notifier.h"
#include "PIDSource.h"
#include "PIDOutput.h"
#include "Synchronized.h"

#include "HVA_PIDController.h"

#define NO_PID_BAND                 10.0
#define NO_PID_TURN_POWER            0.2

//---------------------------------------------------------------------------------
// Routine Summary:
//
//
// Description:
//
//
// Return Value:
//
//    None.
//
// Modification History:
//
//    12/19/11 ARY - First version of routine.
//---------------------------------------------------------------------------------
/**
 * Allocate a PID object with the given constants for P, I, D
 * @param Kp the proportional coefficient
 * @param Ki the integral coefficient
 * @param Kd the derivative coefficient
 * @param source The PIDSource object that is used to get values
 * @param output The PIDOutput object that is set to the output value
 * @param period the loop time for doing calculations. This particularly effects calculations of the
 * integral and differential terms. The default is 50ms.
 */
HVA_PIDController::HVA_PIDController(float Kp, float Ki, float Kd,
                                     PIDSource *source, PIDOutput *output,
                                     float period) : m_semaphore (0)
{
   m_semaphore = semBCreate(SEM_Q_PRIORITY, SEM_FULL);

   m_controlLoop = new Notifier(HVA_PIDController::CallCalculatePID, this);

   m_P = Kp;
   m_I = Ki;
   m_D = Kd;

   m_continuous = false;
   m_enabled    = false;

   m_maximumOutput =  1.0;
   m_minimumOutput = -1.0;

   m_maximumInput = 0;
   m_minimumInput = 0;

   m_setpoint   = 0;

   m_previousError = 0;
   m_integralError = 0;
   m_tolerance     = .05;

   m_result     = 0;

   m_pidInput   = source;
   m_pidOutput  = output;

   m_period     = period;

   m_controlLoop->StartPeriodic(m_period);
}

/**
 * Free the PID object
 */
HVA_PIDController::~HVA_PIDController()
{
   semFlush(m_semaphore);
   delete m_controlLoop;
}

/**
 * Call the Calculate method as a non-static method. This avoids having to prepend
 * all local variables in that method with the class pointer. This way the "this"
 * pointer will be set up and class variables can be called more easily.
 * This method is static and called by the Notifier class.
 * @param controller the address of the PID controller object to use in the background loop
 */
void HVA_PIDController::CallCalculatePID(void *controller)
{
   HVA_PIDController *control = (HVA_PIDController*) controller;
   control->CalculatePID();
}

 /**
  * Read the input, calculate the output accordingly, and write to the output.
  * This should only be called by the Notifier indirectly through CallCalculate
  * and is created during initialization.
  */
void HVA_PIDController::CalculatePID()
{  
   bool enabled;
   PIDSource *pidInput;

   CRITICAL_REGION(m_semaphore)
   {
      // ensure the PID controller contains an input and output source
      if (m_pidInput  == 0) return;
      if (m_pidOutput == 0) return;

      // get the state of PID in the protected region
      enabled  = m_enabled;

      // get the pointer to the PID source
      pidInput = m_pidInput;
   }
   END_REGION;

   // determine if the PID is enabled
   if (enabled == true)
   {
      float      input;
      float      result;
      PIDOutput *pidOutput;

      // read the PID input
      input = pidInput->PIDGet();

      CRITICAL_REGION(m_semaphore)
      {
         // calculate the error
         m_error = m_setpoint - input;

         // determine if the endpoints wrap around? eg. Absolute encoder
         if (m_continuous == true)
         {
            // determine the error based on the distance from the mid-point of the
            // minimum and maximum set points
            // Example: PID with range from 0 to 360
            //          Assume a set point of 10 and a value of 350
            //          m_error = -340, so ABS(-340) > 180
            //          m_error += 360 = -340 + 360 = 20
            if (fabs(m_error) > (m_maximumInput - m_minimumInput) / 2)
            {
               // determine how the error should be calculated base on the error sign
               if (m_error > 0)
                  m_error -= m_maximumInput + m_minimumInput;
               else
                  m_error += m_maximumInput - m_minimumInput;
            }
         }

//         printf("***\n");
//         printf("S: %f  I: %f  E: %f\n", m_setpoint, input, m_error);
                  
         if (m_error > NO_PID_BAND)
         {
        	 m_result = NO_PID_TURN_POWER;
         	 m_integralError = 0.0;
         }
         else if (m_error < -NO_PID_BAND)
         {
        	 m_result = -NO_PID_TURN_POWER;
        	 m_integralError = 0.0;
         }
         else
         {
            // determine if the robot is in tolerance
            if (((m_error > 0) && (m_error < m_tolerance)) ||
                ((m_error < 0) && (m_error > -m_tolerance)))
            {
               m_result = 0.0;
            }
            else
            {
               // only integrate the error if the integral error term does not exceed
        	   // the maximum or minimum output
           	   if ((((m_integralError + m_error) * m_I) < m_maximumOutput) &&
        		   (((m_integralError + m_error) * m_I) > m_minimumOutput))
               {
                  // integrate the error
                  m_integralError += m_error;
               }

               // determine if the PID crossed the setpoint
               if (((m_error > 0) && (m_previousError < 0)) ||
                   ((m_error < 0) && (m_previousError > 0)))
           	      m_integralError = 0.0;

               // perform the PID calculation
               m_result = (m_P * m_error)         +
                          (m_I * m_integralError) +
                          (m_D * (m_error - m_previousError));
         
//printf("Integral: %f\n", m_integralError);       
//printf("P: %f  I: %f  D: %f\n", m_P * m_error, 
//                         		m_I * m_integralError, 
//        		                m_D * (m_error - m_previousError));
            }
         }
         
         // remember the error
         m_previousError = m_error;

         // ensure the PID output is in range
         if (m_result > m_maximumOutput)
            m_result = m_maximumOutput;
         else if (m_result < m_minimumOutput)
            m_result = m_minimumOutput;

         // get the pointer to the PID output
         pidOutput = m_pidOutput;

         // get the PID result for performing the output control
         result = m_result;
      }
      END_REGION;
      
//printf("O: %f\n", result);

      // update the PID output
      pidOutput->PIDWrite(result);
   }
}

/**
 * Set the PID Controller gain parameters.
 * Set the proportional, integral, and differential coefficients.
 * @param p Proportional coefficient
 * @param i Integral coefficient
 * @param d Differential coefficient
 */
void HVA_PIDController::SetPID(float Kp, float Ki, float Kd)
{
   CRITICAL_REGION(m_semaphore)
   {
      m_P = Kp;
      m_I = Ki;
      m_D = Kd;
   }
   END_REGION;
}

/**
 * Get the Proportional coefficient
 * @return proportional coefficient
 */
float HVA_PIDController::GetP()
{
   CRITICAL_REGION(m_semaphore)
   {
      return m_P;
   }
   END_REGION;
}

/**
 * Get the Integral coefficient
 * @return integral coefficient
 */
float HVA_PIDController::GetI()
{
   CRITICAL_REGION(m_semaphore)
   {
      return m_I;
   }
   END_REGION;
}

/**
 * Get the Differential coefficient
 * @return differential coefficient
 */
float HVA_PIDController::GetD()
{
   CRITICAL_REGION(m_semaphore)
   {
      return m_D;
   }
   END_REGION;
}

/**
 * Return the current PID result
 * This is always centered on zero and constrained the the max and min outs
 * @return the latest calculated output
 */
float HVA_PIDController::Get()
{
   float result;

   CRITICAL_REGION(m_semaphore)
   {
       result = m_result;
   }
   END_REGION;

   return result;
}

/**
 *  Set the PID controller to consider the input to be continuous,
 *  Rather then using the max and min in as constraints, it considers them to
 *  be the same point and automatically calculates the shortest route to
 *  the setpoint.
 * @param continuous Set to true turns on continuous, false turns off continuous
 */
void HVA_PIDController::SetInputWrap(bool continuous)
{
   CRITICAL_REGION(m_semaphore)
   {
      m_continuous = continuous;
   }
   END_REGION;
}

/**
 * Sets the maximum and minimum values expected from the input.
 *
 * @param minimumInput the minimum value expected from the input
 * @param maximumInput the maximum value expected from the output
 */
void HVA_PIDController::SetInputRange(float minimumInput, float maximumInput)
{
   CRITICAL_REGION(m_semaphore)
   {
      m_minimumInput = minimumInput;
      m_maximumInput = maximumInput;
   }
   END_REGION;

   SetSetpoint(m_setpoint);
}

/**
 * Sets the minimum and maximum values to write.
 *
 * @param minimumOutput the minimum value to write to the output
 * @param maximumOutput the maximum value to write to the output
 */
void HVA_PIDController::SetOutputRange(float minimumOutput, float maximumOutput)
{
   CRITICAL_REGION(m_semaphore)
   {
      m_minimumOutput = minimumOutput;
      m_maximumOutput = maximumOutput;
   }
   END_REGION;
}

/**
 * Set the setpoint for the HVA_PIDController
 * @param setpoint the desired setpoint
 */
void HVA_PIDController::SetSetpoint(float setpoint)
{
   CRITICAL_REGION(m_semaphore)
   {
      if (m_maximumInput > m_minimumInput)
      {
         if (setpoint > m_maximumInput)
            m_setpoint = m_maximumInput;
         else if (setpoint < m_minimumInput)
            m_setpoint = m_minimumInput;
         else
            m_setpoint = setpoint;
      }
      else
      {
         m_setpoint = setpoint;
      }
   }
   END_REGION;
}

/**
 * Returns the current setpoint of the HVA_PIDController
 * @return the current setpoint
 */
float HVA_PIDController::GetSetpoint()
{
   float setpoint;

   CRITICAL_REGION(m_semaphore)
   {
      setpoint = m_setpoint;
   }
   END_REGION;

   return setpoint;
}

/**
 * Returns the current difference of the input from the setpoint
 * @return the current error
 */
float HVA_PIDController::GetError()
{
   float error;

   CRITICAL_REGION(m_semaphore)
   {
      error = m_error;
   }
   END_REGION;

   return error;
}

/*
 * Set the percentage error which is considered tolerable for use with
 * OnTarget.
 * @param percentage error which is tolerable
 */
void HVA_PIDController::SetTolerance(float percent)
{
   CRITICAL_REGION(m_semaphore)
   {
      m_tolerance = percent;
   }
   END_REGION;
}

/*
 * Return true if the error is within the percentage of the total input range,
 * determined by SetTolerance. This assumes that the maximum and minimum input
 * were set using SetInput.
 */
bool HVA_PIDController::OnTarget()
{
   bool temp;

   CRITICAL_REGION(m_semaphore)
   {
      temp = fabs(m_error) < (m_tolerance / 100 * (m_maximumInput - m_minimumInput));
   }
   END_REGION;

   return temp;
}

/**
 * Begin running the HVA_PIDController
 */
void HVA_PIDController::Enable()
{
   CRITICAL_REGION(m_semaphore)
   {
      m_enabled = true;
   }
   END_REGION;
}
/**
 * Stop running the HVA_PIDController, this sets the output to zero before stopping.
 */
void HVA_PIDController::Disable()
{
   CRITICAL_REGION(m_semaphore)
   {
      m_pidOutput->PIDWrite(0);
      m_enabled = false;
   }
   END_REGION;
}

/**
 * Return true if HVA_PIDController is enabled.
 */
bool HVA_PIDController::IsEnabled()
{
   bool enabled;

   CRITICAL_REGION(m_semaphore)
   {
      enabled = m_enabled;
   }
   END_REGION;

   return enabled;
}

/**
 * Reset the previous error,, the integral term, and disable the controller.
 */
void HVA_PIDController::Reset()
{
   Disable();

   CRITICAL_REGION(m_semaphore)
   {
      m_previousError  = 0;
      m_integralError = 0;
      m_result     = 0;
   }
   END_REGION;
}
