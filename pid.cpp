#include "pid.h"

MotorDriver::MotorDriver(): vState{0}, iState{0}, pGain{0}, iGain{0},
							 dGain{0},preTime{0}, dt{0} {}

MotorDriver::MotorDriver(float Ki, float Kp, float Kd, float min, float max) :
                vState{0}, iState{0}, iGain(Ki), pGain(Kp),
                dGain(Kd), iMax(max), iMin(min) {}


/*float MotorDriver::updatePID(float measurement,unsigned long& cTime)
 {

      dt = (float)(cTime - preTime);*/

 float MotorDriver::updatePID(float measurement,unsigned long& cTime)
 {

      dt = (float)(cTime - preTime);
     /*
        Error
     */
     float error = setPoint - measurement;

     /*
        Proportional
     */
     float pTerm = pGain * error;  //calculate the proportional term

     /*
        Integral
     */
     iState = iState + error;

    /***************************************************************************
    *************** Anti-wind-up via dynamic integrator clamping ***************/

     //Calculate the integral state with appropriate limiting
     if(iState > 40) iState = 80.f;

     else if(iState < -40) iState = -80.f;

     iTerm = iTerm + iGain*error*(dt/1000); //calculate the integral term

    /*************** Anti-wind-up via dynamic integrator clamping ***************
    *****************************************************************************/
    
     float  dTerm = dGain * (vState - measurement) * (dt/1000);
     vState = measurement;

     preTime = cTime;

     float pid_Total = pTerm + iTerm + dTerm;

     if(pid_Total > iMax)
        pid_Total = iMax;
     else if(pid_Total < iMin)
        pid_Total = iMin;

    return pid_Total;
 }

void MotorDriver::set_iGain(float Ki)
{
    iGain = Ki;
}

void MotorDriver::set_pGain(float Kp)
{
    pGain = Kp;
}

void MotorDriver::set_dGain(float Kd)
{
    dGain = Kd;
}

void MotorDriver::set_iMax(float max)
{
    iMax = max;
}

void MotorDriver::set_iMin(float min)
{
    iMin = min;
}

void MotorDriver::set_vState(float vel)
{
    vState = vel;
}

void MotorDriver::set_SetPoint(float s)
{
    setPoint = s;
}

float MotorDriver::get_Kp() const
{
  return pGain;
}

float MotorDriver::get_Ki() const
{
  return iGain;
}

float MotorDriver::get_Kd() const
{
  return dGain;
}
