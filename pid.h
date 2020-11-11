#ifndef _PID_H_
#define _PID_H_

class MotorDriver
{
private:
    float vState;     //Last velocity state
    float iState;
    float setPoint;

    float pGain;      //proportional gain 
    float iGain;      //integral gain
    float dGain;      //proportional gain 

    float iMax;       //Maximum and minimum
    float iMin;       //allowable integrator state

    float iTerm = 0;

    unsigned long preTime;
    float dt;
    
public:
    MotorDriver();
    MotorDriver(float Kp, float Ki, float Kd, float max, float min);

    float updatePID(float,unsigned long&);
    
    void set_iGain(float Ki);
    void set_pGain(float Kp);
    void set_dGain(float Kd);

    void set_iMax(float max);
    void set_iMin(float min);

    void set_vState(float vel);
    void set_SetPoint(float s);

    float get_Kp() const;
    float get_Ki() const;
    float get_Kd() const;
};

#endif //_PID_H_
