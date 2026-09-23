#ifndef PID_H
#define PID_H

class PID {
    private:
       float Kp;
       float Ki;
       float Kd;
       
       float I_sum;
       float e_prev;

       float maxI;
       float maxOut;

    public:
        PID(float Kp, float Ki, float Kd, float maxI, float maxOut);
        float update(float setPoint, float measured, float dt);
        void reset();
};

#endif