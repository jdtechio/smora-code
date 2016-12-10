#ifndef PID_H
#define PID_H 

#include "util.h"

typedef struct STATE {
    float Reference, Output, Y;
    float error, previous_error;
    float integrator, previous_integrator;
} STATE;

typedef struct CONTROLLER {
    float Kp, Ki, Kd, Kf;
    float limit;
    float frequency;
    STATE state;
} CONTROLLER;

class PID {   
    public:
    	CONTROLLER pid;
		STATE state = pid.state; 

    	PID();

    	void setGains(float Kp, float Ki, float Kd, float Kf);
        void setFrequency(float frequency);
        float getFrequency(void);
        void setOutputLimit(float limit);
        void resetIntegrator(void);
        float getOutput(void);
        float compute(float ref, float y);
        int convertOutputToPWM(void);
};

#endif // PID_H