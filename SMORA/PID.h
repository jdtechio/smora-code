#ifndef PID_H
#define PID_H 

typedef struct STATE {
    float Reference, Output;
    float error, previous_error;
    float integrator, previous_integrator;
} STATE;

typedef struct CONTROLLER {
    float Kp, Ki, Kd, Kf;
    float limit_max, limit_min;
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
        void setOutputLimit(float limit_max, float limit_min);
        void resetIntegrator(void);
        float getOutput(void);
        float compute(float ref, float error);
        int convertOutputToPWM(float limit);
};

#endif // PID_H