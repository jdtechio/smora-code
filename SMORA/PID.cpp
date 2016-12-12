#include "PID.h"

PID::PID(){
	pid.Kp = pid.Ki = pid.Kd = pid.Kf = 0.0;
	pid.limit_max = pid.limit_min = 0.0;
	pid.frequency = 100.0;		// Hertz
	state.error = state.previous_error = 0.0;
	state.integrator = state.previous_integrator = 0.0;
	state.Reference = state.Output = 0.0;
}

void PID::setGains(float Kp, float Ki, float Kd, float Kf){
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
	pid.Kf = Kf;
}

void PID::setFrequency(float frequency){
 	pid.frequency = frequency;
}

float PID::getFrequency(){
  	return pid.frequency;
}

void PID::setOutputLimit(float limit_max, float limit_min){
 	pid.limit_max = limit_max;
 	pid.limit_min = limit_min;
}

void PID::resetIntegrator(){
 	state.integrator = 0.0;
}

float PID::getOutput(){
 	return state.Output;
}

float PID::compute(float ref, float error){
	state.Reference = ref;

	state.previous_error = state.error;
	state.error = error;

	state.previous_integrator = state.integrator;
	state.integrator += state.error / pid.frequency;

	// Anti-windup
	if (state.integrator * pid.Ki > pid.limit_max){
		state.integrator = state.previous_integrator;
	} else if (state.integrator * pid.Ki < pid.limit_min){
	    state.integrator = state.previous_integrator;
	}

	// PID Output
	state.Output  = pid.Kp * state.error;
	state.Output += pid.Ki * state.integrator;
	state.Output += pid.Kd * (state.error - state.previous_error) * pid.frequency;  // same as /dt

	// Feed-forward Output
	state.Output += pid.Kf * state.Reference;

	return state.Output;
}

int PID::convertOutputToPWM(float limit){
	if (state.Output > pid.limit_max){
		return (int)(pid.limit_max/limit*1023.0);
	} else if (state.Output < pid.limit_min){
		return (int)(pid.limit_min/limit*1023.0);
	}
	
	return (int)(state.Output/limit*1023.0);
}