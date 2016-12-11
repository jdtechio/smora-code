#include "PID.h"

PID::PID(){
	pid.Kp = pid.Ki = pid.Kd = pid.Kf = 0.0;
	pid.limit_max = 5.0;		// Volts
	pid.limit_min = 0.0;		// Volts
	pid.frequency = 100.0;		// Hertz
	
	state.Reference = state.Output = state.Y = 0.0;

	state.error = state.previous_error = 0.0;
	state.integrator = state.previous_integrator = 0.0;
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

float PID::compute(float ref, float y){
	state.Reference = ref;
	state.Y = y;

	state.previous_error = state.error;
	//state.error = state.Input - state.Y;
	state.error = diffAngleDegrees(state.Y, state.Reference);

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

int PID::convertOutputToPWM(){
	if (state.Output > pid.limit_min)
		return (int)(state.Output/pid.limit_max*1023.0);
	if (pid.limit_min != 0.0)
 		return (int)(state.Output/pid.limit_min*1023.0);
 	return 0;
}