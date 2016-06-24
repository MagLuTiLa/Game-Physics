#include "PIDController.h"

btScalar PIDController::solve(btScalar error, btScalar &dt)
{
	// Possible extension to PID by adding an integral term
	_integral += error * dt;
	btScalar PID = _Kp * error + _Ki * _integral + _Kd * (error - _pre_err) / dt;
	_pre_err = error;
	return PID;
}

PIDController::PIDController(btScalar Kp, btScalar Ki, btScalar Kd){
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_Bpre_err = btVector3(0.0, 0.0, 0.0);
	_Bintegral = btVector3(0.0, 0.0, 0.0);
	_pre_err = 0.0;	
	_integral = 0.0;	
}

btVector3 PIDController::solve(btVector3& _error, btScalar &dt) {
	// Possible extension to PID by adding an integral term
	btVector3 PID;
	_Bintegral += dt * _error;
	PID = _Kp * _error + _Ki * _Bintegral + _Kd * (_error - _Bpre_err) / dt;
	_Bpre_err = _error;
	return PID;
}
