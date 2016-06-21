#include "PIDController.h"

btScalar PIDController::solve(btScalar &error, btScalar &dt)
{
	// Possible extension to PID by adding an integral term
	_integral += error * dt;
	btScalar PID = _Kp * error + _Ki * _integral + _Kd * (error - _pre_err) / dt;
	_pre_err = error;
	return PID;
}
