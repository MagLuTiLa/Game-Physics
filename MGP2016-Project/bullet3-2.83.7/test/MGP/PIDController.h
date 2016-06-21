#pragma once

#include "btBulletDynamicsCommon.h"

class PIDController
{
public:
	// Constructor
	PIDController(btScalar Kp, btScalar Ki, btScalar Kd)
		: _Kp(Kp), _Ki(Ki), _Kd(Kd), _pre_err(0.0f), _integral(0.0f){}
	// Destructor
	~PIDController() {}
	// Solve
	btScalar solve(btScalar &error, btScalar &dt);

private:
	btScalar _Kp;			// P-Proportion
	btScalar _Ki;			// I-Integral
	btScalar _Kd;			// D-Derivative
	btScalar _pre_err;		// Previous error
	btScalar _integral;		// Current integral
};

