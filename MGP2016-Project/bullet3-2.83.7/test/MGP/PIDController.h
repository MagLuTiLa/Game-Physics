#pragma once

#include "btBulletDynamicsCommon.h"

class PIDController
{
public:
	// Constructor
	PIDController(btScalar Kp, btScalar Ki, btScalar Kd);
	// Destructor
	~PIDController() {}
	// Solve
	//btScalar solve(btScalar error, btScalar &dt);
	btVector3 solve(btVector3& error, btScalar &dt);
	btScalar solve(btScalar error, btScalar &dt);

private:
	btScalar _Kp;			// P-Proportion
	btScalar _Ki;			// I-Integral
	btScalar _Kd;			// D-Derivative
	btScalar _pre_err;		// Previous error
	btScalar _integral;		// Current integral
	btVector3 _Bintegral;	// Current integral for ball joint
	btVector3 _Bpre_err;		// Previous error for ball joint
};

