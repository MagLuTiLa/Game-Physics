#pragma once

#include "btBulletDynamicsCommon.h"

class PIDController
{
public:
	// Constructor
	PIDController(btScalar Kp, btScalar Ki, btScalar Kd)
		: _Kp(Kp), _Ki(Ki), _Kd(Kd), _pre_err(0.0f), _integral(0.0f){
		_Bpre_err = btVector3(0.0f, 0.0f, 0.0f);
		_Bintegral = btVector3(0.0f, 0.0f, 0.0f);
	}
	// Destructor
	~PIDController() {}
	// Solve
	btScalar solve(btScalar error, btScalar &dt);		// For basic balancing
	btVector3 solve(btVector3& error, btScalar &dt);	// For advanced balancing
	// get parameters
	btScalar get_Kp() { return _Kp; }
	btScalar get_Ki() { return _Ki; }
	btScalar get_Kd() { return _Kd; }
	// set parameters
	void set_Kp(btScalar kp) { _Kp = kp; }
	void set_Ki(btScalar ki) { _Ki = ki; }
	void set_Kd(btScalar kd) { _Kd = kd; }

private:
	btScalar _Kp;			// P-Proportion
	btScalar _Ki;			// I-Integral
	btScalar _Kd;			// D-Derivative
	btScalar _pre_err;		// Previous error
	btScalar _integral;		// Current integral
	btVector3 _Bintegral;	// Current integral for ball joint
	btVector3 _Bpre_err;	// Previous error for ball joint
};

