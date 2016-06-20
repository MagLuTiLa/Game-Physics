#pragma once

#include "btBulletDynamicsCommon.h"

class PDController
{
public:
	// Constructor
	PDController(btScalar Kd, btScalar Kv) : _Kd(Kd), _Kv(Kv) {}
	// Destructor
	~PDController() {}
	// Solve
	btVector3 control(btScalar &correction);

private:
	btScalar _Kd;
	btScalar _Kv;
};

