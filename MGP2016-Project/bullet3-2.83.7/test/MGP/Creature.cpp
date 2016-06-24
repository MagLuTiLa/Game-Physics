
#include "Creature.h"
#include "PIDController.h"

// TO DEBUG
#include <iostream>
#include <fstream>

#define CONSTRAINT_DEBUG_SIZE 0.2f

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616
#define EPSILON	   0.0000001f
#define ACTION_BIAS	0.00001f

//// Tune parameters here
//#define K_P_ANKLE	200.0f
//#define K_I_ANKLE	0.01f
//#define K_D_ANKLE	10.0f
//
//#define K_P_KNEE	150.0f
//#define K_I_KNEE	0.1f
//#define K_D_KNEE	10.0f

// Tune parameters here
#define K_P_ANKLE	0.1f
#define K_I_ANKLE	0.00f
#define K_D_ANKLE	0.0f

#define K_P_KNEE	0.15f
#define K_I_KNEE	0.0f
#define K_D_KNEE	0.05f


// Switch Modes, modify to extend modes
#if 1
#define BASIC_BALANCE
#elif 0
#define EXTRA_LIMP
#elif 0
#define ADV_BALANCE
#elif 0
#define POS_DEPEND
#endif

Creature::Creature (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld), m_hasFallen(false), lastChange(0), m_showCOM(false), m_time_step(10.0f) { // Constructor
		
		// Setup the rigid bodies
		// ======================

		// Setup the collision shapes
		m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(0.1),btScalar(0.025),btScalar(0.12)));
		m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.50));
		m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.40));
		m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));

		// Setup the body properties
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset); // absolute initial starting position
		btTransform transform;

		// FOOT
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(5.0), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

		// LOWER_LEG
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Creature::BODYPART_LOWER_LEG]);

		// UPPER_LEG
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
		m_bodies[Creature::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Creature::BODYPART_UPPER_LEG]);

		// Add damping to the rigid bodies
		for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
			m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
			m_bodies[i]->setDeactivationTime(btScalar(0.01));
			m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
		}
		m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(0.8), btScalar(0.01)); // Higher friction for foot

	/*	btQuaternion bodyOrientation = m_bodies[Creature::BODYPART_LOWER_LEG]->getOrientation();
		std::cout << bodyOrientation.x() << " " << bodyOrientation.y() << " "
		<< bodyOrientation.z() << " " << bodyOrientation.w() << std::endl;
		btVector3 ooo = QuaternionToEulerXYZ(bodyOrientation);
		std::cout << ooo.x() << " " << ooo.y() << " " << ooo.z() << std::endl;*/

		// Setup the hinge joint constraints
		// ===========================

		//btHingeConstraint* hingeJoint;
		////FYI, another type of joint is for example: btConeTwistConstraint* coneJoint;
		//btTransform localA, localB;
		//
		//// ANKLE
		//localA.setIdentity(); localB.setIdentity();
		//localA.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.025), btScalar(0.0)));
		//localB.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.25), btScalar(0.0)));
		//hingeJoint =  new btHingeConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LOWER_LEG], localA, localB);
		//hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));
		//
		//hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control
		//
		//m_joints[Creature::JOINT_ANKLE] = hingeJoint;
		//hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		//m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE], true);

		//// KNEE
		//localA.setIdentity(); localB.setIdentity();
		//localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.25), btScalar(0.0)));
		//localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.20), btScalar(0.0)));
		//hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG], localA, localB);
		//hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

		//hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

		//m_joints[Creature::JOINT_KNEE] = hingeJoint;
		//hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		//m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);


		// Setup the ball-socket joint constraints
		// ===========================
		btPoint2PointConstraint* balljoint;
		btVector3 foot_lowerleg_joint = btVector3(0.0, 0.025, 0.0);
		btVector3 lowerleg_foot_joint = btVector3(0.0, -0.25, 0.0);
		btVector3 lowerleg_upperleg_joint = btVector3(0.0, 0.25, 0.0);
		btVector3 upperleg_lowerleg_joint = btVector3(0.0, -0.20, 0.0);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LOWER_LEG],
			foot_lowerleg_joint, lowerleg_foot_joint);
		m_joints[Creature::JOINT_ANKLE] = balljoint;
		balljoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE],true);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG],
			lowerleg_upperleg_joint, upperleg_lowerleg_joint);
		m_joints[Creature::JOINT_KNEE] = balljoint;
		balljoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_KNEE],true);

		// Setup the PID controllers (every bodypart has its own PID)
		// =========================
		PIDController* pidController;

		// foot
		pidController = new PIDController(7.0, 0.00, 7.0);
		m_PIDs[Creature::BODYPART_FOOT] = pidController;

		// lower_leg
		pidController = new PIDController(40.0, 0.00, 40.0);
		m_PIDs[Creature::BODYPART_LOWER_LEG] = pidController;

		// upper_leg
		pidController = new PIDController(35.0, 0.00, 35.0);
		m_PIDs[Creature::BODYPART_UPPER_LEG] = pidController;
}

Creature::~Creature() { // Destructor
		// Remove all joint constraints
		for (int i = 0; i < Creature::JOINT_COUNT; ++i) {
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = NULL;
		}		
		// Remove all bodies and shapes
		for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
			m_ownerWorld->removeRigidBody(m_bodies[i]);			
			delete m_bodies[i]->getMotionState();
			delete m_bodies[i]; m_bodies[i] = NULL;
			delete m_shapes[i]; m_shapes[i] = NULL;
		}
		if (m_showCOM) {
			m_ownerWorld->removeRigidBody(m_COM);
			delete m_COM->getMotionState();
			delete m_COM; m_COM = NULL;
			delete m_COMShape; m_COMShape = NULL;
		}
}

void Creature::switchCOM() {
	m_showCOM = !m_showCOM;
	if (m_showCOM) {
		// Shape
		m_COMShape = new btSphereShape(btScalar(0.05));
		m_COMShape->setColor(btVector3(btScalar(0.6),btScalar(1.0),btScalar(0.6)));
		// Body
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		m_COM = m_ownerWorld->localCreateRigidBody(btScalar(0.0), transform, m_COMShape);
		m_COM->setCollisionFlags(m_COM->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
		m_COM->setActivationState(DISABLE_DEACTIVATION);
	}
	else {
		m_ownerWorld->removeRigidBody(m_COM);
		delete m_COM->getMotionState();
		delete m_COM; m_COM = NULL;
		delete m_COMShape; m_COMShape = NULL;	
	}	
}

void Creature::update(int elapsedTime) {
	
	// BALANCE CONTROLLER
	// ==================

	// Step 1.1: Compute the COM in world coordinate system
	btVector3 comInWorld = computeCenterOfMass();
	m_positionCOM = comInWorld;
	if (m_showCOM) { // Visualize COM
		btTransform transform;
		m_COM->getMotionState()->getWorldTransform(transform);
		transform.setOrigin(comInWorld);
		m_COM->getMotionState()->setWorldTransform(transform);
	}

	// Step 1.2: Update pose only if creature did not fall
	if (m_hasFallen) {
		if (((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->getEnableAngularMotor()) { // ragdoll is fallen
			((btHingeConstraint*)m_joints[Creature::JOINT_ANKLE])->enableMotor(false);
			((btHingeConstraint*)m_joints[Creature::JOINT_KNEE])->enableMotor(false);
		}
		return;
	}			

	if (elapsedTime - lastChange > m_time_step) { // Update balance control only every 10 ms
		lastChange = elapsedTime;
		//target orientation is vertical direction
		btQuaternion targetOrientation = btQuaternion(0.0, 0.0, 0.0, 1.0);
		// for each body part
		for (int i = 0; i < 3; ++i) {
			// set angular velocity to 0
			m_bodies[i]->setAngularVelocity(btVector3(0.0,0.0,0.0));
			// get oritentation of this body part
			btQuaternion bodyOrientation = m_bodies[i]->getOrientation();
			// get angle differartion
			btQuaternion deltaOrientation = targetOrientation * bodyOrientation.inverse();
			// compute euler angle
			btVector3 deltaEuler = QuaternionToEulerXYZ(deltaOrientation);
			// PID controller, but apply to vector.
			btVector3 torque = m_PIDs[i]->solve(-deltaEuler, m_time_step);
			// apply torque to body, instead of to joints
			m_bodies[i]->applyTorque(-torque);
		}
	}
}

bool Creature::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // true if already down (cannot get back up here)
	if (m_bodies[BODYPART_LOWER_LEG]->getActivationState() == ISLAND_SLEEPING) m_hasFallen = true; // true if enters in sleeping mode
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY() < 0.15 ||
		m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() < 0.15) m_hasFallen = true; // true if a creature has fallen from platform
	if (m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_UPPER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	if (m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() > m_bodies[BODYPART_LOWER_LEG]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	return m_hasFallen;
}

btVector3 Creature::computeCenterOfMass() {

	//=================== TODO ==================//
	// Compute COM of each object, return the weighted average
	btScalar totalMass = 0.0f;
	btVector3 weightedCOM(0, 0, 0);
	for (int i = 0; i < Creature::BODYPART_COUNT; ++i)
	{
		btScalar bodyMass = 1.0f / m_bodies[i]->getInvMass();
		totalMass += bodyMass;
		btVector3 COM_position = m_bodies[i]->getCenterOfMassPosition() * bodyMass;
		weightedCOM += COM_position;
	}

	weightedCOM /= totalMass;

	return weightedCOM;
}

btVector3 Creature::QuaternionToEulerXYZ(const btQuaternion &quat)
{
	btVector3 euler;
	double w = quat.getW();   double x = quat.getX();   double y = quat.getY();   double z = quat.getZ();
	double sqw = w*w; double sqx = x*x; double sqy = y*y; double sqz = z*z;
	euler.setZ((atan2(2.0 * (x*y + z*w), (sqx - sqy - sqz + sqw))));
	euler.setX((atan2(2.0 * (y*z + x*w), (-sqx - sqy + sqz + sqw))));
	euler.setY((asin(-2.0 * (x*z - y*w))));
	return euler;
}
btVector3 Creature::control(btVector3& in)
{
	// play around with the factor until you find a matching one
	float Kp = 5.5;
	return in * Kp;
}