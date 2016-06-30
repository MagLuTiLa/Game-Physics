
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
	//*m_shapes = new btCollisionShape[6];
		m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(0.1),btScalar(0.025),btScalar(0.12)));
		m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		/*m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.50));
		m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.40));
		m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
*/
		m_shapes[Creature::BODYPART_LEG_0] = new btCapsuleShape(btScalar(0.05), btScalar(0.30));
		m_shapes[Creature::BODYPART_LEG_1] = new btCapsuleShape(btScalar(0.05), btScalar(0.25));
		m_shapes[Creature::BODYPART_LEG_2] = new btCapsuleShape(btScalar(0.05), btScalar(0.20));
		m_shapes[Creature::BODYPART_LEG_3] = new btCapsuleShape(btScalar(0.05), btScalar(0.15));
		m_shapes[Creature::BODYPART_LEG_4] = new btCapsuleShape(btScalar(0.05), btScalar(0.10));
		for (int i = 1; i < 6; ++i) 
			m_shapes[i]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
		
		// Setup the body properties
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset); // absolute initial starting position
		btTransform transform;

		// FOOT
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(5.0), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.175), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LEG_0] = m_ownerWorld->localCreateRigidBody(btScalar(3.0), offset*transform, m_shapes[Creature::BODYPART_LEG_0]);
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.15), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LEG_1] = m_ownerWorld->localCreateRigidBody(btScalar(2.5), offset*transform, m_shapes[Creature::BODYPART_LEG_1]);
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.125), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LEG_2] = m_ownerWorld->localCreateRigidBody(btScalar(2.0), offset*transform, m_shapes[Creature::BODYPART_LEG_2]);
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.1), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LEG_3] = m_ownerWorld->localCreateRigidBody(btScalar(1.5), offset*transform, m_shapes[Creature::BODYPART_LEG_3]);
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.075), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LEG_4] = m_ownerWorld->localCreateRigidBody(btScalar(1.0), offset*transform, m_shapes[Creature::BODYPART_LEG_4]);


		targetOrientation = m_bodies[Creature::BODYPART_FOOT]->getOrientation();

		// Add damping to the rigid bodies
		for (int i = 1; i < 6; ++i) {
			m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
			m_bodies[i]->setDeactivationTime(btScalar(0.01));
			m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
		}
		m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(1.8), btScalar(0.01)); // Higher friction for foot

		// Setup the ball-socket joint constraints
		// ===========================
		btPoint2PointConstraint* balljoint;
		btVector3 foot_leg0_joint = btVector3(0.0, 0.025, 0.0);
		btVector3 leg0_foot_joint = btVector3(0.0, -0.15, 0.0);
		btVector3 leg0_leg1 = btVector3(0.0, 0.15, 0.0);
		btVector3 leg1_leg0 = btVector3(0.0, -0.125, 0.0);
		btVector3 leg1_leg2 = btVector3(0.0, 0.125, 0.0);
		btVector3 leg2_leg1 = btVector3(0.0, -0.10, 0.0);
		btVector3 leg2_leg3 = btVector3(0.0, 0.10, 0.0);
		btVector3 leg3_leg2 = btVector3(0.0, -0.075, 0.0);
		btVector3 leg3_leg4 = btVector3(0.0, 0.075, 0.0);
		btVector3 leg4_leg3 = btVector3(0.0, -0.05, 0.0);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LEG_0],
			foot_leg0_joint, leg0_foot_joint);
		m_joints[Creature::JOINT_0] = balljoint;
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_0],true);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LEG_0], *m_bodies[Creature::BODYPART_LEG_1],
			leg0_leg1, leg1_leg0);
		m_joints[Creature::JOINT_1] = balljoint;
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_1],true);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LEG_1], *m_bodies[Creature::BODYPART_LEG_2],
			leg1_leg2, leg2_leg1);
		m_joints[Creature::JOINT_2] = balljoint;
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_2], true);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LEG_2], *m_bodies[Creature::BODYPART_LEG_3],
			leg2_leg3, leg3_leg2);
		m_joints[Creature::JOINT_3] = balljoint;
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_3], true);

		balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LEG_3], *m_bodies[Creature::BODYPART_LEG_4],
			leg3_leg4, leg4_leg3);
		m_joints[Creature::JOINT_4] = balljoint;
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_4], true);



		// Setup the PID controllers (every bodypart has its own PID)
		// =========================
		PIDController* pidController;

		// foot
		pidController = new PIDController(8.0f, 0.0f, 8.0f);
		m_PIDs[Creature::BODYPART_FOOT] = pidController;

		// lower_leg
		pidController = new PIDController(35.0, 0.0f, 35.0f);
		for (int i = 1; i < BODYPART_COUNT; ++i)
			m_PIDs[i] = pidController;
		
		op_flag = true;
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

void Creature::update(int elapsedTime, float ms) {
	// different computer has different properties...so...
	if (op_flag && ms > 2000 && ms < 3000) {
		// foot
		PIDController* pidController;
		pidController = new PIDController(50.0f, 0.0f, 50.0f);
		m_PIDs[Creature::BODYPART_FOOT] = pidController;

		// lower_leg
		pidController = new PIDController(80.0, 0.03f, 80.0f);
		for (int i = 1; i < 6; ++i) {
			m_PIDs[i] = pidController;
		}

		op_flag = false;
	}
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
		return;
	}			

	if (elapsedTime - lastChange > m_time_step) { // Update balance control only every 10 ms
		//target orientation is vertical direction
		// for each body part
		for (int i = 0; i < 6; ++i) {
			// set angular velocity to 0
			m_bodies[i]->setAngularVelocity(btVector3(0.0,0.0,0.0));
			// get oritentation of this body part
			btQuaternion bodyOrientation = m_bodies[i]->getOrientation();
			// get angle differartion
			btQuaternion deltaOrientation = targetOrientation * bodyOrientation.inverse();
			// compute euler angle
			btVector3 deltaEuler = QuaternionToEulerXYZ(deltaOrientation);
			if (deltaEuler.norm() > 0.01) {
				// PID controller, but apply to vector.
				btVector3 torque;
				if(i!=5) torque = m_PIDs[i]->solve(deltaEuler, m_time_step);
				else torque = m_PIDs[5]->solve(deltaEuler, m_time_step);
				// apply torque impulse to body, instead of to joints
				if (op_flag && ms > 2000 && ms < 3000) {
					m_bodies[i]->applyTorqueImpulse(torque*ms / 100000.0);
				}
				else m_bodies[i]->applyTorqueImpulse(torque*ms / 1000000.0);
			}
		}
		lastChange = elapsedTime;
	}
}

bool Creature::hasFallen() {
	if (m_hasFallen) return m_hasFallen; // true if already down (cannot get back up here)
	for (int i = 1; i < 6; ++i) {
		if (m_bodies[i]->getActivationState() == ISLAND_SLEEPING) m_hasFallen = true; // true if enters in sleeping mode
		if (m_bodies[i]->getCenterOfMassPosition().getY() < 0.20 ||
			m_bodies[i]->getCenterOfMassPosition().getY() < 0.20 ||
			m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() < 0.20) m_hasFallen = true; // true if a creature has fallen from platform
		if (m_bodies[i]->getCenterOfMassPosition().getY() > m_bodies[i]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
		if (m_bodies[BODYPART_FOOT]->getCenterOfMassPosition().getY() > m_bodies[i]->getCenterOfMassPosition().getY()) m_hasFallen = true; // true if align with ground
	}
	return m_hasFallen;
}

btVector3 Creature::computeCenterOfMass() {

	//=================== TODO ==================//
	// Compute COM of each object, return the weighted average
	btScalar totalMass = 0.0f;
	btVector3 weightedCOM(0, 0, 0);
	for (int i = 0; i < 6; ++i)
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
	double w = quat.getW();  
	double x = quat.getX();  
	double y = quat.getY(); 
	double z = quat.getZ();
	double sqw = w*w; 
	double sqx = x*x; 
	double sqy = y*y;
	double sqz = z*z;
	euler.setZ((atan2(2.0 * (x*y + z*w), (sqx - sqy - sqz + sqw))));
	euler.setX((atan2(2.0 * (y*z + x*w), (-sqx - sqy + sqz + sqw))));
	euler.setY((asin(-2.0 * (x*z - y*w))));
	return euler;
}
