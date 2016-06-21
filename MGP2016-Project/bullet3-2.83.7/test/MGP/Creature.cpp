
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

// Tune parameters here
#define K_P_ANKLE	100.0f
#define K_I_ANKLE	0.0f
#define K_D_ANKLE	100.0f

#define K_P_KNEE	150.0f
#define K_I_KNEE	0.0f
#define K_D_KNEE	10.0f

#define ACTION_BIAS	0.00001f

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

		// Setup the joint constraints
		// ===========================

		btHingeConstraint* hingeJoint;
		//FYI, another type of joint is for example: btConeTwistConstraint* coneJoint;
		btTransform localA, localB;
		
		// ANKLE
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.025), btScalar(0.0)));
		localB.getBasis().setEulerZYX(0,btScalar(M_PI_2),0); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.25), btScalar(0.0)));
		hingeJoint =  new btHingeConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LOWER_LEG], localA, localB);
		hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));
		
		hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control
		
		m_joints[Creature::JOINT_ANKLE] = hingeJoint;
		hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE], true);

		// KNEE
		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.25), btScalar(0.0)));
		localB.getBasis().setEulerZYX(0,0,btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.20), btScalar(0.0)));
		hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG], localA, localB);
		hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

		hingeJoint->enableAngularMotor(true,btScalar(0.0),btScalar(50.0)); //uncomment to allow for torque control

		m_joints[Creature::JOINT_KNEE] = hingeJoint;
		hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);

		// Setup the PID controllers
		// =========================
		PIDController* pidController;

		// ANKLE
		pidController = new PIDController(K_P_ANKLE, K_I_ANKLE, K_D_ANKLE);
		m_PIDs[Creature::JOINT_ANKLE] = pidController;
		
		// KNEE
		pidController = new PIDController(K_P_KNEE, K_I_KNEE, K_D_KNEE);
		m_PIDs[Creature::JOINT_KNEE] = pidController;
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

		//=================== TODO ===================//
		// CSP := Centre of Support Polygon
		// Step 2: Describe the ground projected CSP in world coordinate system
		// The Support Polygon should be the foot (the Box shape thing), for now use the COM of the foot
		btVector3 CSP, CSP_project, COM_project;
		CSP = m_bodies[Creature::BODYPART_FOOT]->getCenterOfMassPosition();
		// The ground-projected CSP
		CSP_project = btVector3(CSP.x(), 0.0f, CSP.z());
		// The ground-projected COM
		COM_project = btVector3(m_positionCOM.x(), 0.0f, m_positionCOM.z());
		
		// ANKLE
		// -----
		btVector3 CSP_project_foot, COM_project_foot;
		btTransform foot_system = m_bodies[Creature::BODYPART_FOOT]->getWorldTransform().inverse();
		// Step 3.1: Describe the ground projected CSP in foot coordinate system
		CSP_project_foot = foot_system * CSP_project;	// What for?
		// Step 3.2: Describe the ground projected COM in foot coordinate system
		COM_project_foot = foot_system * COM_project;	// What for?
		// Step 3.3: Calculate the balance error solveable by an ankle rotation (inverted pendulum model)		
		btVector3 errorVect = CSP_project - COM_project;
		btScalar errorSize = errorVect.norm();
		if (abs(errorSize) > ACTION_BIAS)	// Put a threshould here
		{
			// Get the orthonormal axis where the Hinge rotates around
			btVector3 newAxis(	-1.0f*errorVect.z() + EPSILON,
								0.0f,
								1.0f*errorVect.x() + EPSILON	);
			newAxis.normalize();
		
			m_joints[Creature::JOINT_ANKLE]->setAxis(newAxis);
			btScalar torque_ankle = m_PIDs[Creature::JOINT_ANKLE]->solve(errorSize, m_time_step);
			// Step 3.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			m_joints[Creature::JOINT_ANKLE]->setMotorTarget(torque_ankle, m_time_step);
		
			// KNEE
			// ----
			btVector3 CSP_project_leg, COM_project_leg;
			btTransform leg_system = m_bodies[Creature::BODYPART_LOWER_LEG]->getWorldTransform().inverse();
			// Step 4.1: Describe the ground projected CSP in lower leg coordinate system
			CSP_project_leg = leg_system * CSP_project;		// What for?
			// Step 4.2: Describe the ground projected COM in lower leg coordinate system
			COM_project_leg = leg_system * COM_project;		// What for?
			// Step 4.3: Calculate the balance error solveable by a knee rotation (inverted pendulum model)
			m_joints[Creature::JOINT_KNEE]->setAxis(newAxis);
			btScalar torque_knee = m_PIDs[Creature::JOINT_KNEE]->solve(errorSize, m_time_step);
			// Step 4.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			m_joints[Creature::JOINT_KNEE]->setMotorTarget(torque_knee, m_time_step);
		}
		//===========================================//

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
