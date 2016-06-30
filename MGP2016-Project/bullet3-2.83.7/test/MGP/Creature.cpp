
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

// Tune PID parameters here, please tune each set separately, i.e. p&i&d for different modes
#if defined BASIC_BALANCE
	#define K_P_ANKLE	200.0f
	#define K_I_ANKLE	0.01f
	#define K_D_ANKLE	10.0f

	#define K_P_KNEE	150.0f
	#define K_I_KNEE	0.1f
	#define K_D_KNEE	10.0f

#elif defined EXTRA_LIMB
	#define K_P_ANKLE	200.0f
	#define K_I_ANKLE	0.01f
	#define K_D_ANKLE	10.0f

	#define K_P_KNEE	150.0f
	#define K_I_KNEE	0.1f
	#define K_D_KNEE	50.0f

	#define K_P_HIP		100.0f
	#define K_I_HIP		0.0f
	#define K_D_HIP		50.0f

#elif defined ADV_BALANCE
	#define K_P_FOOT	8.0f
	#define K_I_FOOT	0.0f
	#define K_D_FOOT	8.0f

	#define K_P_L_LEG	45.0f
	#define K_I_L_LEG	0.0f
	#define K_D_L_LEG	45.0f

	#define K_P_U_LEG	39.0f
	#define K_I_U_LEG	0.0f
	#define K_D_U_LEG	39.0f

#endif

// Mode swtichtes are moved to header, CREATURE_H_

// Leg geometry properties, including capsule radius, height, box extents, and mass of objects
#if defined BASIC_BALANCE
	#define FOOT_W 0.100		// Width of the foot box
	#define FOOT_L 0.120		// Length of the foot box 
	#define FOOT_H 0.025		// Height of the foot box
	#define FOOT_M 5.0			// Mass of FOOT
	#define FOOT_DAMP 0.80		// Friction of FOOT
	#define LOW_LEG_R 0.05		// Radius of lower leg
	#define LOW_LEG_H 0.50		// Height of lower leg
	#define LOW_LEG_M 3.0		// Mass of lower leg
	#define UP_LEG_R 0.05		// Radius of upper leg
	#define UP_LEG_H 0.40		// Height of upper leg
	#define UP_LEG_M 3.0		// Mass of upper leg

#elif defined EXTRA_LIMB
	#define FOOT_W 0.100		// Width of the foot box
	#define FOOT_L 0.120		// Length of the foot box 
	#define FOOT_H 0.025		// Height of the foot box
	#define FOOT_M 6.0			// Mass of FOOT
	#define FOOT_DAMP 0.80		// Friction of FOOT
	#define LOW_LEG_R 0.05		// Radius of lower leg
	#define LOW_LEG_H 0.50		// Height of lower leg
	#define LOW_LEG_M 3.0		// Mass of lower leg
	#define UP_LEG_R 0.05		// Radius of upper leg
	#define UP_LEG_H 0.40		// Height of upper leg
	#define UP_LEG_M 2.0		// Mass of upper leg
	#define TORSO_R 0.05		// Radius of torso
	#define TORSO_H 0.30		// Height of torso
	#define TORSO_M 1.0			// Mass of torso

#elif defined ADV_BALANCE
	#define FOOT_W 0.100		// Width of the foot box
	#define FOOT_L 0.120		// Length of the foot box 
	#define FOOT_H 0.025		// Height of the foot box
	#define FOOT_M 5.0			// Mass of FOOT
	#define FOOT_DAMP 1.00		// Friction of FOOT
	#define LOW_LEG_R 0.05		// Radius of lower leg
	#define LOW_LEG_H 0.50		// Height of lower leg
	#define LOW_LEG_M 3.0		// Mass of lower leg
	#define UP_LEG_R 0.05		// Radius of upper leg
	#define UP_LEG_H 0.40		// Height of upper leg
	#define UP_LEG_M 3.0		// Mass of upper leg

#endif

Creature::Creature (btDynamicsWorld* ownerWorld, const btVector3& positionOffset) : m_ownerWorld (ownerWorld), m_hasFallen(false), lastChange(0), m_showCOM(false), m_time_step(10.0f) { // Constructor
#if defined BASIC_BALANCE		
		// Setup the rigid bodies
		// ======================

		// Setup the collision shapes
		m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(FOOT_W),btScalar(FOOT_H),btScalar(FOOT_L)));
		m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(LOW_LEG_R), btScalar(LOW_LEG_H));
		m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));
		m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(UP_LEG_R), btScalar(UP_LEG_H));
		m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6),btScalar(0.6),btScalar(0.6)));

		// Setup the body properties
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset); // absolute initial starting position
		btTransform transform;
		
		// FOOT
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
		m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(FOOT_M), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

		// LOWER_LEG
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
		m_bodies[Creature::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(LOW_LEG_M), offset*transform, m_shapes[Creature::BODYPART_LOWER_LEG]);

		// UPPER_LEG
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
		m_bodies[Creature::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(UP_LEG_M), offset*transform, m_shapes[Creature::BODYPART_UPPER_LEG]);

		// Add damping to the rigid bodies
		for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
			m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
			m_bodies[i]->setDeactivationTime(btScalar(0.01));
			m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
		}
		m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(FOOT_DAMP), btScalar(0.01)); // Higher friction for foot

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

#elif defined EXTRA_LIMB
	// Setup rigid bodies
	m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(FOOT_W), btScalar(FOOT_H), btScalar(FOOT_L)));
	m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
	m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(LOW_LEG_R), btScalar(LOW_LEG_H));
	m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
	m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(UP_LEG_R), btScalar(UP_LEG_H));
	m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
	m_shapes[Creature::BODYPART_TORSO] = new btCapsuleShape(btScalar(TORSO_R), btScalar(TORSO_H));
	m_shapes[Creature::BODYPART_TORSO]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));

	// Setup the body properties
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); // absolute initial starting position
	btTransform transform;

	// FOOT
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
	m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(FOOT_M), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

	// LOWER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
	m_bodies[Creature::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(LOW_LEG_M), offset*transform, m_shapes[Creature::BODYPART_LOWER_LEG]);

	// UPPER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
	m_bodies[Creature::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(UP_LEG_M), offset*transform, m_shapes[Creature::BODYPART_UPPER_LEG]);

	// TORSO
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(1.075), btScalar(0.0)));
	m_bodies[Creature::BODYPART_TORSO] = m_ownerWorld->localCreateRigidBody(btScalar(TORSO_M), offset*transform, m_shapes[Creature::BODYPART_TORSO]);

	// Add damping to the rigid bodies
	for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
		m_bodies[i]->setDeactivationTime(btScalar(0.01));
		m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
	}
	m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(1.0), btScalar(0.01)); // Higher friction for foot

	// Setup the joint constraints
	// ===========================

	btHingeConstraint* hingeJoint;
	//FYI, another type of joint is for example: btConeTwistConstraint* coneJoint;
	btTransform localA, localB;

	// ANKLE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, btScalar(M_PI_2), 0); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.025), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0, btScalar(M_PI_2), 0); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.25), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_FOOT], *m_bodies[Creature::BODYPART_LOWER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true, btScalar(0.0), btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Creature::JOINT_ANKLE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE], true);

	// KNEE
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.25), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0, 0, btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.20), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2), btScalar(M_PI_2));

	hingeJoint->enableAngularMotor(true, btScalar(0.0), btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Creature::JOINT_KNEE] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_KNEE], true);

	// HIP
	localA.setIdentity(); localB.setIdentity();
	localA.getBasis().setEulerZYX(0, 0, btScalar(M_PI_2)); localA.setOrigin(btVector3(btScalar(0.0), btScalar(0.20), btScalar(0.0)));
	localB.getBasis().setEulerZYX(0, 0, btScalar(M_PI_2)); localB.setOrigin(btVector3(btScalar(0.0), btScalar(-0.15), btScalar(0.0)));
	hingeJoint = new btHingeConstraint(*m_bodies[Creature::BODYPART_UPPER_LEG], *m_bodies[Creature::BODYPART_TORSO], localA, localB);
	hingeJoint->setLimit(btScalar(-M_PI_2 + EPSILON), btScalar(M_PI_2 - EPSILON));

	hingeJoint->enableAngularMotor(true, btScalar(0.0), btScalar(50.0)); //uncomment to allow for torque control

	m_joints[Creature::JOINT_HIP] = hingeJoint;
	hingeJoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[JOINT_HIP], true);

	// Setup the PID controllers
	// =========================
	PIDController* pidController;

	// ANKLE
	pidController = new PIDController(K_P_ANKLE, K_I_ANKLE, K_D_ANKLE);
	m_PIDs[Creature::JOINT_ANKLE] = pidController;

	// KNEE
	pidController = new PIDController(K_P_KNEE, K_I_KNEE, K_D_KNEE);
	m_PIDs[Creature::JOINT_KNEE] = pidController;

	// HIP
	pidController = new PIDController(K_P_HIP, K_I_HIP, K_D_HIP);
	m_PIDs[Creature::JOINT_HIP] = pidController;

#elif defined ADV_BALANCE
	// Setup the rigid bodies
	// ======================

	// Setup the collision shapes
	m_shapes[Creature::BODYPART_FOOT] = new btBoxShape(btVector3(btScalar(FOOT_W), btScalar(FOOT_H), btScalar(FOOT_L)));
	m_shapes[Creature::BODYPART_FOOT]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
	m_shapes[Creature::BODYPART_LOWER_LEG] = new btCapsuleShape(btScalar(LOW_LEG_R), btScalar(LOW_LEG_H));
	m_shapes[Creature::BODYPART_LOWER_LEG]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));
	m_shapes[Creature::BODYPART_UPPER_LEG] = new btCapsuleShape(btScalar(UP_LEG_R), btScalar(UP_LEG_H));
	m_shapes[Creature::BODYPART_UPPER_LEG]->setColor(btVector3(btScalar(0.6), btScalar(0.6), btScalar(0.6)));

	// Setup the body properties
	btTransform offset; offset.setIdentity();
	offset.setOrigin(positionOffset); // absolute initial starting position
	btTransform transform;

	// FOOT
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.0), btScalar(0.0)));
	m_bodies[Creature::BODYPART_FOOT] = m_ownerWorld->localCreateRigidBody(btScalar(FOOT_M), offset*transform, m_shapes[Creature::BODYPART_FOOT]);

	// LOWER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.275), btScalar(0.0)));
	m_bodies[Creature::BODYPART_LOWER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(LOW_LEG_M), offset*transform, m_shapes[Creature::BODYPART_LOWER_LEG]);

	// UPPER_LEG
	transform.setIdentity();
	transform.setOrigin(btVector3(btScalar(0.0), btScalar(0.725), btScalar(0.0)));
	m_bodies[Creature::BODYPART_UPPER_LEG] = m_ownerWorld->localCreateRigidBody(btScalar(UP_LEG_M), offset*transform, m_shapes[Creature::BODYPART_UPPER_LEG]);

	targetOrientation = m_bodies[Creature::BODYPART_FOOT]->getOrientation();

	// Add damping to the rigid bodies
	for (int i = 0; i < Creature::BODYPART_COUNT; ++i) {
		m_bodies[i]->setDamping(btScalar(0.01), btScalar(0.01));
		m_bodies[i]->setDeactivationTime(btScalar(0.01));
		m_bodies[i]->setSleepingThresholds(btScalar(5.0), btScalar(5.0));
	}
	m_bodies[Creature::BODYPART_FOOT]->setDamping(btScalar(FOOT_DAMP), btScalar(0.01)); // Higher friction for foot
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
	m_ownerWorld->addConstraint(m_joints[Creature::JOINT_ANKLE], true);

	balljoint = new btPoint2PointConstraint(*m_bodies[Creature::BODYPART_LOWER_LEG], *m_bodies[Creature::BODYPART_UPPER_LEG],
											lowerleg_upperleg_joint, upperleg_lowerleg_joint);
	m_joints[Creature::JOINT_KNEE] = balljoint;
	balljoint->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
	m_ownerWorld->addConstraint(m_joints[Creature::JOINT_KNEE], true);

	// Setup the PID controllers (every bodypart has its own PID)
	// =========================
	PIDController* pidController;

	// foot
	pidController = new PIDController(K_P_FOOT, K_I_FOOT, K_D_FOOT);
	m_PIDs[Creature::BODYPART_FOOT] = pidController;					
																		
	// lower_leg
	pidController = new PIDController(K_P_L_LEG, K_I_L_LEG, K_D_L_LEG);
	m_PIDs[Creature::BODYPART_LOWER_LEG] = pidController;				
																		
	// upper_leg
	pidController = new PIDController(K_P_U_LEG, K_I_U_LEG, K_D_U_LEG);
	m_PIDs[Creature::BODYPART_UPPER_LEG] = pidController;				
																		
	op_flag = true;
	
#elif defined MANY_JOINT

#endif
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

#if defined BASIC_BALANCE

	if (elapsedTime - lastChange > m_time_step) { // Update balance control only every 10 ms
		lastChange = elapsedTime;

		//=================== TODO ===================//
		// CSP := Centre of Support Polygon, for now use the COM of the foot, because the box is really thin
		// Step 2: Describe the ground projected CSP in world coordinate system

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

		if (abs(errorVect.norm()) > ACTION_BIAS)	// Put a threshould here
		{
			btVector3 error_foot = CSP_project_foot - COM_project_foot;

			// Step 3.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			btScalar torque_ankle = m_PIDs[Creature::JOINT_ANKLE]->solve(-1.0f*error_foot.z(), m_time_step);
			//m_joints[Creature::JOINT_ANKLE]->setMotorTarget(torque_ankle, m_time_step);			// This one uses 
			m_joints[Creature::JOINT_ANKLE]->setMotorTargetVelocity(torque_ankle / m_time_step);	// This one uses velocity

			// KNEE
			// ----
			btVector3 CSP_project_leg, COM_project_leg;
			btTransform leg_system = m_bodies[Creature::BODYPART_LOWER_LEG]->getWorldTransform().inverse();
			// Step 4.1: Describe the ground projected CSP in lower leg coordinate system
			CSP_project_leg = leg_system * CSP_project;
			
			// Step 4.2: Describe the ground projected COM in lower leg coordinate system
			COM_project_leg = leg_system * COM_project;
			
			// Step 4.3: Calculate the balance error solveable by a knee rotation (inverted pendulum model)
			btVector3 error_leg = CSP_project_leg - COM_project_leg;
			
			// Step 4.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			btScalar torque_knee = m_PIDs[Creature::JOINT_KNEE]->solve(error_leg.x(), m_time_step);
			//m_joints[Creature::JOINT_KNEE]->setMotorTarget(torque_knee, m_time_step);			
			m_joints[Creature::JOINT_KNEE]->setMotorTargetVelocity(torque_knee / m_time_step);
		}
		//===========================================//
	}
#elif defined EXTRA_LIMB
	if (elapsedTime - lastChange > m_time_step) { // Update balance control only every 10 ms
		lastChange = elapsedTime;

		//=================== TODO ===================//
		// CSP := Centre of Support Polygon, for now use the COM of the foot, because the box is really thin
		// Step 2: Describe the ground projected CSP in world coordinate system

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
		CSP_project_foot = foot_system * CSP_project;
		// Step 3.2: Describe the ground projected COM in foot coordinate system
		COM_project_foot = foot_system * COM_project;
		// Step 3.3: Calculate the balance error solveable by an ankle rotation (inverted pendulum model)		
		btVector3 errorVect = CSP_project - COM_project;

		if (abs(errorVect.norm()) > ACTION_BIAS)	// Put a threshould here
		{
			btVector3 error_foot = CSP_project_foot - COM_project_foot;

			// Step 3.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			btScalar torque_ankle = m_PIDs[Creature::JOINT_ANKLE]->solve(-1.0f*error_foot.z(), m_time_step);
			//m_joints[Creature::JOINT_ANKLE]->setMotorTarget(torque_ankle, m_time_step);			// This one uses 
			m_joints[Creature::JOINT_ANKLE]->setMotorTargetVelocity(torque_ankle / m_time_step);	// This one uses velocity

			// KNEE
			// ----
			btVector3 CSP_project_leg, COM_project_leg;
			btTransform leg_system = m_bodies[Creature::BODYPART_LOWER_LEG]->getWorldTransform().inverse();
			// Step 4.1: Describe the ground projected CSP in lower leg coordinate system
			CSP_project_leg = leg_system * CSP_project;

			// Step 4.2: Describe the ground projected COM in lower leg coordinate system
			COM_project_leg = leg_system * COM_project;

			// Step 4.3: Calculate the balance error solveable by a knee rotation (inverted pendulum model)
			btVector3 error_leg = CSP_project_leg - COM_project_leg;
			// Only correct a weighted part
			error_leg *= UP_LEG_M / (UP_LEG_M + TORSO_M);

			// Step 4.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			btScalar torque_knee = m_PIDs[Creature::JOINT_KNEE]->solve(error_leg.x(), m_time_step);
			//m_joints[Creature::JOINT_KNEE]->setMotorTarget(torque_knee, m_time_step);			
			m_joints[Creature::JOINT_KNEE]->setMotorTargetVelocity(torque_knee / m_time_step);

			// TORSO
			// ----
			btVector3 CSP_project_torso, COM_project_torso;
			btTransform torso_system = m_bodies[Creature::BODYPART_TORSO]->getWorldTransform().inverse();
			// Step 5.1: Describe the ground projected CSP in lower leg coordinate system
			CSP_project_torso = torso_system * CSP_project;

			// Step 5.2: Describe the ground projected COM in lower leg coordinate system
			COM_project_torso = torso_system * COM_project;

			// Step 5.3: Calculate the balance error solveable by a knee rotation (inverted pendulum model)
			btVector3 error_torso = CSP_project_torso - COM_project_torso;
			// Only correct a weighted part
			error_torso *= TORSO_M / (UP_LEG_M + TORSO_M);

			// Step 5.4: Feed the error to the PD controller and apply resulting 'torque' (here angular motor velocity)
			// (Conversion between error to torque/motor velocity done by gains in PD controller)
			btScalar torque_hip = m_PIDs[Creature::JOINT_HIP]->solve(error_torso.x(), m_time_step);
			//m_joints[Creature::JOINT_KNEE]->setMotorTarget(torque_knee, m_time_step);			
			m_joints[Creature::JOINT_HIP]->setMotorTargetVelocity(torque_hip / m_time_step);
		}
		//===========================================//
	}
#elif defined MANY_JOINT

#endif
}

#if defined ADV_BALANCE
void Creature::update(int elapsedTime, float ms)
{
	// different computer has different properties, this method is used to better sync timestep
	if (op_flag && ms > 2000 && ms < 3000)
	{
		// foot
		PIDController* pidController;
		pidController = new PIDController(50.0f, 0.0f, 50.0f);
		m_PIDs[Creature::BODYPART_FOOT] = pidController;

		// lower_leg
		pidController = new PIDController(80.0, 0.03f, 80.0f);
		m_PIDs[Creature::BODYPART_LOWER_LEG] = pidController;

		// upper_leg
		pidController = new PIDController(80.0f, 0.05f, 80.0f);
		m_PIDs[Creature::BODYPART_UPPER_LEG] = pidController;

		op_flag = false;
	}
	// BALANCE CONTROLLER
	// ==================

	// Step 1.1: Compute the COM in world coordinate system
	btVector3 comInWorld = computeCenterOfMass();
	m_positionCOM = comInWorld;
	if (m_showCOM)
	{ // Visualize COM
		btTransform transform;
		m_COM->getMotionState()->getWorldTransform(transform);
		transform.setOrigin(comInWorld);
		m_COM->getMotionState()->setWorldTransform(transform);
	}

	// Step 1.2: Update pose only if creature did not fall
	if (m_hasFallen)
	{
		return;
	}

	if (elapsedTime - lastChange > m_time_step)
	{	// Update balance control only every 10 ms
		//target orientation is vertical direction for each body part
		for (int i = 0; i < 3; ++i)
		{
			// set angular velocity to 0
			m_bodies[i]->setAngularVelocity(btVector3(0.0, 0.0, 0.0));
			// get oritentation of this body part
			btQuaternion bodyOrientation = m_bodies[i]->getOrientation();
			// get angle differartion
			btQuaternion deltaOrientation = targetOrientation * bodyOrientation.inverse();
			// compute euler angle
			btVector3 deltaEuler = QuaternionToEulerXYZ(deltaOrientation);
			if (deltaEuler.norm() > 0.01)
			{
				// PID controller, but apply to vector.
				//btVector3 torque = control(deltaEuler);
				btVector3 torque = m_PIDs[i]->solve(deltaEuler, m_time_step);
				// apply torque impulse to body, instead of to joints
				if(!op_flag)
					m_bodies[i]->applyTorqueImpulse(torque*ms * 0.00001);
				else m_bodies[i]->applyTorqueImpulse(torque*ms * 0.000001);
				//m_bodies[i]->applyTorque(torque);
			}
		}
		lastChange = elapsedTime;
	}
}
#endif

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
	double w = quat.getW();
	double x = quat.getX();
	double y = quat.getY();
	double z = quat.getZ();
	double sqw = w*w;
	double sqx = x*x;
	double sqy = y*y;
	double sqz = z*z;
	euler.setZ(float((atan2(2.0f * (x*y + z*w), (sqx - sqy - sqz + sqw)))));
	euler.setX(float((atan2(2.0f * (y*z + x*w), (-sqx - sqy + sqz + sqw)))));
	euler.setY(float((asin(-2.0f * (x*z - y*w)))));
	return euler;
}