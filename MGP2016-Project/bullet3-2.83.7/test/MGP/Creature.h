
#ifndef CREATURE_H
#define CREATURE_H

#include "btBulletDynamicsCommon.h"

// Switch Modes, modify to extend modes
#if 1		
#define BASIC_BALANCE		// Basic balancing mode
#elif 0		
#define EXTRA_LIMB			// Balancing mode with extra limb
#elif 0		
#define ADV_BALANCE			// Advanced balancing mode
#elif 0		
#define POS_DEPEND			// The pose-dependent balancing mode
#endif

class PIDController;

class Creature {

public:
	Creature(btDynamicsWorld* ownerWorld, const btVector3& positionOffset, double* PID_Parameters); // Constructor

	virtual	~Creature();										// Destructor

	void update(int elapsedTime);								// Update the creature state
#if defined ADV_BALANCE
	void update(int elapsedTime, float ms);						// Update the creature state with two parameters, for sync
#endif
	bool hasFallen();											// Return if the creature has fallen down
	void switchCOM();											// Activate / Deactivate the visualization of the COM

	btVector3 getCOM() {return m_positionCOM;}					// Return the position of the COM
	btVector3 QuaternionToEulerXYZ(const btQuaternion &quat);	// Convert quaternion to euler angle

protected:
#if defined BASIC_BALANCE
	enum {BODYPART_FOOT,BODYPART_LOWER_LEG,BODYPART_UPPER_LEG,BODYPART_COUNT}; // Body parts of the creature
	enum {JOINT_ANKLE,JOINT_KNEE,JOINT_COUNT}; // Joints of the creature

#elif defined EXTRA_LIMB
	enum { BODYPART_FOOT, BODYPART_LOWER_LEG, BODYPART_UPPER_LEG, BODYPART_TORSO, BODYPART_COUNT }; // Body parts of the creature
	enum { JOINT_ANKLE, JOINT_KNEE, JOINT_HIP, JOINT_COUNT }; // Joints of the creature

#elif defined ADV_BALANCE
	enum { BODYPART_FOOT, BODYPART_LOWER_LEG, BODYPART_UPPER_LEG, BODYPART_COUNT }; // Body parts of the creature
	enum { JOINT_ANKLE, JOINT_KNEE, JOINT_COUNT }; // Joints of the creature

#endif

	btDynamicsWorld		*	m_ownerWorld;				// The physics world of the simulation
	btCollisionShape	*	m_shapes[BODYPART_COUNT];	// The primitive shape of each body part used in collision
	btRigidBody			*	m_bodies[BODYPART_COUNT];	// The array of body parts
#if defined BASIC_BALANCE
	btHingeConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint: hinge
#elif defined EXTRA_LIMB
	btHingeConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint: hinge
#elif defined ADV_BALANCE
	btPoint2PointConstraint	*	m_joints[JOINT_COUNT];	// The type of each joint constraint: point2point
#elif defined POS_DEPEND
	btHingeConstraint	*	m_joints[JOINT_COUNT];		// The type of each joint constraint: hinge
#endif

	int lastChange;										// Time of last change of balance controller

	bool	m_hasFallen;		// Indicates if the creature has already fallen down

	btCollisionShape	*	m_COMShape;		// Shape for COM
	btRigidBody			*	m_COM;			// Body COM
	bool					m_showCOM;		// Show COM
	btVector3				m_positionCOM;	// Position COM
	btVector3 computeCenterOfMass();		// Compute the COM of the creature in world coordinate system

	btScalar				m_time_step;
	PIDController		*	m_PIDs[JOINT_COUNT];

	double					k_p_ankle;
	double					k_i_ankle;
	double					k_d_ankle;

	double					k_p_knee;
	double					k_i_knee;
	double					k_d_knee;

	double					fitnessFunction();


#if defined ADV_BALANCE
	btQuaternion			targetOrientation;
	bool					op_flag;
#endif
};

#endif
