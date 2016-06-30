
#include "btBulletDynamicsCommon.h"

#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBody.h"

#include "OpenGL/GlutStuff.h"
#include "OpenGL/GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "OpenGL/GLDebugDrawer.h"
#include <iostream>
#include <sstream>
#include <string>

#include "Application.h"
#include "Creature.h"
#include "Scene.h"
#include "TorusMesh.h"

void Application::initPhysics() {
	
	// Setup the basic world
	// =====================
	setTexturing(true);
	setShadows(true);
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_dispatcher =  new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax, 32766);
	m_solver = new btSequentialImpulseConstraintSolver();
	btSoftBodySolver* softBodySolver = 0;

	m_dynamicsWorld = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration, softBodySolver);

	m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;


	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;
	m_softBodyWorldInfo.m_broadphase = m_broadphase;
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	m_softBodyWorldInfo.air_density = 0;// (btScalar)1.2;
	m_softBodyWorldInfo.water_density = 0;
	m_softBodyWorldInfo.water_offset = 0;
	m_softBodyWorldInfo.water_normal = btVector3(0, 0, 0);
	m_softBodyWorldInfo.m_gravity.setValue(0, -10, 0);

	// Setup a big ground box
	// ======================
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.0),btScalar(10.0),btScalar(200.0)));
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));
	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(groundShape);
	fixedGround->setWorldTransform(groundTransform);
	m_dynamicsWorld->addCollisionObject(fixedGround);

	// Init Scene
	// ==========
	btVector3 startOffset(0,0.55,0);
	resetScene(startOffset);
	clientResetScene();
	m_startTime = GetTickCount();
	setCameraDistance(1.5);
}

void Application::Init_Torus(const btVector3 &position)
{
	//TRACEDEMO
	btSoftBody*	psb = btSoftBodyHelpers::CreateFromTriMesh(m_softBodyWorldInfo, gVertices,
		&gIndices[0][0],
		NUM_TRIANGLES);

	//btSoftBody* ball = btSoftBodyHelpers::CreateEllipsoid(m_softBodyWorldInfo, btVector3(btScalar(0), btScalar(0.5), btScalar(0)), btVector3(btScalar(0.1), btScalar(0.1), btScalar(.1)), 50);
	
	psb->m_materials[0]->m_kLST = 0.25;
	psb->m_cfg.kMT = 0.2;
	psb->scale(btVector3(.2, .2, .2));
	psb->generateBendingConstraints(2);

	psb->getCollisionShape()->setMargin(0.12);
	psb->m_cfg.piterations = 20;
	psb->randomizeConstraints();
	psb->getCollisionShape()->setColor(btVector3(btScalar(1), btScalar(0), btScalar(0)));
	
	btMatrix3x3	m;
	m.setEulerZYX(0, 0, 0);
	psb->transform(btTransform(m, position));
	psb->setTotalMass(100, true);
	((btSoftRigidDynamicsWorld*)m_dynamicsWorld)->addSoftBody(psb);
}

void Application::resetScene(const btVector3& startOffset) {

	btSoftBodyArray softBodyArray = ((btSoftRigidDynamicsWorld*)m_dynamicsWorld)->getSoftBodyArray();
	for (int i = 0; i < softBodyArray.size(); i++)
		((btSoftRigidDynamicsWorld*)m_dynamicsWorld)->removeSoftBody(softBodyArray[i]);

	if (spawnTorus)
		Init_Torus(btVector3(0, 2.5, 0));

	if (m_creature != NULL) delete m_creature;
	m_creature = new Creature(m_dynamicsWorld, startOffset);
	if (m_scene != NULL) delete m_scene;
	m_scene = new Scene(m_dynamicsWorld);
	m_startTime = GetTickCount();
}

void Application::clientMoveAndDisplay() {

	// Update the simulator
	// ====================
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	float ms = getDeltaTimeMicroseconds();
	//std::cout << ms << std::endl;
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS) ms = minFPS;
	if (m_dynamicsWorld) {	// Here we force the frame rate to 60pfs in advanced balance mode
#if defined BASIC_BALANCE
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#elif defined EXTRA_LIMB
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#elif defined ADV_BALANCE
		m_dynamicsWorld->stepSimulation(ms / 1000000.f, 1, 1.0f / 60.0f);
#elif defined MANY_JOINT
		m_dynamicsWorld->stepSimulation(ms / 1000000.f, 1, 1.0f / 60.0f);
#elif defined POS_DEPEND
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#endif
		m_dynamicsWorld->debugDrawWorld();
	}

	// Update the Scene
	// ================
#if defined BASIC_BALANCE
	update();
#elif defined EXTRA_LIMB
	update();
#elif defined ADV_BALANCE
	update(ms);
#elif defined MANY_JOINT
	update(ms);
#elif defined POS_DEPEND
	update();
#endif

	// Render the simulation
	// =====================
	renderme(); 
	glFlush();
	glutSwapBuffers();
}

void Application::displayCallback() {

	// Render the simulation
	// =====================
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	renderme();
	if (m_dynamicsWorld) m_dynamicsWorld->debugDrawWorld();
	glFlush();
	glutSwapBuffers();
}

void Application::keyboardCallback(unsigned char key, int x, int y) {
    // You can add your key bindings here.
    // Be careful to not use a key already listed in DemoApplication::keyboardCallback
	switch (key) {
		case 'e':
			{
				btVector3 startOffset(0,0.55,0);
				resetScene(startOffset);
				break;
			}
		case 'r':
			{
				m_scene->switchPlatform();
				break;
			}
		case 's':
		{
			spawnTorus = !spawnTorus;
			btVector3 startOffset(0, 0.55, 0);
			resetScene(startOffset);
		}
		case 't':
			{
				m_scene->switchBall();
				break;
			}
		case 'y':
			{
				m_creature->switchCOM();
				break;
			}
		default :
			DemoApplication::keyboardCallback(key, x, y);
	}	
}

void Application::exitPhysics() {
	delete m_creature;
	delete m_scene;
	//remove the rigidbodies from the dynamics world and delete them	
	for (int i = m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) delete body->getMotionState();
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}
	//delete ground
	delete m_ground;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	//delete collision configuration
	delete m_collisionConfiguration;
	
}

void Application::update() {	

	// Do not update time if creature fallen
	if (!m_creature->hasFallen()) m_currentTime = GetTickCount();
	m_elapsedTime = (int)(((double) m_currentTime - m_startTime)/100.0);

	// Move the platform if not fallen
	m_scene->update( (!m_creature->hasFallen()) ? m_elapsedTime : -1, m_creature->getCOM());

	// Control the creature movements
	m_creature->update((int)(m_currentTime - m_startTime));

	// Display info
	DemoApplication::displayProfileString(10,20,"Q=quit E=reset R=platform S=Toggle Torus T=ball Y=COM U=switch I=pause");

	// Display time elapsed
	std::ostringstream osstmp;
	osstmp << m_elapsedTime;
	std::string s_elapsedTime = osstmp.str();
	std::ostringstream oss;
	if (m_elapsedTime < 10)
		oss << "Time under balance: 0." << s_elapsedTime << " seconds";
	else
		oss << "Time under balance: " << s_elapsedTime.substr(0,s_elapsedTime.size()-1) << "." << s_elapsedTime.substr(s_elapsedTime.size()-1,s_elapsedTime.size()) << " seconds";
	DemoApplication::displayProfileString(10,40,const_cast<char*>(oss.str().c_str()));	

}

void Application::update(float ms) {

	// Do not update time if creature fallen
	if (!m_creature->hasFallen()) m_currentTime = GetTickCount();
	m_elapsedTime = (int)(((double)m_currentTime - m_startTime) / 100.0);

	// Move the platform if not fallen
	m_scene->update((!m_creature->hasFallen()) ? m_elapsedTime : -1, m_creature->getCOM());

#if defined ADV_BALANCE
	// Control the creature movements
	m_creature->update((int)(m_currentTime - m_startTime), ms);
#elif defined MANY_JOINT
	// Control the creature movements
	m_creature->update((int)(m_currentTime - m_startTime), ms);
#endif
	// Display info
	DemoApplication::displayProfileString(10, 20, "Q=quit E=reset R=platform S=Toggle Torus T=ball Y=COM U=switch I=pause");

	// Display time elapsed
	std::ostringstream osstmp;
	osstmp << m_elapsedTime;
	std::string s_elapsedTime = osstmp.str();
	std::ostringstream oss;
	if (m_elapsedTime < 10)
		oss << "Time under balance: 0." << s_elapsedTime << " seconds";
	else
		oss << "Time under balance: " << s_elapsedTime.substr(0, s_elapsedTime.size() - 1) << "." << s_elapsedTime.substr(s_elapsedTime.size() - 1, s_elapsedTime.size()) << " seconds";
	DemoApplication::displayProfileString(10, 40, const_cast<char*>(oss.str().c_str()));

}