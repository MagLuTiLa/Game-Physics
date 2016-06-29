#include <random>
#include "btBulletDynamicsCommon.h"
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

void Application::initPhysics() {
	
	// Setup the basic world
	// =====================
	setTexturing(true);
	setShadows(true);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	m_solver = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

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
	// Init GA Stuff
	// =============
	// Fitness function = Time elapsed
	// Make population
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, 1.0);

	for (int i = 0; i < population_size; i++) {
		for (int j = 0; j < num_pid_param; j++) {
			population[i][j] = distribution(generator) + param_scalar[j];
		}
	}

	// Init Scene
	// ==========
	btVector3 startOffset(0,0.55,0);
	resetScene(startOffset, population[0]);
	clientResetScene();
	m_startTime = GetTickCount();
	setCameraDistance(1.5);



}

void Application::GALoop() {
	// Loop over generations
	const int sz = 10; // Should be population size, but the ide nags.
	double fitness[sz] = {};
	for (int epoch = 0; epoch < max_epochs; epoch++) {
		// Find individual scores
		for (int i = 0; i < population_size; i++) {
			btVector3 startOffset(0, 0.55, 0);
			resetScene(startOffset, population[0]);
			clientResetScene();
			m_startTime = GetTickCount();
			while (!m_creature->hasFallen()) {} // Wait to fall
			fitness[i] = m_elapsedTime;
		}
		// Tournament Selection
		int* chosen_parents = Application::tournamentSelection(fitness, Application::population_size, 2);
		// Generate offspring (combine, somehow)

		// Mutilate offspring (mutate)

		// Update population = add new ones and remove old such that size stays the same.
	}
}

int* Application::tournamentSelection(double* fitness, int length, int k) {
	//Selects index of best fitness with probability p = Application::tournament_prob
	// Selects second best with prob p*(1-p)
	// Selects third best with prob p*(1-p)*(1-p)
	// And so on, until k are selected.

	// length is the size of the fitness.

	// Return k indices.
	int toReturn[2] = { 0, 2 };
	return toReturn;
}

void Application::resetScene(const btVector3& startOffset, double* currentPID_Parameters) {
	if (m_creature != NULL) delete m_creature;
	m_creature = new Creature(m_dynamicsWorld, startOffset, currentPID_Parameters);
	if (m_scene != NULL) delete m_scene;
	m_scene = new Scene(m_dynamicsWorld);
	m_startTime = GetTickCount();
}

void Application::clientMoveAndDisplay() {

	// Update the simulator
	// ====================
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	float ms = getDeltaTimeMicroseconds();
	float minFPS = 1000000.f/60.f;
	if (ms > minFPS) ms = minFPS;
	if (m_dynamicsWorld) {	// Here we force the frame rate to 60pfs in advanced balance mode
#if defined BASIC_BALANCE
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#elif defined EXTRA_LIMB
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
#elif defined ADV_BALANCE
		m_dynamicsWorld->stepSimulation(ms / 1000000.f, 2, 1.0f / 60.0f);
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
	/*	case 'e':
			{
				btVector3 startOffset(0,0.55,0);
				resetScene(startOffset);
				break;
			}*/
		case 'r':
			{
				m_scene->switchPlatform();
				break;
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
	DemoApplication::displayProfileString(10,20,"Q=quit E=reset R=platform T=ball Y=COM U=switch I=pause");

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
#endif
	// Display info
	DemoApplication::displayProfileString(10, 20, "Q=quit E=reset R=platform T=ball Y=COM U=switch I=pause");

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