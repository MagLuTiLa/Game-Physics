#include <list>
#include <mutex>
#include <condition_variable>

#ifndef APPLICATION_H
#define APPLICATION_H

//------------- include/declaration -------------
#include "OpenGL/GlutDemoApplication.h"

class btBroadphaseInterface;
class btCollisionShape;
class btCollisionDispatcher;
class btConstraintSolver;
class btDefaultCollisionConfiguration;

// User include
class Creature;
class Scene;
//-----------------------------------------------

class Application : public GlutDemoApplication {

public:

	Application() : m_creature(NULL), m_scene(NULL), m_elapsedTime(0) {}
	virtual ~Application() {	exitPhysics(); } // Destructor

	void initPhysics();				// Initialize the simulation
	void exitPhysics();				// End the simulation
	void GALoop();

protected:

	virtual void clientMoveAndDisplay();		// Update the simulation
	virtual void displayCallback();				// Render the simulation

	virtual void keyboardCallback(unsigned char key, int x, int y); // Input management

	void resetScene(const btVector3& startOffset, double* currentPID_Parameters);	// Reset the creature
	void update();									// Update objects and display the time elapsed under balance
	void update(float ms);							// Update objects and display the time elapsed under balance with input time
	int* selection(double* fitness, int length, int k);
	double** generate_offspring(double** population, int* chosen_parents, int num_parents, int num_children);
	double** mutate(double** offspring, int num_children);
	void update_population(double** offspring, int num_children);

	Creature						*	m_creature;		// The creature
	Scene							*	m_scene;		// The scene
	btCollisionShape				*	m_ground;		// The ground
	btBroadphaseInterface			*	m_broadphase;	// The broadphase
	btCollisionDispatcher			*	m_dispatcher;	// The displatcher
	btConstraintSolver				*	m_solver;		// The solver
	btDefaultCollisionConfiguration	*	m_collisionConfiguration;	// The collision configuration

	DWORD m_startTime;		// Time starter
	DWORD m_currentTime;	// Time counter
	int m_elapsedTime;		// Time elapsed in 10e-1 sec
	const int population_size = 10;
	const int num_pid_param = 6;
	const double param_scalar[6] = { 200.0f, 0.01f, 10.0f, 150.0f, 0.1f, 10.0f };

	double** population = new double*[population_size]; // Hardcoded due to memory allocation (else it has to be done manually)
	const int max_epochs = 100;
	double tournament_prob = 0.5;

	std::mutex mutex;
	std::unique_lock<std::mutex> lock;
	std::condition_variable wait_for_exec;

	bool done_exec;
};

#endif
