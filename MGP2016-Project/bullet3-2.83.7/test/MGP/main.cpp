
#include "Application.h"

#include "OpenGL/GlutStuff.h"
#include "OpenGL/GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

#include <thread>

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        Application app;

		app.initPhysics();
		app.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
		std::thread thread2 (&Application::GALoop, &app);

        return glutmain(argc, argv,1024,768,"INFOMGP - Project",&app);
}
