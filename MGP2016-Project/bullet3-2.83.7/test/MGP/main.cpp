
#include "Application.h"

#include "OpenGL/GlutStuff.h"
#include "OpenGL/GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"

GLDebugDrawer	gDebugDrawer;

int main(int argc,char* argv[])
{
        Application app;

        app.initPhysics();
		app.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

        return glutmain(argc, argv,1024,768,"INFOMGP - Project",&app);
}
