# Game-Physics

Compiling for Windows Visual Studio (need at least the 2010 edition)

Navigate to "bullet3-2.83.7/build3/" and run "vs2010.bat". This will create the "vs2010" folder wherein you can find the VS project.
Open the solution "0_Bullet3Solution.sln" in Visual Studio.
If you use a VS version that is newer than the 2010 version, chances are that you will get a popup to update the compiler and livraries of the project to your current Visual Studio version. elect all the projects and press OK.
In the solution explorer, right click "App_ExampleBrowser" and select "Set as StartUp Project".
You should now be able to build (this might take a while) and run the example browser from Visual Studio.

If you cannot use Visual Studio or you do not run Windows, there are alternative build options which can be found in 
"bullet3-2.83.7/docs/BulletQuickstart.pdf"
