When compiled in a Mac environment that includes GLFW 3.2 and the files here, 
will create a standalone application with a scene as set in the initScene()
function found in main.cpp.

The application allows users to view the scene with diffuse and ambient shading,
while pressing the 'r' key will cause the application to write out an
output.png file which includes reflectivity and transmittivity in the lighting.
Output images will be created from the same view as the application currently displays.

To change the view, click and drag within the application window to rotate. To
zoom in/out, shift + click and drag. To change the scene itself currently requires
changing the objects in the initScene() function.

NOTE: currently, the files here are more about preserving the source code
than they are about being runnable. I'm not actually entirely sure if this
will compile on another machine.
