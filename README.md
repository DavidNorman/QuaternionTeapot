# QuaternionTeapot

This is a simple test app that I made while developing code for doing smooth
rotation interpolation, using bezier-type curves.

The controls are a little cryptic.

* Click and drag on the teapot area to move the camera around the teapot.  You are not moving the teapot
  this way, only the camera's viewing angle.
* On the right is a list of rotation points, displayed as pitch, yaw, roll.  There is only one at the start.
* Clicking on 'Add' will add another rotation point to the list.
* Clicking on 'Remove' will remove the last point from the list.
* Clicking and dragging up or down on any of the values in the point list will change them.
* At the bottom of the right hand side are three smoothness parameters, which alter the
  shape of the interpolation.  Read up on bezier interpolation and look at the source code.

In the display you can see the teapot.  There are also little blue axis indicators, which show the
rotation points.  Moving the pitch and yaw will move them around the globe, and roll will rotate them.

When there is more than one point, the system will interpolate the orientation between the rotation points.

A red line shows the axis that the teapot is currently rotating around.  The green line shows the path
that the orientation will follow.


# Building

On OS/X I use <pre>gcc teapot.cpp -framework OpenGL -framework GLUT</PRE>.  It is just a simple
GLUT C program, so it shouldn't be any trouble to compile for any OS.
