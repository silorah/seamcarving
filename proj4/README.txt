Smoothing is based on what we did in lecture(on 11/2 if it matters)
io.cpp and io.h were provided

smoothing takes four arguments a .obj file to be read in, a .obj file(optional) to output the smoothed model to, a stepsize to be
used when calculating the new position of a point, and a n value which is the number of iterations the smoothing will be done.
If smoothing is run with less or more than 4 arguments the program will exit with status 1.
If smoothing cannot read the input file it will display a message saying so and end

raytrace parses the .nff file and raytraces the polygons specified in a fan of triangles and saves the resulting image to the .ppm file

Running smoothing:
build with make
./smoothing bunny.obj sbun.obj 1 50

-Sorry I'm turning this in so late in the semester