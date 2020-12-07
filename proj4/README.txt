Smoothing is based on what we did in lecture(on 11/2 if it matters)
io.cpp and io.h were provided and bunny.obj is from the project page

smoothing takes four arguments a .obj file to be read in, a .obj file to output the smoothed model to, a stepsize to be
used when calculating the new position of a point, and a n value which is the number of iterations the smoothing will be done.
If smoothing is run with less or more than 4 arguments the program will exit with status 1.
If smoothing cannot read the input file it will display a message saying so and end

smoothing parses the input .obj file and runs the model through meshfairing n times with a stepsize(stepsize), and outputs the smoothed
image to the output .obj file

Running smoothing:
build with make
./smoothing bunny.obj sbun.obj 1 50

-Sorry I'm turning this in so late in the semester