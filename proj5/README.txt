Fishtank is based on what we did in class
makefile, viewer.cpp, kdTree.cpp, and kdTree.h were provided

fishtank takes two arguments a .in file to be read in, a .out file to output the frames of the animation to.
If fishtank is run with less or more than 2 arguments the program will exit with status 1.
If fishtank cannot read the input file it will display a message saying so and end


Building fishtank and viewer
make (builds both fishtank and viewer)

build just fishtank
make fishtank

build just viewer
make viewer

Running fishtank:
build with make
./fishtank sample.in fish.out

Displaying the animation
build viewer
./viewer fish.out

-Sorry I'm turning this in so late in the semester
-spent the longest time trying to figure out why the boids where comming out all funky and pointing everywhere, turns out I was
calculating the collision avoidance force wrong