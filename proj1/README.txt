Found some info about how solve a matrix in eigen from http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html,
Algorithms are based off of what we went over in class

raytrace takes one to two arguments a .nff file to be read in and a .ppm file(optional) to output the image to.
If a .ppm file is not specified the raytrace will output the image to output.ppm.
If raytrace is run with no arguments it will inform the user it did not detect an input file and will exit with status 1.

raytrace parses the .nff file and raytraces the polygons specified in a fan of triangles and saves the resulting image to the .ppm file

Running raytrace:
build with make
./raytrace input.nff output.ppm


-It took me a while to realize the reason why my algorithms were only giving me a  horizontal small red triangle in the top right 
corner of my ppm file for tetra-3 was because i was negating the r.dir in (v1-v0 + v2-v0 + r.dir=from-v0), and that I was not checking 
if gamma was >0 or if beta was <1 in my Triangle's intersect method