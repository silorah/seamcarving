all: clean run

raytrace: raytracing.o
	c++  -Wall -O3 -g -o raytrace raytracing.o

raytracing.o: raytracing.cpp
	c++  -Wall  -O3 -g -I/usr/include/eigen3/ -c raytracing.cpp

clean: 
	rm -f  raytracing.o raytrace

run: raytrace