all: clean run

rasterizer: rasterizer.o
	c++  -Wall -O3 -g -o rasterizer rasterizer.o

rasterizer.o: rasterizer.cpp
	c++  -Wall  -O3 -g -I/usr/include/eigen3/ -c rasterizer.cpp

clean: 
	rm -f  rasterizer.o rasterizer

run: rasterizer