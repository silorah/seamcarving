all: clean run

smoothing: smoothing.o
	c++  -Wall -O3 -g -o smoothing smoothing.o

smoothing.o: smoothing.cpp
	c++  -Wall  -O3 -g -I/usr/include/eigen3/ -c smoothing.cpp io.cpp

clean: 
	rm -f  smoothing.o smoothing io.o

run: smoothing