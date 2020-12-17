all: clean run
seamcarving: stub.o
	c++  -Wall -O3 -g -o seamcarving stub.o

stub.o: stub.cpp
	c++  -Wall  -O3 -g -I/usr/include/eigen3/ -c stub.cpp

clean: 
	rm -f  stub.o seamcarving

run: seamcarving