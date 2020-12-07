Rasterizer is based off the professors code for project 2 especially the shading in vertexprocessing,createImage,writeImage,and renderer
There are some equations I got from the book, like how to compute Mcam,Morth,Mper,Mvp,M,alpha,beta,and gamma

Rasterizer takes two arguments a .nff file to be read in, a .ppm file to output the image to
If smoothing is run with less or more than 2 arguments the program will exit with status 1.

Rasterizer takes the image described in the nff file and runs it through the graphics pipeline and outputs the resulting image to the
.ppm file

Running rasterizer:
build with make
./rasterizer input.nff output.ppm

-Sorry I'm turning this in so late in the semester
-I know there is an issue with my Mper matrix, i "fixed" it by multiplying Mper(0,0)and MPer(1,1) by 10 because when compared to the 
one produced by the professors code they were off by about a factor of 10(I still have no idea why)
-the teapot currently produced is flipped horizontally and the platform it sits on is all white for some reason