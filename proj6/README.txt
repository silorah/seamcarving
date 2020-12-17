stub.cpp is based on what we did in class
reading in an image, writing an image  were provided in stub


seamcarving takes four arguments a .jpg or .png input image, a .jpg or .png output image, output width, output height
If seamcarving is run with less or more than 4 arguments the program will exit with status 1.


seamcarving takes in an image and uses seam carving to shrink the image to the desired width and height writting the result to the
desired new .jpg or .png file. Uses seam carving to determine the least important vertical and horizontal seams and removes them from
the image as necessary. Writes a greyscaled version of the energy function to grey.jpg

Running seamcarving:
build with make
./seamcarving input.jpg output.jpg width height


-Sorry I'm turning this in so late in the semester
-I noticed that on my laptop at least I had to delete the output image in between runs if I wanted it to update.(might not matter but
figured I should let you know)
-grey.jpg is reddish for some reason no idea why