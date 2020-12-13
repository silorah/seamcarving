#define cimg_display 0
#include "CImg.h"
#include <iostream>
#include <Eigen/Dense>
using namespace cimg_library;

int main(int argc, char *argv[]) {
	CImg<double> input(argv[1]);
	CImg<double> lab = input.RGBtoLab();
	Eigen::Vector3d *image = new Eigen::Vector3d[input.width()*input.height()];
	Eigen::Vector3d *energy =new Eigen::Vector3d[input.width()*input.height()];
	Eigen::Vector3d *grey =new Eigen::Vector3d[input.width()*input.height()];
	for (unsigned int i=0; i<input.width(); i++) {
		for (unsigned int j=0; j<input.height(); j++) {
			image[i*input.height()+j][0] = lab(i, j, 0);
			image[i*input.height()+j][1] = lab(i, j, 1);
			image[i*input.height()+j][2] = lab(i, j, 2);
		}
	}
	Eigen::Vector3d max_energy(0.0,0.0,0.0);
	Eigen::Vector3d x,y;
	for (unsigned int i=0; i<input.width(); i++) {
		for (unsigned int j=0; j<input.height(); j++) {
			if(i==0&&j==0){
				x=image[(i+1)*input.height()+j]-image[i*input.height()+j];
				y=image[i*input.height()+(j+1)]-image[i*input.height()+j];
			}
			else if(i==0){
				x=image[(i+1)*input.height()+j]-image[i*input.height()+j];
				if(j==input.height()-1){
					y=image[i*input.height()+j]-image[i*input.height()+(j-1)];
				}
				else{
					y=image[i*input.height()+(j+1)]-image[i*input.height()+(j-1)];
					y/=2;
				}
			}
			else if(j==0){
				y=image[i*input.height()+(j+1)]-image[i*input.height()+j];
				if(i==input.width()-1){
					x=image[i*input.height()+j]-image[(i-1)*input.height()+j];
				}
				else{
					x=image[(i+1)*input.height()+j]-image[(i-1)*input.height()+j];					
					x/=2;
				}
			}
			else if(i==input.width()-1 && j==input.height()-1){
				x=image[i*input.height()+j]-image[(i-1)*input.height()+j];
				y=image[i*input.height()+j]-image[i*input.height()+(j-1)];
			}
			else if(i==input.width()-1){
				x=image[i*input.height()+j]-image[(i-1)*input.height()+j];
				y=image[i*input.height()+(j+1)]-image[i*input.height()+(j-1)];
				y/=2;
			}
			else if(j==input.height()-1){
				x=image[(i+1)*input.height()+j]-image[(i-1)*input.height()+j];					
				y=image[i*input.height()+j]-image[i*input.height()+(j-1)];
				x/=2;
			}
			else{
				x=image[(i+1)*input.height()+j]-image[(i-1)*input.height()+j];					
				y=image[i*input.height()+(j+1)]-image[i*input.height()+(j-1)];
				x/=2;
				y/=2;
			}
			energy[i*input.height()+j]=(x.array().pow(2)+y.array().pow(2)).sqrt();
			if(energy[i*input.height()+j][0]>max_energy[0]){
				// std::cout<<"max"<<max_energy[0]<<"\tcurr"<<energy[i*input.height()+j][0]<<std::endl;
				max_energy[0]=energy[i*input.height()+j][0];
			}
			if(energy[i*input.height()+j][1]>max_energy[1]){
				max_energy[1]=energy[i*input.height()+j][1];
			}
			if(energy[i*input.height()+j][2]>max_energy[2]){
				max_energy[2]=energy[i*input.height()+j][2];
			}
		}
	}
	Eigen::Vector3d enrg;
	std::cout<<max_energy<<std::endl;
	for(unsigned int i=0;i<input.width();i++){
		for(unsigned int j=0;j<input.height();j++){
			enrg[0]=energy[i*input.height()+j][0]/max_energy[0];
			enrg[1]=energy[i*input.height()+j][1]/max_energy[1];
			enrg[2]=energy[i*input.height()+j][2]/max_energy[2];
			grey[i*input.height()+j][0]=pow(enrg[0],1.0/3.0);
			grey[i*input.height()+j][1]=pow(enrg[1],1.0/3.0);
			grey[i*input.height()+j][2]=pow(enrg[2],1.0/3.0);
		}
	}
	CImg<double> greyScale(input.width(),input.height(),input.depth(),input.spectrum(),0);
	for (unsigned int i=0; i<greyScale.width(); i++) {
		for (unsigned int j=0; j<greyScale.height(); j++) {
			greyScale(i, j, 0) = std::max(0.0,grey[i*greyScale.height()+j][0]*100);
			greyScale(i, j, 1) = std::max(0.0,grey[i*greyScale.height()+j][1]*100);
			greyScale(i, j, 2) = std::max(0.0,grey[i*greyScale.height()+j][2]*100);
		}
	}
	CImg<double> output(atoi(argv[3]), atoi(argv[4]), input.depth(), input.spectrum(), 0);
	for (unsigned int i=0; i<output.width(); i++) {
		for (unsigned int j=0; j<output.height(); j++) {
			output(i, j, 0) = image[i*output.height()+j][0];
			output(i, j, 1) = image[i*output.height()+j][1];
			output(i, j, 2) = image[i*output.height()+j][2];
		}
	}
	CImg<double> greyImg=greyScale.LabtoRGB();
	greyImg.save_jpeg("grey.jpg");
	CImg<double> rgb = output.LabtoRGB();
	if (strstr(argv[2], "png"))
		rgb.save_png(argv[2]);
	else if (strstr(argv[2], "jpg"))
		rgb.save_jpeg(argv[2]);

	delete [] energy;
	delete [] grey;
	delete [] image;
	return 0;
}
