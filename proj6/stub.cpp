#define cimg_display 0
#include "CImg.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace cimg_library;
Eigen::Vector3d min(Eigen::Vector3d a,Eigen::Vector3d b,Eigen::Vector3d c){
	Eigen::Vector3d min;
	if (a[0]<b[0]&&a[0]<c[0]) min[0]=a[0];
	if (a[1]<b[1]&&a[1]<c[1]) min[1]=a[1];
	return a;
}
void computeCost(Eigen::Vector3d *energy,Eigen::Vector3d *cost,int width,int height){
	int row;
	for(unsigned int i=0;i<width;i++){
		for(unsigned int j=0;j<height;j++){
			if(i==0){
				cost[i*height+j]=energy[i*height+j];
			}
			else{
				row=(i-1)*height;
				if(j!=0||j!=height-1){
					cost[i*height+j]=energy[i*height+j];
					if(cost[row+(j-1)].norm()<cost[row+j].norm()){
						if(cost[row+(j-1)].norm()<cost[row+(j+1)].norm()){
							cost[i*height+j]+=cost[row+(j-1)];
						}
						else{
							cost[i*height+j]+=cost[row+(j+1)];
						}
					}
					else if(cost[row+j].norm()<cost[row+(j+1)].norm()){
						cost[i*height+j]+=cost[row+j];
					}
					else{
						cost[i*height+j]+=cost[row+(j+1)];
					}
					// cost[i*height+j]+=min(cost[row+j],cost[row+j-1],cost[row+j+1]);
				}
				else if(j==0){
					// cost[i*height+j]+=min(cost[row+j],cost[row+j+1]);
					if(cost[row+j].norm()<cost[row+j+1].norm()){
						cost[i*height+j]+=cost[row+j];
					}
					else{
						cost[i*height+j]+=cost[row+j+1];
					}
				}
				else{
					// cost[i*height+j]+=min(cost[row+j],cost[row+j-1]);
					if(cost[row+j].norm()<cost[row+j-1].norm()){
						cost[i*height+j]+=cost[row+j];
					}
					else{
						cost[i*height+j]+=cost[row+j-1];
					}
				}
			}
		}
	}
}
void calcCost(double *energy,double *cost,int width,int height){
	double c;
	for (unsigned int i=0; i<height; i++) {
		for (unsigned int j=0; j<width; j++) {
			c=energy[i*width+j];
			if(i==0){}
		}
	}
}
void energyCalc(Eigen::Vector3d *energy,double *e,Eigen::Vector3d *image,int width,int height){
	Eigen::Vector3d x,y;
	for (unsigned int i=0; i<width; i++) {
		for (unsigned int j=0; j<height; j++) {
			if(i==0&&j==0){
				x=image[(i+1)*height+j]-image[i*height+j];
				y=image[i*height+(j+1)]-image[i*height+j];
			}
			else if(i==0){
				x=image[(i+1)*height+j]-image[i*height+j];
				if(j==height-1){
					y=image[i*height+j]-image[i*height+(j-1)];
				}
				else{
					y=image[i*height+(j+1)]-image[i*height+(j-1)];
					y/=2;
				}
			}
			else if(j==0){
				y=image[i*height+(j+1)]-image[i*height+j];
				if(i==width-1){
					x=image[i*height+j]-image[(i-1)*height+j];
				}
				else{
					x=image[(i+1)*height+j]-image[(i-1)*height+j];					
					x/=2;
				}
			}
			else if(i==width-1 && j==height-1){
				x=image[i*height+j]-image[(i-1)*height+j];
				y=image[i*height+j]-image[i*height+(j-1)];
			}
			else if(i==width-1){
				x=image[i*height+j]-image[(i-1)*height+j];
				y=image[i*height+(j+1)]-image[i*height+(j-1)];
				y/=2;
			}
			else if(j==height-1){
				x=image[(i+1)*height+j]-image[(i-1)*height+j];					
				y=image[i*height+j]-image[i*height+(j-1)];
				x/=2;
			}
			else{
				x=image[(i+1)*height+j]-image[(i-1)*height+j];					
				y=image[i*height+(j+1)]-image[i*height+(j-1)];
				x/=2;
				y/=2;
			}
			energy[i*height+j]=(x.array().pow(2)+y.array().pow(2)).sqrt();
			e[i*height+j]=energy[i*height+j].norm();
		}
	}
}
int main(int argc, char *argv[]) {
	CImg<double> input(argv[1]);
	CImg<double> lab = input.RGBtoLab();
	Eigen::Vector3d *image = new Eigen::Vector3d[input.width()*input.height()];
	Eigen::Vector3d *energy =new Eigen::Vector3d[input.width()*input.height()];
	Eigen::Vector3d *grey =new Eigen::Vector3d[input.width()*input.height()];
	double *eng=new double[input.width()*input.height()];
	for (unsigned int i=0; i<input.width(); i++) {
		for (unsigned int j=0; j<input.height(); j++) {
			image[i*input.height()+j][0] = lab(i, j, 0);
			image[i*input.height()+j][1] = lab(i, j, 1);
			image[i*input.height()+j][2] = lab(i, j, 2);
		}
	}
	//Calculate energy for the greyscale image
	energyCalc(energy,eng,image,input.width(),input.height());
	// Eigen::Vector3d max_energy(0.0,0.0,0.0);
	double max_energy=0;
	//find max_energy
	for(int i=0;i<input.width();i++){
		for(int j=0;j<input.height();j++){
			if(eng[i*input.height()+j]>max_energy){
				max_energy=eng[i*input.height()+j];
			}
		}
	}
	Eigen::Vector3d enrg;
	//fill in grey with the greyscaled values
	for(unsigned int i=0;i<input.width();i++){
		for(unsigned int j=0;j<input.height();j++){
			enrg=energy[i*input.height()+j]/max_energy;
			grey[i*input.height()+j][0]=pow(enrg[0],1.0/3.0);
			grey[i*input.height()+j][1]=pow(enrg[1],1.0/3.0);
			grey[i*input.height()+j][2]=pow(enrg[2],1.0/3.0);
		}
	}
	//store the image in grey into greyScale
	CImg<double> greyScale(input.width(),input.height(),input.depth(),input.spectrum(),0);
	for (unsigned int i=0; i<greyScale.width(); i++) {
		for (unsigned int j=0; j<greyScale.height(); j++) {
			greyScale(i, j, 0) = std::max(0.0,grey[i*greyScale.height()+j][0]*100);
			greyScale(i, j, 1) = std::max(0.0,grey[i*greyScale.height()+j][1]*100);
			greyScale(i, j, 2) = std::max(0.0,grey[i*greyScale.height()+j][2]*100);
		}
	}
	//convert greyScale to RGB, save to grey.jpg
	CImg<double> greyImg=greyScale.LabtoRGB();
	greyImg.save_jpeg("grey.jpg");
	std::cout<<"starting shrink"<<std::endl;
	bool done=false;
	int height=input.height(),width=input.width();
	Eigen::Vector3d *cost= new Eigen::Vector3d[input.width()*input.height()];
	std::vector<int> vseam;
	int index,l,r,m;
	double min_cost;
	while(!done){
		std::cout<<width<<std::endl;
		if(width>atoi(argv[3])){
			energyCalc(energy,eng,image,width,height);
			std::cout<<"Compute Cost"<<std::endl;
			computeCost(energy,cost,width,height);
			std::cout<<"Cost computed"<<std::endl;
			for(int i=width-1;i>=0;i--){
				min_cost=DBL_MAX;
				index=0;
				for(int j=height-1;j>=0;j--){
					if((min_cost>cost[i*height+j].norm())&&(i==width-1||(i*height+j==l||i*height+j==m||i*height+j==r))){
						min_cost=cost[i*height+j].norm();
						index=i*height+j;
						l=(i-1)*height+j-1;
						m=(i-1)*height+j;
						r=(i-1)*height+j+1;
						// std::cout<<"big if true"<<std::endl;
					}
				}
				// std::cout<<"i"<<i<<"\tindex\t"<<index<<std::endl;
				vseam.push_back(index);
			}
			std::cout<<"seam created\tseam size"<<vseam.size()<<" "<<vseam[0]<<std::endl;
			Eigen::Vector3d *temp;
			temp=image;
			image=new Eigen::Vector3d[(width)*height];
			for(int i=0;i<input.width();i++){
				index=vseam[vseam.size()-1];
				vseam.pop_back();
				// if(i==0) std::cout<<index<<std::endl;
				for(int j=0;j<height;j++){
					if(index!=i*height+j){
						image[i*height+j]=temp[i*height+j];
					}
					else{
						// std::cout<<index<<std::endl;
						image[i*height+j]=Eigen::Vector3d(53.23,80.11,67.22);
					}
				}
			}
			std::cout<<"seam removed"<<std::endl;
			delete [] temp;
			width--;
		}
		if(width==atoi(argv[3])&&height==atoi(argv[4])){
			done=true;
		}
	}
	//begin seam finding
	CImg<double> output(input.width(), input.height(), input.depth(), input.spectrum(), 0);
	for (unsigned int i=0; i<output.width(); i++) {
		// std::cout<<"i"<<i<<std::endl;
		for (unsigned int j=0; j<output.height(); j++) {
			output(i, j, 0) = image[i*output.height()+j][0];
			output(i, j, 1) = image[i*output.height()+j][1];
			output(i, j, 2) = image[i*output.height()+j][2];
		}
	}

	CImg<double> rgb = output.LabtoRGB();
	if (strstr(argv[2], "png"))
		rgb.save_png(argv[2]);
	else if (strstr(argv[2], "jpg"))
		rgb.save_jpeg(argv[2]);
	delete [] eng;
	delete [] cost;
	delete [] energy;
	delete [] grey;
	delete [] image;
	return 0;
}
