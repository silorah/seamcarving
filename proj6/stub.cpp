#define cimg_display 0
#include "CImg.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
using namespace cimg_library;
void calcCost(double *energy,double *cost,int width,int height){
	double c;
	for(int j=height-1;j>=0;j--){
		// std::cout<<"J "<<j<<std::endl;
		for (unsigned int i=0; i<width; i++) {
			c=energy[i*height+j];
			if(j<height-1){
				if(i!=0&&i!=width-1){
					c+=std::min({cost[i*height+j+1],cost[(i-1)*height+j+1],cost[(i+1)*height+j+1]});
				}
				else if(i==0){
					c+=std::min(cost[i*height+j+1],cost[(i+1)*height+j+1]);
				}
				else{
					c+=std::min(cost[i*height+j+1],cost[(i-1)*height+j+1]);
				}
			}
			cost[i*height+j]=c;
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
void transPose(Eigen::Vector3d *&img,double *&cost,int width,int height){
	Eigen::Vector3d *tempImg=img;
	double *tempCost=cost;
	img=new Eigen::Vector3d [width*height];
	cost=new double[width*height];
	for(unsigned int i=0;i<width;i++){
		for(unsigned int j=0;j<height;j++){
			img[j*width+i]=tempImg[i*height+j];
			cost[j*width+i]=tempCost[i*height+j];
		}
	}
	delete[] tempImg;
	delete[] tempCost;
}
void findSeam(std::vector<int> &seam,double *cost,int width,int height){
	double min_cost=DBL_MAX;
	int index,u,d,m;
	for(int i=0;i<width;i++){
		if(min_cost>=cost[i*height]){
			min_cost=cost[i*height];
			index=i;
		}
	}
	// std::cout<<cost[0]<<" push_back "<<min_cost<<std::endl;
	double min;
	seam.push_back(index);
	for(int i=1;i<height;i++){
		m=index*height+i;
		if(index!=0&&index!=width-1){
			u=(index-1)*height+i;
			d=(index+1)*height+i;
			min=std::min({cost[u],cost[d],cost[m]});
			// std::cout<<"MIN"<<min<<" m "<<cost[m]<<" u "<<cost[u]<<" d "<<cost[d]<<std::endl;
			if(min==cost[u]){
				// std::cout<<"UP"<<std::endl;
				index-=1;
			}
			else if(min==cost[d]){
				// std::cout<<"DOWN"<<std::endl;
				index+=1;
			}
		}
		else if(index==width-1){
			u=(index-1)*height+i;
			min=std::min(cost[u],cost[m]);
			if(min==cost[u]){
				index-=1;
			}
		}
		else{
			d=(index+1)*height+i;
			min=std::min(cost[d],cost[m]);
			if(min==cost[d]){
				index+=1;
			}
		}
		seam.push_back(index);
	}
	// std::cout<<"end"<<std::endl;
}
int main(int argc, char *argv[]) {
	CImg<double> input(argv[1]);
	CImg<double> lab = input.RGBtoLab();
	if(argc!=5){
		exit(1);
	}
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
	// std::cout<<"starting shrink"<<std::endl;
	bool done=false;
	int height=input.height(),width=input.width();
	double *cost= new double[input.width()*input.height()];
	std::vector<int> seam;
	int index,l,r,m;
	double min_cost;
	bool next=true,removed=false;
	//seam removal
	while(!done){
		// std::cout<<width<<std::endl;
		if(height>atoi(argv[4])){
			// std::cout<<"Shrink Width"<<std::endl;
			energyCalc(energy,eng,image,width,height);
			// std::cout<<"Compute Cost"<<std::endl;
			calcCost(eng,cost,width,height);
			// std::cout<<"Cost computed"<<std::endl;
			transPose(image,cost,width,height);
			// std::cout<<"transposed"<<std::endl;
			findSeam(seam,cost,height,width);
			double max=seam[0];
			// std::cout<<"seam created\tseam size"<<seam.size()<<" "<<seam[0]<<" "<<seam[seam.size()-1]<<std::endl;
			Eigen::Vector3d *temp;
			std::sort(seam.begin(),seam.end());
			temp=image;
			image=new Eigen::Vector3d[(width)*(height)];
			for(int i=0;i<seam.size();i++){
				for(int j=0;j<seam[i];j++){
					image[j*width+i]=temp[j*width+i];
					
				}
				for(int j=seam[i];j<height-1;j++){
					image[j*width+i]=temp[(j+1)*width+i];					
				}
			}
			
			// std::cout<<"seam removed"<<std::endl;
			transPose(image,cost,height-1,width);
			height-=1;
			delete [] temp;
			temp=image;
			image=new Eigen::Vector3d[height*width];
			for(int i=0;i<width;i++){
				for(int j=0;j<height;j++){
					image[i*height+j]=temp[i*height+j];
				}
			}
			delete[] energy;
			delete[] eng;
			delete[] cost;
			delete[] temp;
			energy=new Eigen::Vector3d[(width)*height];
			eng=new double[(width)*height];
			seam.clear();
			cost=new double[(width)*height];
		}
		next=true;
		removed=false;
		if(width>atoi(argv[3])){
			energyCalc(energy,eng,image,width,height);
			// std::cout<<"Compute Cost"<<std::endl;
			calcCost(eng,cost,width,height);
			// std::cout<<"Cost computed"<<std::endl;
			findSeam(seam,cost,width,height);
			double max=seam[0];
			// std::cout<<"seam created\tseam size"<<seam.size()<<" "<<seam[0]<<" "<<seam[1]<<" "<<seam[2]<<std::endl;
			Eigen::Vector3d *temp;
			std::sort(seam.begin(),seam.end());
			temp=image;
			image=new Eigen::Vector3d[(width-1)*(height)];
			for(int i=0;i<seam.size();i++){
				for(int j=0;j<seam[i];j++){
					image[j*height+i]=temp[j*height+i];
					
				}
				for(int j=seam[i];j<width-1;j++){
					image[j*height+i]=temp[(j+1)*height+i];					
				}
			}
			width--;
			// std::cout<<"seam removed"<<std::endl;
			seam.clear();
			delete[] temp;
			delete[] energy;
			delete[] eng;
			delete[] cost;
			energy=new Eigen::Vector3d[(width)*height];
			eng=new double[(width)*height];
			cost=new double[(width)*height];
		}
		removed=false;
		if(width==atoi(argv[3])&&height==atoi(argv[4])){
			done=true;
		}
	}
	CImg<double> output(atoi(argv[3]),atoi(argv[4]), input.depth(), input.spectrum(), 0);
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
