#include "rasterizer.h"
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <getopt.h>
#define cimg_display 0
#include "CImg.h"
#ifdef __APPLE__
#define MAX std::numeric_limits<double>::max()
#else
#include <values.h>
#define MAX DBL_MAX
#endif

using namespace cimg_library;

// return the determinant of the matrix with columns a, b, c.
double det(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c) {
	return a[0]* (b[1] * c[2] - c[1] * b[2]) +
	b[0] * (c[1] * a[2] - a[1] * c[2]) +
	c[0] * (a[1] * b[2] - b[1] * a[2]);
}

inline double sqr(double x) {return x*x;}
inline double dot(const Eigen::Vector3d &x, const Eigen::Vector3d &y) {return x.dot(y);}
inline double cross2d(const Eigen::Vector2d &x, const Eigen::Vector2d &y) {return x[0]*y[1]-x[1]*y[0];}
/**
 * transforms the triangle
 * @param m the transformation matrix
 */
void Triangle::transform(const Eigen::Matrix4d &m){
	ap=m*a;//a'=Ma
	bp=m*b;//b'=Mb
	cp=m*c;//c'=Mc
	// std::cout<<"\na"<<a<<"\naP"<<ap<<"\nb"<<b<<"\nbp"<<bp<<"\nc"<<c<<"\ncP"<<cp<<std::endl;
}
/**
 * Creates the camera
 * @param eye the eye of the camera
 * @param at the at of the camera
 * @param up the up of the camera
 */
Camera createCamera(Eigen::Vector3d &eye, Eigen::Vector3d &at,Eigen::Vector3d &up){
	Camera cam;
	//cam=new Camera;
	cam.eye=eye;
	cam.at=at;
	cam.up=up;
	return cam;
}
/**
 * reads in the file, constructs the Renderer
 * @param fname the input file name
 */
Renderer::Renderer(const std::string &fname) {
	std::ifstream in(fname.c_str(), std::ios_base::in);
	std::string line;
	char ch;
	Fill fill;
	bool coloredlights = false;

	while (in) {
	getline(in, line);
	switch (line[0]) {
	case 'b': {//backgorund color
		std::stringstream ss(line);
		ss>>ch>>bcolor[0]>>bcolor[1]>>bcolor[2];
		break;
	}

	case 'v': {//viewing
		getline(in, line);
		std::string junk;
		std::stringstream fromss(line);
		fromss>>junk>>eye[0]>>eye[1]>>eye[2];

		getline(in, line);
		std::stringstream atss(line);
		atss>>junk>>at[0]>>at[1]>>at[2];

		getline(in, line);
		std::stringstream upss(line);
		upss>>junk>>up[0]>>up[1]>>up[2];

		getline(in, line);
		std::stringstream angless(line);
		angless>>junk>>angle;

		getline(in, line);
		std::stringstream hitherss(line);
		hitherss>>junk>>hither;

		getline(in, line);
		std::stringstream resolutionss(line);
		resolutionss>>junk>>res[0]>>res[1];
		break;
	}

	case 'p': {//polygons/triangles
		bool patch = false;
		std::stringstream ssn(line);
		unsigned int nverts;
		if (line[1] == 'p') {
		patch = true;
		ssn>>ch;
		}
		ssn>>ch>>nverts;
		std::vector<Eigen::Vector4d> vertices;
		std::vector<Eigen::Vector4d> normals;
		for (unsigned int i=0; i<nverts; i++) {
		getline(in, line);
		std::stringstream ss(line);
		Eigen::Vector4d v,n;
		if (patch){
			ss>>v[0]>>v[1]>>v[2]>>n[0]>>n[1]>>n[2];
			v[3]=1;
			n[3]=0;
		}
		else{
			ss>>v[0]>>v[1]>>v[2];
			v[3]=1;
		}
		vertices.push_back(v);
		if (patch) {
			n.normalize();
			normals.push_back(n);
		}
		}

		bool makeTriangles = false;
		if (vertices.size() == 3) {
			makeTriangles = true;
		if (patch) {
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
						normals[0], normals[1], normals[2]), fill));
		} else {
			//calculate normals of the triangle 
			Eigen::Vector4d n0 = (vertices[1] - vertices[0]).cross3(vertices[2] - vertices[0]);
			Eigen::Vector4d n1 = (vertices[2] - vertices[1]).cross3(vertices[0] - vertices[1]);
			Eigen::Vector4d n2 = (vertices[0] - vertices[2]).cross3(vertices[1] - vertices[2]);
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
		}
		} else if (vertices.size() == 4) {
		//calculate normals of the triangles
		Eigen::Vector4d n0 = (vertices[1] - vertices[0]).cross3(vertices[2] - vertices[0]);
		Eigen::Vector4d n1 = (vertices[2] - vertices[1]).cross3(vertices[3] - vertices[1]);
		Eigen::Vector4d n2 = (vertices[3] - vertices[2]).cross3(vertices[0] - vertices[2]);
		Eigen::Vector4d n3 = (vertices[0] - vertices[3]).cross3(vertices[1] - vertices[3]);
		if (n0.dot(n1) > 0 && n0.dot(n2) > 0 && n0.dot(n3) > 0) {
			makeTriangles = true;
			if (patch) {
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
						normals[0], normals[1], normals[2]), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3], 
						normals[0], normals[2], normals[3]), fill));
			} else {
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3],n0,n2,n3), fill));
			}
		}
		}
		break;
	}
	case 'f' : {//fill
		std::stringstream ss(line);
		ss>>ch>>fill.color[0]>>fill.color[1]>>fill.color[2]>>fill.kd>>fill.ks>>fill.shine>>fill.transmittance>>fill.ior;
		break;
	}

	case 'l' : {//light
		std::stringstream ss(line);
		Light l;
		ss>>ch>>l.p[0]>>l.p[1]>>l.p[2];
		l.p[3]=1;
		if (!ss.eof()) {
		ss>>l.c[0]>>l.c[1]>>l.c[2];
		coloredlights = true;
		}
		lights.push_back(l);
		break;
	}

	default:
		break;
	}
	}


	if (!coloredlights) {
	for (unsigned int i=0; i<lights.size(); i++) {
		lights[i].c[0] = 1.0/sqrt(lights.size());
		lights[i].c[1] = 1.0/sqrt(lights.size());
		lights[i].c[2] = 1.0/sqrt(lights.size());
	}
	}
	//create im or the image array
	im = new Eigen::Vector3d[res[0]*res[1]];
	aperture = 0.0;
}
//destructs Renderer
Renderer::~Renderer() {
	if (im) delete [] im;
	for (unsigned int i=0;i<triangles.size();i++) delete triangles[i].first;
}
/**
 * does the vertex processing including vertex transformation and shading
 * @param s pair of a Triangle* and a Fill used for transforming the triangle and shading the verticies
 * @param cam the camera
 */
void Renderer::vertexProcessing(std::pair<Triangle*,Fill> s,Camera cam){
	Triangle *tri=s.first;
	Fill f=s.second;
	Eigen::Vector4d v[3];
	Eigen::Vector4d eye;
	Eigen::Vector3d color[3];
	tri->transform(M);//transforms the triangle
	color[0]<<0.0,0.0,0.0;
	color[1]<<0.0,0.0,0.0;
	color[2]<<0.0,0.0,0.0;
	eye<<cam.eye,1.0;
	//calculate viewing of the vertices from the eye
	v[0]= eye - tri->a;
	v[1]= eye - tri->b;
	v[2]= eye - tri->c;
	v[0].normalize();
	v[1].normalize();
	v[2].normalize();
	//for every light do shading
	for (unsigned int i=0; i<lights.size(); i++) {
		Light &light = lights[i];
		Eigen::Vector4d l[3];
		//find the light dir
		l[0]= light.p - tri->a;
		l[1]= light.p - tri->b;
		l[2]= light.p - tri->c;
		l[0].normalize();
		l[1].normalize();
		l[2].normalize();
		Eigen::Vector4d h[3];
		//find the halfway vector
		h[0]=l[0]+v[0];
		h[1]=l[1]+v[1];
		h[2]=l[2]+v[2];
		h[0].normalize();
		h[1].normalize();
		h[2].normalize();
		double diffuse[3];
		//find diffuse for each vertex
		diffuse[0]= std::max(0.0, (tri->n1).dot(l[0]));
		diffuse[1]= std::max(0.0, (tri->n2).dot(l[1]));
		diffuse[2]= std::max(0.0, (tri->n3).dot(l[2]));
		double specular[3];
		//find specular for each vertex
		specular[0]=pow(std::max(0.0, (tri->n1).dot(h[0])), f.shine);
		specular[1]=pow(std::max(0.0, (tri->n2).dot(h[1])), f.shine);
		specular[2]=pow(std::max(0.0, (tri->n3).dot(h[2])), f.shine);
		Eigen::Vector3d c[3];
		//find the color at each vertex
		c[0]=(f.kd * diffuse[0] * f.color + f.ks * specular[0] * Eigen::Vector3d(1.0,1.0,1.0));
		c[1]=(f.kd * diffuse[1] * f.color + f.ks * specular[1] * Eigen::Vector3d(1.0,1.0,1.0));
		c[2]=(f.kd * diffuse[2] * f.color + f.ks * specular[2] * Eigen::Vector3d(1.0,1.0,1.0));
		c[0][0] *= light.c[0], c[0][1] *= light.c[1], c[0][2] *= light.c[2];
		c[1][0] *= light.c[0], c[1][1] *= light.c[1], c[1][2] *= light.c[2];
		c[2][0] *= light.c[0], c[2][1] *= light.c[1], c[2][2] *= light.c[2];
		color[0] += c[0];
		color[1] += c[1];
		color[2] += c[2];
	}
	//set the vertex color to color
	tri->c0=color[0];
	tri->c1=color[1];
	tri->c2=color[2];
}
/**
 *Does the rasterization of the image
 * @param t pair of a Triangle* and a Fill used for making the fragments
 * @param resx x resolution of the image used to loop over pixels
 * @param resy y resolution of the image used to loop over pixels
 */
void Renderer::rasterization(std::pair<Triangle*,Fill> t,int resx, int resy){
	int minX,minY,maxX,maxY;
	Triangle *tri=t.first;
	// std::cout<<"a"<<tri->ap<<"b"<<tri->bp<<"c"<<tri->cp<<std::endl;
	//Homogenous divide
	tri->ap[0]/=tri->ap[3];
	tri->ap[1]/=tri->ap[3];
	tri->ap[2]/=tri->ap[3];
	tri->bp[0]/=tri->bp[3];
	tri->bp[1]/=tri->bp[3];
	tri->bp[2]/=tri->bp[3];
	tri->cp[0]/=tri->cp[3];
	tri->cp[1]/=tri->cp[3];
	tri->cp[2]/=tri->cp[3];
	//Find min and max xvals for bounding box
	if(tri->ap[0]<tri->bp[0]){
		if(tri->bp[0]<tri->cp[0]){
			minX=round(tri->ap[0]);
			maxX=round(tri->cp[0]);
		}
		else if(tri->ap[0]<tri->cp[0]){
			minX=round(tri->ap[0]);
			maxX=round(tri->bp[0]);
		}
		else{
			minX=round(tri->cp[0]);
			maxX=round(tri->bp[0]);
		}
	}
	else{
		if(tri->ap[0]<tri->cp[0]){
			minX=round(tri->bp[0]);
			maxX=round(tri->cp[0]);
		}
		else if(tri->bp[0]<tri->cp[0]){
			minX=round(tri->bp[0]);
			maxX=round(tri->ap[0]);
		}
		else{
			minX=round(tri->cp[0]);
			maxX=round(tri->ap[0]);
		}
	}
	//find min and max y vals for bounding box
	if(tri->ap[1]<tri->bp[1]){
		if(tri->bp[1]<tri->cp[1]){
			minY=round(tri->ap[1]);
			maxY=round(tri->cp[1]);
		}
		else if(tri->ap[1]<tri->cp[1]){
			minY=round(tri->ap[1]);
			maxY=round(tri->bp[1]);
		}
		else{
			minY=round(tri->cp[1]);
			maxY=round(tri->bp[1]);
		}
	}
	else{
		if(tri->ap[1]<tri->cp[1]){
			minY=round(tri->bp[1]);
			maxY=round(tri->cp[1]);
		}
		else if(tri->bp[1]<tri->cp[1]){
			minY=round(tri->bp[1]);
			maxY=round(tri->ap[1]);
		}
		else{
			minY=round(tri->cp[1]);
			maxY=round(tri->ap[1]);
		}
	}
	minX-=1;minY-=1;maxX+=1;maxY+=1;//make sure bounding box includes all of the triangle
	Eigen::Vector2d v0,v1,v2,p;
	v0<<tri->ap[0],tri->ap[1];
	v1<<tri->bp[0],tri->bp[1];
	v2<<tri->cp[0],tri->cp[1];
	double alpha,beta,gamma,alF,beF,gaF;
	//compute area of the triangle for alpha beta and gamma
	alF=((v1[1]-v2[1])*v0[0] + (v2[0]-v1[0])*v0[1] +(v1[0]*v2[1])-(v2[0]*v1[1]));
	beF=((v2[1]-v0[1])*v1[0] + (v0[0]-v2[0])*v1[1] +(v2[0]*v0[1])-(v0[0]*v2[1]));
	gaF=((v0[1]-v1[1])*v2[0] + (v1[0]-v0[0])*v2[1] +(v0[0]*v1[1])-(v1[0]*v0[1]));
	Eigen::Vector3d color;
	// std::cout<<"minx "<<minX<<" maxx "<<maxX<<" miny "<<minY<<" maxy "<<maxY<<std::endl;
	for(unsigned int i=minX;i<maxX&&i<resx;i++){
		for(unsigned int j=minY;j<maxY&&j<resy;j++){
			p<<i,j;
			//find alpha beta gamma, area of relavent triangle with p / alF||beF||gaF respectively
			alpha=((v1[1]-v2[1])*p[0] + (v2[0]-v1[0])*p[1] +(v1[0]*v2[1])-(v2[0]*v1[1]))/((v1[1]-v2[1])*v0[0] + (v2[0]-v1[0])*v0[1] +(v1[0]*v2[1])-(v2[0]*v1[1]));
			beta=((v2[1]-v0[1])*p[0] + (v0[0]-v2[0])*p[1] +(v2[0]*v0[1])-(v0[0]*v2[1]))/((v2[1]-v0[1])*v1[0] + (v0[0]-v2[0])*v1[1] +(v2[0]*v0[1])-(v0[0]*v2[1]));
			gamma=((v0[1]-v1[1])*p[0] + (v1[0]-v0[0])*p[1] +(v0[0]*v1[1])-(v1[0]*v0[1]))/((v0[1]-v1[1])*v2[0] + (v1[0]-v0[0])*v2[1] +(v0[0]*v1[1])-(v1[0]*v0[1]));
			if(alpha>=0&&beta>=0&&gamma>=0){//if the point is in the triangle
				if((alpha>0 || (alF*((v1[1]-v2[1])*(-1) + (v2[0]-v1[0])*(-1) +(v1[0]*v2[1])-(v2[0]*v1[1])))>0)&&(beta>0 ||
					(beF*((v2[1]-v0[1])*(-1) + (v0[0]-v2[0])*(-1) +(v2[0]*v0[1])-(v0[0]*v2[1])))>0)&&(gamma>0 || 
					(gaF*((v0[1]-v1[1])*(-1) + (v1[0]-v0[0])*(-1) +(v0[0]*v1[1])-(v1[0]*v0[1])))>0)){//got this if statement from the book
					//interpolate colors and z, then add them to the fragment
					color=(alpha*tri->c0)+(beta*tri->c1)+(gamma*tri->c2);
					fragments[j*resy+i].push_back(Fragment((alpha*tri->ap[2])+(beta*tri->bp[2])+(gamma*tri->cp[2]),color));
				}
			}
		}
	}
	// std::cout<<"EoR"<<std::endl;
}
/**
 * Does the blending
 * @param resx x resolution of the image used to loop over pixels
 * @param resy y resolution of the image used to loop over pixels
 */
void Renderer::blending(int resx,int resy){
	double minZ;
	Eigen::Vector3d color;
	for(unsigned int i=0;i<resx;i++){
		for(unsigned int j=0;j<resy;j++){
			minZ=DBL_MAX;
			color=bcolor;
			// std::cout<<fragments[i*resy+j].size()<<std::endl;
			for(unsigned int f=0;f<fragments[j*resy+i].size();f++){//for each fragment at this pixel
				if(fragments[j*resy+i][f].z<minZ){
					minZ=fragments[j*resy+i][f].z;
					color=fragments[j*resy+i][f].color;
				}
			}
			im[j*resy+i]=color;
		}
	}
}
/**
 * Does the Graphics pipeline and renders the image
 * @param cam Camera used during vertexProcessing
 * @param resx x resolution of the image used in rasterization and blending
 * @param resy y resolution of the image used in rasterization and blending
 */
void Renderer::render(Camera cam,int resx,int resy){
	std::cout<<"VP & Raster"<<std::endl;
	//vertex processing and rasterization for each triangle
	for(unsigned int i=0;i<triangles.size();i++){
		vertexProcessing(triangles[i],cam);
		rasterization(triangles[i],resx,resy);
	}
	std::cout<<"EoR"<<std::endl;
	std::cout<<"blending"<<std::endl;
	//blending
	blending(resx,resy);
}
/**
 *Sets up the M matrix
 @param cam the camera in the scene
 @param l left bound
 @param r right bound
 @param t top boubd
 @param b bottom bound
 @param n near
 @param resx the x resolution of the image
 @param resy the y resolution of the image
 @param u eigen vector 3d u
 @param w eigen vector 3d v
 @param w eigen vector 3d w
 */
void Renderer::setUpM(Camera cam, double l, double r, double t, double b, double n, double resx,double resy,const Eigen::Vector3d &u, const Eigen::Vector3d &v, const Eigen::Vector3d &w){
	Eigen::Matrix4d Mcam,Mvp,Mper,Morth,eyeinv;
	// std::cout<<cam.eye<<std::endl;
	double f=1000*n;//far is 1000*near
	//setup Mcam
	Mcam<<u[0],u[1],u[2],0.0,
		v[0],v[1],v[2],0.0,
		w[0],w[1],w[2],0.0,
		0.0,0.0,0.0,1.0;
	eyeinv<<1.0,0.0,0.0,-cam.eye[0],
		0.0,1.0,0.0,-cam.eye[1],
		0.0,0.0,1.0,-cam.eye[2],
		0.0,0.0,0.0,1.0;
	Mcam=Mcam*eyeinv;
	//setup Mper
	Morth<<(2/(r-l)), 0.0, 0.0, (-(r+l)/(r-l)),
		0.0, (2/(t-b)), 0.0, (-(t+b)/(t-b)),
		0.0, 0.0, (2/(n-f)), (-(n+f)/(n-f)),
		0.0, 0.0, 0.0, 1.0;
	Mper<<n,0.0,0.0,0.0,
		0.0,n,0.0,0.0,
		0.0,0.0,n+f,-f*n,
		0.0,0.0,1.0,0.0;
	Mper=Morth*Mper;
	//Mper 0,0 and 1,1 were originally off by a factor of 10(they were 0.2xxx instead of 2.xxx)
	//couldn't figure out where Mper went wrong because it's exactly like the book, but this seems to fix it
	Mper(0,0)*=10;
	Mper(1,1)*=10;
	//Mper<<-2.41421,0,0,0,0,-2.41421,0,0,0,0,-1.002,2.002,0,0,1,0; was used for testing
	//setup MVP
	Mvp<<resx/2.0,0.0,0.0,(resx-1)/2.0,
		0.0,resy/2.0,0.0,(resy-1)/2.0,
		0.0,0.0,1.0,0.0,
		0.0,0.0,0.0,1.0;
	//M=Mvp*Morth*Mp*Mcam
	M=Mvp*Mper*Mcam;
	std::cout<<"Mvp=\n"<<Mvp<<std::endl;
	std::cout<<"Mper=\n"<<Mper<<std::endl;
	std::cout<<"Mcam=\n"<<Mcam<<std::endl;
	std::cout<<"M=\n"<<M<<std::endl;
	
}
//creates the image
void Renderer::createImage() {
	// set up coordinate system
	Eigen::Vector3d w = eye - at;
	w /= w.norm();
	Eigen::Vector3d u = up.cross(w);
	u.normalize();
	Eigen::Vector3d v = w.cross(u);

	//std::cout<<u<<" "<<v<<" "<<w<<std::endl;

	double d = (eye - at).norm();
	double h = tan((M_PI/180.0) * (angle/2.0)) * d;
	double increment = (2*h) / res[0];
	double l = -h + 0.5*increment;
	double t = h*(((double)res[1])/res[0]) - 0.5*increment;

	if (verbose) {
	std::cout<<"u: ["<<u[0]<<", "<<u[1]<<", "<<u[2]<<"]"<<std::endl;
	std::cout<<"v: ["<<v[0]<<", "<<v[1]<<", "<<v[2]<<"]"<<std::endl;
	std::cout<<"w: ["<<w[0]<<", "<<w[1]<<", "<<w[2]<<"]"<<std::endl;
	std::cout<<"d: "<<d<<std::endl;
	std::cout<<"deltax / h: "<<increment<<std::endl;
	std::cout<<"left limit: "<<l<<std::endl;
	std::cout<<"top limit: "<<t<<std::endl;
	}
	Camera cam;
	fragments=new std::vector<Fragment>[res[0]*res[1]];
	// std::cout<<"Camera"<<std::endl;
	//makes the camera
	cam= createCamera(eye,at,up);
	// std::cout<<"setUpM"<<std::endl;
	// std::cout<<"l:"<<l<<std::endl;
	// std::cout<<"t:"<<t<<std::endl;
	//sets up M
	Renderer::setUpM(cam,l,-l,t,-t,hither,res[0],res[1],u,v,w);
	// std::cout<<"Render"<<std::endl;
	//render the image
	Renderer::render(cam,res[0],res[1]);
	// std::cout<<"ended render"<<std::endl;
	delete fragments;
}
//writes the image
void Renderer::writeImage(const std::string &fname) {
	CImg<unsigned char> output(res[0], res[1], 1, 3);
	for (unsigned int i=0; i<output.width(); i++) {
	for (unsigned int j=0; j<output.height(); j++) {
		output(i, j, 0, 0) = std::min(1.0, std::max(0.0, im[j*output.width()+i][0]))*255.0;
		output(i, j, 0, 1) = std::min(1.0, std::max(0.0, im[j*output.width()+i][1]))*255.0;
		output(i, j, 0, 2) = std::min(1.0, std::max(0.0, im[j*output.width()+i][2]))*255.0;
	}
	}
	if (strstr(fname.c_str(), "png")) {
		output.save_png(fname.c_str());
		return;
	} else if (strstr(fname.c_str(), "jpg")) {
		output.save_jpeg(fname.c_str());
		return;
	}

	#ifdef __APPLE__
		std::ofstream out(fname, std::ios::out | std::ios::binary);
	#else
		std::ofstream out(fname.c_str(), std::ios_base::binary);
	#endif
	out<<"P6"<<"\n"<<res[0]<<" "<<res[1]<<"\n"<<255<<"\n";
	Eigen::Vector3d *pixel = im;
	char val;
	for (unsigned int i=0; i<res[0]*res[1]; i++, pixel++) {
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[0])) * 255.0);
		out.write (&val, sizeof(unsigned char));
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[1])) * 255.0);
		out.write (&val, sizeof(unsigned char));
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[2])) * 255.0);
		out.write (&val, sizeof(unsigned char));
	}
	out.close();
}

int main(int argc, char *argv[]) {
	int c;
	double aperture = 0.0;
	bool verbose = false;
	int mode = 3;

	if (argc-optind != 2) {
	std::cout<<"usage: raster input.nff output.ppm"<<std::endl;
	for (int i=0; i<argc; i++) std::cout<<argv[i]<<std::endl;
	exit(0);
	}	

	Renderer renderer(argv[optind++]);
	renderer.aperture = aperture;
	renderer.verbose = verbose;
	std::cout<<"creating"<<std::endl;
	renderer.createImage();
	std::cout<<"writing "<<argv[optind]<<std::endl;
	renderer.writeImage(argv[optind++]);
};
