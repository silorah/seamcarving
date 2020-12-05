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
//inline Eigen::Vector4d cross(const Eigen::Vector4d &x, const Eigen::Vector4d &y) {return x.cross(y);}
inline double dot(const Eigen::Vector3d &x, const Eigen::Vector3d &y) {return x.dot(y);}
inline double cross2d(const Eigen::Vector2d &x, const Eigen::Vector2d &y) {return x[0]*y[1]-x[1]*y[0];}
// Eigen::Vector4d cross4d(Eigen::Vector4d &x,Eigen::Vector4d &y){
// 	Eigen::Vector4d res;
// 	res[0]=
// 	res[1]=
// 	res[2]=
// 	res[3]=
// }
// Eigen::Vector4d Triangle::normal(const Eigen::Vector4d &x) const {
// 	Eigen::Vector4d n = cross(b-a, c-a);
// 	n.normalize();
// 	return n;
// }
void Triangle::transform(const Eigen::Matrix4d &m){
	ap=m*a;
	bp=m*b;
	cp=m*c;
	// std::cout<<"\na"<<a<<"\naP"<<ap<<"\nb"<<b<<"\nbp"<<bp<<"\nc"<<c<<"\ncP"<<cp<<std::endl;
}
// Eigen::Vector4d Triangle/*Patch*/::normal(const Eigen::Vector4d &x) const {
// 	Eigen::Vector4d n = cross(b-a, c-a);
// 	double area = n.norm();
// 	n = cross(x-a, c-a);
// 	double alpha = n.norm()/area;
// 	n = cross(x-a, b-a);
// 	double beta = n.norm()/area;
// 	n = (1-alpha-beta)*n1 + alpha*n2 + beta*n3;
// 	n.normalize();
// 	return n;
// }

// Eigen::Vector4d Poly::normal(const Eigen::Vector4d &x) const {
// 	Eigen::Vector4d n = cross(verts[1]-verts[0],verts[2]-verts[0]);
// 	n.normalize();
// 	return n;
// }
void Poly::transform(Eigen::Matrix4d &m){
	for(int i=0;i<verts.size();i++){
		vertsP.push_back(m*verts[i]);
	}
}
Camera createCamera(Eigen::Vector3d &eye, Eigen::Vector3d &at,Eigen::Vector3d &up){
	Camera cam;
	//cam=new Camera;
	cam.eye=eye;
	cam.at=at;
	cam.up=up;
	return cam;
}

Renderer::Renderer(const std::string &fname) {
	std::ifstream in(fname.c_str(), std::ios_base::in);
	std::string line;
	char ch;
	Fill fill;
	bool coloredlights = false;

	while (in) {
	getline(in, line);
	switch (line[0]) {
	case 'b': {
		std::stringstream ss(line);
		ss>>ch>>bcolor[0]>>bcolor[1]>>bcolor[2];
		break;
	}

	case 'v': {
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

	case 'p': {
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
			/*surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
						normals[0], normals[1], normals[2]), fill));*/
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
						normals[0], normals[1], normals[2]), fill));
		} else {
			Eigen::Vector4d n0 = (vertices[1] - vertices[0]).cross3(vertices[2] - vertices[0]);
			Eigen::Vector4d n1 = (vertices[2] - vertices[1]).cross3(vertices[0] - vertices[1]);
			Eigen::Vector4d n2 = (vertices[0] - vertices[2]).cross3(vertices[1] - vertices[2]);
			//surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
		}
		} else if (vertices.size() == 4) {
		Eigen::Vector4d n0 = (vertices[1] - vertices[0]).cross3(vertices[2] - vertices[0]);
		Eigen::Vector4d n1 = (vertices[2] - vertices[1]).cross3(vertices[3] - vertices[1]);
		Eigen::Vector4d n2 = (vertices[3] - vertices[2]).cross3(vertices[0] - vertices[2]);
		Eigen::Vector4d n3 = (vertices[0] - vertices[3]).cross3(vertices[1] - vertices[3]);
		if (n0.dot(n1) > 0 && n0.dot(n2) > 0 && n0.dot(n3) > 0) {
			makeTriangles = true;
			if (patch) {
			// surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
			// 			normals[0], normals[1], normals[2]), fill));
			// surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3], 
			// 			normals[0], normals[2], normals[3]), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2], 
						normals[0], normals[1], normals[2]), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3], 
						normals[0], normals[2], normals[3]), fill));
			} else {
			// surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
			// surfaces.push_back(std::pair<Surface *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3],n0,n2,n3), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[1], vertices[2],n0,n1,n2), fill));
			triangles.push_back(std::pair<Triangle *, Fill>(new Triangle(vertices[0], vertices[2], vertices[3],n0,n2,n3), fill));
			}
		}
		}
		if (!makeTriangles) {
		if (patch) {
			surfaces.push_back(std::pair<Surface *, Fill>(new PolyPatch(vertices, normals), fill));
		} else {
			surfaces.push_back(std::pair<Surface *, Fill>(new Poly(vertices), fill));
		}
		}
		break;
	}
	case 'f' : {
		std::stringstream ss(line);
		ss>>ch>>fill.color[0]>>fill.color[1]>>fill.color[2]>>fill.kd>>fill.ks>>fill.shine>>fill.transmittance>>fill.ior;
		break;
	}

	case 'l' : {
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
	im = new Eigen::Vector3d[res[0]*res[1]];
	shadowbias = 1e-6;
	samples = 1;
	aperture = 0.0;


}

Renderer::~Renderer() {
	if (im) delete [] im;
	for (unsigned int i=0; i<surfaces.size(); i++) delete surfaces[i].first;
	for (unsigned int i=0;i<triangles.size();i++) delete triangles[i].first;
}
void Renderer::vertexProcessing(std::pair<Triangle*,Fill> s,Camera cam){
	Triangle *tri=s.first;
	Fill f=s.second;
	Eigen::Vector4d v[3];
	Eigen::Vector4d eye;
	Eigen::Vector3d color[3];
	tri->transform(M);
	color[0]<<0.0,0.0,0.0;
	color[1]<<0.0,0.0,0.0;
	color[2]<<0.0,0.0,0.0;
	eye<<cam.eye,1.0;
	v[0]= eye - tri->a;
	v[1]= eye - tri->b;
	v[2]= eye - tri->c;
	v[0].normalize();
	v[1].normalize();
	v[2].normalize();
	for (unsigned int i=0; i<lights.size(); i++) {
		Light &light = lights[i];
		Eigen::Vector4d l[3];
		l[0]= light.p - tri->a;
		l[1]= light.p - tri->b;
		l[2]= light.p - tri->c;
		l[0].normalize();
		l[1].normalize();
		l[2].normalize();
		Eigen::Vector4d h[3];
		h[0]=l[0]+v[0];
		h[1]=l[1]+v[1];
		h[2]=l[2]+v[2];
		h[0].normalize();
		h[1].normalize();
		h[2].normalize();
		double diffuse[3];
		diffuse[0]= std::max(0.0, (tri->n1).dot(l[0]));
		diffuse[1]= std::max(0.0, (tri->n2).dot(l[1]));
		diffuse[2]= std::max(0.0, (tri->n3).dot(l[2]));
		double specular[3];
		specular[0]=pow(std::max(0.0, (tri->n1).dot(h[0])), f.shine);
		specular[1]=pow(std::max(0.0, (tri->n2).dot(h[1])), f.shine);
		specular[2]=pow(std::max(0.0, (tri->n3).dot(h[2])), f.shine);
		Eigen::Vector3d c[3];
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
	tri->c0=color[0];
	tri->c1=color[1];
	tri->c2=color[2];
}
void Renderer::rasterization(std::pair<Triangle*,Fill> t,int resx, int resy){
	int minX,minY,maxX,maxY;
	Triangle *tri=t.first;
	// std::cout<<"a"<<tri->ap<<"b"<<tri->bp<<"c"<<tri->cp<<std::endl;
	//Find min and max xvals for bounding box
	tri->ap[0]/=tri->ap[3];
	tri->ap[1]/=tri->ap[3];
	tri->ap[2]/=tri->ap[3];
	tri->bp[0]/=tri->bp[3];
	tri->bp[1]/=tri->bp[3];
	tri->bp[2]/=tri->bp[3];
	tri->cp[0]/=tri->cp[3];
	tri->cp[1]/=tri->cp[3];
	tri->cp[2]/=tri->cp[3];
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
	// 2dcross(v1-v0,v2-v0);
	double alpha,beta,gamma;
	Eigen::Vector3d color;
	// std::cout<<"minx "<<minX<<" maxx "<<maxX<<" miny "<<minY<<" maxy "<<maxY<<std::endl;
	for(unsigned int i=minX;i<maxX&&i<resx;i++){
		for(unsigned int j=minY;j<maxY&&j<resy;j++){
			p<<i,j;
			alpha=((v1[1]-v2[1])*p[0] + (v2[0]-v1[0])*p[1] +(v1[0]*v2[1])-(v2[0]*v1[1]))/((v1[1]-v2[1])*v0[0] + (v2[0]-v1[0])*v0[1] +(v1[0]*v2[1])-(v2[0]*v1[1]));
			beta=((v2[1]-v0[1])*p[0] + (v0[0]-v2[0])*p[1] +(v2[0]*v0[1])-(v0[0]*v2[1]))/((v2[1]-v0[1])*v1[0] + (v0[0]-v2[0])*v1[1] +(v2[0]*v0[1])-(v0[0]*v2[1]));;
			gamma=((v0[1]-v1[1])*p[0] + (v1[0]-v0[0])*p[1] +(v0[0]*v1[1])-(v1[0]*v0[1]))/((v0[1]-v1[1])*v2[0] + (v1[0]-v0[0])*v2[1] +(v0[0]*v1[1])-(v1[0]*v0[1]));;
			if(alpha>0&&beta>0&&gamma>0){
				color=alpha*tri->c0+beta*tri->c1+gamma*tri->c2;
				fragments[i*resy+j].push_back(Fragment((alpha*tri->ap[3])+(beta*tri->bp[3])+(gamma*tri->cp[3]),color));
			}
		}
	}
	// std::cout<<"EoR"<<std::endl;
}
void Renderer::blending(int resx,int resy){
	double minZ;
	Eigen::Vector3d color;
	for(unsigned int i=0;i<resx;i++){
		for(unsigned int j=0;j<resy;j++){
			minZ=DBL_MAX;
			color=bcolor;
			// std::cout<<fragments[i*resy+j].size()<<std::endl;
			for(unsigned int f=0;f<fragments[i*resy+j].size();f++){
				if(fragments[i*resy+j][f].z<minZ){
					// std::cout<<"What the Fuck!"<<std::endl;
					minZ=fragments[i*resy+j][f].z;
					color=fragments[i*resy+j][f].color;
				}
			}
			im[i*resy+j]=color;
		}
	}
}
void Renderer::render(Camera cam,int resx,int resy){
	std::cout<<"VP"<<std::endl;
	for(unsigned int i=0;i<triangles.size();i++){
		vertexProcessing(triangles[i],cam);
		std::cout<<"Raster "<<i<<std::endl;
		rasterization(triangles[i],resx,resy);
	}
	std::cout<<"EoR"<<std::endl;
	std::cout<<"blending"<<std::endl;
	blending(resx,resy);
}
void Renderer::setUpM(Camera cam, double l, double r, double t, double b, double n, double resx,double resy,const Eigen::Vector3d &u, const Eigen::Vector3d &v, const Eigen::Vector3d &w){
	Eigen::Matrix4d Mcam,Mvp,Mper,Morth,eyeinv;
	std::cout<<cam.eye<<std::endl;
	double f=1000*n;
	Mcam<<u[0],u[1],u[2],0.0,
		v[0],v[1],v[2],0.0,
		w[0],w[1],w[2],0.0,
		0.0,0.0,0.0,1.0;
	eyeinv<<1.0,0.0,0.0,-cam.eye[0],
		0.0,1.0,0.0,-cam.eye[1],
		0.0,0.0,1.0,-cam.eye[2],
		0.0,0.0,0.0,1.0;
	Mcam=Mcam*eyeinv;
	Morth<<(2/(r-l)), 0.0, 0.0, (-(r+l)/(r-l)),
		0.0, (2/(t-b)), 0.0, (-(t+b)/(t-b)),
		0.0, 0.0, (2/(n-f)), (-(n+f)/(n-f)),
		0.0, 0.0, 0.0, 1.0;
	Mper<<n,0.0,0.0,0.0,
		0.0,n,0.0,0.0,
		0.0,0.0,n+f,f*n,
		0.0,0.0,1.0,0.0;
	Mper=Morth*Mper;
	Mper<<-2.41421,0,0,0,0,-2.41421,0,0,0,0,-1.002,-2.002,0,0,1,0;
	Mvp<<resx/2.0,0.0,0.0,(resx-1)/2.0,
		0.0,resy/2.0,0.0,(resy-1)/2.0,
		0.0,0.0,1.0,0.0,
		0.0,0.0,0.0,1.0;
	M=Mvp*Mper*Mcam;
	std::cout<<"Mvp=\n"<<Mvp<<std::endl;
	std::cout<<"Mper=\n"<<Mper<<std::endl;
	std::cout<<"Mcam=\n"<<Mcam<<std::endl;
	std::cout<<"M=\n"<<M<<std::endl;
	
}
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
	std::cout<<"Camera"<<std::endl;
	cam= createCamera(eye,at,up);
	std::cout<<"setUpM"<<std::endl;
	std::cout<<"l:"<<l<<std::endl;
	std::cout<<"t:"<<t<<std::endl;
	Renderer::setUpM(cam,l,-l,t,-t,hither,res[0],res[1],u,v,w);
	std::cout<<"Render"<<std::endl;
	Renderer::render(cam,res[0],res[1]);
	std::cout<<"ended render"<<std::endl;

}

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
	int samples = 1;
	int maxraydepth = 5;
	bool color = false;
	bool phong = false;
	bool stratified = false;
	bool reflections = true;
	bool shadows = true;
	bool verbose = false;
	int mode = 3;
	// while ((c = getopt(argc, argv, "a:s:d:cpjm:v")) != -1) {
	// 	switch(c) {
	// 	case 'a':
	// 		aperture = atof(optarg);
	// 		break;
	// 	case 's':
	// 		samples = atoi(optarg);
	// 		break;
	// 	case 'j':
	// 		stratified = true;
	// 		break;
	// 	case 'c':
	// 		color = true;
	// 		break;
	// 	case 'p':
	// 		phong = true;
	// 		break;
	// 	case 'm':
	// 		mode = atoi(optarg);
	// 		break;
	// 	case 'd':
	// 		maxraydepth = atoi(optarg);
	// 		break;
	// 	case 'v':
	// 		verbose = true;
	// 		break;
	// 	default:
	// 		abort();
	// 	}
	// }

	if (argc-optind != 2) {
	std::cout<<"usage: trace input.nff output.ppm"<<std::endl;
	for (int i=0; i<argc; i++) std::cout<<argv[i]<<std::endl;
	exit(0);
	}	

	switch(mode) {
	case 0:
	reflections = shadows = false;
	break;
	case 1:
	reflections = false;
	break;
	case 2:
	shadows = false;
	break;
	}

	Renderer renderer(argv[optind++]);
	renderer.aperture = aperture;
	renderer.samples = samples;
	renderer.stratified = stratified;
	renderer.color = color;
	renderer.phong = phong;
	renderer.reflections = reflections;
	renderer.shadows = shadows;
	renderer.maxraydepth = maxraydepth;
	renderer.verbose = verbose;
	std::cout<<"creating"<<std::endl;
	renderer.createImage();
	std::cout<<"writing "<<argv[optind]<<std::endl;
	renderer.writeImage(argv[optind++]);
};
