#ifndef RASTERIZER_H
#define RASTERIZER_H

//#include "slVector.H"
#include <vector>
#include <Eigen/Dense>


class Fill {
public: 
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d color;
	double kd, ks, shine, transmittance, ior;
};


class Light {
public:
	Eigen::Vector4d p;
	Eigen::Vector3d c;
};



class Triangle {	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector4d n1, n2, n3,a,b,c,ap,bp,cp;
	Eigen::Vector3d c0,c1,c2;
	Triangle(const Eigen::Vector4d &_a, const Eigen::Vector4d &_b, const Eigen::Vector4d &_c,
		const Eigen::Vector4d &_n1, const Eigen::Vector4d &_n2, const Eigen::Vector4d &_n3) 
	: a(_a), b(_b), c(_c), n1(_n1), n2(_n2), n3(_n3) {};
	virtual void transform(const Eigen::Matrix4d &m);
	virtual ~Triangle() {};
};

class Camera{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		Eigen::Vector3d eye,at,up;
};
class Fragment{
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Fragment(double _z,const Eigen::Vector3d &c):z(_z),color(c){};
		double z;
		Eigen::Vector3d color;
};
class Renderer {
	Eigen::Vector3d bcolor, eye, at, up;
	double angle, hither;
	unsigned int res[2];
	std::vector<std::pair<Triangle *, Fill> > triangles;
	std::vector<Light> lights;
	double shadowbias;

	Eigen::Vector3d *im;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Renderer(const std::string &fname);
	~Renderer();
	void createImage();
	void setUpM(Camera cam, double l, double r, double t, double b, double n,double resx,double resy, const Eigen::Vector3d &u, const Eigen::Vector3d &v, const Eigen::Vector3d &w);  
	void vertexProcessing(std::pair<Triangle*,Fill> s,Camera cam);
	void rasterization(std::pair<Triangle*,Fill> t,int resx,int resy);
	void fragmentProcessing();
	void blending(int resx,int resy);
	void clipping();
	void render(Camera cam,int resx,int resy);
	void writeImage(const std::string &fname);
	std::vector<Fragment> *fragments;
	Eigen::Matrix4d M;
	bool verbose;
	double aperture;
};
	
#endif
