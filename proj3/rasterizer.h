#ifndef RASTERIZER_H
#define RASTERIZER_H

//#include "slVector.H"
#include <vector>
#include <Eigen/Dense>


class Fill {
public: 
	Eigen::Vector3d color;
	double kd, ks, shine, transmittance, ior;
};

class HitRecord {
public:
	double t, alpha, beta, gamma;
	Eigen::Vector3d p, n, v;
	Fill f;
	int raydepth;
};

class Light {
public:
	Eigen::Vector4d p;
	Eigen::Vector3d c;
};

class Surface {
public:
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const = 0;
	virtual void transform(Eigen::Matrix4d m);
	virtual ~Surface() {};
};

// class Triangle : public Surface {
// protected:
// public:
// 	Triangle(const Eigen::Vector4d &_a, const Eigen::Vector4d &_b, const Eigen::Vector4d &_c) : a(_a), b(_b), c(_c) {};
// 	virtual void transform(Eigen::Matrix4d m);
// 	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const;
// };

class Triangle : public Surface {	
public:
	Eigen::Vector4d n1, n2, n3,a,b,c,ap,bp,cp;
	Eigen::Vector3d c0,c1,c2;
	Triangle(const Eigen::Vector4d &_a, const Eigen::Vector4d &_b, const Eigen::Vector4d &_c,
		const Eigen::Vector4d &_n1, const Eigen::Vector4d &_n2, const Eigen::Vector4d &_n3) 
	: /*Triangle(_a,_b,_c)*/a(_a), b(_b), c(_c), n1(_n1), n2(_n2), n3(_n3) {};
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const;
	virtual void transform(Eigen::Matrix4d m);
};

class Poly : public Surface {
public:
	std::vector<Eigen::Vector4d> verts;
	std::vector<Eigen::Vector4d> vertsP;
	Poly(const std::vector<Eigen::Vector4d> &_verts) : verts(_verts) {};
	virtual void transform(Eigen::Matrix4d m); 
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const;
};

class PolyPatch : public Poly {
	std::vector<Eigen::Vector4d> normals;
public:
	PolyPatch(const std::vector<Eigen::Vector4d> &_verts, const std::vector<Eigen::Vector4d> &_normals) : Poly(_verts), normals(_normals) {}; 
};
class Camera{
	public:
		Eigen::Vector3d eye,at,up;
};
class Fragment{
	public:
	Fragment(double _z,Eigen::Vector3d c):z(_z),color(c){};
		double z;
		Eigen::Vector3d color;
};
class Renderer {
	Eigen::Vector3d bcolor, eye, at, up;
	double angle, hither;
	unsigned int res[2];
	std::vector<std::pair<Surface *, Fill> > surfaces;
	std::vector<std::pair<Triangle *, Fill> > triangles;
	std::vector<Light> lights;
	double shadowbias;

	Eigen::Vector3d *im;
public:
	Renderer(const std::string &fname);
	~Renderer();
	void createImage();
	void setUpM(Camera cam, double l, double r, double t, double b, double n,double resx,double resy, Eigen::Vector3d u, Eigen::Vector3d v, Eigen::Vector3d w);  
	void vertexProcessing(std::pair<Triangle*,Fill> s,Camera cam);
	void rasterization(std::pair<Triangle*,Fill> t,int resy);
	void fragmentProcessing();
	void blending(int resx,int resy);
	void clipping();
	void render(Camera cam,int resx,int resy);
	Eigen::Vector3d shade(const HitRecord &hr) const;
	void writeImage(const std::string &fname);
	std::vector<Fragment> *fragments;
	Eigen::Matrix4d M;
	bool color;
	bool phong;
	bool stratified;
	bool reflections;
	bool shadows;
	bool verbose;
	int samples;
	double aperture;
	int maxraydepth;
};
	
#endif
