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
	Eigen::Vector3d p, c;
};

class Surface {
public:
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const = 0;
	virtual ~Surface() {};
};

class Triangle : public Surface {
protected:
	Eigen::Vector3d a,b,c;
public:
	Triangle(const Eigen::Vector4d &_a, const Eigen::Vector4d &_b, const Eigen::Vector4d &_c) : a(_a), b(_b), c(_c) {};
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const;
};

class TrianglePatch : public Triangle {
	Eigen::Vector4d n1, n2, n3;
public:
	TrianglePatch(const Eigen::Vector4d &_a, const Eigen::Vector4d &_b, const Eigen::Vector4d &_c,
		const Eigen::Vector4d &_n1, const Eigen::Vector4d &_n2, const Eigen::Vector4d &_n3) 
	: Triangle(_a,_b,_c), n1(_n1), n2(_n2), n3(_n3) {};
	virtual Eigen::Vector4d normal(const Eigen::Vector4d &x) const;
};

class Poly : public Surface {
	std::vector<Eigen::Vector4d> verts;
public:
	Poly(const std::vector<Eigen::Vector4d> &_verts) : verts(_verts) {}; 
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
class Renderer {
	Eigen::Vector3d bcolor, eye, at, up;
	double angle, hither;
	unsigned int res[2];
	std::vector<std::pair<Surface *, Fill> > surfaces;
	std::vector<Light> lights;
	double shadowbias;

	Eigen::Vector3d *im;
public:
	Renderer(const std::string &fname);
	~Renderer();
	void createImage();
	void setUpM();  
	void vertexProcessing();
	void rasterization();
	void fragmentProcessing();
	void blending(Eigen::Vector3d * &img);
	void clipping();
	void render();
	Eigen::Vector3d shade(const HitRecord &hr) const;
	void writeImage(const std::string &fname);

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
