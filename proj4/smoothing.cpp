#include "io.h"
int main(int argc, char* argv[]){
    std::vector<Eigen::Vector3d> pts;
    std::vector<Tri> triangles;
    std::vector<double> m;//mass
    std::vector<Eigen::Vector3d> lap;//laplacian op
    readObjFile(argv[0],pts,triangles);
    writeObjFile(argv[1],pts,triangles);
}