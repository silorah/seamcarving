#include "io.cpp"
void laplacian(const std::vector<Tri> &triangles, const std::vector<Eigen::Vector3d> &x, std::vector<Eigen::Vector3d> &y){
	std::vector<double> m;
    std::vector<Eigen::Vector3d> lap;
	for(int j=0;j<x.size();j++){
        m.push_back(0);
        lap.push_back(Eigen::Vector3d(0,0,0));
    }
	for(int j=0;j<x.size();j++){
        	m[j]=0;
            lap[j]=Eigen::Vector3d(0,0,0);
        }

        for(int t=0;t<triangles.size();t++){
            lap[triangles[t][0]]+=x[triangles[t][1]]-x[triangles[t][0]];
            lap[triangles[t][0]]+=x[triangles[t][2]]-x[triangles[t][0]];

            lap[triangles[t][1]]+=x[triangles[t][0]]-x[triangles[t][1]];
            lap[triangles[t][1]]+=x[triangles[t][2]]-x[triangles[t][1]];

            lap[triangles[t][2]]+=x[triangles[t][0]]-x[triangles[t][2]];
            lap[triangles[t][2]]+=x[triangles[t][1]]-x[triangles[t][2]];

            m[triangles[t][0]]+=2;
            m[triangles[t][1]]+=2;
            m[triangles[t][2]]+=2;
        }
        for(int j=0;j<x.size();j++){
			y[j]=lap[j]/m[j];
		}
}
int main(int argc, char* argv[]){
    std::vector<Eigen::Vector3d> pts,y;
    std::vector<Tri> triangles;
    int n;

    readObjFile(argv[1],pts,triangles);
    std::cout<<"in"<<argv[1]<<std::endl;
    std::cout<<"out"<<argv[2]<<std::endl;
    std::cout<<argc<<std::endl;
    n=atoi(argv[4]);
    std::cout<<n<<std::endl;
    
    for(int j=0;j<pts.size();j++){
        y.push_back(Eigen::Vector3d(0,0,0));
    }
    for(int i =0;i<n;i++){
       	laplacian(triangles,pts,y);
        for(int j=0;j<pts.size();j++){
            pts[j]+=atof(argv[3])*y[j];
        }
    }
    std::cout<<"Writing"<<std::endl;
    writeObjFile(argv[2],pts,triangles);
}