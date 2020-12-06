#include "io.cpp"
/**
 *Laplacian is the umbrella operator to calculate the Laplacian
 @param triangles a vector of Tri objects
 @param x a vector of eigen vector3ds which represents the points
 @param y the laplacian
 */
void laplacian(const std::vector<Tri> &triangles, const std::vector<Eigen::Vector3d> &x, std::vector<Eigen::Vector3d> &y){
	std::vector<double> m;//vector of masses
    std::vector<Eigen::Vector3d> lap;//laplacian vector
	for(int j=0;j<x.size();j++){//setup lap and m
        m.push_back(0);
        lap.push_back(Eigen::Vector3d(0,0,0));
    }
	for(int j=0;j<x.size();j++){//zero out lap and m
        	m[j]=0;
            lap[j]=Eigen::Vector3d(0,0,0);
        }

        for(int t=0;t<triangles.size();t++){//fill in lap and m
            lap[triangles[t][0]]+=x[triangles[t][1]]-x[triangles[t][0]];//lap[the point stored at[0] in triangle t] += x[point at [1]]-x[point at [0]]
            lap[triangles[t][0]]+=x[triangles[t][2]]-x[triangles[t][0]];//lap[the point stored at[0] in triangle t] += x[point at [2]]-x[point at [0]]

            lap[triangles[t][1]]+=x[triangles[t][0]]-x[triangles[t][1]];//lap[the point stored at[1] in triangle t] += x[point at [0]]-x[point at [1]]
            lap[triangles[t][1]]+=x[triangles[t][2]]-x[triangles[t][1]];//lap[the point stored at[1] in triangle t] += x[point at [2]]-x[point at [1]]

            lap[triangles[t][2]]+=x[triangles[t][0]]-x[triangles[t][2]];//lap[the point stored at[2] in triangle t] += x[point at [0]]-x[point at [2]]
            lap[triangles[t][2]]+=x[triangles[t][1]]-x[triangles[t][2]];//lap[the point stored at[2] in triangle t] += x[point at [1]]-x[point at [2]]

            m[triangles[t][0]]+=2;//increment m by 2 (double counting )
            m[triangles[t][1]]+=2;
            m[triangles[t][2]]+=2;
        }
        for(int j=0;j<x.size();j++){
			y[j]=lap[j]/m[j];//fill in y[j] with the lap[j] / mass[j]
		}
}
int main(int argc, char* argv[]){
    std::vector<Eigen::Vector3d> pts,y;
    std::vector<Tri> triangles;
    int n;
    bool r;
    r=readObjFile(argv[1],pts,triangles);
    if(argc!=5){//Making sure the number of arguments are valid
        std::cout<<"Usage: smoothing bunny.obj sbun.obj 1 50"<<std::endl;
        exit(0);
    }
    // std::cout<<r<<std::endl;
    // std::cout<<"in"<<argv[1]<<std::endl;
    // std::cout<<"out"<<argv[2]<<std::endl;
    // std::cout<<argc<<std::endl;
    n=atoi(argv[4]);//num iterations
    // std::cout<<n<<std::endl;
    if(r){//if we could read the file continue
        for(int j=0;j<pts.size();j++){
            y.push_back(Eigen::Vector3d(0,0,0));//prime the lap vector
        }
        for(int i =0;i<n;i++){//loop n times
        	laplacian(triangles,pts,y);//call laplacian on triangles and pts
             for(int j=0;j<pts.size();j++){
                pts[j]+=atof(argv[3])*y[j];//pts[j]+=stepsize*lap
             }
        }
        //std::cout<<"Writing"<<std::endl;
        writeObjFile(argv[2],pts,triangles);//write to file
    }
    else std::cout<<"Unable to read file"<<std::endl;
}