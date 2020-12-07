#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

class Boid{
    public:
    Boid(const Eigen::Vector3d &_pos,const Eigen::Vector3d &_vel):pos(_pos),vel(_vel){};
    ~Boid(){};
    Eigen::Vector3d pos,vel;
    double mass,radius,maxNeighbors;
};
class Animation{
    public:
        bool read(const std::string &fname);
        void write(char *fname);
        double numBoids;
        std::vector<Boid> fish;
};
bool Animation::read(const std::string &fname){
    std::ifstream in(fname.c_str(), std::ios_base::in);
	std::string line;
    double size,radius,numNeighbors,mass,collision,centering,velocity,hunger,dt,length;
    std::string junk;
    getline(in,line);
    std::stringstream first(line);

    first>>size>>radius>>numNeighbors>>mass>>collision>>centering>>velocity>>hunger>>dt>>length;
    getline(in,line);
    std::stringstream nf(line);
    nf>>numBoids;
    int i=0;
    while(in){
        getline(in,line);
        if(i<numBoids){
            std::stringstream ss(line);
            Eigen::Vector3d pos,vel;
            ss>>junk>>pos[0]>>junk>>pos[1]>>junk>>pos[2]>>junk>>junk>>vel[0]>>junk>>vel[1]>>junk>>vel[2]>>junk;
            Boid b(pos,vel);
            fish.push_back(b);
            i++;
        }
    }
    return true;
}
void Animation::write(char *fname){
    for(int i=0;i<fish.size();i++){
        std::cout<<i<<", "<<fish[i].pos<<std::endl;
    }
}
int main(int argc,char *argv[]){
    if(argc!=3){
        std::cout<<"usage fishtank sample.in sample.out"<<std::endl;
        exit(1);
    }
    Animation anim;
    bool r=anim.read(argv[1]);
    if(r){
        anim.write(argv[2]);
    }
    else{
        std::cout<<"couldn't read file "<<argv[1]<<std::endl;
    }
    return 0;
}