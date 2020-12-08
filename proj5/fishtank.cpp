#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

class Boid{
    public:
    Boid(){};
    Boid(const Eigen::Vector3d &_pos,const Eigen::Vector3d &_vel):pos(_pos),vel(_vel){};
    ~Boid(){};
    Eigen::Vector3d pos,vel,colF,centF,velF;//position,velocity,collision avoidance force, centering force,
};
class Animation{
    public:
        bool read(const std::string &fname);
        void write(char *fname);
        void animate();
        double size,radius,numNeighbors,mass,collision,centering,velocity,hunger,damping,dt,length,xbound,ybound,zbound;
        double numBoids;
        std::vector<std::vector<Boid>> frames;
        std::vector<Boid> fish;
};
void Animation::animate(){
    int frame=0,n=0;//n is number of neighbors considered
    double time=0.0;
    double frameTime=-1.0;
    double distx,disty,distz,colRadius=2;
    std::ofstream out;
    out.open("foo.text");
    for(int i=0;i<numBoids;i++){
        out<<"["<<fish[i].pos[0]<<","<<fish[i].pos[1]<<","<<fish[i].pos[2]<<"] ";
        out<<"["<<fish[i].vel[0]<<","<<fish[i].vel[1]<<","<<fish[i].vel[2]<<"]"<<std::endl;
    }
    out.close();
    Eigen::Vector3d colF=Eigen::Vector3d(0,0,0),centF=Eigen::Vector3d(0,0,0),velF=Eigen::Vector3d(0,0,0);
    Boid curr,neigh;
    while(time<length){
        if(frameTime<0){
            frames.push_back(fish);
            frameTime=1/30.00;
            frame++;
        }
        for(int i=0;i<numBoids;i++){
            colF=Eigen::Vector3d(0,0,0),centF=Eigen::Vector3d(0,0,0),velF=Eigen::Vector3d(0,0,0);
            n=0;
            for(int j=0;j<numBoids;j++){
                distx=fabs(fish[j].pos[0]-fish[i].pos[0]);
                disty=fabs(fish[j].pos[1]-fish[i].pos[1]);
                distz=fabs(fish[j].pos[2]-fish[i].pos[2]);
                if(j!=i&&n<numNeighbors&&(distx<=radius && disty<=radius && distz<=radius )){
                    centF+=fish[j].pos;
                    velF+=fish[j].vel;
                    colF-=fish[j].pos-fish[i].pos;
                    n++;
                }
            }
            centF/=n;
            colF/=n;
            velF/=n;
            fish[i].centF=(centF-fish[i].pos)/dt;
            fish[i].velF=(velF-fish[i].vel)/dt;
            fish[i].colF=(colF-fish[i].pos)/dt;
        }
        for(int i=0;i<numBoids;i++){
            fish[i].pos[0]=fish[i].pos[0]+velF[0]*dt;
            fish[i].pos[1]=fish[i].pos[1]+velF[1]*dt;
            fish[i].pos[2]=fish[i].pos[2]+velF[2]*dt;
            fish[i].vel[0]=(fish[i].vel[0])+(fish[i].velF[0]*velocity)+(fish[i].colF[0]*collision)+(fish[i].centF[0]*centering);
            fish[i].vel[1]=(fish[i].vel[1])+(fish[i].velF[1]*velocity)+(fish[i].colF[1]*collision)+(fish[i].centF[1]*centering);
            fish[i].vel[2]=(fish[i].vel[2])+(fish[i].velF[2]*velocity)+(fish[i].colF[2]*collision)+(fish[i].centF[2]*centering);
            if(fabs(fish[i].pos[0])>=xbound){
                fish[i].vel[0]*=-1;
            }
            if(fabs(fish[i].pos[1])>=ybound){
                fish[i].vel[1]*=-1;
            }
            if(fabs(fish[i].pos[2])>=zbound){
                fish[i].vel[2]*=-1;
            }
            fish[i].vel*=damping;
        }
        time+=dt;
        frameTime-=dt;
    }
}
bool Animation::read(const std::string &fname){
    std::ifstream in(fname.c_str(), std::ios_base::in);
	std::string line;
    std::string junk;
    getline(in,line);
    std::stringstream first(line);

    first>>size>>radius>>numNeighbors>>mass>>collision>>centering>>velocity>>hunger>>damping>>dt>>length;
    getline(in,line);
    std::stringstream nf(line);
    nf>>numBoids;
    int i=0;
    while(in){
        getline(in,line);
        if(i<numBoids){
            std::stringstream ss(line);
            // std::cout<<line<<std::endl;
            Eigen::Vector3d pos=Eigen::Vector3d(0.0,0.0,0.0),vel=Eigen::Vector3d(0.0,0.0,0.0);
            ss>>junk>>pos[0]>>junk>>pos[1]>>junk>>pos[2]>>junk>>junk>>vel[0]>>junk>>vel[1]>>junk>>vel[2]>>junk;
            // std::cout<<"["<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"junk:"<<junk<<std::endl;
            fish.push_back(Boid(pos,vel));
            i++;
        }
    }
    xbound=.5;
    ybound=.25;
    zbound=.125;
    return true;
}
void Animation::write(char *fname){
    std::ofstream out;
    out.open(fname);
    out<<30*length<<std::endl;
    for(int i=0;i<frames.size();i++){
        out<<numBoids<<std::endl;
        for(int j=0;j<numBoids;j++){
            out<<"["<<frames[i][j].pos[0]<<","<<frames[i][j].pos[1]<<","<<frames[i][j].pos[2]<<"] ";
            out<<"["<<frames[i][j].vel[0]<<","<<frames[i][j].vel[1]<<","<<frames[i][j].vel[2]<<"]"<<std::endl;
        }
        out<<0<<std::endl;
    }
    out.close();
}
int main(int argc,char *argv[]){
    if(argc!=3){
        std::cout<<"usage fishtank sample.in sample.out"<<std::endl;
        exit(1);
    }
    Animation anim;
    bool r=anim.read(argv[1]);
    if(r){
        anim.animate();
        anim.write(argv[2]);
    }
    else{
        std::cout<<"couldn't read file "<<argv[1]<<std::endl;
    }
    return 0;
}