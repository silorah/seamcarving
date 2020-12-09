#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

class Boid{
    public:
    Boid(){};
    Boid(const Eigen::Vector3d &_pos,const Eigen::Vector3d &_vel,double _mass):pos(_pos),vel(_vel),mass(_mass){};
    ~Boid(){};
    double mass;
    Eigen::Vector3d pos,vel;//position,velocity
    Eigen::Vector3d netForce;//the net force acting on the boid calculated by mass*( collision*(colF/dt)+centering*(centF/dt)+velocity*(velF/dt)+vel/dt)
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
    double distx,disty,distz,m;
    // std::ofstream out;
    // out.open("foo.text");
    // for(int i=0;i<numBoids;i++){
    //     out<<"["<<fish[i].pos[0]<<","<<fish[i].pos[1]<<","<<fish[i].pos[2]<<"] ";
    //     out<<"["<<fish[i].vel[0]<<","<<fish[i].vel[1]<<","<<fish[i].vel[2]<<"]"<<std::endl;
    // }
    // out.close();
    Eigen::Vector3d avgPos=Eigen::Vector3d(0,0,0),avgVel=Eigen::Vector3d(0,0,0),colForce=Eigen::Vector3d(0,0,0);
    Boid curr,neigh;
    while(time<length){
        if(frameTime<0){
            frames.push_back(fish);
            frameTime=1/30.00;
            frame++;
        }
        for(int i=0;i<numBoids;i++){
            fish[i].netForce=Eigen::Vector3d(0,0,0),avgPos=Eigen::Vector3d(0,0,0),avgVel=Eigen::Vector3d(0,0,0),colForce=Eigen::Vector3d(0,0,0);
            n=0;
            for(int j=0;j<numBoids;j++){
                distx=fabs(fish[j].pos[0]-fish[i].pos[0]);
                disty=fabs(fish[j].pos[1]-fish[i].pos[1]);
                distz=fabs(fish[j].pos[2]-fish[i].pos[2]);
                if(j!=i&&n<numNeighbors&&(distx<=radius && disty<=radius && distz<=radius )){
                    // if(distx>0&&disty>0&&distz>0){
                    //     // centF[0]+=fish[j].pos[0]/(distx*distx);
                    //     // centF[1]+=fish[j].pos[1]/(disty*disty);
                    //     // centF[2]+=fish[j].pos[2]/(distz*distz);
                    //     // colF[0]-=((fish[j].pos[0]-fish[i].pos[0])/(distx*distx));
                    //     // colF[1]-=((fish[j].pos[1]-fish[i].pos[1])/(disty*disty));
                    //     // colF[2]-=((fish[j].pos[2]-fish[i].pos[2])/(distz*distz));
                    //     centF[0]+=(fish[j].pos[0]-fish[i].pos[0]);
                    //     centF[1]+=(fish[j].pos[0]-fish[i].pos[1]);
                    //     centF[2]+=(fish[j].pos[0]-fish[i].pos[2]);
                    //     colF[0]-=(fish[j].pos[0]-fish[i].pos[0]);
                    //     colF[1]-=(fish[j].pos[1]-fish[i].pos[1]);
                    //     colF[2]-=(fish[j].pos[2]-fish[i].pos[2]);
                    // }
                    // else{                        
                    //     centF[0]+=(fish[j].pos[0]-fish[i].pos[0]);
                    //     centF[1]+=(fish[j].pos[0]-fish[i].pos[1]);
                    //     centF[2]+=(fish[j].pos[0]-fish[i].pos[2]);
                    //     colF[0]-=(fish[j].pos[0]-fish[i].pos[0]);
                    //     colF[1]-=(fish[j].pos[1]-fish[i].pos[1]);
                    //     colF[2]-=(fish[j].pos[2]-fish[i].pos[2]);
                    
                    // }
                    colForce=fish[i].pos-fish[j].pos;
                    m=colForce.norm();
                    if(m>.0001){
                        fish[i].netForce+=(collision*colForce)/(m*m*m);
                    }
                    avgPos+=fish[j].pos;
                    avgVel+=fish[j].vel;
                // fish[i].colF[0]*=sqrt(fabs(colF[0]-fish[i].pos[0]));
                // fish[i].colF[1]*=sqrt(fabs(colF[1]-fish[i].pos[1]));
                // fish[i].colF[2]*=sqrt(fabs(colF[2]-fish[i].pos[2]));
                    n++;
                }
            }
            if(n>0){
                // centF=(centF-fish[i].pos)/dt;
                // velF=(velF-fish[i].vel);
                // colF=(colF-fish[i].pos)/dt;
                // centF=fish[i].mass*(centF/dt);
                // colF=fish[i].mass*(colF/dt);
                // velF=fish[i].mass*(velF/dt);
                // centF/=n;
                // colF/=n;
                // velF/=n;
                // centF/=centF.norm();
                avgPos/=n;
                avgVel/=n;
                m=(avgPos-fish[i].pos).norm();
                if(m>.0001){
                    fish[i].netForce+=(centering*(avgPos-fish[i].pos))/m;
                }
                fish[i].netForce+=velocity*(avgVel-fish[i].vel);
            }
            else{
                // fish[i].colF=Eigen::Vector3d(0.0,0.0,0.0);
                // fish[i].centF=Eigen::Vector3d(0.0,0.0,0.0);
                // fish[i].velF=Eigen::Vector3d(0.0,0.0,0.0);
            }
            // fish[i].netForce=centering*avgPos+velocity*avgVel;
        }
        for(int i=0;i<numBoids;i++){
            fish[i].vel+=dt*(fish[i].netForce/fish[i].mass);
            fish[i].vel*=damping;
            fish[i].pos+=fish[i].vel*dt;
            if(fish[i].pos[0]<=-xbound){
                fish[i].pos[0]=-xbound;
                fish[i].vel[0]=fabs(fish[i].vel[0]);
            }
            if(fish[i].pos[0]>=xbound){
                fish[i].pos[0]=xbound;
                fish[i].vel[0]=-fabs(fish[i].vel[0]);
            }
            if(fish[i].pos[1]<=-ybound){
                fish[i].pos[1]=-ybound;
                fish[i].vel[1]=fabs(fish[i].vel[1]);
            }
            if(fish[i].pos[1]>=ybound){
                fish[i].pos[1]=ybound;
                fish[i].vel[1]=-fabs(fish[i].vel[1]);
            }
            if(fish[i].pos[2]<=-zbound){
                fish[i].pos[2]=-zbound;
                fish[i].vel[2]=fabs(fish[i].vel[2]);
            }
            if(fish[i].pos[2]>=zbound){
                fish[i].pos[2]=zbound;
                fish[i].vel[2]=-fabs(fish[i].vel[2]);
            }
            // if(fabs(fish[i].pos[0])>=xbound){
            //     fish[i].vel[0]*=-1;
            // }
            // if(fabs(fish[i].pos[1])>=ybound){
            //     fish[i].vel[1]*=-1;
            // }
            // if(fabs(fish[i].pos[2])>=zbound){
            //     fish[i].vel[2]*=-1;
            // }
            // if(fabs(fish[i].pos[0])>=xbound||fabs(fish[i].pos[1])>=ybound||fabs(fish[i].pos[2])>=zbound){
            //     fish[i].vel*=-1;
            // }
            // fish[i].pos[0]=fish[i].pos[0]+fish[i].vel[0]*dt+(((fish[i].netForce[0]/mass)*(dt*dt))/2);
            // fish[i].pos[0]=fish[i].pos[1]+fish[i].vel[1]*dt+(((fish[i].netForce[1]/mass)*(dt*dt))/2);
            // fish[i].pos[0]=fish[i].pos[2]+fish[i].vel[2]*dt+(((fish[i].netForce[2]/mass)*(dt*dt))/2);
            // // fish[i].vel[0]=(fish[i].vel[0])+(fish[i].velF[0]*velocity)+(fish[i].colF[0]*collision)+(fish[i].centF[0]*centering);
            // // fish[i].vel[1]=(fish[i].vel[1])+(fish[i].velF[1]*velocity)+(fish[i].colF[1]*collision)+(fish[i].centF[1]*centering);
            // // fish[i].vel[2]=(fish[i].vel[2])+(fish[i].velF[2]*velocity)+(fish[i].colF[2]*collision)+(fish[i].centF[2]*centering);
            // fish[i].vel-=((fish[i].netForce/fish[i].mass)*dt);
            // fish[i].vel*=damping;
        }
        time+=dt;
        frameTime-=dt;
    }
}
bool Animation::read(const std::string &fname){
    std::ifstream in(fname, std::ios_base::in);
	std::string line;
    std::string junk;
    double x,y,z;
    in>>size>>radius>>numNeighbors>>mass>>collision>>centering>>velocity>>hunger>>damping>>dt>>length;
    std::cout<<size<<" "<<radius<<" "<<numNeighbors<<" "<<mass<<" "<<collision<<" "<<centering<<" "<<velocity<<" "<<hunger<<" "<<damping<<" "<<dt<<" "<<length<<std::endl;
    in>>numBoids;
    // numBoids=0;
    int i=0;
    getline(in,line);
   while(in){
        getline(in,line);
        if(i<numBoids){
            std::stringstream ss(line);
            Eigen::Vector3d pos=Eigen::Vector3d(0.0,0.0,0.0),vel=Eigen::Vector3d(0.0,0.0,0.0);
            getline(ss,junk,',');
            junk.erase(0,1);
            pos[0]=atof(junk.c_str());
            getline(ss,junk,',');
            pos[1]=atof(junk.c_str());
            getline(ss,junk,']');
            pos[2]=atof(junk.c_str());
            getline(ss,junk,',');
            junk.erase(0,1);
            vel[0]=atof(junk.c_str());
            getline(ss,junk,',');
            vel[1]=atof(junk.c_str());
            getline(ss,junk,']');
            vel[2]=atof(junk.c_str());
            std::cout<<"i: "<<i<<"\tpos: ["<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"]"<<std::endl;
            fish.push_back(Boid(pos,vel,mass));
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