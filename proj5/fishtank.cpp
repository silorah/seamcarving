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
    Eigen::Vector3d netForce;//the net force acting on the boid
};
class Animation{
    public:
        bool read(const std::string &fname);
        void write(char *fname);
        void animate();
        //constraints of the animation
        double size,radius,numNeighbors,mass,collision,centering,velocity,hunger,damping,dt,length,xbound,ybound,zbound;
        double numBoids;
        //frames holds each frame which is a vector of boids
        std::vector<std::vector<Boid>> frames;
        //holds all the boids in the fishtank
        std::vector<Boid> fish;
};
/**
 * Animates the scene, one frame at a time
 */
void Animation::animate(){
    int frame=0,n=0;//n is number of neighbors considered
    double time=0.0;//time elapsed in the animation
    double frameTime=-1.0;//time since last frame
    double distx,disty,distz,m;//distance from current boid and the neighbor void, m is used when calculating forces
    Eigen::Vector3d avgPos=Eigen::Vector3d(0,0,0),avgVel=Eigen::Vector3d(0,0,0),colForce=Eigen::Vector3d(0,0,0);//used to calc forces
    while(time<length){//while there is still time in the animation
        if(frameTime<0){//if we're ready for a new frame
            frames.push_back(fish);//add the current state of the fish to frames
            frameTime=1/30.00;//set frame time = 1/frame rate
            frame++;
        }
        for(int i=0;i<numBoids;i++){//calculates forces acting on each fish
            fish[i].netForce=Eigen::Vector3d(0,0,0),avgPos=Eigen::Vector3d(0,0,0),avgVel=Eigen::Vector3d(0,0,0),colForce=Eigen::Vector3d(0,0,0);
            n=0;//num neighbors
            for(int j=0;j<numBoids;j++){//loop through the fish to find possible neighbors
                distx=fabs(fish[j].pos[0]-fish[i].pos[0]);
                disty=fabs(fish[j].pos[1]-fish[i].pos[1]);
                distz=fabs(fish[j].pos[2]-fish[i].pos[2]);
                //if we haven't reached maximum neighbors for this fish and the fish being considered is within the view radius
                if(j!=i&&n<numNeighbors&&(distx<=radius && disty<=radius && distz<=radius )){
                    colForce=fish[i].pos-fish[j].pos;//set the direction of the avoidance force to be away from the neighbor
                    m=colForce.norm();
                    if(m>.0001){//if m > epsilon
                        fish[i].netForce+=(collision*colForce)/(m*m*m);//calculate the collision avoidance force and divide it by it's norm^3 to scale it
                    }
                    //increment average neighbor position and velocity, and n
                    avgPos+=fish[j].pos;
                    avgVel+=fish[j].vel;
                    n++;
                }
            }
            if(n>0){//if there is more than one neighbor for the current fish
                avgPos/=n;//truly make it an average by dividing by number of neighbors
                avgVel/=n;
                m=(avgPos-fish[i].pos).norm();//get the norm of the average position vector
                if(m>.0001){
                    fish[i].netForce+=(centering*(avgPos-fish[i].pos))/m;//calculate the centering force and make it unit magnitude
                }
                fish[i].netForce+=velocity*(avgVel-fish[i].vel);//calculate the velocity matching force
            }
        }
        for(int i=0;i<numBoids;i++){//for each fish/boid
            fish[i].vel+=dt*(fish[i].netForce/fish[i].mass);//dv=dt*a
            fish[i].vel*=damping;//apply velocity damping
            fish[i].pos+=fish[i].vel*dt;//dp=v*dt
            //make sure the fish is inside the tank
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
        }
        time+=dt;//increment time by the time step
        frameTime-=dt;//decrement frame time by the time step
    }
}
/**
 * Reads the scenario data from a file
 * @param fname the file to be read from
 */
bool Animation::read(const std::string &fname){
    std::ifstream in(fname, std::ios_base::in);
	std::string line;
    std::string junk;
    double x,y,z;
    if(!in.is_open()){
        return false;
    }
    //fill in the animation constraints
    in>>size>>radius>>numNeighbors>>mass>>collision>>centering>>velocity>>hunger>>damping>>dt>>length;
    // std::cout<<size<<" "<<radius<<" "<<numNeighbors<<" "<<mass<<" "<<collision<<" "<<centering<<" "<<velocity<<" "<<hunger<<" "<<damping<<" "<<dt<<" "<<length<<std::endl;
    in>>numBoids;//read in num boids
    int i=0;
    getline(in,line);//get the newline char after numBoids in the file
   while(in){
        getline(in,line);
        if(i<numBoids){
            std::stringstream ss(line);
            Eigen::Vector3d pos=Eigen::Vector3d(0.0,0.0,0.0),vel=Eigen::Vector3d(0.0,0.0,0.0);//position and velocity vectors for the boid
            getline(ss,junk,',');//get xpos
            junk.erase(0,1);//remove the '[' from [xpos
            pos[0]=atof(junk.c_str());//store xpos in pos[0]
            getline(ss,junk,',');//get ypos
            pos[1]=atof(junk.c_str());//store ypos in pos[1]
            getline(ss,junk,']');//get zpos
            pos[2]=atof(junk.c_str());//store zpos in pos[2]
            getline(ss,junk,',');//get xvel
            junk.erase(0,1);//remove the '[' from [xvel
            vel[0]=atof(junk.c_str());//store xvel in vel[0]
            getline(ss,junk,',');//get yvel
            vel[1]=atof(junk.c_str());//store yvel in vel[1]
            getline(ss,junk,']');//get zvel 
            vel[2]=atof(junk.c_str());// store zvel in vel[2]
            // std::cout<<"i: "<<i<<"\tpos: ["<<pos[0]<<","<<pos[1]<<","<<pos[2]<<"]"<<std::endl;
            fish.push_back(Boid(pos,vel,mass));//add the fish to the fish tank
            i++;//next fish/boid
       }
    }
    xbound=.5;//xbound of tank
    ybound=.25;//ybound of tank
    zbound=.125;//zbound of tank
    return true;
}
/**
 * Writes the frames of the animation to a file
 * @param fname the file to be written to
 */
void Animation::write(char *fname){
    std::ofstream out;
    out.open(fname);
    out<<30*length<<std::endl;//write the number of frames to the file, frame rate is 30fps
    for(int i=0;i<frames.size();i++){
        out<<numBoids<<std::endl;//write number of boids/fish to the file
        for(int j=0;j<numBoids;j++){//for each boid
            //write a line to [xpos,ypos,zpos] [xvel,yvel,zvel] to the file
            out<<"["<<frames[i][j].pos[0]<<","<<frames[i][j].pos[1]<<","<<frames[i][j].pos[2]<<"] ";
            out<<"["<<frames[i][j].vel[0]<<","<<frames[i][j].vel[1]<<","<<frames[i][j].vel[2]<<"]"<<std::endl;
        }
        out<<0<<std::endl;//since we're currently ignoring food we write that their are 0 frames of food to the file
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