#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

struct hitRecord{
    Eigen::Vector3d color;
    int t;
    bool inside;
};
struct ray{
    Eigen::Vector3d from;
    Eigen::Vector3d dir;
};
class Geo{
   public:
        virtual hitRecord intersect(ray r)=0;
};
class Triangle: public Geo{
    public:
        Triangle(Eigen::Vector3d vert[3],Eigen::Vector3d color){
            this->vert[0]=vert[0];
            this->vert[1]=vert[1];
            this->vert[2]=vert[2];
            this->color=color;
        }
        hitRecord intersect(ray r){
            hitRecord hr;
            Eigen::Matrix3d m;
            // m<<(vert[1]-vert[0]),(vert[2]-vert[0]),(-r.dir);
            m.col(0)=vert[1]-vert[0];
            m.col(1)=vert[2]-vert[0];
            m.col(2)=-r.dir;
            Eigen::Vector3d sol=m.colPivHouseholderQr().solve(r.from-vert[0]);
            //std::cout<<m<<std::endl;
            hr.t=sol[2];
            if(sol[0]>0&&sol[1]<1&&(sol[0]+sol[1]<1)){
                hr.color=color;
                hr.inside=true;
                // std::cout<<"Hit!"<<hr.t<<std::endl;
            }
            else{
                hr.inside=false;
            }
            return hr;
        }
    private:
        Eigen::Vector3d vert[3];
        Eigen::Vector3d color;
};

std::vector<Geo*> polygons;
Eigen::Vector3d background;
Eigen::Vector3d fill;
Eigen::Vector3d from;
Eigen::Vector3d at;
Eigen::Vector3d up;
Eigen::Vector3d w,u,v;
double imgHalfWidth,pixelWidth,aspect,angle,hither;
int resx,resy;
bool nextTri,nextSphere,nextPoly,nextView;

void readInput(std::string in){
    int numVert;
    std::vector<Eigen::Vector3d> vertexs;
    std::ifstream fil(in);
    std::string line,junk;
    while(!fil.eof()){
        std::getline(fil,line);
        // std::cout<<line<<std::endl;
        std::stringstream str(line);
        if(line[0]=='b'){
            str>>junk>>background[0]>>background[1]>>background[2];
        }
        else if(line[0]=='v'){
            nextView=true;
        }
        else if(nextView){
            std::getline(str,junk,' ');
            if(junk=="from"){
                str>>from[0]>>from[1]>>from[2];
            }
            else if(junk=="at"){
                str>>at[0]>>at[1]>>at[2];
            }
            else if(junk=="up"){
                str>>up[0]>>up[1]>>up[2];
            }
            else if(junk=="angle"){
                str>>angle;
            }
            else if(junk=="hither"){
                str>>hither;
            }
            else if(junk=="resolution"){
                str>>resx>>resy;
                nextView=false;
            }
        }
        else if(line[0]=='f'){
            str>>junk>>fill[0]>>fill[1]>>fill[2];
        }
        else if(line[0]=='p'){
            nextTri=false;
            nextPoly=false;
            str>>junk>>numVert;
            vertexs.clear();
            if(numVert>=3){
                nextTri=true;
            }
        }
        else if(nextTri){
            Eigen::Vector3d vert;
            str>>vert[0]>>vert[1]>>vert[2];
            vertexs.push_back(vert);
        }
        if((nextTri||nextPoly)&&vertexs.size()==numVert){
            if(nextTri){
                for(int i =0;i<numVert-2;i++){
                    Eigen::Vector3d triVert[3];
                    triVert[0]=vertexs.at(0);
                    triVert[1]=vertexs.at(i+1);
                    triVert[2]=vertexs.at(i+2);
                    polygons.push_back(new Triangle(triVert,fill));
                }
            }
            if(nextPoly){
                std::cout<<"Hasn't been implemented yet"<<std::endl;
            }
        }
    }
    fil.close();
}
ray calcRay(int i,int j){
    double m=(-imgHalfWidth+pixelWidth/2.0)+j*pixelWidth;
    double n=-(imgHalfWidth+pixelWidth/2.0)/aspect+i*pixelWidth;
    double o=1;
    ray r;
    r.dir=m*u+n*v+hither*w;
    r.from=from;
    return r;
}
Eigen::Vector3d trace(ray r){
    int min_t=INT32_MAX;
    Eigen::Vector3d color=background;
    for(int i=0;i<polygons.size();i++){
        hitRecord hr=polygons.at(i)->intersect(r);
        if(hr.t<min_t&&hr.t>0&&hr.inside){
            min_t=hr.t;
            color=hr.color;
            // std::cout<<polygons.size()<<std::endl;
        }
    }
    return color;
}

int main(int argc, char* argv[]){
    
    if(argc<2){
        std::cout<<"No input file detected"<<std::endl;
        exit(1);
    }
    if (argc<3)
    {
        std::cout<<"No output file detected, using default output file: output.ppm"<<std::endl;
    }
    readInput(argv[1]);
    unsigned char pixels[resy][resx][3];
    Eigen::Vector3d color;
    w=(from-at).normalized();
    u=up.cross(w).normalized();
    v=w.cross(u);
    imgHalfWidth=tan(M_PI*angle/180.0/2.0)*hither;
    pixelWidth=(imgHalfWidth*2)/resx*2.0;
    aspect=resx/resy;
    for(int i=0;i<resy;i++){
        for(int j=0;j<resx;j++){
            ray r=calcRay(i,j);
            color=trace(r);
            pixels[i][j][0]=color[0]*255;
            pixels[i][j][1]=color[1]*255;
            pixels[i][j][2]=color[2]*255;
        }
    }
    FILE *f = fopen("output.ppm","wb");
    fprintf(f, "P6\n%d %d\n%d\n", resx, resy, 255);
    fwrite(pixels, 1, resx*resy*3, f);
    fclose(f);
    while(!polygons.empty()){
        Geo* temp;
        temp=polygons.back();
        delete temp;
        polygons.pop_back();
    }
    return 0;
}