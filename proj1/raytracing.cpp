#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

struct hitRecord{
    Eigen::Vector3d color;
    double t;
    bool inside;
};
struct ray{
    Eigen::Vector3d from;
    Eigen::Vector3d dir;
};
class Geo{
   public:
        //Should check if the ray r intersects the geo
        virtual hitRecord intersect(ray r)=0;
};
//A triangle class based off Geo that implements an intersect function
class Triangle: public Geo{
    public:
        /**
         *Creates a triangle with the vertices vert and the color color 
         *@param vert a size 3 array of Eigen::Vector3ds each specifying a vertex
         *@param color an Eigen::Vector3d specifying the color of the Triangle
         */
        Triangle(Eigen::Vector3d vert[3],Eigen::Vector3d color){
            this->vert[0]=vert[0];
            this->vert[1]=vert[1];
            this->vert[2]=vert[2];
            this->color=color;
        }
        /**
         * Checks to see if the ray r intersects with the triangle
         * @param r the ray that is being tested
         * @return hitRecord the hitRecord of r hitting the triangle
         */
        hitRecord intersect(ray r){
            hitRecord hr;
            Eigen::Matrix3d m;//A matrix that can be solved to find beta gamma and t
            m<<(vert[1]-vert[0]),(vert[2]-vert[0]),((r.dir));//the columns of the matrix from the equation from-v0=beta(v1-v0)+gamma(v2-v0)+td
            Eigen::Vector3d sol=m.colPivHouseholderQr().solve((r.from-vert[0]));//solves the matrix and stores beta,gamma,and t into sol
            hr.t=sol[2];
            if(sol[0]>0&&sol[0]<1&&sol[1]>0&&sol[1]<1&&(sol[0]+sol[1]<1)){//if beta>0 &&beta<1 && gamma>0 &&beta<1 &&beta+gamma<1
                hr.color=color;//sets the color of the hitrecord to the color of the triangle
                hr.inside=true;//the ray ended inside the triangle
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

class Polygon :public Geo{
    public:
        Polygon(std::vector<Eigen::Vector3d> verts,Eigen::Vector3d color){
            this->verts=verts;
            this->color=color;
        }
        hitRecord intersect(ray r){
            hitRecord hr;
            // for(int i =0; i<verts.size()-1;i++){
        
            // }
            return hr;
        }
    private:
        std::vector<Eigen::Vector3d> verts;
        Eigen::Vector3d color;
};

std::vector<Geo*> polygons;//vector of polygons in the image
Eigen::Vector3d background;//background color
Eigen::Vector3d fill;//fill color
Eigen::Vector3d from;
Eigen::Vector3d at;
Eigen::Vector3d up;
Eigen::Vector3d w,u,v;
double imgHalfWidth,pixelWidth,aspect,angle,hither;
int resx,resy;
bool nextTri,nextSphere,nextPoly,nextView;//signals readInput that a multi line section is being read in
/**
 * Reads in the input from the specified .nff file
 * @param in the .nff file to be read
 */
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
            if(numVert==3){
                nextTri=true;
            }
            if(numVert>3){
                nextPoly=true;
            }
        }
        else if(nextTri){
            Eigen::Vector3d vert;
            str>>vert[0]>>vert[1]>>vert[2];
            vertexs.push_back(vert);
        }
        else if(nextPoly){
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
                // std::cout<<"Hasn't been implemented yet"<<std::endl;
                polygons.push_back(new Polygon(vertexs,fill));
            }
        }
    }
    fil.close();
}
/**
 * Calculates a ray for a pixel(j,i)
 * @param i the y value of the pixel
 * @param j the x value of the pixel
 * @return r the ray that was calculated
 */
ray calcRay(int i,int j){
    double m=(-imgHalfWidth+pixelWidth/2.0)+j*pixelWidth;
    double n=-(imgHalfWidth+pixelWidth/2.0)/aspect+i*pixelWidth;
    ray r;
    r.dir=m*u+n*v+hither*w;
    r.from=from;
    return r;
}
/**
 * Traces a ray and returns the appropriate color
 * @param r the ray to be traced
 * @return color an Eigen::Vector3d that corresponds to the color of the pixel
 */
Eigen::Vector3d trace(ray r){
    double min_t=INFINITY;
    Eigen::Vector3d color=background;
    for(int i=0;i<polygons.size();i++){
        hitRecord hr=polygons.at(i)->intersect(r);
        if(hr.t<min_t&&hr.t>0&&hr.inside){
            min_t=hr.t;
            color=hr.color;
            // std::cout<<"INSIDE!"<<min_t<<std::endl;
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
    std::string outfile="output.ppm";//default output file
    if(argc==3){
        outfile=argv[2];//changes output file if one is specifide
    }
    char out[outfile.size()+1];
    std::strcpy(out,outfile.c_str());
    readInput(argv[1]);
    unsigned char pixels[resy][resx][3];
    Eigen::Vector3d color;
    w=(from-at).normalized();
    u=up.cross(w).normalized();
    v=w.cross(u);
    imgHalfWidth=tan(M_PI*angle/180.0/2.0)*hither;
    pixelWidth=(imgHalfWidth*2)/double(resx);
    aspect=double(resx)/double(resy);
    //for every pixel it calculates a ray and then traces it assigning the color to the appropriate pixels rgb channels
    for(int i=0;i<resy;i++){
        for(int j=0;j<resx;j++){
            ray r=calcRay(i,j);
            color=trace(r);
            pixels[i][j][0]=color[0]*255;
            pixels[i][j][1]=color[1]*255;
            pixels[i][j][2]=color[2]*255;
        }
    }
    //Not quite sure why but tetra-3 was comming out flipped so this flips it back
    unsigned char pixelsFlipped[resy][resx][3];
     for(int i=0;i<resy;i++){
        for(int j=resx-1;j>=0;j--){
            pixelsFlipped[i][j][0]=pixels[i][resx-(j+1)][0];
            pixelsFlipped[i][j][1]=pixels[i][resx-(j+1)][1];
            pixelsFlipped[i][j][2]=pixels[i][resx-(j+1)][2];
        }
    }
    //Writes the pixels to the specified ppm file
    FILE *f = fopen(out,"wb");
    fprintf(f, "P6\n%d %d\n%d\n", resx, resy, 255);
    fwrite(pixelsFlipped, 1, resx*resy*3, f);
    fclose(f);
    //goes through and deallocates memory for the polygons
    while(!polygons.empty()){
        Geo* temp;
        temp=polygons.back();
        delete temp;
        polygons.pop_back();
    }
    return 0;
}