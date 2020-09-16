#include <Eigen/Dense>
struct hitRecord{
    Eigen::Vector3d color;
    int t;
};
struct ray{
    Eigen::Vector3d from;
    Eigen::Vector3d dir;
};
class Geo{
   public:
        virtual hitRecord intersect()=0;
};
class Triangle:Geo{
    public:
        Triangle(Eigen::Vector3d vert[3]){
            this->vert[0]=vert[0];
            this->vert[1]=vert[1];
            this->vert[2]=vert[2];
        }
        hitRecord intersect(ray r){
            hitRecord hr;
            Eigen::Matrix3d m;
            m<<vert[1]-vert[0],vert[2]-vert[0],-r.dir;
            Eigen::Vector3d sol=m.colPivHouseholderQr().solve(r.from-vert[0]);
            hr.t=sol[2];
            if(sol[0]>0&&sol[1]<1&&(sol[0]+sol[1]<1)&&hr.t>0){
                hr.color=color;
            }
            return hr;
        }
    private:
        Eigen::Vector3d vert[3];
        Eigen::Vector3d color;
};
int main(int argc, char* argv[]){
    return 0;
}