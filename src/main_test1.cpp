#include "reedshepp.h"
#include "planner.h"
#define PI 3.1415926
using namespace std;
Smoother smoother;
Node3D* getNodes(vector<Node3D>& vec){
    int num_vec =vec.size();
    Node3D* nPred;
    Node3D* nSucc;
    Node3D tmp();
    Node3D* newNodes = new Node3D [num_vec];
    for(int i = 0; i<num_vec; i++){
        if(i == 0){
            Node3D tt(vec[i].getX(),vec[i].getY(),vec[i].getT(),0,0,nullptr);
            newNodes[i] = tt;
            nPred = &newNodes[i];
        }else{
            Node3D tt(vec[i].getX(),vec[i].getY(),vec[i].getT(),0,0,nPred);
            newNodes[i] = tt;
            nPred = &newNodes[i];
        }
        
        cout << "nPred in get nodes: " << nPred<<endl;
        cout<<"get nodes list size:  " << i+1 <<endl;
        
    }
    cout << "newNodes[0] Pred: " << newNodes[0].getPred()<<endl;
    smoother.tracePath(&newNodes[num_vec-1]);
    cout << "smoother.tracePath(&newNodes[num_vec-1]); : check" <<endl;
    

    return &newNodes[num_vec-1];
}
int main(int argc, char** argv){
    ros::init(argc, argv, "a_star");
    double q0[3] = {40,40,0};
    double q1[3] = {40,30,3*PI/2.0};
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath rsPath;
    reedshepp_init(q0, q1, Constants::r, rsPath);
    for(int i = 0;i<5;i++){
        cout<< "rsPath length 1:"<<rsPath.length_[i]<<endl;
        cout << "rsPath type 1: "<<rsPath.type_[i]<<endl;
    }
    vector<Node3D> vec;
    double newStart[3]={q0[0],q0[1],q0[2]};

    /*
    get_right_turn(newStart,Constants::r * rsPath.length_[0],vec);
    cout<< "vec size: " << vec.size() << endl;
    for(int i = 0; i<vec.size(); i++){
        cout << "x : "<<vec[i][0]<<"y : "<<vec[i][1] << "theta : "<<vec[i][2]<<endl;
    }
    */
    
    
    for(int i = 0; i<5; i++){
        if(rsPath.type_[i] == 0){
            continue;
        }

        if(rsPath.type_[i] == 1) // left turn 
        {
            get_left_turn(newStart,Constants::r * rsPath.length_[i],vec);

        }
        else if(rsPath.type_[i] == 2) //straight
        {
            get_straight(newStart,Constants::r * rsPath.length_[i],vec);

        }
        else if(rsPath.type_[i] == 3) // right turn
        {
            get_right_turn(newStart,Constants::r * rsPath.length_[i],vec);

        }
    }
    cout<< "vec size: " << vec.size() << endl;
    for(int i = 0; i<vec.size(); i++){
        cout << "x : "<<vec[i].getX()<<"y : "<<vec[i].getY() << "theta : "<<vec[i].getT()<<endl;
    }

    Node3D* newNodes;
    Path jjpath;
    
    newNodes = getNodes(vec);
    //cout << "in main newNOdes Pred:  "<<newNodes->getPred()<<endl;
    cout << "newNodes = :" << newNodes <<endl;
    jjpath.clear();
    
    jjpath.updatePath(smoother.getPath());
    
    

    
    
    return 0;
}