#include "reedshepp.h"
#include "constants.h"
#define PI 3.1415926535
using namespace std;
using namespace HybridAStar;

double fmodr(double x, double y)
{
    return x - y * floor(x / y);
}

double mod2pi(double theta)
{
    return fmodr(theta, 2 * PI);
}
/*
Node3D *reedshepp_curve(Node3D t0, Node3D t1)
{

    ompl::base::ReedsSheppStateSpace::ReedsSheppPath rsPath;
    double q0[3] = {t0.getX(), t0.getY(), t0.getT()};
    double q1[3] = {t1.getX(), t1.getY(), t1.getT()};
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();
    rsStart->setXY(q0[0], q0[1]);
    rsStart->setYaw(q0[2]);
    rsEnd->setXY(q1[0], q1[1]);
    rsEnd->setYaw(q1[2]);
    rsPath = reedsSheppPath.reedsShepp(rsStart, rsEnd);

    vector<vector<float>> vec;
    double newStart[3] = {q0[0], q0[1], q0[2]};
    for (int i = 0; i < 5; i++)
    {
        if (rsPath.type_[i] == 0)
        {
            continue;
        }

        if (rsPath.type_[i] == 1) // left turn
        {
            get_left_turn(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
        else if (rsPath.type_[i] == 2) //straight
        {
            get_straight(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
        else if (rsPath.type_[i] == 3) // right turn
        {
            get_right_turn(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
    }

    int num_vec = vec.size();
    Node3D *nPred;
    Node3D *nSucc;
    Node3D tmp();
    Node3D *newNodes = new Node3D[num_vec];
    for (int i = 0; i < num_vec; i++)
    {
        if (i == 0)
        {
            Node3D tt(vec[i][0], vec[i][1], vec[i][2], 0, 0, nullptr);
            newNodes[i] = tt;
            nPred = &newNodes[i];
        }
        else
        {
            Node3D tt(vec[i][0], vec[i][1], vec[i][2], 0, 0, nPred);
            newNodes[i] = tt;
            nPred = &newNodes[i];
        }
    }
    return &newNodes[vec.size() - 1];
}
*/
void reedshepp_init(double q0[3], double q1[3], double rho, ompl::base::ReedsSheppStateSpace::ReedsSheppPath &rsPath)
{
    std::cout << "q0 x:" << q0[0] << std::endl;
    std::cout << "q0 y:" << q0[1] << std::endl;
    std::cout << "q0 z:" << q0[2] << std::endl;
    std::cout << "q1 x:" << q1[0] << std::endl;
    std::cout << "q1 y:" << q1[1] << std::endl;
    std::cout << "q1 z:" << q1[2] << std::endl;
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();
    rsStart->setXY(q0[0], q0[1]);
    rsStart->setYaw(q0[2]);
    rsEnd->setXY(q1[0], q1[1]);
    rsEnd->setYaw(q1[2]);
    rsPath = reedsSheppPath.reedsShepp(rsStart, rsEnd);
}

void reedshepp_init(Node3D t0, Node3D t1, double rho, ompl::base::ReedsSheppStateSpace::ReedsSheppPath &rsPath)
{
    double q0[3] = {t0.getX(), t0.getY(), t0.getT()};
    double q1[3] = {t1.getX(), t1.getY(), t1.getT()};
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();
    rsStart->setXY(q0[0], q0[1]);
    rsStart->setYaw(q0[2]);
    rsEnd->setXY(q1[0], q1[1]);
    rsEnd->setYaw(q1[2]);
    rsPath = reedsSheppPath.reedsShepp(rsStart, rsEnd);
}

/*
void reedshepp_sample(ompl::base::ReedsSheppStateSpace::ReedsSheppPath &rsPath, double q0[3], vector<vector<float>> &vec)
{
    double newStart[3] = {q0[0], q0[1], q0[2]};

    for (int i = 0; i < 5; i++)
    {
        if (rsPath.type_[i] == 0)
        {
            continue;
        }

        if (rsPath.type_[i] == 1) // left turn
        {
            get_left_turn(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
        else if (rsPath.type_[i] == 2) //straight
        {
            get_straight(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
        else if (rsPath.type_[i] == 3) // right turn
        {
            get_right_turn(newStart, Constants::r * rsPath.length_[i], vec);
            newStart[0] = vec[vec.size() - 1][0];
            newStart[1] = vec[vec.size() - 1][1];
            newStart[2] = vec[vec.size() - 1][2];
        }
    }
}
*/
void rotate_z(const double theta, const double q0[2], double q1[2])
{
    q1[0] = q0[0] * cos(theta) - q0[1] * sin(theta);
    q1[1] = q0[0] * sin(theta) + q0[1] * cos(theta);
}

void get_left_turn(double q0[3], const double path_length, vector<Node3D> &vec)
{
    double unit_pt_heading[2] = {cos(q0[2]), sin(q0[2])};
    double unit_pt_heading_perp[2];
    rotate_z(PI / 2.0, unit_pt_heading, unit_pt_heading_perp);
    double pt_heading_perp[2] = {Constants::r * unit_pt_heading_perp[0], Constants::r * unit_pt_heading_perp[1]};
    double rotation_center[2] = {q0[0] + pt_heading_perp[0], q0[1] + pt_heading_perp[1]};
    double pt_center_to_start[2] = {-pt_heading_perp[0], -pt_heading_perp[1]};

    int numsOfSample = (int)(abs(path_length) / Constants::dubinsStepSize) + 1;
    int signflag = (path_length > 0) ? 1 : -1;
    double theta_res = signflag * Constants::dubinsStepSize / Constants::r;
    double new_pt_center_to_start[2];

    double new_unit_pt_heading[2];

    //put the first node in to the vec
    Node3D tmpNode;
    rotate_z(0.0 * theta_res, pt_center_to_start, new_pt_center_to_start);
    tmpNode.setX(rotation_center[0] + new_pt_center_to_start[0]);
    tmpNode.setY(rotation_center[1] + new_pt_center_to_start[1]);
    rotate_z(0.0 * theta_res, unit_pt_heading, new_unit_pt_heading);
    tmpNode.setT(mod2pi(atan2(new_unit_pt_heading[1], new_unit_pt_heading[0])));
    tmpNode.setIsRS(true);
    tmpNode.setRSMotionType(int(signflag*1));//left turn 
    vec.push_back(tmpNode);

    int tmpNumber = 0;
    for (int i = 0; i < numsOfSample - 1; i++)
    {

        rotate_z((i + 1) * theta_res, pt_center_to_start, new_pt_center_to_start);
        tmpNode.setX(rotation_center[0] + new_pt_center_to_start[0]);
        tmpNode.setY(rotation_center[1] + new_pt_center_to_start[1]);
        rotate_z((i + 1) * theta_res, unit_pt_heading, new_unit_pt_heading);
        tmpNode.setT(mod2pi(atan2(new_unit_pt_heading[1], new_unit_pt_heading[0])));
        vec[vec.size()-1].setRemainingDist(Constants::dubinsStepSize);
        vec.push_back(tmpNode);
        tmpNumber++;
    }
    vec[vec.size()-1].setRemainingDist(path_length - tmpNumber*Constants::dubinsStepSize); 

    
    rotate_z(path_length / Constants::r, pt_center_to_start, new_pt_center_to_start);
    rotate_z(path_length / Constants::r, unit_pt_heading, new_unit_pt_heading);

    q0[0] = rotation_center[0] + new_pt_center_to_start[0];
    q0[1] = rotation_center[1] + new_pt_center_to_start[1];
    q0[2] = mod2pi(atan2(new_unit_pt_heading[1],new_unit_pt_heading[0]));
}

void get_right_turn(double q0[3], const double path_length, vector<Node3D> &vec)
{
    double unit_pt_heading[2] = {cos(q0[2]), sin(q0[2])};
    //cout<< "q0[2] :  " << q0[2] << endl;
    //cout<<"unit_pt_heading:  "<<unit_pt_heading[0] << "  " << unit_pt_heading[1] << endl;
    double unit_pt_heading_perp[2];
    rotate_z(-PI / 2.0, unit_pt_heading, unit_pt_heading_perp);
    double pt_heading_perp[2] = {Constants::r * unit_pt_heading_perp[0], Constants::r * unit_pt_heading_perp[1]};
    double rotation_center[2] = {q0[0] + pt_heading_perp[0], q0[1] + pt_heading_perp[1]};
    double pt_center_to_start[2] = {-pt_heading_perp[0], -pt_heading_perp[1]};

    int numsOfSample = (int)(abs(path_length) / Constants::dubinsStepSize) + 1;
    int signflag = (path_length > 0) ? -1 : 1;
    double theta_res = signflag * Constants::dubinsStepSize / Constants::r;
    double new_pt_center_to_start[2];

    double new_unit_pt_heading[2];

    Node3D tmpNode;
    rotate_z(0.0 * theta_res, pt_center_to_start, new_pt_center_to_start);
    tmpNode.setX(rotation_center[0] + new_pt_center_to_start[0]);
    tmpNode.setY(rotation_center[1] + new_pt_center_to_start[1]);
    rotate_z(0.0 * theta_res, unit_pt_heading, new_unit_pt_heading);
    tmpNode.setT(mod2pi(atan2(new_unit_pt_heading[1], new_unit_pt_heading[0])));
    tmpNode.setIsRS(true);
    tmpNode.setRSMotionType(int(signflag*3));//right turn 
    vec.push_back(tmpNode);

    int tmpNumber = 0;
    for (int i = 0; i < numsOfSample - 1; i++)
    {
        rotate_z((i + 1) * theta_res, pt_center_to_start, new_pt_center_to_start);
        tmpNode.setX(rotation_center[0] + new_pt_center_to_start[0]);
        tmpNode.setY(rotation_center[1] + new_pt_center_to_start[1]);
        rotate_z((i + 1) * theta_res, unit_pt_heading, new_unit_pt_heading);
        tmpNode.setT(mod2pi(atan2(new_unit_pt_heading[1], new_unit_pt_heading[0])));
        vec[vec.size()-1].setRemainingDist(Constants::dubinsStepSize);
        vec.push_back(tmpNode);
        tmpNumber++;
    }
    vec[vec.size()-1].setRemainingDist(path_length - tmpNumber*Constants::dubinsStepSize); 
    rotate_z(-path_length / Constants::r, pt_center_to_start, new_pt_center_to_start);
    
    rotate_z(-path_length / Constants::r, unit_pt_heading, new_unit_pt_heading);
    

    q0[0] = rotation_center[0] + new_pt_center_to_start[0];
    q0[1] = rotation_center[1] + new_pt_center_to_start[1];
    q0[2] = mod2pi(atan2(new_unit_pt_heading[1],new_unit_pt_heading[0]));


}

void get_straight(double q0[3], const double path_length, vector<Node3D> &vec)
{
    double unit_pt_heading[2] = {cos(q0[2]), sin(q0[2])};

    int numsOfSample = (int)(abs(path_length) / Constants::dubinsStepSize) + 1;
    int signflag = (path_length > 0) ? 1 : -1;
    double d_res = signflag * Constants::dubinsStepSize;

    Node3D tmpNode;
    
    tmpNode.setX(q0[0]);
    tmpNode.setY(q0[1]);
    tmpNode.setT(q0[2]);
    tmpNode.setIsRS(true);
    tmpNode.setRSMotionType(int(signflag*2));//straight
    vec.push_back(tmpNode);

    int tmpNumber = 0;
    for (int i = 0; i < numsOfSample - 1; i++)
    {
        tmpNode.setX((i + 1) * d_res * unit_pt_heading[0] + q0[0]);
        tmpNode.setY((i + 1) * d_res * unit_pt_heading[1] + q0[1]);
        tmpNode.setT(q0[2]);
        vec[vec.size()-1].setRemainingDist(Constants::dubinsStepSize);
        vec.push_back(tmpNode);
        tmpNumber++;
    }
    vec[vec.size()-1].setRemainingDist(path_length - tmpNumber*Constants::dubinsStepSize);
    
    q0[0] = path_length * unit_pt_heading[0] + q0[0];
    q0[1] = path_length * unit_pt_heading[1] + q0[1];
    q0[2] = q0[2];
}