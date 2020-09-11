#ifndef REEDSHEPP_H
#define REEDSHEPP_H
#include "constants.h"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <vector>
#include <math.h>
#include "node3d.h"
using namespace std;
typedef ompl::base::SE2StateSpace::StateType State;
using namespace HybridAStar;


void reedshepp_init(double q0[3], double q1[3], double rho,ompl::base::ReedsSheppStateSpace::ReedsSheppPath& rsPath);
void reedshepp_init(Node3D t0, Node3D t1, double rho,ompl::base::ReedsSheppStateSpace::ReedsSheppPath& rsPath);
void reedshepp_sample(ompl::base::ReedsSheppStateSpace::ReedsSheppPath& rsPath, double q0[3], vector<vector<float>>& vec);
void get_left_turn(double q0[3], const double path_length, vector<Node3D>& vec);
void get_right_turn(double q0[3], const double path_length, vector<Node3D>& vec);
void get_straight(double q0[3], const double path_length, vector<Node3D>& vec);
Node3D* reedshepp_curve(Node3D t0, Node3D t1);
/*
void reedshepp_path_sample(State* fr, State* to, double t, double p[3]){
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* midState;
    reedsSheppPath.interpolate(fr, to, t, midState);
    p[0] = midState->getX();
    p[1] = midState->getY();
    p[2] = midState->getYaw();

}
*/
#endif