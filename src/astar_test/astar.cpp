#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "planner.h"
#include <boost/heap/binomial_heap.hpp>
#include "node2d.h"

using namespace cv;
using namespace std;
nav_msgs::OccupancyGridPtr grid;
struct CompareNodes1
{
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D *lhs, const Node3D *rhs) const
    {
        return lhs->getG() > rhs->getG();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D *lhs, const Node2D *rhs) const
    {
        return lhs->getG() > rhs->getG();
    }
};
struct CompareNodes
{
    /// Sorting 3D nodes by increasing C value - the total estimated cost
    bool operator()(const Node3D *lhs, const Node3D *rhs) const
    {
        return lhs->getG() > rhs->getG();
    }
    /// Sorting 2D nodes by increasing C value - the total estimated cost
    bool operator()(const Node2D *lhs, const Node2D *rhs) const
    {
        return lhs->getG() > rhs->getG();
    }
};
float a1Star(Node2D &start,
             Node2D &goal,
             Node2D *nodes2D,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization);
float jjStar(Node2D &start,
             Node2D &goal,
             Node2D *nodes2D,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization);

void updateMap(const nav_msgs::OccupancyGrid::Ptr map)
{
    grid = map;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "jj_star");
    ros::NodeHandle n;
    ros::Subscriber subMap;
    subMap = n.subscribe("/map", 1, updateMap);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
    int width = grid->info.width;
    int height = grid->info.height;

    Node2D *nodes2D = new Node2D[width * height]();
    //Node2D start2d(61, 124, 0, 0, nullptr);
    // create a 2d goal node
    //Node2D goal2d(61, 77, 0, 0, nullptr);
    Node2D start2d(1, 1, 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(100,105, 0, 0, nullptr);
    goal2d.setIdx(width);
    CollisionDetection configurationSpace;
    configurationSpace.updateGrid(grid);
    Visualize visualization;
    ros::WallTime start_, end_;
    visualization.clear();

    std::cout << "height : " << height << std::endl;
    std::cout << "width : " << width << std::endl;
    start_ = ros::WallTime::now();
    float astart_cost = a1Star(start2d, goal2d, nodes2D, width, height, configurationSpace, visualization);
    end_ = ros::WallTime::now();
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Astar time (ms): " << execution_time);
    
    
 
    
    start_ = ros::WallTime::now();
    jjStar(start2d, goal2d, nodes2D, width, height, configurationSpace, visualization);
    end_ = ros::WallTime::now();
    execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("jjStar time (ms): " << execution_time);

    float jjstar_cost = nodes2D[goal2d.getIdx()].getG();
    std::cout << "astar cost : " << astart_cost << std::endl;
    std::cout << "jjstar cost : " << jjstar_cost << std::endl;
    Mat image;
    image = imread("/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/jj_map_parking.png", -1);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    float Max_C = 0;
    for(int i = 0; i < width * height ; i++){
        if(nodes2D[i].getG() > Max_C){
            Max_C = nodes2D[i].getG();
        }
    }
    std::cout << "Max cost : " << Max_C << std::endl;
    
    int nodecount = 0;
    for (int i = 0; i < width * height ; i++){
        if(nodes2D[i].isClosed()){
            int r = i / width;

            int c = i % width;
            //std::cout <<"row : "<<r<<", col : "<<c<<std::endl;
            Vec3b &intensity = image.at<Vec3b>(124 - r, c);

            intensity.val[0] = saturate_cast< uchar >(255 * nodes2D[i].getG() / Max_C);
            intensity.val[1] = 1;
            intensity.val[2] = saturate_cast< uchar >(255 * nodes2D[i].getG() / Max_C);
            //intensity.val[0] = 1;
            //intensity.val[1] = 2;
            //intensity.val[2] = 3;
            nodecount++;
        }
    }
    //std::cout << "total node number : " << nodecount << std::endl;
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    imwrite("/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/jj_map_parking1.png", image);
    waitKey(0);

    delete[] nodes2D;
    return 0;
}
//###################################################
float jjStar(Node2D &start,
             Node2D &goal,
             Node2D *nodes2D,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization)
{
    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;

    // reset the open and closed list
    for (int i = 0; i < width * height; ++i)
    {
        nodes2D[i].open();
    }
    boost::heap::binomial_heap<Node2D *,
                               boost::heap::compare<CompareNodes1>>
        O;
    // update h value
    start.updateH(goal);
    // mark start as open
    start.open();
    // push on priority queue
    O.push(&start);
    iPred = start.setIdx(width);
    nodes2D[iPred] = start;
    int tt = nodes2D[iPred].isOpen() ? 1 : 0;
    std::cout << "node2D[iPred].isOpen : "<< tt <<std::endl;
    // NODE POINTER
    Node2D *nPred;
    Node2D *nSucc;
    int node_count = 0;
    while (!O.empty())
    {
        // pop node with lowest cost from priority queue
        //std::cout << "in while()"<<std::endl;
        nPred = O.top();
        O.pop();
        // set index
        iPred = nPred->setIdx(width);

        for (int i = 0; i < Node2D::dir; i++)
        {
            // create possible successor
            nSucc = nPred->createSuccessor(i);
            // set index of the successor
            iSucc = nSucc->setIdx(width);
            
            int isongrid = nSucc->isOnGrid(width, height)? 1:0;
            int isTrav = configurationSpace.isTraversable(nSucc)?1:0;
            //std::cout << "succ X : "<< nSucc->getX()
             //         << "succ Y : "<< nSucc->getY() <<std::endl;
            //std::cout << "isOnGrid(width, height) : "<< isongrid <<std::endl;
            //std::cout << "isTraversable(nSucc) : "<< isTrav <<std::endl;
            if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc))
            {
                // calculate new G value
                nSucc->updateG();
                newG = nSucc->getG();
                //int isopen = nodes2D[iSucc].isOpen()?1:0;
                //int lower = newG < nodes2D[iSucc].getG() ? 1:0;
                //std::cout << "nodes2D[iSucc].isOpen() : "<< isopen <<std::endl;
                //std::cout << "newG < nodes2D[iSucc].getG() : "<< lower <<std::endl;
                if (nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG())
                {
                    nodes2D[iSucc] = *nSucc;
                    nodes2D[iSucc].close();
                    nodes2D[iSucc].setG(newG);
                    O.push(&nodes2D[iSucc]);
                    node_count ++ ;
                    //std::cout << "total node number : "<<node_count<<std::endl;

                }
                else
                {
                    delete nSucc;
                }
            }
            else
            {
                delete nSucc;
            }
        }
        
    }
}
//###################################################
float a1Star(Node2D &start,
             Node2D &goal,
             Node2D *nodes2D,
             int width,
             int height,
             CollisionDetection &configurationSpace,
             Visualize &visualization)
{

    // PREDECESSOR AND SUCCESSOR INDEX
    int iPred, iSucc;
    float newG;

    // reset the open and closed list
    for (int i = 0; i < width * height; ++i)
    {
        nodes2D[i].reset();
    }

    // VISUALIZATION DELAY
    ros::Duration d(0.001);

    boost::heap::binomial_heap<Node2D *,
                               boost::heap::compare<CompareNodes>>
        O;
    // update h value
    start.updateH(goal);
    // mark start as open
    start.open();
    // push on priority queue
    O.push(&start);
    iPred = start.setIdx(width);
    nodes2D[iPred] = start;

    // NODE POINTER
    Node2D *nPred;
    Node2D *nSucc;

    // continue until O empty
    while (!O.empty())
    {
        // pop node with lowest cost from priority queue
        nPred = O.top();
        // set index
        iPred = nPred->setIdx(width);

        // _____________________________
        // LAZY DELETION of rewired node
        // if there exists a pointer this node has already been expanded
        if (nodes2D[iPred].isClosed())
        {
            // pop node from the open list and start with a fresh node
            O.pop();
            continue;
        }
        // _________________
        // EXPANSION OF NODE
        else if (nodes2D[iPred].isOpen())
        {
            // add node to closed list
            nodes2D[iPred].close();
            nodes2D[iPred].discover();

            // RViz visualization
            if (Constants::visualization2D)
            {
                visualization.publishNode2DPoses(*nPred);
                visualization.publishNode2DPose(*nPred);
                //        d.sleep();
            }

            // remove node from open list
            O.pop();

            // _________
            // GOAL TEST
            if (*nPred == goal)
            {
                return nPred->getG();
            }
            // ____________________
            // CONTINUE WITH SEARCH
            else
            {
                // _______________________________
                // CREATE POSSIBLE SUCCESSOR NODES
                for (int i = 0; i < Node2D::dir; i++)
                {
                    // create possible successor
                    nSucc = nPred->createSuccessor(i);
                    // set index of the successor
                    iSucc = nSucc->setIdx(width);

                    // ensure successor is on grid ROW MAJOR
                    // ensure successor is not blocked by obstacle
                    // ensure successor is not on closed list
                    if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed())
                    {
                        // calculate new G value
                        nSucc->updateG();
                        newG = nSucc->getG();

                        // if successor not on open list or g value lower than before put it on open list
                        if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG())
                        {
                            // calculate the H value
                            nSucc->updateH(goal);
                            // put successor on open list
                            nSucc->open();
                            nodes2D[iSucc] = *nSucc;
                            O.push(&nodes2D[iSucc]);
                            delete nSucc;
                        }
                        else
                        {
                            delete nSucc;
                        }
                    }
                    else
                    {
                        delete nSucc;
                    }
                }
            }
        }
    }

    // return large number to guide search away
    return 1000;
}