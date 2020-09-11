#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "planner.h"

using namespace cv;
using namespace std;

bool comp(int a, int b)
{
    return (a < b);
}

bool FindPoint(float x1, float y1, float x2,
               float y2, float x, float y)
{
    if (x > x1 and x < x2 and y > y1 and y < y2)
        return true;

    return false;
}

void jjlookup(Constants::config *lookup)
{
    const float cSize = Constants::cellSize;
    // bounding box size length/width
    const int size = Constants::bbSize;
    cout << "bbsize: " << size << endl;
    struct point
    {
        double x;
        double y;
    };

    // ______________________
    // VARIABLES FOR ROTATION
    //center of the rectangle
    point c;
    point temp;
    // points of the rectangle
    point p[4];
    point nP[4];

    // turning angle
    double theta;

    // ____________________________
    // VARIABLES FOR GRID TRAVERSAL
    // vector for grid traversal
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;
    // grid
    bool cSpace[size * size];
    bool inside = false;
    int hcross1 = 0;
    int hcross2 = 0;
    float upper_left_rect = 0;
    float lower_right_rect = 0;
    // _____________________________
    // VARIABLES FOR LOOKUP CREATION
    int count = 0;
    const int positionResolution = Constants::positionResolution;
    const int positions = Constants::positions;
    point points[positions];

    // generate all discrete positions within one cell
    for (int i = 0; i < positionResolution; ++i)
    {
        for (int j = 0; j < positionResolution; ++j)
        {
            points[positionResolution * i + j].x = 1.f / positionResolution * j;
            points[positionResolution * i + j].y = 1.f / positionResolution * i;
        }
    }
    for (int q = 0; q < positions; ++q)
    {
        // set the starting angle to zero;
        theta = 0;

        // set points of rectangle
        c.x = points[q].x;
        c.y = points[q].y;
        //cout << "c.x,c.y = " << c.x << " , " << c.y << endl;
        p[0].x = -Constants::length / 2 / cSize;
        p[0].y = -Constants::width / 2 / cSize;

        p[1].x = -Constants::length / 2 / cSize;
        p[1].y = +Constants::width / 2 / cSize;

        p[2].x = +Constants::length / 2 / cSize;
        p[2].y = +Constants::width / 2 / cSize;

        p[3].x = +Constants::length / 2 / cSize;
        p[3].y = -Constants::width / 2 / cSize;

        for (int o = 0; o < Constants::headings; ++o)
        {
            // shape rotation
            for (int j = 0; j < 4; ++j)
            {
                // translate point to origin
                temp.x = p[j].x;
                temp.y = p[j].y;

                // rotate and shift back
                nP[j].x = temp.x * cos(o * Constants::deltaHeadingRad) - temp.y * sin(o * Constants::deltaHeadingRad) + c.x;
                nP[j].y = temp.x * sin(o * Constants::deltaHeadingRad) + temp.y * cos(o * Constants::deltaHeadingRad) + c.y;
            }

            float xmin = std::min({nP[0].x, nP[1].x, nP[2].x, nP[3].x}, comp);
            float xmax = std::max({nP[0].x, nP[1].x, nP[2].x, nP[3].x}, comp);
            float ymin = std::min({nP[0].y, nP[1].y, nP[2].y, nP[3].y}, comp);
            float ymax = std::max({nP[0].y, nP[1].y, nP[2].y, nP[3].y}, comp);

            //cout << "xmin: " << xmin << " "
            //     << "xmax: " << xmax << " "
            //     << "ymin: " << ymin << " "
            //     << "ymax: " << ymax << endl;

            int ixmin = std::floor(xmin);
            int ixmax = std::ceil(xmax);
            int iymin = std::floor(ymin);
            int iymax = std::ceil(ymax);
            //cout << "ixmin: " << ixmin << " "
            //     << "ixmax: " << ixmax << " "
            //     << "iymin: " << iymin << " "
            //     << "iymax: " << iymax << endl;

            uint grid_width = ixmax - ixmin;
            uint grid_height = iymax - iymin;
            point *upper_left_points = new point[(grid_width + 1) * (grid_height + 1)];
            point *upper_left_points_rotate_back = new point[(grid_width + 1) * (grid_height + 1)];
            bool *upper_left_in_rect_bool = new bool[(grid_width + 1) * (grid_height + 1)];
            bool *rect_is_occupy = new bool[grid_width * grid_height];
            for (int cy = 0; cy <= grid_height; cy++)
            {
                for (int cx = 0; cx <= grid_width; cx++)
                {
                    temp.x = ixmin + cx - c.x;
                    temp.y = iymax - cy - c.y;
                    upper_left_points[cy * grid_width + cx].x = temp.x;
                    upper_left_points[cy * grid_width + cx].y = temp.y;

                    upper_left_points_rotate_back[cy * grid_width + cx].x = temp.x * cos(-o * Constants::deltaHeadingRad) - temp.y * sin(-o * Constants::deltaHeadingRad);
                    upper_left_points_rotate_back[cy * grid_width + cx].y = temp.x * sin(-o * Constants::deltaHeadingRad) + temp.y * cos(-o * Constants::deltaHeadingRad);
                    upper_left_in_rect_bool[cy * grid_width + cx] = FindPoint(p[0].x, p[0].y, p[2].x, p[2].y, upper_left_points_rotate_back[cy * grid_width + cx].x, upper_left_points_rotate_back[cy * grid_width + cx].y);
                    //cout << "point("<<upper_left_points_rotate_back[cy * grid_width + cx].x<<","<<upper_left_points_rotate_back[cy * grid_width + cx].y
                    //<<") in rect is " <<(upper_left_in_rect_bool[cy * grid_width + cx]?1:0) <<endl;
                }
            }

            for (int cy = 0; cy < grid_height; cy++)
            {
                for (int cx = 0; cx < grid_width; cx++)
                {
                    if (upper_left_in_rect_bool[cy * grid_width + cx] ||
                        upper_left_in_rect_bool[cy * grid_width + cx + 1] ||
                        upper_left_in_rect_bool[(cy + 1) * grid_width + cx] ||
                        upper_left_in_rect_bool[(cy + 1) * grid_width + cx + 1])
                    {
                        rect_is_occupy[cy * grid_width + cx] = true;
                    }
                    else
                    {
                        rect_is_occupy[cy * grid_width + cx] = false;
                    }
                } 
            }
            //cout << "HAH"<<endl;
            
            count = 0;
            for (int cy = 0; cy < grid_height; cy++)
            {
                for (int cx = 0; cx < grid_width; cx++)
                {
                    if (rect_is_occupy[cy * grid_width + cx])
                    {
                        
                        //cout << q * Constants::headings + o<<endl;
                       
                        
                        lookup[q * Constants::headings + o].pos[count].x = ixmin + cx;
                        lookup[q * Constants::headings + o].pos[count].y = iymax - cy;
                        // add one for the length of the current list
                        count++;
                        //cout<<"count : "<<count<<endl;
                        

                       //vec[q * Constants::headings + o][count] = make_pair(ixmin + cx,iymax - cy);
                       //count++;
                    }
                }
            }
            lookup[q * Constants::headings + o].length = count;
            
            delete rect_is_occupy;
            delete upper_left_in_rect_bool;
            delete upper_left_points_rotate_back;
            delete upper_left_points;
        }
    }
}
int main()
{
    float x = 62.6807;
    float y = 77.5736;
    float t = 269.427/360.0*2*3.141592653;
    int X = (int)x;
    int Y = (int)y;
    int iX = (int)((x - (long)x) * Constants::positionResolution);
    iX = iX > 0 ? iX : 0;
    int iY = (int)((y - (long)y) * Constants::positionResolution);
    iY = iY > 0 ? iY : 0;
    int iT = (int)(t / Constants::deltaHeadingRad);
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
    cout << "iT : "<<iT<<endl;
    int cX;
    int cY;
    vector<vector<pair<int,int>>> vec;
    Constants::config * collisionLookup = new Constants::config[Constants::headings * Constants::positions * 2];
    //jjlookup(collisionLookup);
    Lookup::collisionLookup(collisionLookup);

    Mat image;
    image = imread("/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/jj_map_parking.png", -1);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }
    for (int i = 0; i < collisionLookup[idx].length; ++i)
    {
        cX = (X + collisionLookup[idx].pos[i].x);
        cY = (Y + collisionLookup[idx].pos[i].y);

        // make sure the configuration coordinates are actually on the grid

        if (cX >= 0 && cX < 160 && cY >= 0 && cY < 150)
        {

            std::cout << "cX : " << cX << " "
                      << "cY : " << cY << " " << std::endl;
            Vec3b &intensity = image.at<Vec3b>(125 - cY, cX);
            intensity.val[0] = 100;
            intensity.val[1] = 12;
            intensity.val[2] = 122;
        }

        /*
    else{
      return false;
    }
*/
    }
    for (int i = 20; i < 100; i++)
    {
        Vec3b &intensity = image.at<Vec3b>(i, 20);
        intensity.val[0] = 100;
        intensity.val[1] = 12;
        intensity.val[2] = 122;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    imwrite("/home/ljj/code/APA/pathplannerOnGit/src/path_planner/maps/jj_map_parking1.png", image);
    waitKey(0);

    return 0;
}