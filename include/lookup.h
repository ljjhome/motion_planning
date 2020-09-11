#ifndef COLLISIONLOOKUP
#define COLLISIONLOOKUP

#include "dubins.h"
#include "constants.h"
/*
inline bool comp(int a, int b)
{
    return (a < b);
}

inline bool FindPoint(float x1, float y1, float x2,
               float y2, float x, float y)
{
    if (x > x1 and x < x2 and y > y1 and y < y2)
        return true;

    return false;
}
*/
namespace HybridAStar {
namespace Lookup {

//###################################################
//                                      DUBINS LOOKUP
//###################################################
inline void dubinsLookup(float* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the Dubin's lookup table...";

  DubinsPath path;

  int width = Constants::dubinsWidth / Constants::cellSize;

  //  // increase the width by one to make it square
  //  if (width % 2 != 0) {
  //    width++;
  //  }

  const int headings = Constants::headings;

  // start and goal vector
  double start[3];
  double goal[] = {0, 0, 0};

  // iterate over the X index of a grid cell
  for (int X = 0; X < width; ++X) {
    start[0] = X;

    // iterate over the Y index of a grid cell
    for (int Y = 0; Y < width; ++Y) {
      start[1] = Y;

      // iterate over the start headings
      for (int h0 = 0; h0 < headings; ++h0) {
        start[2] = Constants::deltaHeadingRad * h0;

        // iterate over the goal headings
        for (int h1 = 0; h1 < headings; ++h1) {
          goal[2] = Constants::deltaHeadingRad * h1;

          // calculate the actual cost
          dubins_init(start, goal, Constants::r, &path);
          lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] = dubins_path_length(&path);

          if (DEBUG && lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] < sqrt(X * X + Y * Y) * 1.000001) {
            std::cout << X << " | " << Y << " | "
                      << Constants::deltaHeadingDeg* h0 << " | "
                      << Constants::deltaHeadingDeg* h1 << " length: "
                      << lookup[X * headings * headings * width + Y * headings * headings + h0 * headings + h1] << "\n";

          }
        }
      }
    }
  }

  std::cout << " done!" << std::endl;
}

//###################################################
//                                   COLLISION LOOKUP
//###################################################

// _____________
// SIGN FUNCTION
inline int sign(double x) {
  if (x >= 0) { return 1; }
  else { return -1; }
}

// _________________________
// COLLISION LOOKUP CREATION
inline void collisionLookup(Constants::config* lookup) {
  bool DEBUG = false;
  std::cout << "I am building the collision lookup table...";
  // cell size
  const float cSize = Constants::cellSize;
  // bounding box size length/width
  const int size = Constants::bbSize;

  struct point {
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

  // _____________________________
  // VARIABLES FOR LOOKUP CREATION
  int count = 0;
  const int positionResolution = Constants::positionResolution;
  const int positions = Constants::positions;
  point points[positions];

  // generate all discrete positions within one cell
  for (int i = 0; i < positionResolution; ++i) {
    for (int j = 0; j < positionResolution; ++j) {
      points[positionResolution * i + j].x = 1.f / positionResolution * j;
      points[positionResolution * i + j].y = 1.f / positionResolution * i;
    }
  }


  
  for (int q = 0; q < positions; ++q) {
    // set the starting angle to zero;
    theta = 0;

    // set points of rectangle
    c.x = (double)size / 2 + points[q].x;
    c.y = (double)size / 2 + points[q].y;

    p[0].x = c.x - Constants::length / 2 / cSize;
    p[0].y = c.y - Constants::width / 2 / cSize;

    p[1].x = c.x - Constants::length / 2 / cSize;
    p[1].y = c.y + Constants::width / 2 / cSize;

    p[2].x = c.x + Constants::length / 2 / cSize;
    p[2].y = c.y + Constants::width / 2 / cSize;

    p[3].x = c.x + Constants::length / 2 / cSize;
    p[3].y = c.y - Constants::width / 2 / cSize;

    for (int o = 0; o < Constants::headings; ++o) {
      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180.f / M_PI << std::endl; }

      // initialize cSpace
      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          cSpace[i * size + j] = false;
        }
      }

      // shape rotation
      for (int j = 0; j < 4; ++j) {
        // translate point to origin
        temp.x = p[j].x - c.x;
        temp.y = p[j].y - c.y;

        // rotate and shift back
        nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
        nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
      }

      // create the next angle
      theta += Constants::deltaHeadingRad;

      // cell traversal clockwise
      for (int k = 0; k < 4; ++k) {
        // create the vectors clockwise
        if (k < 3) {
          start = nP[k];
          end = nP[k + 1];
        } else {
          start = nP[k];
          end = nP[0];
        }

        //set indexes
        X = (int)start.x;
        Y = (int)start.y;
        //      std::cout << "StartCell: " << X << "," << Y << std::endl;
        cSpace[Y * size + X] = true;
        t.x = end.x - start.x;
        t.y = end.y - start.y;
        stepX = sign(t.x);
        stepY = sign(t.y);

        // width and height normalized by t
        if (t.x != 0) {
          tDeltaX = 1.f / std::abs(t.x);
        } else {
          tDeltaX = 1000;
        }

        if (t.y != 0) {
          tDeltaY = 1.f / std::abs(t.y);
        } else {
          tDeltaY = 1000;
        }

        // set maximum traversal values
        if (stepX > 0) {
          tMaxX = tDeltaX * (1 - (start.x - (long)start.x));
        } else {
          tMaxX = tDeltaX * (start.x - (long)start.x);
        }

        if (stepY > 0) {
          tMaxY = tDeltaY * (1 - (start.y - (long)start.y));
        } else {
          tMaxY = tDeltaY * (start.y - (long)start.y);
        }

        while ((int)end.x != X || (int)end.y != Y) {
          // only increment x if the t length is smaller and the result will be closer to the goal
          if (tMaxX < tMaxY && std::abs(X + stepX - (int)end.x) < std::abs(X - (int)end.x)) {
            tMaxX = tMaxX + tDeltaX;
            X = X + stepX;
            cSpace[Y * size + X] = true;
            // only increment y if the t length is smaller and the result will be closer to the goal
          } else if (tMaxY < tMaxX && std::abs(Y + stepY - (int)end.y) < std::abs(Y - (int)end.y)) {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
          } else if (2 >= std::abs(X - (int)end.x) + std::abs(Y - (int)end.y)) {
            if (std::abs(X - (int)end.x) > std::abs(Y - (int)end.y)) {
              X = X + stepX;
              cSpace[Y * size + X] = true;
            } else {
              Y = Y + stepY;
              cSpace[Y * size + X] = true;
            }
          } else {
            // this SHOULD NOT happen
            std::cout << "\n--->tie occured, please check for error in script\n";
            break;
          }
        }
      }

      // FILL THE SHAPE
      for (int i = 0; i < size; ++i) {
        // set inside to false
        inside = false;

        for (int j = 0; j < size; ++j) {

          // determine horizontal crossings
          for (int k = 0; k < size; ++k) {
            if (cSpace[i * size + k] && !inside) {
              hcross1 = k;
              inside = true;
            }

            if (cSpace[i * size + k] && inside) {
              hcross2 = k;
            }
          }

          // if inside fill
          if (j > hcross1 && j < hcross2 && inside) {
            cSpace[i * size + j] = true;
          }
        }
      }

      // GENERATE THE ACTUAL LOOKUP
      count = 0;

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {
            // compute the relative position of the car cells
            lookup[q * Constants::headings + o].pos[count].x = j - (int)c.x;
            lookup[q * Constants::headings + o].pos[count].y = i - (int)c.y;
            // add one for the length of the current list
            count++;
          }
        }
      }

      lookup[q * Constants::headings + o].length = count;

      if (DEBUG) {
        //DEBUG
        for (int i = 0; i < size; ++i) {
          std::cout << "\n";

          for (int j = 0; j < size; ++j) {
            if (cSpace[i * size + j]) {
              std::cout << "#";
            } else {
              std::cout << ".";
            }
          }
        }

        //TESTING
        std::cout << "\n\nthe center of " << q* Constants::headings + o << " is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[q * Constants::headings + o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[q * Constants::headings + o].pos[i].x << " | " << lookup[q * Constants::headings + o].pos[i].y << std::endl;
        }
      }
    }
  }
  
  std::cout << " done!" << std::endl;
}

/*
inline void jjlookup(Constants::config *lookup)
{
    const float cSize = Constants::cellSize;
    // bounding box size length/width
    const int size = Constants::bbSize;
    std::cout << "bbsize: " << size << std::endl;
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
*/
}
}
#endif // LOOKUP

