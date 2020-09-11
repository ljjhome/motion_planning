#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  collisionLookup = new Constants::config[Constants::headings * Constants::positions*2];
  Lookup::collisionLookup(collisionLookup);
  //Lookup::jjlookup(collisionLookup);
}
CollisionDetection::~CollisionDetection(){
  delete [] collisionLookup;
}
bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int cX;
  int cY;
  //std::cout<<"grid width: " <<grid->info.width << std::endl;
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {

        //std::cout<<"(cX,cY)="<<cX<<","<<cY<<std::endl;
        return false;
      }
    }
    /*
    else{
      return false;
    }
*/
  }

  return true;
}
