#include "path.h"

using namespace HybridAStar;

//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear()
{
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePath(std::vector<Node3D> nodePath)
{
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i)
  {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D &node)
{
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}
void Path::publishLineSeg(Node3D *nstart, Node3D *ngoal)
{
  nav_msgs::Path jjpath;
  jjpath.header.frame_id = "path";
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = nstart->getX() * Constants::cellSize;
  vertex.pose.position.y = nstart->getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  jjpath.poses.push_back(vertex);

  vertex.pose.position.x = ngoal->getX() * Constants::cellSize;
  vertex.pose.position.y = ngoal->getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  jjpath.poses.push_back(vertex);
  pubPath.publish(jjpath);
  ROS_INFO("haha_qqw");

}

void Path::publish_search_tree(Node3D *nstart, Node3D *ngoal){
  line_list.header.frame_id = "path";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.2;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = nstart->getX() * Constants::cellSize;
  p.y = nstart->getY() * Constants::cellSize;
  p.z = 0;
  line_list.points.push_back(p);

  p.x = ngoal->getX() * Constants::cellSize;
  p.y = ngoal->getY() * Constants::cellSize;
  p.z = 0;
  line_list.points.push_back(p);

  pubPathTree.publish(line_list);


}
void Path::cleanPathTree(){
  line_list.header.frame_id = "path";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 2;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.2;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  line_list.points.clear();
  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  line_list.points.push_back(p);

  p.x = 0;
  p.y = 0;
  p.z = 0;
  line_list.points.push_back(p);

  pubPathTree.publish(line_list);
}

// ________
// ADD NODE
void Path::addNode(const Node3D &node, int i)
{
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0)
  {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed)
  {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  }
  else
  {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D &node, int i)
{
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1)
  {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed)
  {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  }
  else
  {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}
