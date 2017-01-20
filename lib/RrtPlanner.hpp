#pragma once

#include "planning-utils/Node.hpp"
#include "planning-utils/geom/Coord.hpp"
#include <deque>
#include <set>
#include <cmath>  //TODO make sure this is the right library
#include "planning-utils/utils.hpp"
#include "planning-utils/geom/utils.hpp"

using namespace std;

class RrtPlanner
{
private:
  int sampleGoal;
  bool pathCompleteBool;
  vector<vector<bool>>* obstacleHash;
  int width, height;
  set<Coord> myCoords;
  const double MAX_DIST = 2.0;  // maximum distance between nodes
public:
  shared_ptr<Node> startNode, goalNode;
  
  RrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>* newObstacleHash);
  ~RrtPlanner();
  deque<Coord> getPath();
  bool pathComplete();
  void nextIteration();
  Coord getCoordInDerection(Coord nodeCoord, Coord goalCoord);
  shared_ptr<Node> getNearestNode(Coord coord);
  shared_ptr<Node> getNearestNode(Coord coord, shared_ptr<Node> node, shared_ptr<Node> curClosesestNode, double curShortDist);
  Coord getUnusedRandomCoord();
};
