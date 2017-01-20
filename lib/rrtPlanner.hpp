#pragma once

#include "planning-utils/Node.hpp"
#include "planning-utils/geom/Coord.hpp"
#include <deque>
#include <set>
#include <cmath>  //TODO make sure this is the right library

class rrtPlanner
{
private:
  Node startNode, goalNode;
  int sampleGoal;
  bool pathCompleteBool;
  vector<vector<bool>>& obstacleHash;
  int width, height;
  set<Coord> myCoords;
  final double MAX_DIST = 2.0;  // maximum distance between nodes
public:
  rrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>& newObstacleHash);
  ~rrtPlanner();
  deque<Coord> getPath();
  bool pathComplete();
  void nextIteration();
  Coord getCoordInDerection(Coord nodeCoord, Coord goalCoord);
  shared_ptr<Node> getNearestNode(Coord coord);
  shared_ptr<Node> getNearestNode(Coord coord, shared_ptr<Node> node, shared_ptr<Node> curClosesestNode, double curShortDist);
  Coord getUnusedRandomCoord();
}
