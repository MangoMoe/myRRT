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
public:
  deque<Coord> getPath();
  void nextIteration();
  bool pathComplete();
  rrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>& newObstacleHash);
  ~rrtPlanner();
  Coord findNextRRTPoint();
  Coord getUnusedRandomNode();
  shared_ptr<Node> getNearestNode()
  {
    
  }
}
