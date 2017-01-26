#pragma once

#include <deque>
//#include <set>
#include <vector>
#include <iterator> //std::back_inserter

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "planning-utils/Node.hpp"
#include "planning-utils/geom/Coord.hpp"
#include "planning-utils/utils.hpp"
#include "planning-utils/geom/utils.hpp"

//copied from example code to make scope and namespaces easer when using boost
namespace bgi = boost::geometry::index;

// Copied these from bryant's code, but they make sense
typedef boost::geometry::model::box<point> box;
typedef std::pair<point, std::shared_ptr<Node>> RtreeValue;
typedef boost::geometry::index::rtree<RtreeValue, boost::geometry::index::rstar<16>> Rtree;
//TODO find out if we want a different algorithm for the rtree instead of rstar

using namespace std;

class RrtPlanner
{
private:
  int sampleGoal;
  bool pathCompleteBool;
  vector<vector<bool>>* obstacleHash;
  int width, height;

  Rtree myRtree;

  const double MAX_DIST = 0.10;  // maximum distance between nodes

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
};
