#include "RrtPlanner.hpp"

using namespace std;

RrtPlanner::RrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>* newObstacleHash)
{
  nodeCount = 1;

  sampleGoal = 0;
  pathCompleteBool = false;

  startNode = make_shared<Node>();
  goalNode = make_shared<Node>();

  startNode->coord = start;
  startNode->parent = startNode;
  myRtree.insert(RtreeValue(startNode->coord.getBoostPoint(),startNode)); // insert the starting node into the list of known nodes

  goalNode->coord = goal;
  goalNode->parent = goalNode;

  width = newWidth;
  height = newHeight;
  obstacleHash = newObstacleHash;
}

RrtPlanner::~RrtPlanner()
{
    //TODO make a destructor
}

void RrtPlanner::saturate(int numNodes)
{
  if(numNodes < 1 || numNodes > MAX_NUM_NODES)  // you craycray??
  {
    return;
  }
  while(nodeCount < numNodes)
  {
    nextIteration();
  }
}

deque<Coord> RrtPlanner::getPath(Coord newGoal)
{
  //TODO validate newGoal
  // try to create new goalNode
  vector<shared_ptr<Node>>* closestNodes = getNearestNodes(newGoal, 10);
  shared_ptr<Node> newGoalNode;
  deque<Coord> path;

  for(auto node : *closestNodes)
  {
    if(!lineIntersectsObstacles(node->coord, newGoal, obstacleHash, width, height))
    { // WOOOOOT this is a valid part of our tree, lets add it to the family
      newGoalNode = make_shared<Node>(newGoal, node);
      myRtree.insert(RtreeValue(newGoal.getBoostPoint(), newGoalNode));
      node->children.push_back(newGoalNode); // make him a child of the parent
      nodeCount++;
      break;
    }
  }

  // handle node creation failure
  if(newGoalNode == NULL)
    return path;  // return empty path if none of the nearby nodes could connect

  // generate path
  path.push_front(newGoal);
  shared_ptr<Node> currentNode = newGoalNode->parent;
  do
  {
    path.push_front(currentNode->coord); // TODO pass by pointer instead?
    currentNode = currentNode->parent;
  } while(currentNode->parent != currentNode);  // if the parent is itself, we have reached the root
  path.push_front(currentNode->parent->coord); // add start coordinate
  return path;  // return the completed path
}

deque<Coord> RrtPlanner::getPath()
{
  deque<Coord> returnValue;
  if(pathComplete() || true)
  {
    returnValue.push_front(goalNode->coord); // start with the goal
    shared_ptr<Node> currentNode = goalNode->parent;
    do
    {
      returnValue.push_front(currentNode->coord); // TODO pass by pointer instead?
      currentNode = currentNode->parent;
    } while(currentNode->parent != currentNode);  // if the parent is itself, we have reached the root
    returnValue.push_front(currentNode->parent->coord); // add start coordinate
  }
  return returnValue;
}

bool RrtPlanner::pathComplete()
{
  return pathCompleteBool;
}

void RrtPlanner::nextIteration()
{
  if(!pathComplete())
  {
    Coord sample;
    // switch(sampleGoal)
    // {
    //   case 0 : sample = goalNode->coord;
    //     sampleGoal++;
    //   break;
    //   default :
    //   case 5 : sampleGoal = 0;
    //     sample = randomPoint(width, height);
    //     break;
    //   case 4 :
    //   case 3 :
    //   case 2 :
    //   case 1 : sample = randomPoint(width, height);
    //     sampleGoal++;
    //   break;
    // }

    // Random sampling, but if guess close to goal, go there.
    sample = randomPoint(width, height);
    //
    // if(euclideanDistance(sample, goalNode->coord) < 20.0)
    // {
    //   sample = goalNode->coord;
    // }

    shared_ptr<Node> closestNode = getNearestNode(sample);

    sample = getCoordInDerection(closestNode->coord, sample);  // trim the sample to the maximum distance

    if(!lineIntersectsObstacles(closestNode->coord, sample, obstacleHash, width, height))
    { // WOOOOOT this is a valid part of our tree, lets add it to the family
      shared_ptr<Node> newGuy = make_shared<Node>(sample, closestNode);
      myRtree.insert(RtreeValue(sample.getBoostPoint(), newGuy));
      closestNode->children.push_back(newGuy); // make him a child of the parent
      nodeCount++;
    }
    else  // OH NOOOOOO the line intersects an obstacle. Darn, just give up
    {
      return; // empty handed, no new nodes were added in the calling of this function
    }
  }
}



Coord RrtPlanner::getCoordInDerection(Coord nodeCoord, Coord goalCoord)
{
  if(euclideanDistance(nodeCoord, goalCoord) <= MAX_DIST)
  {
    return goalCoord; // goalCoord is well within range, it is what we want
  }
  else  // return a coord that is closer to the node
  {
    double angle = angleBetweenCoords(nodeCoord, goalCoord);
    return Coord(nodeCoord.x + cos(angle) * MAX_DIST, nodeCoord.y + sin(angle) * MAX_DIST);
  }
}

shared_ptr<Node> RrtPlanner::getNearestNode(Coord coord)
{
  vector<RtreeValue> rtreeQueryResults;
  myRtree.query(bgi::nearest(coord.getBoostPoint(), 1), back_inserter(rtreeQueryResults));

  return rtreeQueryResults[0].second; // return the nearest node
}

vector<shared_ptr<Node>>* RrtPlanner::getNearestNodes(Coord coord, int numNodes)
{
  vector<RtreeValue> rtreeQueryResults;
  //TODO validate numNodes
  myRtree.query(bgi::nearest(coord.getBoostPoint(), numNodes), back_inserter(rtreeQueryResults));

  vector<shared_ptr<Node>>* results = new vector<shared_ptr<Node>>();
  for(auto x : rtreeQueryResults) //cycle through results and get the nodes
  {
    results->push_back(x.second);
  }
  return results;
}
