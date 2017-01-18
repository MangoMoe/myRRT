#include "rrtPlanner.hpp"

rrtPlanner::rrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>& newObstacleHash)
{
  sampleGoal = 0;
  pathCompleteBool = false;

  startNode = new Node();

  startNode.coord = start;
  startNode.parent = (std::shared_ptr<Node>)&startNode;
  myCoords.insert(startNode.coord);

  goalNode.coord = goal;
  goalNode.parent = (std::shared_ptr<Node>)&goalNode;

  width = newWidth;
  height = newHeight;
  obstacleHash = newObstacleHash;
}

rrtPlanner::~rrtPlanner()
{
    //TODO make a destructor
}

deque<Coord> rrtPlanner::getPath()
{
  deque<Coord> returnValue = new deque<Coord>();
  if(pathComplete())
  {
    returnValue.push_front(goalNode.coord); // start with the goal
    shared_ptr<Node> currentNode = goalNode.parent;
    do
    {
      returnValue.push_front(currentNode->coord) // TODO pass by pointer instead?
      currentNode = currentNode->parent;
    } while(currentNode.parent != (shared_ptr<Node>)&currentNode);  // if the parent is itself, we have reached the root
  }
  return returnValue;
}

bool rrtPlanner::pathComplete()
{
  return pathCompleteBool;
}

void rrtPlanner::nextIteration()
{
  while(!pathCompleteBool)
  {
    Coord sample;
    switch(sampleGoal)
    {
      case 0 : sample = (shared_ptr<Node>)&goalNode;
      break;
      default :
      case 5 : sampleGoal = 0;
      case 4 :
      case 3 :
      case 2 :
      case 1 : sample = getUnusedRandomNode();
      break;
    }

    // TODO add point coordinates to the set of known coordinates
  }
}

shared_ptr<Node> rrtPlanner::getNearestNode()
{
  
}

Coord rrtPlanner::getUnusedRandomNode()
{
  Coord potentialPoint = randomPoint(width, height);
  while(myCoords.find(potentialPoint) != myCoords.end())
  {
    potentialPoint = randomPoint(width, height);
  }
  return potentialPoint;
}
