#include "RrtPlanner.hpp"

using namespace std;

RrtPlanner::RrtPlanner(Coord start, Coord goal, int newWidth, int newHeight, vector<vector<bool>>* newObstacleHash)
{
  sampleGoal = 0;
  pathCompleteBool = false;

  // startNode();
  // goalNode();
  startNode = make_shared<Node>();
  goalNode = make_shared<Node>();

  startNode->coord = start;
  startNode->parent = startNode;
  myCoords.insert(startNode->coord);

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

deque<Coord> RrtPlanner::getPath()
{
  deque<Coord> returnValue; // = new deque<Coord>();
  if(pathComplete())
  {
    returnValue.push_front(goalNode->coord); // start with the goal
    shared_ptr<Node> currentNode = goalNode->parent;
    do
    {
      returnValue.push_front(currentNode->coord); // TODO pass by pointer instead?
      currentNode = currentNode->parent;
    } while(currentNode->parent != currentNode);  // if the parent is itself, we have reached the root
  }
  return returnValue;
}

bool RrtPlanner::pathComplete()
{
  return pathCompleteBool;
}

void RrtPlanner::nextIteration()
{
  //while(!pathCompleteBool)
  if(!pathCompleteBool)
  {
    Coord sample;
    switch(sampleGoal)
    {
      case 0 : sample = goalNode->coord;
      break;
      default :
      case 5 : sampleGoal = 0;
      case 4 :
      case 3 :
      case 2 :
      case 1 : sample = getUnusedRandomCoord();
      break;
    }

    shared_ptr<Node> closestNode = getNearestNode(sample);

    sample = getCoordInDerection(closestNode->coord, sample);  // trim the sample to the maximum distance

    if(!lineIntersectsObstacles(closestNode->coord, sample, obstacleHash, width, height))
    { // WOOOOOT this is a valid part of our tree, lets add it to the family

      myCoords.insert(sample);  // we know this coord is in our tree
      if(sample == goalNode->coord)
      {
        closestNode->children.push_back(goalNode);
        goalNode->parent = closestNode;
        pathCompleteBool = true;
      }
      else
      {
        closestNode->children.push_back(make_shared<Node>(sample, closestNode)); // make him a child of the parent
      }
    }
    else  // OH NOOOOOO the line intersects an obstacle darn, just give up
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
  //shared_ptr<Node> nearestNode = (shared_ptr<Node>)&start;
  return getNearestNode(coord, startNode, startNode, euclideanDistance(startNode->coord, coord));
}

shared_ptr<Node> RrtPlanner::getNearestNode(Coord coord, shared_ptr<Node> node, shared_ptr<Node> curClosesestNode, double curShortDist) //recursive
{
  // first initialize
  shared_ptr<Node> temp = curClosesestNode;

  // Then check if this node is closer to the point we want
  if(euclideanDistance(node->coord, coord) < curShortDist)
  {
    curShortDist = euclideanDistance(node->coord, coord);
    curClosesestNode = node;
  }

  //Then check the children if they are closer
  if(!node->children.empty()) // base case: node has no children (leaf node)
  {
    for(auto child : node->children)
    {
      temp = getNearestNode(coord, child, curClosesestNode, curShortDist);
      if(temp == curClosesestNode)  // the call to that child didn't find a closer node
      {
        continue;
      }
      else  // update the values
      {
        curClosesestNode = temp;
        curShortDist = euclideanDistance(temp->coord, coord);
      }
    }
  }

  return curClosesestNode;  // this should now be pointing to the closest node to the coordinate
}

Coord RrtPlanner::getUnusedRandomCoord()  //TODO make this actually pick a point not in the obstacles
{
  Coord potentialPoint = randomPoint(width, height);
  while(myCoords.find(potentialPoint) != myCoords.end() && (*obstacleHash)[(int)potentialPoint.y][(int)potentialPoint.x])
  {
    potentialPoint = randomPoint(width, height);
  }
  return potentialPoint;
}
