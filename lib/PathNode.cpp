#include "PathNode.hpp"

int PathNode::getCost(std::shared_ptr<PathNode> root) {
  if(parent == root)
  {
    return 1;
  }
  else return parent->getCost(root) + 1;
}
