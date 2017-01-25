#pragma once

#include "planning-utils/Node.hpp"
#include "planning-utils/geom/Coord.hpp"

class PathNode: public Node {
public:
  int getCost(std::shared_ptr<PathNode> root);

  std::shared_ptr<PathNode> parent;
	std::list<std::shared_ptr<PathNode>> children;

	PathNode(Coord coord, std::shared_ptr<PathNode> parent) {
		this->coord = coord;
		this->parent = parent;
	}
protected:

};
