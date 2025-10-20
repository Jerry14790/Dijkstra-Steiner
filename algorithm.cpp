#include <iostream>
#include "algorithm.h"

using namespace Grid; //should I put using namespace here?

namespace Algorithm {


Coordinate dijkstra_steiner(std::vector<Terminal> const& terminals)
{

  GridGraph grid = GridGraph(terminals);
  return grid.RunDijkstraSteiner();
  
}

}
