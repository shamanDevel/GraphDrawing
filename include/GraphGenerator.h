#pragma once

#include <Graph.h>
#include <boost/optional.hpp>

using namespace std;

namespace shaman {

class GraphGenerator
{
public:
	GraphGenerator(void);
	~GraphGenerator(void);

	boost::optional<Graph> createRandomGraph(unsigned int nodeCount, unsigned int edgeCount);

	boost::optional<Graph> createRandomPlanarGraph(unsigned int sqrtNodeCount, unsigned int edgeCount);
};

}