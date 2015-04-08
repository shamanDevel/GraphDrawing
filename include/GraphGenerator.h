#pragma once

#include <Graph.h>
#include <cstdlib>
#include <boost/optional.hpp>

using namespace std;

namespace shaman {

class GraphGenerator
{
public:
	GraphGenerator(void);
	~GraphGenerator(void);

	boost::optional<Graph> createRandomGraph(unsigned int nodeCount, unsigned int vertexCount);

	boost::optional<Graph> createRandomPlanarGraph(unsigned int nodeCount, unsigned int vertexCount);
};

}