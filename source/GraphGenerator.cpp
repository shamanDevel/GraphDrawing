#include "stdafx.h"
#include <GraphGenerator.h>

namespace shaman {

GraphGenerator::GraphGenerator(void)
{
}


GraphGenerator::~GraphGenerator(void)
{
}

boost::optional<Graph> GraphGenerator::createRandomGraph(unsigned int nodeCount, unsigned int vertexCount)
{
	return boost::optional<Graph>();
}

boost::optional<Graph> GraphGenerator::createRandomPlanarGraph(unsigned int nodeCount, unsigned int vertexCount)
{
	return boost::optional<Graph>();
}

}