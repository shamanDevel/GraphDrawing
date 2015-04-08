#include "stdafx.h"
#include <GraphGenerator.h>
#include <cstdlib>
#include <cmath>

namespace shaman {

GraphGenerator::GraphGenerator(void)
{
}


GraphGenerator::~GraphGenerator(void)
{
}

boost::optional<Graph> GraphGenerator::createRandomGraph(unsigned int n, unsigned int e)
{
	if (e < n-1) {
		//too less edges (less than a straight line)
		return boost::optional<Graph>();
	}
	if (e > n*(n-1)/2) {
		//too many edges (more than the complete graph)
		return boost::optional<Graph>();
	}
	//create random graph
	Graph g;
	return boost::optional<Graph>(g);
}

boost::optional<Graph> GraphGenerator::createRandomPlanarGraph(unsigned int s, unsigned int e)
{
	unsigned int nodeCount = s*s;
	if (e < nodeCount - 1) {
		//too less edges (less than a straight line)
		return boost::optional<Graph>();
	}
	if (e > s*s*3) {
		//too many edges (more than the complete graph)
		return boost::optional<Graph>();
	}
	//create random planar graph
	Graph g;
	return boost::optional<Graph>(g);
}

}