#pragma once

#include <Graph.h>
#include <boost/optional.hpp>
#include <random>
#include <vector>

using namespace std;

namespace shaman {

class GraphGenerator
{
public:
	GraphGenerator(void);
	~GraphGenerator(void);

	boost::optional<Graph> createRandomGraph(unsigned int nodeCount, unsigned int edgeCount);

	boost::optional<Graph> createRandomPlanarGraph(unsigned int sqrtNodeCount, unsigned int edgeCount);

private:
	//removes a random edge without disconnecting the graph
	bool removeEdge(Graph& g, vector<pair<int, int> >& edges);

	std::default_random_engine randomEngine;
};

}