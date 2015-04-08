#pragma once

#include <Graph.h>
#include <ogdf_include.h>

namespace shaman {

// Converts the Boost-Graph to the OGDF-Graph
class GraphConverter
{
private:
	GraphConverter(void);
	~GraphConverter(void);

public:
	// Converts the Boost-Graph as defined as a typedef in Graph.h to an OGDF-Graph
	static ogdf::Graph convert(const Graph& g);
};

}