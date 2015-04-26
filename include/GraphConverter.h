#pragma once

#include <ogdf_include.h>
#include <utility>

namespace shaman {

// Converts the Boost-Graph to the OGDF-Graph
class GraphConverter
{
private:
	GraphConverter(void);
	~GraphConverter(void);

public:

	///	\brief	Converts the Boost-Graph as defined as a typedef in Graph.h to an OGDF-Graph
	///	\param g		The input Boost-Graph (should be an adjacency list)
	///	\param outG		The output OGDF-Graph that is filled
	template<class Graph_t>
	static void convert(const Graph_t& g, ogdf::Graph& outG);
};

}