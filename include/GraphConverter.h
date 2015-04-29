#pragma once

#include <ogdf_include.h>
#include <utility>
#include <GraphConverter.h>
#include <unordered_map>
#include <sstream>

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
	static void convert(const Graph_t& g, ogdf::Graph& G)
	{
		typedef boost::graph_traits<Graph_t>::vertex_iterator vertex_iterator;
		typedef boost::graph_traits<Graph_t>::edge_iterator edge_iterator;

		std::unordered_map<int, ogdf::node> nodeMap;
		//add nodes
		std::pair<vertex_iterator, vertex_iterator> nodeIter = vertices(g);
		for(vertex_iterator it = nodeIter.first; it!=nodeIter.second; ++it) {
			int index = (*it);
			ogdf::node n = G.newNode();
			nodeMap.insert(std::unordered_map<int, ogdf::node>::value_type(index, n));
		}
		//add edges
		std::pair<edge_iterator, edge_iterator> edgeIter = edges(g);
		for (edge_iterator it = edgeIter.first; it!=edgeIter.second; ++it) {
			int u = min(it->m_source, it->m_target);
			int v = max(it->m_source, it->m_target);
			ogdf::node un = nodeMap[u];
			ogdf::node vn = nodeMap[v];
			ogdf::edge e = G.newEdge(un, vn);
		}

	}
	//template<class Graph_t>
	//static void convert(const Graph_t& g, ogdf::Graph& outG);
};

}