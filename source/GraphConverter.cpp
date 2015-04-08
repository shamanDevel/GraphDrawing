#include "stdafx.h"
#include <GraphConverter.h>
#include <unordered_map>

namespace shaman {

GraphConverter::GraphConverter(void)
{
}


GraphConverter::~GraphConverter(void)
{
}

ogdf::Graph GraphConverter::convert(const Graph& g)
{
	ogdf::Graph G;
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
		int u = it->m_source;
		int v = it->m_target;
		ogdf::node un = nodeMap[u];
		ogdf::node vn = nodeMap[v];
		G.newEdge(un, vn);
	}
	return G;
}

}