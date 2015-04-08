#include "stdafx.h"
#include <GraphConverter.h>
#include <unordered_map>
#include <sstream>

namespace shaman {

GraphConverter::GraphConverter(void)
{
}


GraphConverter::~GraphConverter(void)
{
}

void GraphConverter::convert(const Graph& g, ogdf::Graph& G, ogdf::GraphAttributes& GA, const Settings& settings)
{
	std::unordered_map<int, ogdf::node> nodeMap;
	//add nodes
	std::pair<vertex_iterator, vertex_iterator> nodeIter = vertices(g);
	for(vertex_iterator it = nodeIter.first; it!=nodeIter.second; ++it) {
		int index = (*it);
		ogdf::node n = G.newNode();
		nodeMap.insert(std::unordered_map<int, ogdf::node>::value_type(index, n));
		//TODO: check node type
		GA.width(n) = GA.height(n) = settings.nodeSize;

		if (settings.labelNodes) {
			std::stringstream s;
			s << (index + 1);
			ogdf::String os (s.str().c_str());
			GA.labelNode(n) = os;
			GA.colorNode(n) = settings.nodeColor;
		}
	}
	//add edges
	std::pair<edge_iterator, edge_iterator> edgeIter = edges(g);
	for (edge_iterator it = edgeIter.first; it!=edgeIter.second; ++it) {
		int u = it->m_source;
		int v = it->m_target;
		ogdf::node un = nodeMap[u];
		ogdf::node vn = nodeMap[v];
		ogdf::edge e = G.newEdge(un, vn);
		GA.edgeWidth(e) = settings.edgeWidth;
	}

}

}