#include "stdafx.h"
#include <CrossingMinimization.h>
#include <vector>

using namespace std;

namespace shaman {

CrossingMinimization::CrossingMinimization(void)
{
}


CrossingMinimization::~CrossingMinimization(void)
{
}

void CrossingMinimization::splitGraph(Graph& g)
{
	boost::property_map<Graph, node_data_t>::type nodeProps = get(node_data_t(), g);
	int l = num_edges(g);
	//collect edges
	vector<pair<int, int> > edgeVector;
	edgeVector.reserve(l);
	std::pair<edge_iterator, edge_iterator> edgeIter = edges(g);
	for (edge_iterator it = edgeIter.first; it!=edgeIter.second; ++it) {
		int u = it->m_source;
		int v = it->m_target;
		edgeVector.push_back(make_pair(u, v));
	}
	//remove every edge and replace with a path of length l
	int nodeIndex = num_vertices(g);
	for (pair<int, int> edge : edgeVector) {
		int u = edge.first;
		int v = edge.second;
		//remove edge
		remove_edge(u, v, g);
		//create nodes
		for (int i=0; i<l-1; ++i) {
			int index = add_vertex(g);
			NodeData data;
			data.type = NodeType::SUBDIVISION;
			nodeProps[index] = data;
		}
		//create edges
		add_edge(u, nodeIndex, g);
		add_edge(nodeIndex + l-2, v, g);
		for (int i=0; i<l-2; ++i) {
			add_edge(nodeIndex+i, nodeIndex+i+1, g);
		}
		nodeIndex += l-1;
	}
}

}