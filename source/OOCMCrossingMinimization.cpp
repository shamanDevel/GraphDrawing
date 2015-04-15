#include "stdafx.h"
#include <OOCMCrossingMinimization.h>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <algorithm>

namespace shaman {

using namespace std;

OOCMCrossingMinimization::OOCMCrossingMinimization(MILP* lp)
	: lp(lp)
{
}


OOCMCrossingMinimization::~OOCMCrossingMinimization(void)
{
}

boost::optional< pair<Graph, unsigned int> > OOCMCrossingMinimization::solve(const Graph& originalGraph)
{
	return boost::optional< pair<Graph, unsigned int> >();
}

void OOCMCrossingMinimization::createVariables(
	const Graph& originalG, vector<crossing>& outCrossings, vector<crossingOrder>& outCrossingOrders)
{
	outCrossings.clear();
	outCrossingOrders.clear();
	int m = num_edges(originalG);
	vector<edge> edgeVector;
	std::pair<edge_iterator, edge_iterator> edgeIter = edges(originalG);
	for (edge_iterator it = edgeIter.first; it!=edgeIter.second; ++it) {
		edge e = minmax(it->m_source, it->m_target);
		edgeVector.push_back(e);
	}
	//create x_{e,f} variables: all combinations of edges (unordered)
	for (vector<edge>::iterator it1 = edgeVector.begin(); it1 != edgeVector.end(); ++it1) {
		for (vector<edge>::iterator it2 = it1 + 1; it2 != edgeVector.end(); ++it2) {
			crossing c = minmax(*it1, *it2);
			outCrossings.push_back(c);
		}
	}
	//create y_(e,f,g) variables: all ordered combinations of three edges
	for (edge e : edgeVector) {
		for (edge f : edgeVector) {
			if (e==f) continue;
			for (edge g : edgeVector) {
				if (e==g || f==g) continue;
				crossingOrder o = make_tuple(e, f, g);
				outCrossingOrders.push_back(o);
			}
		}
	}
}

Graph OOCMCrossingMinimization::realize(
	const Graph& originalG, vector<crossing>& crossings, vector<crossingOrder>& crossingOrders,
	vector<bool> variableAssignment, stringstream& s)
{
	Graph G = originalG;
	boost::property_map<Graph, node_data_t>::type nodeProps = get(node_data_t(), G);
	
	int crossingsSize = crossings.size();
	int io = 0;
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].first;

		//collect all crossings with this edge
		vector<edge> ex;
		int j = i;
		for (; j<crossingsSize && crossings[j].first==e; ++j) {
			if (variableAssignment[j])
				ex.push_back(crossings[j].second);
		}
		i = j;

		//specify order
		if (ex.size() > 1) {
			map< pair<edge, edge>, int > orderMap;
			for (; get<0>(crossingOrders[io])!=e; ++io);
			for (; io<crossingOrders.size() && get<0>(crossingOrders[io])==e; ++io) {
				orderMap[make_pair(get<1>(crossingOrders[io]), get<2>(crossingOrders[io]))] = io + crossingsSize;
			}
			sort(ex.begin(), ex.end(), [=](const edge& f, const edge& g)
			{
				map< pair<edge,edge>, int >::const_iterator it = orderMap.find(make_pair(f, g));
				if (it == orderMap.end()) 
					cerr << "Not found" << endl;
				int index = it->second;
				bool result = variableAssignment[index];
				return result;
			});
		}

		s << "Introduce crossing between "
			<< "(" << e.first << "," << e.second << ")"
			<< " and";
		for (edge f : ex) {
			s << " (" << f.first << "," << f.second << ")";
		}
		s << endl;

		//split edges
		int u = e.first;
		for (edge f : ex) {
			//introduce crossing node
			int node = add_vertex(G);
			NodeData data;
			data.type = NodeType::CROSSING;
			data.variable = i;
			nodeProps[node] = data;
			int m1 = num_edges(G);
			remove_edge(u, e.second, G);
			remove_edge(f.first, f.second, G);
			add_edge(u, node, G);
			add_edge(e.second, node, G);
			add_edge(f.first, node, G);
			add_edge(f.second, node, G);
			int m2 = num_edges(G);
			if (m1+2 != m2) {
				s << "unable to remove all edges" << endl;
				//throw "Unable to remove all edges";
			}
			u = node;
		}
	}

	return G;
}

}