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

void OOCMCrossingMinimization::createCrossingOrdersMap(
	const vector<crossingOrder>& crossingOrders, map<crossingOrder, int>& outMap)
{
	outMap.clear();
	for (int i=0; i<crossingOrders.size(); ++i) {
		outMap.emplace(crossingOrders[i], i);
	}
}

Graph OOCMCrossingMinimization::realize(
	const Graph& originalG, vector<crossing>& crossings, map<crossingOrder, int>& crossingOrdersMap,
	vector<bool> variableAssignment, ostream& s)
{
	Graph G = originalG;
	boost::property_map<Graph, node_data_t>::type nodeProps = get(node_data_t(), G);
	
	int crossingsSize = crossings.size();
	int io = 0;
	map< edge, vector<edge> > crossingMap;
	//Improve performance of the crossingMap creation
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].first;

		//collect all crossings with this edge
		vector<edge>& ex = crossingMap[e];
		int j = i;
		for (; j<crossingsSize && crossings[j].first==e; ++j) {
			if (variableAssignment[j])
				ex.push_back(crossings[j].second);
		}
		i = j;
	}
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].second;

		//collect all crossings with this edge
		vector<edge>& ex = crossingMap[e];
		for (int j = 0; j<crossingsSize; ++j) {
			if (variableAssignment[j] && crossings[j].second==e && indexOf(ex, crossings[j].first)==-1)
				ex.push_back(crossings[j].first);
		}
	}

	//specify order
	for (auto& a : crossingMap) {
		edge e = a.first;
		vector<edge>& ex = a.second;

		if (ex.size() > 1) {
			map<crossingOrder, bool> cache;
			for (edge f : ex) {
				for (edge g : ex) {
					crossingOrder o = make_tuple(e, f, g);
					map< crossingOrder, int >::const_iterator it = crossingOrdersMap.find(o);
					if (it == crossingOrdersMap.end()) 
						continue;
					int index = it->second + crossings.size();
					bool result = variableAssignment[index];
					cache[o] = result;
				}
			}
			s;
			sort(ex.begin(), ex.end(), [=](const edge& f, const edge& g) -> bool
			{
				crossingOrder o = make_tuple(e, f, g);
				bool result = cache.find(o)->second;
				return result;
			});
			s;
		}

	}

	for (auto a : crossingMap) {
		edge e = a.first;
		vector<edge> ex = a.second;
		s << "edge (" << e.first << "," << e.second << ") crosses with";
		for (edge f : ex)
			s << " (" << f.first << "," << f.second << ")";
		s << endl;
	}

	while (!crossingMap.empty())
	{
		edge e = crossingMap.begin()->first;
		vector<edge> ex = crossingMap.begin()->second;
		int s1 = crossingMap.size();
		crossingMap.erase(e);
		assert(crossingMap.size() == s1-1);

		s << "Introduce crossing between "
			<< "(" << e.first << "," << e.second << ")"
			<< " and";
		s << endl;

		//split edges
		int u = e.first;
		for (edge f : ex) {
			s << " (" << f.first << "," << f.second << ")";
			//introduce crossing node
			int node = add_vertex(G);
			s << " -> node " << node;
			NodeData data;
			data.type = NodeType::CROSSING;
			data.variable = -1;
			nodeProps[node] = data;
			//check where f crosses e
			vector<edge> fx = crossingMap[f];
			crossingMap.erase(f);
			int index = indexOf(fx, e);
			//assert(index >= 0);
			if (index < 0) {
				s << "ERROR: edge (" << e.first << "," << e.second << ")"
					<< " not found in edge list of (" << f.first << "," << f.second << "): ";
				for (edge h : fx) {
					s << " (" << h.first << "," << h.second << ")";
				}
				s << endl;
				return G;
			}
			//split fx
			vector<edge> fx1, fx2;
			s << " fx1:";
			for (int i=0; i<index; ++i) {
				fx1.push_back(fx[i]);
				s << " (" << fx[i].first << "," << fx[i].second << ")";
				assert (crossingMap.count(fx[i]) > 0);
				vector<edge>& fxx = crossingMap[fx[i]];
				replace(fxx.begin(), fxx.end(), f, (edge) minmax(f.first, node));
				s << "[fxx:";
				for (edge h : crossingMap[fx[i]])
					s << "(" << h.first << "," << h.second << ")";
				s << "]";
			}
			s << " fx2:";
			for (int i=index+1; i<fx.size(); ++i) {
				fx2.push_back(fx[i]);
				s << " (" << fx[i].first << "," << fx[i].second << ")";
				assert (crossingMap.count(fx[i]) > 0);
				vector<edge>& fxx = crossingMap[fx[i]];
				replace(fxx.begin(), fxx.end(), f, (edge) minmax(f.second, node));
				s << "[fxx:";
				for (edge h : crossingMap[fx[i]])
					s << "(" << h.first << "," << h.second << ")";
				s << "]";
			}
			reverse(fx2.begin(), fx2.end());
			if (!fx1.empty()) {
				crossingMap.emplace(minmax(f.first, node), fx1); 
			}
			if (!fx2.empty()) {
				crossingMap.emplace(minmax(f.second, node), fx2);
			}
			//remove old and add new edges
			int m1 = num_edges(G);
			remove_edge(u, e.second, G);
			remove_edge(f.first, f.second, G);
			add_edge(u, node, G);
			add_edge(e.second, node, G);
			add_edge(f.first, node, G);
			add_edge(f.second, node, G);
			s << " create (" << u << "," << node << "), ("
				<< e.second << "," << node << "), ("
				<< f.first << "," << node << "), ("
				<< f.second << "," << node << ")";
			int m2 = num_edges(G);
			if (m1+2 != m2) {
				s << "unable to remove all edges" << endl;
				//throw "Unable to remove all edges";
			}
			u = node;
			s << endl;
		}
	}

	return G;
}

}