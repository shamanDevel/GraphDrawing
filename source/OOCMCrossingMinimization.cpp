#include "stdafx.h"
#include <OOCMCrossingMinimization.h>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include "boost/iostreams/stream.hpp"
#include "boost/iostreams/device/null.hpp"
#include <algorithm>
#include <limits>

namespace shaman {

using namespace std;

OOCMCrossingMinimization::OOCMCrossingMinimization(MILP* lp)
	: lp(lp)
{
}


OOCMCrossingMinimization::~OOCMCrossingMinimization(void)
{
}

OOCMCrossingMinimization::solve_result_t OOCMCrossingMinimization::solve(const Graph& originalGraph)
{
	//pre-check for planarity
	if (boost::boyer_myrvold_planarity_test(originalGraph)) {
		return solve_result_t(make_pair(originalGraph, 0));
	}
	int n = num_vertices(originalGraph);
	int m = num_edges(originalGraph);
	boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );

	//create variables
	vector<edge> edgeVector;
	std::pair<edge_iterator, edge_iterator> edgeIter = edges(originalGraph);
	for (edge_iterator it = edgeIter.first; it!=edgeIter.second; ++it) {
		edge e = minmax(it->m_source, it->m_target);
		edgeVector.push_back(e);
	}
	vector<crossing> crossings;
	vector<crossingOrder> crossingOrders;
	crossingOrderMap_t crossingOrderMap;
	createVariables(originalGraph, crossings, crossingOrders);
	createCrossingOrdersMap(crossingOrders, crossingOrderMap);
	vector<bool> variables (crossings.size() + crossingOrders.size());

	//setup lp model
	if (!lp->initialize(crossings.size() + crossingOrders.size()))
		return solve_result_t();
	if (!setObjectiveFunction(crossings, lp))
		return solve_result_t();
	int crLower = crGLower(n, m);
	int crUpper = crKnUpper(n);
	if (!addCrossingNumberConstraints(crossings, crLower, crUpper, lp))
		return solve_result_t();
	if (!addLinearOrderingConstraints(edgeVector, crossings, crossingOrderMap, lp))
		return solve_result_t();

	while (true) 
	{
		MILP::real* resultVariables;
		MILP::real objective;
		MILP::SolveResult result = lp->solve(&objective, &resultVariables);
		cout << "Solved, result: " << (int) result << endl;
		if (result != MILP::SolveResult::Optimal) {
			lp->printDebug();
			return solve_result_t();
		}
		cout << "objective: " << objective << endl;
		cout << "variables:";
		for (int i=0; i<variables.size(); ++i) {
			variables[i] = resultVariables[i] >= 0.999;
			cout << " " << resultVariables[i];
		}
		cout << endl;
		for (int i=0; i<variables.size(); ++i) {
			if (variables[i]) {
				edge e = crossings[i].first;
				edge f = crossings[i].second;
				cout << "introduce crossing between (" << e.first << "," << e.second
				<< ") and (" << f.first << "," << f.second << ")" << endl;
			}
		}

		//realize graph
		Graph G = realize(originalGraph, crossings, crossingOrderMap, variables, nullOstream);

		// Initialize the interior edge index
		boost::property_map<Graph, boost::edge_index_t>::type e_index = get(boost::edge_index, G);
		boost::graph_traits<Graph>::edges_size_type edge_count = 0;
		boost::graph_traits<Graph>::edge_iterator ei, ei_end;
		for(boost::tie(ei, ei_end) = boost::edges(G); ei != ei_end; ++ei) {
			put(e_index, *ei, edge_count);
			edge_count++;
		}
		//check if the graph is now planar
		typedef vector< edge_descriptor > kuratowski_edges_t;
		kuratowski_edges_t kuratowski_edges;
		if (boost::boyer_myrvold_planarity_test(
			boost::boyer_myrvold_params::graph = G,
			boost::boyer_myrvold_params::kuratowski_subgraph = back_inserter(kuratowski_edges))) 
		{
			//graph is planar
			return solve_result_t(make_pair(G, (unsigned int) objective));
		}
		else
		{
			cout << "Kuratowki-Subgraph:";
			for (auto e : kuratowski_edges) {
				cout << " (" << e.m_source << "," << e.m_target << ")" ;
			}
			cout << "  count=" << kuratowski_edges.size();
			cout << endl;

			if (!addKuratowkiConstraints(crossings, crossingOrderMap, variables, kuratowski_edges, lp))
				return solve_result_t();
		}

	}

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
	const vector<crossingOrder>& crossingOrders, crossingOrderMap_t& outMap)
{
	outMap.clear();
	for (int i=0; i<crossingOrders.size(); ++i) {
		//outMap.emplace(crossingOrders[i], i);
		const crossingOrder& o = crossingOrders[i];
		outMap[get<0>(o)][get<1>(o)][get<2>(o)] = i;
	}
}

Graph OOCMCrossingMinimization::realize(
	const Graph& originalG, const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrdersMap,
	const vector<bool>& variableAssignment, ostream& s)
{
	Graph G = originalG;
	boost::property_map<Graph, node_data_t>::type nodeProps = get(node_data_t(), G);
	
	int crossingsSize = crossings.size();
	int io = 0;
	//maps the source edges to the crossing edges with the inducing variable
	map< edge, vector< pair<edge, int> > > crossingMap;
	//Improve performance of the crossingMap creation
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].first;

		//collect all crossings with this edge
		vector< pair<edge, int> >& ex = crossingMap[e];
		int j = i;
		for (; j<crossingsSize && crossings[j].first==e; ++j) {
			if (variableAssignment[j])
				ex.push_back(make_pair(crossings[j].second, j));
		}
		i = j;
	}
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].second;

		//collect all crossings with this edge
		vector< pair<edge, int> >& ex = crossingMap[e];
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
					//map< crossingOrder, int >::const_iterator it = crossingOrdersMap.find(o);
					//if (it == crossingOrdersMap.end()) 
					//	continue;
					//int index = it->second + crossings.size();
					const auto& it1 = crossingOrdersMap.find(e);
					if (it1 == crossingOrdersMap.end()) continue;
					const auto& it2 = it1->second.find(f);
					if (it2 == it1->second.end()) continue;
					const auto& it3 = it2->second.find(g);
					if (it3 == it2->second.end()) continue;
					int index = it3->second + crossings.size();
					bool result = variableAssignment[index];
					cache[o] = result;
				}
			}
			s;
			sort(ex.begin(), ex.end(), [=](const edge& f, const edge& g) -> bool
			{
				crossingOrder o = make_tuple(e, f, g);
				map<crossingOrder, bool>::const_iterator it = cache.find(o);
				if (it == cache.end())
					return false;
				else
					return it->second;
			});
			s;
		}

	}

	for (const auto& a : crossingMap) {
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

bool OOCMCrossingMinimization::setObjectiveFunction(const vector<crossing>& crossings, MILP* lp)
{
	vector<MILP::real> row (crossings.size());
	vector<int> colno (crossings.size());
	for (int i=0; i<crossings.size(); ++i) {
		row[i] = 1; //TODO: edge weight (from preprocessing)
		colno[i] = i+1;
	}
	bool result = lp->setObjectiveFunction(crossings.size(), &row[0], &colno[0], MILP::Direction::Minimize);
	return result;
}

bool OOCMCrossingMinimization::addCrossingNumberConstraints(
	const vector<crossing>& crossings, int crLower, int crUpper, MILP* lp)
{
	int count = crossings.size();
	vector<MILP::real> row (count);
	vector<int> colno (count);
	for (int i=0; i<count; ++i) {
		row[i] = 1;
		colno[i] = i+1;
	}
	return lp->addConstraint(count, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, crLower)
		&& lp->addConstraint(count, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, crUpper);
}

bool OOCMCrossingMinimization::addLinearOrderingConstraints(
	const vector<edge>& edges, const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap, MILP* lp)
{
	//This method is very slow (~15sec for the K8)
	//Improve it

	int countX = crossings.size();
	int countY = crossingOrderMap.size();
	
	if (!lp->setAddConstraintMode(true)) 
		return false;

	map<crossing, int> crossingMap;
	for (int i=0; i<countX; ++i)
		crossingMap[crossings[i]] = i;

	vector<MILP::real> row(4);
	vector<int> colno(4);
	for (const auto& m1 : crossingOrderMap) {
		const edge& e = m1.first;
		for (const auto& m2 : m1.second) {
			const edge& f = m2.first;
			for (const auto& m3 : m2.second) {
				const edge& g = m3.first;
				int index = m3.second;
				//add crossing-existence constraint
				row[0] = 1; colno[0] = crossingMap.at(minmax(e, f)) + 1;
				row[1] = -1; colno[1] = index + countX + 1;
				if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
					return false;
				colno[0] = crossingMap.at(minmax(e, g)) + 1;
				if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
					return false;
				//add order-existence constraint
				//TODO: constraint is added twice, check f<g
				row[0] = 1; colno[0] = index + countX + 1;
				row[1] = 1; colno[1] = crossingOrderMap.at(e).at(g).at(f) + countX + 1;
				row[2] = -1; colno[2] = crossingMap.at(minmax(e, f)) + 1;
				row[3] = -1; colno[3] = crossingMap.at(minmax(e, g)) + 1;
				if (!lp->addConstraint(4, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, -1))
					return false;
				//add mirror-order constraint
				//TODO: constraint is added twice, check f<g
				row[0] = 1; colno[0] = index + countX + 1;
				row[1] = 1; colno[1] = crossingOrderMap.at(e).at(g).at(f) + countX + 1;
				if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, 1))
				return false;
			}
		}
	}

	//add cyclic-order-constraint
	for (const edge& e : edges) {
		const auto& me = crossingOrderMap.at(e);
		for (const auto& m1 : me) {
			const edge& f = m1.first;
			for (const auto& m2 : m1.second) {
				const edge& g = m2.first;
				const int i1 = m2.second;
				for (edge h : edges) {
					if (e!=h && h>f && h>g) {
						const auto& it1 = me.find(g);
						if (it1 == me.end()) continue;
						const auto& it2 = it1->second.find(h);
						if (it2 == it1->second.end()) continue;
						const int i2 = it2->second;
						const auto& it3 = me.find(h);
						if (it3 == me.end()) continue;
						const auto& it4 = it3->second.find(f);
						if (it4 == it3->second.end()) continue;
						const int i3 = it4->second;

						row[0] = 1; colno[0] = i1 + countX + 1;
						row[1] = 1; colno[1] = i2 + countX + 1;
						row[2] = 1; colno[2] = i3 + countX + 1;
						if (!lp->addConstraint(3, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, 2))
							return false;
					}
				}
			}
		}
	}
	//	for (const edge& f : edges) {
	//		if (e==f) continue;
	//		for (const edge& g : edges) {
	//			if (e==g || f==g) continue;
	//			for (const edge& h : edges) {
	//				if (e==h || f==h || g==h) continue;

	//				int i1, i2, i3;
	//				crossingOrderMap_t::const_iterator it;
	//				it = crossingOrderMap.find(crossingOrder(e, f, g));
	//				if (it == crossingOrderMap.end()) continue;
	//				i1 = it->second + countX + 1;
	//				it = crossingOrderMap.find(crossingOrder(e, g, h));
	//				if (it == crossingOrderMap.end()) continue;
	//				i2 = it->second + countX + 1;
	//				it = crossingOrderMap.find(crossingOrder(e, h, f));
	//				if (it == crossingOrderMap.end()) continue;
	//				i3 = it->second + countX + 1;

	//				row[0] = 1; colno[0] = i1;
	//				row[1] = 1; colno[1] = i2;
	//				row[2] = 1; colno[2] = i3;
	//				/*if (!lp->addConstraint(3, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, 2))
	//					return false;*/
	//			}
	//		}
	//	}
	//}

	if (!lp->setAddConstraintMode(false))
		return false;

	return true;
}

bool OOCMCrossingMinimization::addKuratowkiConstraints(
	const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
	const vector<bool>& variableAssignment, kuratowski_edges_t& kuratowski_edges, MILP* lp)
{
	simplifyKuratowskiSubgraph(kuratowski_edges);

	//TODO

	return true;
}

void OOCMCrossingMinimization::simplifyKuratowskiSubgraph(
	kuratowski_edges_t& kuratowski_edges)
{
	int size = kuratowski_edges.size();
	//removes all redundant edges from the kuratowski subgraph
	unordered_map<int, int> nodeOccurence;
	//add node occurence
	for (auto e : kuratowski_edges) {
		nodeOccurence[e.m_source]++;
		nodeOccurence[e.m_target]++;
	}
	//loop as long as nodes with an occurence of 1 are there
	while(true)
	{
		bool found = false;
		for(unordered_map<int, int>::value_type entry : nodeOccurence) {
			if (entry.second == 1) {
				//remove the edge going out from this node
				remove_if(kuratowski_edges.begin(), kuratowski_edges.end(),
					[=](boost::graph_traits<Graph>::edge_descriptor e) mutable -> bool {
						if (e.m_source == entry.first) {
							nodeOccurence[e.m_target]--;
							return true;
						} else if (e.m_target == entry.first) {
							nodeOccurence[e.m_source]--;
							return true;
						} else {
							return false;
						}
				});
				nodeOccurence.erase(entry.first);
				size--;
				found = true;
				break;
			}
		}
		if (!found) break;
	}
	kuratowski_edges.resize(size);
}

}