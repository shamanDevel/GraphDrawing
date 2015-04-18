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
	for (unsigned int i=1; i<=crossings.size() + crossingOrders.size(); ++i) {
		if (!lp->setVariableType(i, MILP::VariableType::Binary))
			return solve_result_t();
	}
	if (!setObjectiveFunction(crossings, lp))
		return solve_result_t();
	int crLower = crGLower(n, m);
	int crUpper = crKnUpper(n);
	if (!addCrossingNumberConstraints(crossings, crLower, crUpper, lp))
		return solve_result_t();
	if (!addLinearOrderingConstraints(edgeVector, crossings, crossingOrderMap, lp))
		return solve_result_t();

	//Main loop
	vector<bool> oldVariables;
	while (true) 
	{
		cout << endl;

		MILP::real* resultVariables;
		MILP::real objective;
		MILP::SolveResult result = lp->solve(&objective, &resultVariables);
		cout << "Solved, result: " << (int) result << endl;
		if (result != MILP::SolveResult::Optimal) {
			return solve_result_t();
		}
		cout << "objective: " << objective << endl;
		for (unsigned int i=0; i<variables.size(); ++i) {
			variables[i] = resultVariables[i] >= 0.999;
		}
		cout << "variables set to one:";
		for (unsigned int i=0; i<crossings.size(); ++i) {
			if (variables[i]) {
				const crossing& c = crossings[i];
				cout << " (" << c.first.first << "," << c.first.second << ")x("
					<< c.second.first << "," << c.second.second << ")";
			}
		}
		for (unsigned int i=0; i<crossingOrders.size(); ++i) {
			if (variables[i + crossings.size()]) {
				const crossingOrder& o = crossingOrders[i];
				cout << " (" << get<0>(o).first << "," << get<0>(o).second << "),("
					<< get<1>(o).first << "," << get<1>(o).second << "),("
					<< get<2>(o).first << "," << get<2>(o).second << ")";
			}
		}
		cout << endl;
		if (variables == oldVariables) {
			cout << "Nothing has changed between this loop and the last loop -> terminate" << endl;
			return solve_result_t();
		}
		oldVariables = variables;

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

			if (!addKuratowkiConstraints(edgeVector, crossings, crossingOrderMap, variables, kuratowski_edges, G, lp))
				return solve_result_t();
		}

		//if (true) {
		//	return solve_result_t();
		//}
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
			//ignore adjacent and same edges
			const edge& e = *it1;
			const edge& f = *it2;
			if (e.first == f.first || e.first == f.second
				|| e.second == f.first || e.second == f.second)
				continue;

			crossing c = minmax(*it1, *it2);
			outCrossings.push_back(c);
		}
	}
	sort(outCrossings.begin(), outCrossings.end());
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
	for (unsigned int i=0; i<crossingOrders.size(); ++i) {
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
			if (variableAssignment[j] && crossings[j].second==e 
				&& indexOf(ex, pair<edge, int>(crossings[j].first, j))==-1)
			{
				ex.push_back(make_pair(crossings[j].first, j));
			}
		}
	}

	//specify order
	for (auto& a : crossingMap) {
		edge e = a.first;
		vector< pair<edge, int> >& ex = a.second;

		if (ex.size() > 1) {
			map<crossingOrder, bool> cache;
			for (pair<edge, int> f : ex) {
				for (pair<edge, int> g : ex) {
					crossingOrder o = make_tuple(e, f.first, g.first);
					//map< crossingOrder, int >::const_iterator it = crossingOrdersMap.find(o);
					//if (it == crossingOrdersMap.end()) 
					//	continue;
					//int index = it->second + crossings.size();
					const auto& it1 = crossingOrdersMap.find(e);
					if (it1 == crossingOrdersMap.end()) continue;
					const auto& it2 = it1->second.find(f.first);
					if (it2 == it1->second.end()) continue;
					const auto& it3 = it2->second.find(g.first);
					if (it3 == it2->second.end()) continue;
					int index = it3->second + crossings.size();
					bool result = variableAssignment[index];
					cache[o] = result;
				}
			}
			s;
			sort(ex.begin(), ex.end(), [=](const pair<edge, int>& f, const pair<edge, int>& g) -> bool
			{
				crossingOrder o = make_tuple(e, f.first, g.first);
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
		vector< pair<edge, int> > ex = a.second;
		s << "edge (" << e.first << "," << e.second << ") crosses with";
		for (pair<edge, int> f : ex)
			s << " (" << f.first.first << "," << f.first.second << ")->" << f.second;
		s << endl;
	}

	while (!crossingMap.empty())
	{
		edge e = crossingMap.begin()->first;
		vector< pair<edge, int> > ex = crossingMap.begin()->second;
		int s1 = crossingMap.size();
		crossingMap.erase(e);
		assert(crossingMap.size() == s1-1);

		s << "Introduce crossing between "
			<< "(" << e.first << "," << e.second << ")"
			<< " and";
		s << endl;

		//split edges
		int u = e.first;
		for (pair<edge, int> f : ex) {
			s << " (" << f.first.first << "," << f.first.second << ")->" << f.second;
			//introduce crossing node
			int node = add_vertex(G);
			s << " -> node " << node;
			NodeData data;
			data.type = NodeType::CROSSING;
			data.variable = f.second;
			nodeProps[node] = data;
			//check where f crosses e
			vector< pair<edge, int> > fx = crossingMap[f.first];
			crossingMap.erase(f.first);
			int index = indexOf(fx, make_pair(e, f.second));
			//assert(index >= 0);
			if (index < 0) {
				s << "ERROR: edge (" << e.first << "," << e.second << ")"
					<< " not found in edge list of (" << f.first.first << "," << f.first.second << "): ";
				for (pair<edge, int> h : fx) {
					s << " (" << h.first.first << "," << h.first.second << ")";
				}
				s << endl;
				return G;
			}
			//split fx
			vector<pair<edge, int>> fx1, fx2;
			s << " fx1:";
			for (int i=0; i<index; ++i) {
				fx1.push_back(fx[i]);
				s << " (" << fx[i].first.first << "," << fx[i].first.second << ")";
				assert (crossingMap.count(fx[i].first) > 0);
				vector<pair<edge, int>>& fxx = crossingMap[fx[i].first];
				//replace(fxx.begin(), fxx.end(), f, make_pair((edge) minmax(f.first.first, node), f.second));
				for (pair<edge, int>& h : fxx) {
					if (h.first == f.first) h.first = minmax(f.first.first, node);
				}
				s << "[fxx:";
				for (pair<edge, int> h : crossingMap[fx[i].first])
					s << "(" << h.first.first << "," << h.first.second << ")";
				s << "]";
			}
			s << " fx2:";
			for (unsigned int i=index+1; i<fx.size(); ++i) {
				fx2.push_back(fx[i]);
				s << " (" << fx[i].first.first << "," << fx[i].first.second << ")";
				assert (crossingMap.count(fx[i].first) > 0);
				vector<pair<edge, int>>& fxx = crossingMap[fx[i].first];
				//replace(fxx.begin(), fxx.end(), f, make_pair((edge) minmax(f.first.second, node), f.second));
				for (pair<edge, int>& h : fxx) {
					if (h.first == f.first) h.first = minmax(f.first.second, node);
				}
				s << "[fxx:";
				for (pair<edge, int> h : crossingMap[fx[i].first])
					s << "(" << h.first.first << "," << h.first.second << ")";
				s << "]";
			}
			reverse(fx2.begin(), fx2.end());
			if (!fx1.empty()) {
				crossingMap.emplace(minmax(f.first.first, node), fx1); 
			}
			if (!fx2.empty()) {
				crossingMap.emplace(minmax(f.first.second, node), fx2);
			}
			//remove old and add new edges
			int m1 = num_edges(G);
			remove_edge(u, e.second, G);
			remove_edge(f.first.first, f.first.second, G);
			add_edge(u, node, G);
			add_edge(e.second, node, G);
			add_edge(f.first.first, node, G);
			add_edge(f.first.second, node, G);
			s << " create (" << u << "," << node << "), ("
				<< e.second << "," << node << "), ("
				<< f.first.first << "," << node << "), ("
				<< f.first.second << "," << node << ")";
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
	for (unsigned int i=0; i<crossings.size(); ++i) {
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
				map<crossing, int>::iterator it1 = crossingMap.find(minmax(e, f));
				int x_ef = -1;
				if (it1 != crossingMap.end()) x_ef = it1->second + 1;
				map<crossing, int>::iterator it2 = crossingMap.find(minmax(e, g));
				int x_eg = -1;
				if (it2 != crossingMap.end()) x_eg = it2->second + 1;
				row[1] = -1; colno[1] = index + countX + 1;
				if (x_ef > 0) {
					row[0] = 1; colno[0] = x_ef;
					if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
						return false;
				}
				if (x_eg > 0) {
					row[0] = 1; colno[0] = x_eg;
					if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
						return false;
				}
				//add order-existence constraint
				//TODO: constraint is added twice, check f<g
				if (x_ef > 0 && x_eg > 0) {
					row[0] = 1; colno[0] = index + countX + 1;
					row[1] = 1; colno[1] = crossingOrderMap.at(e).at(g).at(f) + countX + 1;
					row[2] = -1; colno[2] = x_ef;
					row[3] = -1; colno[3] = x_eg;
					if (!lp->addConstraint(4, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, -1))
						return false;
				}
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

bool OOCMCrossingMinimization::addKuratowkiConstraints(const vector<edge>& edges,
	const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
	const vector<bool>& variableAssignment, kuratowski_edges_t& kuratowski_edges, 
	const Graph& realizedGraph, MILP* lp)
{
	int crossingCount = crossings.size();

	simplifyKuratowskiSubgraph(kuratowski_edges);

	//create a mapping from the variable index to the crossings (0-based because the assignment is 0-based)
	map<int, crossing> crossingVariableMap;
	for (int i=0; i<crossingCount; ++i) {
		crossingVariableMap[i] = crossings[i];
	}

	//collect set of all nodes in the kuratowski subdivision
	map<int, int> kuratowskiNodes;
	for (edge_descriptor e : kuratowski_edges) {
		kuratowskiNodes[e.m_source]++;
		kuratowskiNodes[e.m_target]++;
	}
	//maps the nodes created by the path subdivisions to their end-nodes (called kuratowski nodes)
	map<int, pair<int, int>> kuratowskiPathToNodes;
	for (pair<int, int> p : kuratowskiNodes) {
		if (p.second != 2) continue; //a subdivision node always has degree two
		//follow the path in the edge list until a node with deg(n)>2 is reached
		const int n = p.first;
		int u = -1, v = -1;
		for (edge_descriptor e : kuratowski_edges) {
			int t;
			if (e.m_source == n)
				t = e.m_target;
			else if (e.m_target == n)
				t = e.m_source;
			else
				continue;
			int ot = n;
			while (kuratowskiNodes.at(t) == 2) {
				int t2 = -1;
				for (edge_descriptor e2 : kuratowski_edges) {
					if (e2.m_source == t && e2.m_target != ot)
						t2 = e2.m_target;
					else if (e2.m_target == t && e2.m_source != ot)
						t2 = e2.m_source;
					else
						continue;
					break;
				}
				assert (t2 >= 0);
				//next node
				ot = t;
				t = t2;
			}
			if (u == -1)
				u = t;
			else {
				v = t;
				break;
			}
		}
		assert (u>=0);
		assert (v>=0);
		kuratowskiPathToNodes[n] = minmax(u, v);
	}

	cout << "Kuratowski-Nodes:";
	for (pair<int, int> p : kuratowskiNodes) {
		cout << "  " << p.second << "x" << p.first;
	}
	cout << endl;
	cout << "  Subdivision-Nodes go to:";
	for (pair<int, pair<int, int> > p : kuratowskiPathToNodes) {
		cout << "  " << p.first << "->{" << p.second.first << "," << p.second.second << "}";
	}
	cout << endl;

	//create Zk, the set of all induced crossings whose dummy nodes are part of the kuratowski subdivision
	set<crossing> Zk;
	for (pair<int, int> p : kuratowskiNodes) {
		int node = p.first;
		NodeData data = get(node_data_t(), realizedGraph, node);
		if (data.type == NodeType::CROSSING) {
			int variable = data.variable;
			Zk.insert(crossingVariableMap.at(variable));
		}
	}
	cout << "Zk:";
	for (const crossing& c : Zk) {
		cout << " (" << c.first.first << "," << c.first.second << ")x("
			<< c.second.first << "," << c.second.second << ")";
	}
	cout << endl;

	//create Yk
	set<crossingOrder> Yk;
	for (const crossing& c1 : Zk) {
		for (const crossing& c2 : Zk) {
			edge e,f,g;
			if (c1.first == c2.first) {
				e = c1.first; f = c1.second; g = c2.second;
			} else if (c1.first == c2.second) {
				e = c1.first; f = c1.second; g = c2.first;
			} else if (c1.second == c2.first) {
				e = c1.second; f = c1.first; g = c2.second;
			} else if (c1.second == c2.second) {
				e = c1.second; f = c1.first; g = c2.first;
			} else {
				continue;
			}
			int y_efg = getCrossingOrderVariable(crossingOrderMap, e, f, g, -1);
			if (y_efg == -1) continue;
			if (!variableAssignment[y_efg + crossingCount]) continue; //y_e,f,g does not exists or is false

			//check if there is no other edge h in between
			bool existsH = false;
			for (const crossing& c3 : Zk) {
				edge h;
				if (c3.first == e) {
					h = c3.second;
				} else if (c3.second == e) {
					h = c3.first;
				} else {
					continue;
				}
				if (h==f || h==g) continue;
				int y_efh = getCrossingOrderVariable(crossingOrderMap, e, f, h, -1);
				if (y_efh == -1) continue;
				int y_ehg = getCrossingOrderVariable(crossingOrderMap, e, h, g, -1);
				if (y_ehg == -1) continue;
				if (variableAssignment[y_efh + crossingCount] && variableAssignment[y_ehg + crossingCount]) {
					existsH = true; //an edge h found that crosses between f and g
					break;
				}
			}
			if (existsH) continue;

			//triple found
			Yk.insert(crossingOrder(e, f, g));
		}
	}
	cout << "Yk:";
	for (const crossingOrder& o : Yk) {
		cout << " (" << get<0>(o).first << "," <<get<0>(o).second << "),("
			<< get<1>(o).first << "," << get<1>(o).second << "),("
			<< get<2>(o).first << "," << get<2>(o).second << ")";
	}
	cout << endl;

	//Create Xk
	set<crossing> Xk;
	for (const crossing& c : Zk) {
		const edge& e = c.first;
		const edge& f = c.second;
		for (const edge& g : edges) {
			set<crossingOrder> s;
			s.insert(crossingOrder(e, f, g));
			s.insert(crossingOrder(e, g, f));
			s.insert(crossingOrder(f, e, g));
			s.insert(crossingOrder(f, g, e));
			if (is_disjoint(s, Yk)) {
				Xk.insert(c);
			}
		}
	}
	cout << "Xk:";
	for (const crossing& c : Xk) {
		cout << " (" << c.first.first << "," << c.first.second << ")x("
			<< c.second.first << "," << c.second.second << ")";
	}
	cout << endl;

	//Create CrPairs(K)
	set<crossing> CrPairs;
	set<edge> kuratowskiEdgesInG; //the kuratowski edges in the original graph
	for (pair<int, int> p : kuratowskiNodes) {
		int n = p.first;
		NodeData data = get(node_data_t(), realizedGraph, n);
		if (data.type == NodeType::CROSSING) continue;
		//it is a node in the original graph, no walk along all outgoing edges until another original node is found
		vector<int> targetNodes;
		//set<int> visitedNodes;
		//visitedNodes.insert(n);
		//findNextNormalNodes(kuratowski_edges, n, targetNodes, visitedNodes, realizedGraph);
		for (const auto& e : kuratowski_edges) {
			int v;
			if (e.m_source == n)
				v = e.m_target;
			else if (e.m_target == n)
				v = e.m_source;
			else
				continue;
			NodeData data = get(node_data_t(), realizedGraph, v);
			if (data.type == NodeType::CROSSING) continue;
			targetNodes.push_back(v);
		}
		//add found targets
		for (int v : targetNodes)
			kuratowskiEdgesInG.insert(minmax(n, v));
	}
	cout << "Paths in K that are edges in G:";
	for (const edge& e : kuratowskiEdgesInG) {
		cout << " (" << e.first << "," << e.second << ")";
	}
	cout << endl;
	//now loop over all pairs of these edges, find non-adjacent ones
	for (const edge& e : kuratowskiEdgesInG) {
		int u1 = e.first;
		int v1 = e.second;
		set<int> s1;
		if (kuratowskiPathToNodes.count(u1)>0) {
			//subdivision-node, replace it
			pair<int, int> p = kuratowskiPathToNodes.at(u1);
			s1.insert(p.first); s1.insert(p.second);
		} else {
			s1.insert(u1);
		}
		if (kuratowskiPathToNodes.count(v1)>0) {
			//subdivision-node, replace it
			pair<int, int> p = kuratowskiPathToNodes.at(v1);
			s1.insert(p.first); s1.insert(p.second);
		} else {
			s1.insert(v1);
		}

		for (const edge& f : kuratowskiEdgesInG) {
			int u2 = f.first;
			int v2 = f.second;
			set<int> s2;
			if (kuratowskiPathToNodes.count(u2)>0) {
				//subdivision-node, replace it
				pair<int, int> p = kuratowskiPathToNodes.at(u2);
				s2.insert(p.first); s1.insert(p.second);
			} else {
				s2.insert(u2);
			}
			if (kuratowskiPathToNodes.count(v2)>0) {
				//subdivision-node, replace it
				pair<int, int> p = kuratowskiPathToNodes.at(v2);
				s2.insert(p.first); s2.insert(p.second);
			} else {
				s2.insert(v2);
			}

			if (is_disjoint(s1, s2)) {
				//add this crossing
				crossing c = minmax(e, f);
				//if (binary_search(crossings.begin(), crossings.end(), c)) {
				vector<crossing>::const_iterator it = find(crossings.begin(), crossings.end(), c);
				if (it ==  crossings.end()) continue;
				int index = it - crossings.begin();
				if (variableAssignment[index]) continue; //Already form a crossing, but not used
				//add it
				CrPairs.insert(c);
			}
		}
	}
	cout << "CrPairs:";
	for (const crossing& c : CrPairs) {
		cout << " (" << c.first.first << "," << c.first.second << ")x("
			<< c.second.first << "," << c.second.second << ")";
	}
	cout << endl;

	//Validation
	//Every variable in CrPairs must be assigned to zero
	for (const crossing& c : CrPairs) {
		int index = indexOf(crossings, c); //TODO: binary search
		assert (index >= 0);
		bool assignment = variableAssignment[index];
		assert (assignment == false);
	}
	//Every variable in Xk and Yk must be assigned to one
	for (const crossing& c : Xk) {
		int index = indexOf(crossings, c);
		assert (index >= 0);
		bool assignment = variableAssignment[index];
		assert (assignment == true);
	}
	for (const crossingOrder& o : Yk) {
		int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
		bool assignment = variableAssignment[index + crossingCount];
		assert (assignment == true);
	}

	//now setup the equation
	vector<MILP::real> row;
	vector<int> colno;
	for (const crossing& c : CrPairs) {
		int index = indexOf(crossings, c); //TODO: binary search
		if (index >= 0) {
			row.push_back(1);
			colno.push_back(index + 1);
		}
	}
	for (const crossing& c : Xk) {
		int index = indexOf(crossings, c); //TODO: binary search
		assert (index >= 0);
		int index2 = indexOf(colno, index + 1);
		if (index2 >= 0) {
			//variable already used
			//row[index2]--;
			row[index2] = -1;
		} else {
			row.push_back(-1);
			colno.push_back(index + 1);
		}
	}
	for (const crossingOrder& o : Yk) {
		int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
		row.push_back(-1);
		colno.push_back(index + crossingCount + 1);
	}
	int rhs = 1 - Xk.size() - Yk.size();

	//Debug
	cout << "new constraint:" << endl;
	for (unsigned int i=0; i<colno.size(); ++i) {
		if (row[i] > 0)
			cout << " + ";
		else if (row[i] == 0)
			cout << " + 0*";
		else
			cout << " - ";
		if (colno[i] <= crossingCount) {
			const crossing& c = crossings[colno[i]-1];
			cout << "x(" << c.first.first << "," << c.first.second << ")x("
				<< c.second.first << "," << c.second.second << ")";
		} else {
			for (const crossingOrder o : Yk) {
				int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
				if (index == colno[i] - 1 - crossingCount) {
					cout << "y(" << get<0>(o).first << "," <<get<0>(o).second << "),("
						<< get<1>(o).first << "," << get<1>(o).second << "),("
						<< get<2>(o).first << "," << get<2>(o).second << ")";
					break;
				}
			}
		}
	}
	cout << " >= " << rhs << endl;

	//send to LP
	return lp->addConstraint(colno.size(), &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, rhs);
}

void OOCMCrossingMinimization::findNextNormalNodes(
	const kuratowski_edges_t& kuratowski_edges, int startNode, vector<int>& targetNodes, 
	set<int>& visitedNodes, const Graph& realizedGraph)
{
	for (const auto& e : kuratowski_edges) {
		int v;
		if (e.m_source == startNode)
			v = e.m_target;
		else if (e.m_target == startNode)
			v = e.m_source;
		else
			continue;
		if (visitedNodes.count(v) > 0) continue; //already visited
		visitedNodes.insert(v);
		NodeData data = get(node_data_t(), realizedGraph, v);
		if (data.type != NodeType::CROSSING) {
			//we found one
			targetNodes.push_back(v);
		} else {
			//go on
			findNextNormalNodes(kuratowski_edges, v, targetNodes, visitedNodes, realizedGraph);
		}
	}
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