#include "stdafx.h"
#include <SOCMCrossingMinimization.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <set>
#include <map>
#include <unordered_map>
#include <cassert>
#include <boost/graph/boyer_myrvold_planar_test.hpp>

using namespace std;

namespace shaman {

SOCMCrossingMinimization::SOCMCrossingMinimization(MILP* lp)
	: lp(lp)
{
}


SOCMCrossingMinimization::~SOCMCrossingMinimization(void)
{
}

void SOCMCrossingMinimization::splitGraph(Graph& g)
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
		int u = min(edge.first, edge.second);
		int v = max(edge.first, edge.second);
		//remove edge
		remove_edge(u, v, g);
		//create nodes
		for (int i=0; i<l-1; ++i) {
			int index = add_vertex(g);
			NodeData data;
			data.type = NodeType::SUBDIVISION;
			data.pathU = u;
			data.pathV = v;
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

vector<SOCMCrossingMinimization::crossingInfo> SOCMCrossingMinimization::createVariables(const Graph& g)
{
	vector<crossingInfo> info;
	//iterate through every E x E
	std::pair<edge_iterator, edge_iterator> edgeIter = edges(g);
	for (edge_iterator it1 = edgeIter.first; it1!=edgeIter.second; ++it1) {
		int u1Orig = min(it1->m_source, it1->m_target);
		int v1Orig = max(it1->m_source, it1->m_target);
		int u1 = u1Orig;
		int v1 = v1Orig;
		NodeData u1data = get(node_data_t(), g, u1);
		NodeData v1data = get(node_data_t(), g, v1);
		if (u1data.type == NodeType::SUBDIVISION) {
			u1 = u1data.pathU;
			v1 = u1data.pathV;
		} else if (v1data.type == NodeType::SUBDIVISION) {
			u1 = v1data.pathU;
			v1 = v1data.pathV;
		}
		for (edge_iterator it2 = it1; it2!=edgeIter.second; ++it2) {
			int u2Orig = min(it2->m_source, it2->m_target);
			int v2Orig = max(it2->m_source, it2->m_target);
			int u2 = u2Orig;
			int v2 = v2Orig;
			NodeData u2data = get(node_data_t(), g, u2);
			NodeData v2data = get(node_data_t(), g, v2);
			if (u2data.type == NodeType::SUBDIVISION) {
				u2 = u2data.pathU;
				v2 = u2data.pathV;
			} else if (v2data.type == NodeType::SUBDIVISION) {
				u2 = v2data.pathU;
				v2 = v2data.pathV;
			}
			//create a variable for the edge pair e and f if
			// - e != f
			// - e and f are not adjacent
			//all edges that are created during subdivision are threaten as being the same edge
			if (u1==u2 || u1==v2 || v1==u2 || v1==v2)
				continue;

			//add edge
			if (u1Orig < u2Orig) {
				info.push_back(crossingInfo(edge(u1Orig,v1Orig), edge(u2Orig,v2Orig)));
			} else if (u1Orig > u2Orig) {
				info.push_back(crossingInfo(edge(u2Orig,v2Orig), edge(u1Orig,v1Orig)));
			} else if (v1Orig < v2Orig) {
				info.push_back(crossingInfo(edge(u1Orig,v1Orig), edge(u2Orig,v2Orig)));
			} else {
				info.push_back(crossingInfo(edge(u2Orig,v2Orig), edge(u1Orig,v1Orig)));
			}
		}
	}

	return info;
}

Graph SOCMCrossingMinimization::realize(const Graph& originalG, 
									const vector<crossingInfo>& variableInfo, const vector<bool>& variables)
{
	//clone graph
	Graph G = originalG;
	boost::property_map<Graph, node_data_t>::type nodeProps = get(node_data_t(), G);
	//iterate through all variables and introduce crossing nodes
	for (int i=0; i<variables.size(); ++i) {
		if (variables[i]) {
			edge e = variableInfo[i].first;
			edge f = variableInfo[i].second;
			//introduce crossing node
			int node = add_vertex(G);
			NodeData data;
			data.type = NodeType::CROSSING;
			data.variable = i;
			nodeProps[node] = data;
			int m1 = num_edges(G);
			remove_edge(e.first, e.second, G);
			remove_edge(f.first, f.second, G);
			add_edge(e.first, node, G);
			add_edge(e.second, node, G);
			add_edge(f.first, node, G);
			add_edge(f.second, node, G);
			int m2 = num_edges(G);
			if (m1+2 != m2) {
				cerr << "unable to remove all edges" << endl;
			}
		}
	}

	return G;
}

boost::optional< std::pair<Graph, unsigned int> > SOCMCrossingMinimization::solve(const Graph& originalG)
{
	//return solveBacktracking(originalG);
	return solveLp(originalG);
}

boost::optional< pair<Graph, unsigned int> > SOCMCrossingMinimization::solveLp(const Graph& originalG)
{
	//pre-test: check if the graph is not already planar
	if (boost::boyer_myrvold_planarity_test(originalG)) {
		//already planar
		return boost::optional <pair<Graph, unsigned int> >(pair<Graph, unsigned int>(originalG, 0));
	}

	int n = num_vertices(originalG);
	int m = num_edges(originalG);

	//create variables
	vector<crossingInfo> variableInfos = createVariables(originalG);
	unsigned int varCount = variableInfos.size();
	map<crossingInfo, int> variableMap; //maps the edge crossing to variable index
	map<edge, set<edge> > crossingEdges; //maps each edge to the edges it can cross
	set<edge> edges;
	for (int i=0; i<varCount; ++i) {
		edge e = variableInfos[i].first;
		edge f = variableInfos[i].second;
		variableMap.emplace(crossingInfo(e, f), i+1);
		variableMap.emplace(crossingInfo(f, e), i+1);
		crossingEdges[e].insert(f);
		crossingEdges[f].insert(e);
		edges.insert(e);
		edges.insert(f);
	}
	//debug
	cout << "crossing edges contains:" << endl;
	for (pair<edge, set<edge>> i : crossingEdges)
	{
		std::cout << "(" << i.first.first << "," << i.first.second << ") =>";
		for (edge j : i.second)
			std::cout << " (" << j.first << "," << j.second << ")";
		std::cout << endl;
	}
	cout << "variable map contains:" << endl;
	for (pair< crossingInfo, int > vi : variableMap)
	{
		cout << "(" << vi.first.first.first << "," << vi.first.first.second << ") x (";
		cout << vi.first.second.first << "," << vi.first.second.second << ") => ";
		cout << vi.second << endl;
	}

	//initialize LP
	if (!lp->initialize(varCount)) {
		return boost::optional< pair<Graph, unsigned int> >();
	}
	for (int i=1; i<=varCount; ++i) {
		if (!lp->setVariableType(i, MILP::VariableType::Binary)) {
			return boost::optional< pair<Graph, unsigned int> >();
		}
	}
	vector<bool> variables(varCount);
	vector<int> completeColno(varCount);
	vector<MILP::real> completeRow(varCount);
	for (int i=0; i<varCount; ++i) completeColno[i] = i+1;

	//set objective function
	fill(completeRow.begin(), completeRow.end(), 1);
	if (!lp->setObjectiveFunction(varCount, &completeRow[0], &completeColno[0], MILP::Direction::Minimize)) {
		return boost::optional< pair<Graph, unsigned int> >();
	}

	if (!lp->setAddConstraintMode(true)) {
		return boost::optional< pair<Graph, unsigned int> >();
	}

	//set upper and lower bound
	int crLower = 0;//crGLower(n, m);
	int crUpper = crKnUpper(n);
	if (!lp->addConstraint(varCount, &completeRow[0], &completeColno[0], MILP::ConstraintType::GreaterThanEqual, crLower)
		|| !lp->addConstraint(varCount, &completeRow[0], &completeColno[0], MILP::ConstraintType::LessThanEqual, crUpper)) {
		return boost::optional< pair<Graph, unsigned int> >();
	}

	//set constraints to ensure at most one crossing per edge
	if (!addSimpleEdgeCrossingConstraints(crossingEdges, variableMap, lp))
		return boost::optional< pair<Graph, unsigned int> >();

	if (!lp->setAddConstraintMode(false)) {
		return boost::optional< pair<Graph, unsigned int> >();
	}

	//debug
	//lp->printDebug();

	//run the loop
	while(true)
	{
		//solve the LP-model
		cout << "Solving LP-model ... ";
		MILP::real* resultVariables;
		MILP::real objective;
		MILP::SolveResult result = lp->solve(&objective, &resultVariables);
		cout << "Solved, result: " << (int) result << endl;
		if (result != MILP::SolveResult::Optimal) {
			lp->printDebug();
			return boost::optional< pair<Graph, unsigned int> >();
		}
		cout << "objective: " << objective << endl;
		cout << "variables:";
		for (int i=0; i<varCount; ++i) {
			variables[i] = resultVariables[i] == 1;
			cout << " " << resultVariables[i];
		}
		cout << endl;
		for (int i=0; i<varCount; ++i) {
			if (variables[i]) {
				edge e = variableInfos[i].first;
				edge f = variableInfos[i].second;
				cout << "introduce crossing between (" << e.first << "," << e.second
				<< ") and (" << f.first << "," << f.second << ")" << endl;
			}
		}

		//realize graph
		Graph G = realize(originalG, variableInfos, variables);
		// Initialize the interior edge index
		boost::property_map<Graph, boost::edge_index_t>::type e_index = get(boost::edge_index, G);
		boost::graph_traits<Graph>::edges_size_type edge_count = 0;
		boost::graph_traits<Graph>::edge_iterator ei, ei_end;
		for(boost::tie(ei, ei_end) = boost::edges(G); ei != ei_end; ++ei) {
			put(e_index, *ei, edge_count);
			edge_count++;
		}

		//check if the graph is now planar
		kuratowski_edges_t kuratowski_edges;
		if (boost::boyer_myrvold_planarity_test(
			boost::boyer_myrvold_params::graph = G,
			boost::boyer_myrvold_params::kuratowski_subgraph = back_inserter(kuratowski_edges))) 
		{
			//graph is planar
			return boost::optional <pair<Graph, unsigned int> >(pair<Graph, unsigned int>(G, (unsigned int) objective));
		}
		else
		{
			cout << "Kuratowki-Subgraph:";
			for (auto e : kuratowski_edges) {
				cout << " (" << e.m_source << "," << e.m_target << ")" ;
			}
			cout << "  count=" << kuratowski_edges.size();
			cout << endl;
			//clear
			int countOfVisitedCrossingNodes = 
				simplifyKuratowskiSubgraph(G, originalG, kuratowski_edges);
			cout << "after cleaning:";
			for (auto e : kuratowski_edges) {
				cout << " (" << e.m_source << "," << e.m_target << ")" ;
			}
			cout << "  count=" << kuratowski_edges.size();
			cout << endl;

			if (!addKuratowskiConstraints(originalG, G, crossingEdges, variableMap, variableInfos, kuratowski_edges, lp))
				return boost::optional< pair<Graph, unsigned int> >();

			//lp->printDebug();

			
		}


	}
}

bool SOCMCrossingMinimization::addSimpleEdgeCrossingConstraints(
	const map<edge, set<edge> >& crossingEdges, map<crossingInfo, int>& variableMap, MILP* lp)
{
	vector<int> oneCrossingColno;
	vector<MILP::real> oneCrossingRow;
	for (pair<edge, set<edge> > i : crossingEdges)
	{
		edge e = i.first;
		oneCrossingColno.clear();
		oneCrossingRow.clear();
		for (edge f : i.second) {
			oneCrossingColno.push_back(variableMap[crossingInfo(e, f)]);
			oneCrossingRow.push_back(1);
		}
		if (!lp->addConstraint(oneCrossingColno.size(), &oneCrossingRow[0], &oneCrossingColno[0], MILP::LessThanEqual, 1)) {
			return false;
		}
		cout << "edge (" << e.first << "," << e.second << ") can cross with " << oneCrossingColno.size() << " other edges" << endl;
	}
	return true;
}

bool SOCMCrossingMinimization::addKuratowskiConstraints(
		const Graph& originalG, const Graph& G, const map<edge, set<edge> >& crossingEdges, 
		const map<crossingInfo, int>& variableMap, const vector<crossingInfo>& variableInfos, 
		const kuratowski_edges_t& kuratowski_edges, MILP* lp)
{
	int m = num_edges(originalG);
	vector<int> kuratowskiColno;
	vector<MILP::real> kuratowskiRow;
	set<int> kuratowskiVars;
	set<int> kuratowskiNodes;
	int countOfVisitedCrossingNodes = 0;
	//set<crossingInfo> H;
	//set<crossingInfo> D;

	//H.clear();
	//D.clear();

	////extract kuratowski constraint
	//for (auto e : kuratowski_edges) {
	//	for (auto f : kuratowski_edges) {
	//		addCrossingToSet(H, crossingInfo(edge(e.m_source, e.m_target), edge(f.m_source, f.m_target)));
	//	}
	//}
	//for (int i=0; i<varCount; ++i) {
	//	if (variables[i]) {
	//		addCrossingToSet(D, variableInfos[i]);
	//	}
	//}
	//cout << "H^2: "; printCrossingSet(H); cout << endl;
	//cout << "D': "; printCrossingSet(D); cout << endl;
	//vector<crossingInfo> H_diff_D;
	//set_difference(H.begin(), H.end(), D.begin(), D.end(), back_inserter(H_diff_D));
	//vector<crossingInfo> H_cut_D;
	//set_intersection(H.begin(), H.end(), D.begin(), D.end(), back_inserter(H_cut_D));
	//cout << "H^2 \\ D': "; printCrossingSet(H_diff_D); cout << endl;
	//cout << "H^2 cut D': "; printCrossingSet(H_cut_D); cout << endl;

	////add additional constraints on the edges in the subgraph to exclude D
	//kuratowkiVars.clear();
	//for (crossingInfo i : H_diff_D) {
	//	map<crossingInfo, int>::iterator it = variableMap.find(i);
	//	if (it != variableMap.end()) {
	//		kuratowkiVars.insert(it->second);
	//	} else {
	//		crossingInfo i2 = crossingInfo(edge(i.first.first, i.second.second),
	//			edge(i.second.first, i.first.second) );
	//		it = variableMap.find(i2);
	//		if (it != variableMap.end()) {
	//			kuratowkiVars.insert(it->second);
	//		}
	//	}
	//}
	//cout << "constraints: (+)";
	//for (int var : kuratowkiVars) {
	//	kuratowskiColno.push_back(var);
	//	kuratowskiRow.push_back(1);
	//	crossingInfo info = variableInfos[var - 1];
	//	cout << " (" << info.first.first << "," << info.first.second
	//		<< ")x(" << info.second.first << "," << info.second.second << ")";
	//}
	//cout << "  (-)";
	//for (crossingInfo i : H_cut_D) {
	//	kuratowskiColno.push_back(variableMap[i]);
	//	kuratowskiRow.push_back(-1);
	//	cout << " (" << i.first.first << "," << i.first.second
	//		<< ")x(" << i.second.first << "," << i.second.second << ")";
	//}
	//cout << endl;
	//lp->addConstraint(kuratowskiColno.size(), &kuratowskiRow[0], &kuratowskiColno[0], 
	//	MILP::ConstraintType::GreaterThanEqual, 1 - H_cut_D.size());

	//add additional constraints on the edges in the subgraph -> force one to 1
	for (auto e : kuratowski_edges) {
		if (e.m_source < m) kuratowskiNodes.insert(e.m_source);
		if (e.m_target < m) kuratowskiNodes.insert(e.m_target);
	}
	cout << "Kuratowski nodes:";
	for (int n : kuratowskiNodes) {
		cout << " " << n;
	}
	cout << endl;

	kuratowskiVars.clear();
	for (int u1 : kuratowskiNodes) {
		for (int v1 : kuratowskiNodes) {
			edge e (min(u1, v1), max(u1, v1));
			if (crossingEdges.count(e) == 0) continue;
			for (int u2 : kuratowskiNodes) {
				for (int v2 : kuratowskiNodes) {
					edge f (min(u2, v2), max(u2, v2));
					if (crossingEdges.count(f) == 0) continue;
					crossingInfo i (e, f);
					map<crossingInfo, int>::const_iterator it = variableMap.find(i);
					if (it != variableMap.end()) {
						kuratowskiVars.insert(it->second);
					}
				}
			}
		}
	}
	//for (auto e : kuratowski_edges) {
	//	for (auto f : kuratowski_edges) {
	//		int u,v,n; //u:left, v:right, n:middle
	//		if (e.m_source == f.m_source) {
	//			u = e.m_target; v = f.m_target;
	//			n = e.m_source;
	//		} else if (e.m_source == f.m_target) {
	//			u = e.m_target; v = f.m_source;
	//			n = e.m_source;
	//		} else if (e.m_target == f.m_source) {
	//			u = e.m_source; v = f.m_target;
	//			n = e.m_target;
	//		} else { //e.m_target == f.m_target
	//			u = e.m_source; v = f.m_source;
	//			n = e.m_target;
	//		}
	//		//check if the middle node is a crossing node
	//		NodeData data = get(node_data_t(), G, n);
	//		if (data.type == NodeType::CROSSING) { 
	//			//it is a crossing node, add crossing variable directly
	//			kuratowsiVars.insert(data.variable + 1);
	//		} else {
	//			//a normal node
	//			crossingInfo i = crossingInfo(edge(e.m_source, e.m_target), edge(f.m_source, f.m_target));
	//			map<crossingInfo, int>::const_iterator it = variableMap.find(i);
	//			if (it != variableMap.end()) {
	//				kuratowskiVars.insert(it->second);
	//			}
	//		}
	//	}
	//}
	cout << "Add constraint on crossings";
	for (int var : kuratowskiVars) {
		kuratowskiColno.push_back(var);
		kuratowskiRow.push_back(1);
		crossingInfo info = variableInfos[var - 1];
		cout << " (" << info.first.first << "," << info.first.second
			<< ")x(" << info.second.first << "," << info.second.second << ")";
	}
	cout << endl;
	bool result = lp->addConstraint(kuratowskiColno.size(), &kuratowskiRow[0], &kuratowskiColno[0], 
		MILP::ConstraintType::GreaterThanEqual, 1 + countOfVisitedCrossingNodes);

	return result;
}

void SOCMCrossingMinimization::addCrossingToSet(set<crossingInfo>& set, crossingInfo crossing)
{
	int u1 = min(crossing.first.first, crossing.first.second);
	int v1 = max(crossing.first.first, crossing.first.second);
	int u2 = min(crossing.second.first, crossing.second.second);
	int v2 = max(crossing.second.first, crossing.second.second);
	if (u1<u2) {
		set.insert(crossingInfo(edge(u1,v1), edge(u2,v2)));
	} else if (u1>u2) {
		set.insert(crossingInfo(edge(u2,v2), edge(u1,v1)));
	} else if (v1<v2) {
		set.insert(crossingInfo(edge(u1,v1), edge(u2,v2)));
	} else {
		set.insert(crossingInfo(edge(u2,v2), edge(u1,v1)));
	}
}

void SOCMCrossingMinimization::printCrossingSet(const set<crossingInfo>& set)
{
	for (const crossingInfo& v : set) {
		cout << "{{" << v.first.first << "," << v.first.second << "};{"
			<< v.second.first << "," << v.second.second << "} ";
	}
}
void SOCMCrossingMinimization::printCrossingSet(const vector<crossingInfo>& set)
{
	for (const crossingInfo& v : set) {
		cout << "{{" << v.first.first << "," << v.first.second << "};{"
			<< v.second.first << "," << v.second.second << "} ";
	}
}

bool SOCMCrossingMinimization::areNodesAdjacent(const Graph& G, int u, int v) {
	boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
	for(boost::tie(ei, ei_end) = boost::out_edges(u, G); ei != ei_end; ++ei) {
		if (ei->m_source == v || ei->m_target == v) {
			return true;
		}
	}
	return false;
}

int SOCMCrossingMinimization::simplifyKuratowskiSubgraph(const Graph& G, const Graph& originalG,
													  kuratowski_edges_t& kuratowski_edges)
{
	int countOfVisitedCrossingNodes = 0;
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
	//check if some edges are introduced because of crossing-nodes
	//for (int i=0; i<kuratowski_edges.size(); ++i) {
	//	edge_descriptor e = kuratowski_edges[i];
	//	edge_descriptor f = kuratowski_edges[(i+1) % kuratowski_edges.size()];
	//	int u,v,n; //u:left, v:right, n:middle
	//	if (e.m_source == f.m_source) {
	//		u = e.m_target; v = f.m_target;
	//		n = e.m_source;
	//	} else if (e.m_source == f.m_target) {
	//		u = e.m_target; v = f.m_source;
	//		n = e.m_source;
	//	} else if (e.m_target == f.m_source) {
	//		u = e.m_source; v = f.m_target;
	//		n = e.m_target;
	//	} else { //e.m_target == f.m_target
	//		u = e.m_source; v = f.m_source;
	//		n = e.m_target;
	//	}
	//	//check if the middle node is a crossing node
	//	NodeData data = get(node_data_t(), G, n);
	//	if (data.type == NodeType::CROSSING) {
	//		//check if (u,v) is in the graph
	//		if (areNodesAdjacent(originalG, u, v)) {
	//			//replace the edge
	//			e.m_source = u;
	//			e.m_target = v;
	//			kuratowski_edges[i] = e;
	//			//delete next element (i+1)
	//			kuratowski_edges.erase(kuratowski_edges.begin() + ((i+1)%kuratowski_edges.size()) );

	//			countOfVisitedCrossingNodes++;
	//		}
	//	}
	//}
	return countOfVisitedCrossingNodes;
}

boost::optional< pair<Graph, unsigned int> > SOCMCrossingMinimization::solveBacktracking(const Graph& originalG)
{
	//pre-test: check if the graph is not already planar
	if (boost::boyer_myrvold_planarity_test(originalG)) {
		//already planar
		return boost::optional <pair<Graph, unsigned int> >(pair<Graph, unsigned int>(originalG, 0));
	}

	int n = num_vertices(originalG);
	int m = num_edges(originalG);

	//create variables
	vector<crossingInfo> variableInfos = createVariables(originalG);
	unsigned int varCount = variableInfos.size();
	vector<bool> variableAssignment(varCount);
	for (int i=0; i<varCount; ++i) variableAssignment[i] = false;

	int crLower = crGLower(n, m);
	int crUpper = crKnUpper(n);

	//setup backtracking cache
	set<edge> usedEdges;

	//run backtracking
	return solveBacktrackingRec(originalG, variableInfos, variableAssignment, 0, usedEdges, crLower, crUpper, 0);
}

boost::optional< pair<Graph, unsigned int> > SOCMCrossingMinimization::solveBacktrackingRec(
		const Graph& originalG, vector<crossingInfo>& variableInfo, vector<bool>& variableAssignment,
		int startVariable, set<edge> usedEdges, int crLower, int crUpper, int numCrossings ) 
{
	if (numCrossings >= crUpper) {
		//reached upper crossing number -> break, not optimal
		return boost::optional< pair<Graph, unsigned int> >();
	}

	for (int i=startVariable; i<variableInfo.size(); ++i)
	{
		crossingInfo& crossing = variableInfo[i];
		if (usedEdges.count(crossing.first) == 0 && usedEdges.count(crossing.second) == 0) {
			//try this edge
			variableAssignment[i] = true;

			Graph G = realize(originalG, variableInfo, variableAssignment);
			if (boost::boyer_myrvold_planarity_test(G)) {
				//we found it
				return boost::optional< pair<Graph, unsigned int> > (make_pair(G, numCrossings + 1));
			}

			//reserve edge
			usedEdges.insert(crossing.first);
			usedEdges.insert(crossing.second);

			//do recursion
			boost::optional< pair<Graph, unsigned int> > result
				= solveBacktrackingRec(originalG, variableInfo, variableAssignment, i+1, usedEdges, 
					crLower, crUpper, numCrossings + 1);
			if (result) {
				//found in recursion
				return result;
			}

			//undo changes
			variableAssignment[i] = false;
			usedEdges.erase(crossing.first);
			usedEdges.erase(crossing.second);
		}
	}

	//not found
	return boost::optional< pair<Graph, unsigned int> >();
}

}