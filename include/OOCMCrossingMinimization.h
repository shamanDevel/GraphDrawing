#pragma once

#include <Graph.h>
#include <CrossingMinimization.h>
#include <MILP.h>
#include <vector>
#include <utility>
#include <tuple>
#include <ostream>
#include <unordered_map>

namespace shaman {

using namespace std;

class OOCMCrossingMinimization :
	public CrossingMinimization
{
public:
	OOCMCrossingMinimization(MILP* lp);
	virtual ~OOCMCrossingMinimization(void);

	virtual solve_result_t solve(const Graph& originalGraph);

public: //only for unit tests

	//Describes an edge. edge.first < edge.second
	typedef pair<int, int> edge;
	//Describes a crossing. crossing.first < crossing.second
	typedef pair<edge, edge> crossing;
	struct edgehash {
	public:
		std::size_t operator()(const edge &e) const
		{
			int u = e.first;
			int v = e.second;
			__asm rol u, 32
			return u ^ v;
		}
	};

	//Let e:=get<0>(crossingOrder), f:=get<1>(crossingOrder), g:=get<2>(crossingOrder)
	//A variable with the index y_e,f,g is one iff f and g cross e and the crossing 
	//(e,f) is nearaer to e's source node than crossing (e,g).
	typedef tuple<edge, edge, edge> crossingOrder;

	//typedef map<crossingOrder, int> crossingOrderMap_t;
	typedef unordered_map<edge, 
				unordered_map<edge, 
					unordered_map<edge, int, edgehash>,
				edgehash >,
			edgehash >
		crossingOrderMap_t; //accessed by map[e][f][g] = index

	///	\brief	Creates the variables for the ILP formulation
	void createVariables(const Graph& originalG, vector<crossing>& outCrossings, vector<crossingOrder>& outCrossingOrders);

	void createCrossingOrdersMap(const vector<crossingOrder>& crossingOrders, crossingOrderMap_t& outMap);

	///	\brief	Realizes the graph
	///			The first outCrossings.size() elements of variableAssignment contain the assignment of the variables
	///			described in outCrossings. The next (and last) outCrossingOrders.size() elements contain
	///			the assignment of the variables described in outCrossingOrders.
	Graph realize(const Graph& originalG, const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
		const vector<bool>& variableAssignment, ostream& s);

	bool setObjectiveFunction(const vector<crossing>& crossings, MILP* lp);

	bool addCrossingNumberConstraints(const vector<crossing>& crossings, int crLower, int crUpper, MILP* lp);

	bool addLinearOrderingConstraints(const vector<edge>& edges, const vector<crossing>& crossings,
		const crossingOrderMap_t& crossingOrderMap, MILP* lp);

	typedef vector< edge_descriptor > kuratowski_edges_t;
	bool addKuratowkiConstraints(const vector<edge>& edges,
		const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
		const vector<bool>& variableAssignment, kuratowski_edges_t& kuratowski_edges, 
		const Graph& realizedGraph, MILP* lp);

private:
	template<class T>
	inline
	int indexOf(const vector<T>& vec, const T& t) {
		for (unsigned int i=0; i<vec.size(); ++i) {
			if (vec[i] == t) return i;
		}
		return -1;
	}

	inline int getCrossingOrderVariable(
		const crossingOrderMap_t& map, const edge& e, const edge& f, const edge& g, int nullValue)
	{
		const auto& it1 = map.find(e);
		if (it1 == map.end()) return nullValue;
		const auto& it2 = it1->second.find(f);
		if (it2 == it1->second.end()) return nullValue;
		const auto& it3 = it2->second.find(g);
		if (it3 == it2->second.end()) return nullValue;
		return it3->second;
	}

	template<class Set1, class Set2> 
	bool is_disjoint(const Set1 &set1, const Set2 &set2)
	{
		if(set1.empty() || set2.empty()) return true;

		typename Set1::const_iterator 
			it1 = set1.begin(), 
			it1End = set1.end();
		typename Set2::const_iterator 
			it2 = set2.begin(), 
			it2End = set2.end();

		if(*it1 > *set2.rbegin() || *it2 > *set1.rbegin()) return true;

		while(it1 != it1End && it2 != it2End)
		{
			if(*it1 == *it2) return false;
			if(*it1 < *it2) { it1++; }
			else { it2++; }
		}

		return true;
	}

	void simplifyKuratowskiSubgraph(kuratowski_edges_t& kuratowski_edges);
	void findNextNormalNodes(const kuratowski_edges_t& kuratowski_edges, int startNode, 
		vector<int>& targetNodes, set<int>& visitedNodes, const Graph& realizedGraph);

	MILP* lp;
};

}