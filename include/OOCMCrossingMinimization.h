#pragma once

#include <CrossingMinimization.h>
#include <MILP.h>
#include <vector>
#include <utility>
#include <tuple>
#include <ostream>
#include <unordered_map>
#include <set>
#include <ogdf/basic/SList.h>
#include <ogdf/planarity/BoyerMyrvold.h>
#include <ogdf/basic/GraphCopy.h>

namespace shaman {

using namespace std;

class OOCMCrossingMinimization :
	public CrossingMinimization
{
public:
	OOCMCrossingMinimization(MILP* lp);
	virtual ~OOCMCrossingMinimization(void);

	virtual solve_result_t solve(const ogdf::GraphCopy& originalGraph, const edge_cost_t& edgeCosts);

	void enableRealizeDebugOutput(bool enabled) {
		debugRealize = enabled;
	}
	void enableKuratowskiDebugEnabled(bool enabled) {
		debugKuratowski = enabled;
	}

public: //only for unit tests

	//Describes a crossing. crossing.first < crossing.second
	typedef pair<ogdf::edge, ogdf::edge> crossing;
	struct edgehash {
	public:
		std::size_t operator()(const ogdf::edge &e) const
		{
			return (size_t) e->index();
		}
	};

	//Let e:=get<0>(crossingOrder), f:=get<1>(crossingOrder), g:=get<2>(crossingOrder)
	//A variable with the index y_e,f,g is one iff f and g cross e and the crossing 
	//(e,f) is nearaer to e's source node than crossing (e,g).
	typedef tuple<ogdf::edge, ogdf::edge, ogdf::edge> crossingOrder;

	//typedef map<crossingOrder, int> crossingOrderMap_t;
	typedef unordered_map<ogdf::edge, 
				unordered_map<ogdf::edge, 
					unordered_map<ogdf::edge, int, edgehash>,
				edgehash >,
			edgehash >
		crossingOrderMap_t; //accessed by map[e][f][g] = index

	///	\brief	Creates the variables for the ILP formulation.
	///			outCrossings is sorted
	void createVariables(const ogdf::Graph& originalG, vector<crossing>& outCrossings, vector<crossingOrder>& outCrossingOrders);

	void createCrossingOrdersMap(const vector<crossingOrder>& crossingOrders, crossingOrderMap_t& outMap);

	///	\brief	Realizes the graph
	///			The first outCrossings.size() elements of variableAssignment contain the assignment of the variables
	///			described in outCrossings. The next (and last) outCrossingOrders.size() elements contain
	///			the assignment of the variables described in outCrossingOrders.
	void realize(const ogdf::GraphCopy& originalG, ogdf::GraphCopy& G, 
		const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
		const vector<bool>& variableAssignment, unordered_map<ogdf::node, int>& crossingNodes);

	bool setObjectiveFunction(const vector<crossing>& crossings, MILP* lp, 
		const ogdf::GraphCopy& originalGraph, const edge_cost_t& edgeCosts);

	bool addCrossingNumberConstraints(const vector<crossing>& crossings, int crLower, int crUpper, MILP* lp);

	bool addLinearOrderingConstraints(const ogdf::SList<ogdf::edge>& edges, const vector<crossing>& crossings,
		const crossingOrderMap_t& crossingOrderMap, MILP* lp);

	typedef vector< ogdf::edge > kuratowski_edges_t;
	bool addKuratowkiConstraints(const ogdf::SList<ogdf::edge>& edges,
		const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
		const vector<bool>& variableAssignment, const ogdf::KuratowskiWrapper& kuratowski_edges, 
		const ogdf::GraphCopy& realizedGraph, const unordered_map<ogdf::node, int>& crossingNodes, MILP* lp);

protected:
	template<class T>
	inline
	int indexOf(const vector<T>& vec, const T& t) {
		for (unsigned int i=0; i<vec.size(); ++i) {
			if (vec[i] == t) return i;
		}
		return -1;
	}

	inline int getCrossingOrderVariable(
		const crossingOrderMap_t& map, const ogdf::edge& e, const ogdf::edge& f, const ogdf::edge& g, int nullValue)
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

	MILP* lp;
	ogdf::BoyerMyrvold boyerMyrvold;
	bool debugRealize;
	bool debugKuratowski;
};

}

namespace std {
	bool operator==(const shaman::OOCMCrossingMinimization::crossing& a,
		const shaman::OOCMCrossingMinimization::crossing& b);
	bool operator!=(const shaman::OOCMCrossingMinimization::crossing& a, 
		const shaman::OOCMCrossingMinimization::crossing& b);
	bool operator==(const shaman::OOCMCrossingMinimization::crossingOrder& a,
		const shaman::OOCMCrossingMinimization::crossingOrder& b);
	bool operator!=(const shaman::OOCMCrossingMinimization::crossingOrder& a, 
		const shaman::OOCMCrossingMinimization::crossingOrder& b);
}