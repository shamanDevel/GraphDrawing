#pragma once

#include <Graph.h>
#include <CrossingMinimization.h>
#include <MILP.h>
#include <vector>
#include <utility>
#include <tuple>
#include <sstream>

namespace shaman {

using namespace std;

class OOCMCrossingMinimization :
	public CrossingMinimization
{
public:
	OOCMCrossingMinimization(MILP* lp);
	virtual ~OOCMCrossingMinimization(void);

	virtual boost::optional< pair<Graph, unsigned int> > solve(const Graph& originalGraph);

	//Describes an edge. edge.first < edge.second
	typedef pair<int, int> edge;
	//Describes a crossing. crossing.first < crossing.second
	typedef pair<edge, edge> crossing;
	//Let e:=get<0>(crossingOrder), f:=get<1>(crossingOrder), g:=get<2>(crossingOrder)
	//A variable with the index y_e,f,g is one iff f and g cross e and the crossing 
	//(e,f) is nearaer to e's source node than crossing (e,g).
	typedef tuple<edge, edge, edge> crossingOrder;

	///	\brief	Creates the variables for the ILP formulation
	void createVariables(const Graph& originalG, vector<crossing>& outCrossings, vector<crossingOrder>& outCrossingOrders);

	///	\brief	Realizes the graph
	///			The first outCrossings.size() elements of variableAssignment contain the assignment of the variables
	///			described in outCrossings. The next (and last) outCrossingOrders.size() elements contain
	///			the assignment of the variables described in outCrossingOrders.
	Graph realize(const Graph& originalG, vector<crossing>& crossings, vector<crossingOrder>& crossingOrders,
		vector<bool> variableAssignment, stringstream& s);

private:
	MILP* lp;
};

}