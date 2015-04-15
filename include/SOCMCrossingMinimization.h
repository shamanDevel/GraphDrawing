#pragma once

#include <math.h>
#include <Graph.h>
#include <utility>
#include <boost/optional.hpp>
#include <set>
#include <MILP.h>
#include <CrossingMinimization.h>

using namespace std;

namespace shaman {

/// Performs the crossing minimization to the optimum.
/// Contains also some helper functions
class SOCMCrossingMinimization : public CrossingMinimization
{
public:
	SOCMCrossingMinimization(MILP* lp);
	virtual ~SOCMCrossingMinimization(void);

	///	\brief	Replaces every edge of g with a path of length |E|.
	///			This is needed to enshure that a simpe drawing is also the optimum in a drawing where
	///			edges can cross multiple times.
	///
	///	\param g	The graph (input and output)
	virtual void splitGraph(Graph& g);

	typedef pair<int, int> edge;
	typedef pair<edge, edge> crossingInfo;

	///	\brief	Creates the variables for the ILP model.
	///			The returned vector contains all possible edge-edge crossings.
	///			The ILP model then creates a zero-one variable for each entry of the vector.
	///			After solving the ILP, this vector and the resulting vector of the zero-one variables is then
	///			passed back to realize(...) to create the new graph with edge crossings.
	///
	///			If the result is ((u1,v1),(u2,v2)), then the following equations are valid:
	///				u1 < v1 and u2 < v2
	///				u1 <= u2
	///				if u1 == u2 then v1 < v2
	///				
	///
	///	\param g	The initial graph
	///
	///	\return		A vector with an entry for every zero-one variable.
	virtual vector<crossingInfo> createVariables(const Graph& g);

	///	\brief	Realizes the graph from the specified ILP solution.
	///			Every zero-one variable that is set to true creates a crossing in the associated edge-edge pair.
	///			IMPORTANT: The crossings must be simple, one edge must cross at most one other edge (not multiple edges)!
	///
	///	\param originalG	The original graph
	///	\param variableInfo	The infos for the variables, returned by createVariables(const Graph& g)
	///	\param variables	The result of the ILP
	///
	///	\return	The created graph
	virtual Graph realize(const Graph& originalG, const vector<crossingInfo>& variableInfo, const vector<bool>& variables);

	///	\brief	Solves the crossing problem to optimum.
	///			Input: the original graph, simple and connected
	///			Output: The optional contains the graph with inserted crossing nodes and is now planar and the crossing number.
	///				If the problem could not be solved (too many crossing for a simple crossing), the optional is empty.
	///
	///	\param	originalG	The original graph
	///	\return	The resulting graph with crossing nodes and the crossing number, or an empty optional
	virtual boost::optional< std::pair<Graph, unsigned int> > solve(const Graph& originalGraph);

	virtual boost::optional< pair<Graph, unsigned int> > solveLp(const Graph& originalG);

	virtual boost::optional< pair<Graph, unsigned int> > solveBacktracking(const Graph& originalG);

private:
	typedef vector< edge_descriptor > kuratowski_edges_t;
	bool addSimpleEdgeCrossingConstraints(
		const map<edge, set<edge> >& crossingEdges, map<crossingInfo, int>& variableMap, MILP* lp);
	bool addKuratowskiConstraints(
		const Graph& originalG, const Graph& G, const map<edge, set<edge> >& crossingEdges, 
		const map<crossingInfo, int>& variableMap, const vector<crossingInfo>& variableInfos, 
		const kuratowski_edges_t& kuratowski_edges, MILP* lp);
	int simplifyKuratowskiSubgraph(const Graph& G, const Graph& originalG, kuratowski_edges_t& kuratowski_edges);
	bool areNodesAdjacent(const Graph& G, int u, int v);
	void addCrossingToSet(set<crossingInfo>& set, crossingInfo crossing);
	void printCrossingSet(const set<crossingInfo>& set);
	void printCrossingSet(const vector<crossingInfo>& set);
	boost::optional< pair<Graph, unsigned int> > solveBacktrackingRec(
		const Graph& originalG, vector<crossingInfo>& variableInfo, vector<bool>& variableAssignment,
		int startVariable, set<edge> usedEdges, int crLower, int crUpper, int numCrossings );

	MILP* lp;
};

}