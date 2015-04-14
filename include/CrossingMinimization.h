#pragma once

#include <math.h>
#include <Graph.h>
#include <utility>
#include <boost/optional.hpp>
#include <set>
#include <MILP.h>

using namespace std;

namespace shaman {

/// Performs the crossing minimization to the optimum.
/// Contains also some helper functions
class CrossingMinimization
{
public:
	CrossingMinimization(void);
	~CrossingMinimization(void);

	// Known bounds for the crossing number

	// Returns the upper bound of the crossing number of the complete graph Kn.
	static int crKnUpper(int n) {
		return (int) ceil( (n/2) * ((n-1)/2) * ((n-2)/2) * ((n-3)/2) / 4.0);
	}

	// Returns a lower bound of the crossing number of an arbitrary (simple and connected) graph
	// with n nodes and m edges
	static int crGLower(int n, int m) {
		int cr = 0;
		if (n>2) {
			cr = max(cr, m - 3*n + 6); //Euler's formula
		}
		if (m >= 4*n) {
			cr = max(cr, m*m*m/(n*n*100)); //Leighton
		}
		cr = max(cr, (int) floor(m*m*m/(n*n*33.75) - 0.9*n)); //Pach and Tóth
		return cr;
	}

	///	\brief	Replaces every edge of g with a path of length |E|.
	///			This is needed to enshure that a simpe drawing is also the optimum in a drawing where
	///			edges can cross multiple times.
	///
	///	\param g	The graph (input and output)
	void splitGraph(Graph& g);

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
	vector<crossingInfo> createVariables(const Graph& g);

	///	\brief	Realizes the graph from the specified ILP solution.
	///			Every zero-one variable that is set to true creates a crossing in the associated edge-edge pair.
	///			IMPORTANT: The crossings must be simple, one edge must cross at most one other edge (not multiple edges)!
	///
	///	\param originalG	The original graph
	///	\param variableInfo	The infos for the variables, returned by createVariables(const Graph& g)
	///	\param variables	The result of the ILP
	///
	///	\return	The created graph
	Graph realize(const Graph& originalG, const vector<crossingInfo>& variableInfo, const vector<bool>& variables);

	///	\brief	Solves the crossing problem to optimum.
	///			Input: the original graph, simple and connected
	///			Output: The optional contains the graph with inserted crossing nodes and is now planar and the crossing number.
	///				If the problem could not be solved (too many crossing for a simple crossing), the optional is empty.
	///
	///	\param	originalG	The original graph
	///	\param	lp			The mixed-integer-linear-program solver implementation
	///	\return	The resulting graph with crossing nodes and the crossing number, or an empty optional
	boost::optional< pair<Graph, unsigned int> > solve(const Graph& originalG, MILP* lp);

	boost::optional< pair<Graph, unsigned int> > solveLp(const Graph& originalG, MILP* lp);

	boost::optional< pair<Graph, unsigned int> > solveBacktracking(const Graph& originalG);

private:
	typedef vector< edge_descriptor > kuratowski_edges_t;
	int simplifyKuratowskiSubgraph(const Graph& G, const Graph& originalG, kuratowski_edges_t& kuratowski_edges);
	bool areNodesAdjacent(const Graph& G, int u, int v);
	void addCrossingToSet(set<crossingInfo>& set, crossingInfo crossing);
	void printCrossingSet(const set<crossingInfo>& set);
	void printCrossingSet(const vector<crossingInfo>& set);
	boost::optional< pair<Graph, unsigned int> > solveBacktrackingRec(
		const Graph& originalG, vector<crossingInfo>& variableInfo, vector<bool>& variableAssignment,
		int startVariable, set<edge> usedEdges, int crLower, int crUpper, int numCrossings );
};

}