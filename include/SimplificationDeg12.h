#pragma once

#include <ogdf_include.h>
#include <ogdf/basic/GraphCopy.h>
#include <unordered_map>
#include <vector>

namespace shaman {

using namespace std;
using namespace ogdf;

///	\brief	Simplifies a graph by removing nodes of degree 1 and 2.
///			The removal of degree 2 nodes can lead to multi-edges.
///			Therefore, this class also returns a map that maps the edges to their edge weight/cost.
///
///			By contract, every preprocessing step is only allowed to remove nodes and edges.
///			The final Minimization Step then only adds crossings. 
///			Therefore, a regular node (no crossing node) can always be identified over its original node.
class SimplificationDeg12
{
public:
	///	\brief	Creates a new instance that simplifies the given graph
	SimplificationDeg12(const GraphCopy& originalGraph);
	~SimplificationDeg12(void);

	///	\brief	Returns the simplified graph
	///			This graph is then passed to the CrossingMinimization.
	const GraphCopy& getSimplifiedGraph() const;

	///	\brief	Returns edge costs associated with every edge of the simplified graph.
	///			These costs are used in the objective function of the lp model
	const unordered_map<edge, int>& getEdgeCosts() const;

	///	\brief	Reverts the simplification
	///	\param modifiedCopy	A copy of the GraphCopy returned by getSimplifiedGraph(),
	///						modified by the CrossingMinimization (dummy nodes are introduced)
	///	\return	The original graph again with the dummy nodes at edge crossings.
	GraphCopy reverseSimplification(const GraphCopy& modifiedCopy) const;

private:
	GraphCopy simplifiedGraph;
	unordered_map<edge, int> edgeCosts;

	void removeDeg1Nodes();
	void reverseDeg1Nodes(GraphCopy& C) const;

	unordered_map<node, vector<node> > deg1Nodes;
};

}