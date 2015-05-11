#pragma once

#include <ogdf_include.h>
#include <ogdf/basic/GraphCopy.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace shaman {

using namespace std;
using namespace ogdf;

///	\brief	Simplifies a graph by removing nodes of degree 1 and 2.
///			The removal of degree 2 nodes can lead to multi-edges.
///			Therefore, this class also returns a map that maps the edges to their edge weight/cost.
///
///			It is assumend that the input graph is simple, connected and non-planar.
///			The simplified graph will also be simple, connected and non-planar.
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
	void mergeDeg2Nodes();
	void unmergeDeg2Nodes(GraphCopy& C) const;
	void followDeg2Path(node last, node current, vector<node>& nodes);

	//Deprecated
	edge newEdge(GraphCopy& C, edge origE, node copyU, node copyV) const;

	struct nodePairHash {
	public:
		std::size_t operator()(const pair<node, node> &nodes) const
		{
			size_t hash = nodes.first->index();
			__asm rol hash,16
			hash |= nodes.second->index();
			return hash;
		}
	};

	unordered_map<node, vector< pair<node, edge> > > deg1Nodes;
	//unordered_multimap<pair<node, node>, vector<edge>, nodePairHash > deg2Edges;
	//unordered_multimap<edge, vector<edge> > deg2Edges;
	//unordered_set<edge> deg2Case3Edges;
	//unordered_multimap<node, vector<edge> > deg2Circles;

	//Describes a path. The first vector contains the inner nodes, 
	//the second vector the edges between the nodes (including the first and last edge).
	//Therefore, the vector<edge> has one element more than the vector<node>.
	//All nodes and edges refer to the original graph
	typedef pair< vector<node>, vector<edge> > path_t;
	//Describes a simplification induced by a node of degree 2
	struct Deg2Info {
		node sourceOriginal;
		node targetOriginal;
		vector< path_t > paths;
		//for case 2 and 3:
		edge edgeOriginal;
		bool deleteEdge;
	};
	vector<Deg2Info> deg2Infos;

	void reverseCircle(GraphCopy& C, const Deg2Info& info) const;
	void reversePath(GraphCopy& C, const Deg2Info& info) const;
};

}