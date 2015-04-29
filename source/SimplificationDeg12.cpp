#include "stdafx.h"
#include <SimplificationDeg12.h>
#include <queue>
#include <cassert>

namespace shaman {

using namespace std;
using namespace ogdf;

SimplificationDeg12::SimplificationDeg12(const GraphCopy& originalGraph)
	: simplifiedGraph(originalGraph)
{
	//simplify graph
	removeDeg1Nodes();
}
SimplificationDeg12::~SimplificationDeg12(void)
{
}

void SimplificationDeg12::removeDeg1Nodes()
{
	queue<node> queue;
	node v;
	forall_nodes(v,simplifiedGraph) {
		if (v->degree() == 1)
			queue.push(v);
	}
	while (!queue.empty()) {
		node v = queue.front(); queue.pop();
		assert (v->degree() == 1);
		node u = v->firstAdj()->twinNode();
		//remove node v and the edge (u,v)
		deg1Nodes[simplifiedGraph.original(u)].push_back(simplifiedGraph.original(v));
		simplifiedGraph.delCopy(v->firstAdj()->theEdge());
		simplifiedGraph.delCopy(v);
		if (u->degree() == 1)
			queue.push(u);
	}
}

void SimplificationDeg12::reverseDeg1Nodes(GraphCopy& C) const
{
	queue<node> queue;
	node v;
	forall_nodes(v, C)
		queue.push(v);
	while (!queue.empty())
	{
		node v = queue.front(); queue.pop();
		node sv = NULL;
		//The GraphCopy C might not be a direct copy of simplifiedGraph,
		//Test, if C.original(v) still works and returns a node of simplifiedGraph
		sv = C.original(v);
		if (sv == NULL) sv = v;
		//search node in the original graph
		node ov = simplifiedGraph.original(sv);
		if (ov == NULL) ov = sv;
		//check, if a 1-deg node was removed
		unordered_map<node, vector<node> >::const_iterator it = deg1Nodes.find(ov);
		if (it != deg1Nodes.end()) {
			for (node ou : it->second) {
				//add new node
				node u = C.newNode(ou);
				C.newEdge(v, u);
				queue.push(u);
			}
		}
	}
}

const GraphCopy& SimplificationDeg12::getSimplifiedGraph() const
{
	return simplifiedGraph;
}

const unordered_map<edge, int>& SimplificationDeg12::getEdgeCosts() const
{
	return edgeCosts;
}

GraphCopy SimplificationDeg12::reverseSimplification(const GraphCopy& modifiedCopy) const
{
	GraphCopy C (modifiedCopy);

	reverseDeg1Nodes(C);

	return C;
}

}