#include "stdafx.h"
#include <SimplificationDeg12.h>
#include <queue>
#include <cassert>
#include <unordered_set>

namespace shaman {

using namespace std;
using namespace ogdf;

SimplificationDeg12::SimplificationDeg12(const GraphCopy& originalGraph)
	: simplifiedGraph(originalGraph)
{
	//initialize edge costs
	edge e;
	forall_edges(e, simplifiedGraph)
		edgeCosts[e] = 1;

	//simplify graph
	removeDeg1Nodes();
	mergeDeg2Nodes();
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
		edge e = v->firstAdj()->theEdge();
		deg1Nodes[simplifiedGraph.original(u)].push_back(
			make_pair(simplifiedGraph.original(v), simplifiedGraph.original(e)));
		simplifiedGraph.delCopy(e);
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
		unordered_map<node, vector<pair<node, edge> > >::const_iterator it = deg1Nodes.find(ov);
		if (it != deg1Nodes.end()) {
			for (pair<node, edge> ou : it->second) {
				//add new node
				node u = C.newNode(ou.first);
				edge e = C.newEdge(v, u); //TODO: add link to original node
				C.setEdge(ou.second, e);
				queue.push(u);
			}
		}
	}
}

void SimplificationDeg12::mergeDeg2Nodes()
{
	unordered_set<node> deg2Nodes;
	node v;
	forall_nodes(v, simplifiedGraph) {
		if (v->degree() == 2)
			deg2Nodes.insert(v);
	}

	while (!deg2Nodes.empty())
	{
		node v = *deg2Nodes.begin();
		deg2Nodes.erase(v);
		assert (v->degree() == 2);
		//follow path in both directions until a node with deg > 2 is reached (last node is of degree > 2)
		vector<node> nodes1, nodes2;
		followDeg2Path(v, v->firstAdj()->twinNode(), nodes1);
		followDeg2Path(v, v->lastAdj()->twinNode(), nodes2);
		node last1 = nodes1[nodes1.size() - 1];
		node last2 = nodes2[nodes2.size() - 1];
		//remove these edges from the set of deg 2 nodes
		for (vector<node>::iterator it = nodes1.begin(); it!=nodes1.end()-1; ++it) {
			int i = deg2Nodes.erase(*it);
			assert (i==1);
		}
		for (vector<node>::iterator it = nodes2.begin(); it!=nodes2.end()-1; ++it) {
			int i = deg2Nodes.erase(*it);
			assert (i==1);
		}

		//Build edge list and remove edges and nodes
		vector<edge> edges;
		for (int i=nodes1.size()-1; i>0; --i) {
			node u = nodes1[i];
			node v = nodes1[i-1];
			edge e = simplifiedGraph.searchEdge(u, v);
			assert (e != NULL);
			edge eo = simplifiedGraph.original(e);
			assert (eo != NULL);
			edges.push_back(eo);
			simplifiedGraph.delCopy(e);
			if (i<nodes1.size()-1)
				simplifiedGraph.delCopy(u);
		}
		node u = nodes1[0];
		edge e = simplifiedGraph.searchEdge(u, v);
		assert (e != NULL);
		edge eo = simplifiedGraph.original(e);
		assert (eo != NULL);
		edges.push_back(eo);
		simplifiedGraph.delCopy(e);
		if (nodes1.size() > 1)
			simplifiedGraph.delCopy(u);
		u = nodes2[0];
		e = simplifiedGraph.searchEdge(v, u);
		assert (e != NULL);
		eo = simplifiedGraph.original(e);
		assert (eo != NULL);
		edges.push_back(eo);
		simplifiedGraph.delCopy(e);
		simplifiedGraph.delCopy(v);
		for (int i=1; i<nodes2.size(); ++i) {
			node u = nodes2[i-1];
			node v = nodes2[i];
			edge e = simplifiedGraph.searchEdge(u, v);
			assert (e != NULL);
			edge eo = simplifiedGraph.original(e);
			assert (eo != NULL);
			edges.push_back(eo);
			simplifiedGraph.delCopy(e);
			simplifiedGraph.delCopy(u);
		}

		//Now there exists three cases:
		//1. last1 == last2: The path of degree 2 nodes is a circle around one single node
		//2. adj(last1, last2)==true: The merging of the nodes would result in a double edge
		//3. otherwise: Normal case, simply add the merged edge

		if (last1 == last2) {
			//Case 1:
			deg2Circles.emplace(simplifiedGraph.original(last1), edges);
			//It can happen that the handle node of the circle now has degree = 2
			if (last1->degree() == 2) {
				deg2Nodes.insert(last1); //add it
			}
		} else {
			edge e = simplifiedGraph.searchEdge(last1, last2);
			if (e != NULL) {
				//Case 2:
				edge oe = simplifiedGraph.original(e);
				assert (oe != NULL);
				edgeCosts[e]++;
				deg2Edges.emplace(oe, edges);
			} else {
				//Case 3:
				edge oe = edges[0];
				node oeSourceCopy = simplifiedGraph.copy(oe->source());
				assert (oeSourceCopy != NULL);
				/*edge e = simplifiedGraph.newEdge(last1, last2);
				simplifiedGraph.setEdge(oe, e);*/
				edge e;
				if (oeSourceCopy == last1)
					e = simplifiedGraph.newEdge(oe, last1, last2->lastAdj());
				else
					e = simplifiedGraph.newEdge(oe, last2, last1->lastAdj());
				deg2Edges.emplace(eo, edges);
			}
		}
	} //while (!deg2Nodes.empty())
}
void SimplificationDeg12::followDeg2Path(node last, node current, vector<node>& nodes)
{
	if (current->degree() != 2) {
		nodes.push_back(current);
		return;
	}
	nodes.push_back(current);
	node v;
	adjEntry adj;
	forall_adj(adj, current) {
		v = adj->twinNode();
		if (v != last) {
			followDeg2Path(current, v, nodes);
			return;
		}
	}
}
void SimplificationDeg12::unmergeDeg2Nodes(GraphCopy& C) const
{

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

	unmergeDeg2Nodes(C);
	reverseDeg1Nodes(C);

	return C;
}

}