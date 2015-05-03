#include "stdafx.h"
#include <SimplificationDeg12.h>
#include <queue>
#include <cassert>
#include <unordered_set>
#include <sstream>
#include <boost/log/trivial.hpp>

namespace shaman {

using namespace std;
using namespace ogdf;

#define LOG(level) BOOST_LOG_TRIVIAL(level)
#define LOG_LEVEL_DEBUG debug

SimplificationDeg12::SimplificationDeg12(const GraphCopy& originalGraph)
	: simplifiedGraph(originalGraph)
{
	//initialize edge costs
	edge e;
	forall_edges(e, simplifiedGraph)
		edgeCosts[simplifiedGraph.original(e)] = 1;

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
		int id = v->index();
		node u = v->firstAdj()->twinNode();
		//remove node v and the edge (u,v)
		edge e = v->firstAdj()->theEdge();
		deg1Nodes[simplifiedGraph.original(u)].push_back(
			make_pair(simplifiedGraph.original(v), simplifiedGraph.original(e)));
		simplifiedGraph.delCopy(e);
		simplifiedGraph.delCopy(v);
		if (u->degree() == 1)
			queue.push(u);
		LOG(LOG_LEVEL_DEBUG) << "node " << id << " of degree 1, connected to " << u->index() << ", is removed";
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
				LOG(LOG_LEVEL_DEBUG) << "node " << u->index() << " of degree 1, connected to " << v->index() << ", is re-added";
				//edge e = C.newEdge(v, u);
				//C.setEdge(ou.second, e);
				edge e = newEdge(C, ou.second, v, u);
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

		stringstream str;
		str << "path of degree 2 nodes starting at " << last1->index() << " going to " << last2->index() << " removed:";
		for (int i=nodes1.size()-2; i>=0; --i)
			str << " " << nodes1[i]->index();
		str << " " << v->index();
		for (int i=0; i<nodes2.size()-1; ++i)
			str << " " << nodes2[i]->index();
		LOG(LOG_LEVEL_DEBUG) << str.str();

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
			node olast1 = simplifiedGraph.original(last1);
			node olast2 = simplifiedGraph.original(last2);
			assert (olast1 != NULL);
			assert (olast2 != NULL);
			edge e = simplifiedGraph.searchEdge(last1, last2);
			if (e != NULL) {
				//Case 2:
				edge oe = simplifiedGraph.original(e);
				assert (oe != NULL);
				edgeCosts[oe]++;
				deg2Edges.emplace(oe, edges);
				LOG(LOG_LEVEL_DEBUG) << "Reuse edge (" << e->source()->index() << "," << e->target()->index() << ")"
					<< " linked to original edge (" << oe->source()->index() << "," << oe->target()->index() << ")";
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
				oe = simplifiedGraph.original(e);
				assert (oe != NULL);
				edgeCosts[oe] = 1;
				deg2Edges.emplace(oe, edges);
				deg2Case3Edges.insert(oe);
				LOG(LOG_LEVEL_DEBUG) << "Create new edge (" << e->source()->index() << "," << e->target()->index() << ")"
					<< " and link to original edge (" << oe->source()->index() << "," << oe->target()->index() << ")";
			}
		}

	} //while (!deg2Nodes.empty())

	LOG(info) << "count of removed circles: " << deg2Circles.size();
	LOG(info) << "count of removed paths: " << deg2Edges.size();
	stringstream str;
	str << "deg2Edges contains:";
	for (const auto& entry : deg2Edges) {
		str << endl;
		str << "  (" << entry.first->source()->index() << "," << entry.first->target()->index() << ")";
		str << " => path of length " << entry.second.size();
	}
	LOG(LOG_LEVEL_DEBUG) << str.str();
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
	SList<node> nodes;
	C.allNodes(nodes);
	SList<edge> edges;
	C.allEdges(edges);
	int reversedCircles = 0;
	int reversedPaths = 0;
	unordered_multimap<edge, vector<edge> > deg2EdgesCopy = deg2Edges;

	while (!nodes.empty() || !edges.empty())
	{
		while (!nodes.empty()) {
			//Undo Case 1 (circles around a node)
			const node n =  nodes.front(); nodes.popFront();
			auto itPair = deg2Circles.equal_range(C.original(n));
			for (auto it = itPair.first; it != itPair.second; ++it) {
				const vector<edge>& circle = it->second;
				assert (circle.size() >= 2);

				stringstream str;
				str << "Reverse circle of degree 2 nodes around node " << n->index() << ":";
				for (edge e : circle)
					str << " (" << e->source()->index() << "," << e->target()->index() << ")";
				LOG(LOG_LEVEL_DEBUG) << str.str();

				node u = n;
				node v = NULL;
				for (int i=0; i<circle.size()-1; ++i) {
					edge oe = circle[i];
					node ov = oe->target();
					if (ov == C.original(u)) ov = oe->source();
					v = C.newNode(ov);
					nodes.pushBack(v);
					edge e = newEdge(C, oe, u, v);
					edges.pushBack(e);
					u = v;
				}
				edges.pushBack(newEdge(C, circle[circle.size() - 1], v, n));

				reversedCircles++;
			}
		}

		while (!edges.empty()) {
			//Undo Case 2 and 3
			const edge e = edges.front(); edges.popFront();
			edge oe = C.original(e);
			assert (oe != NULL);
			if (C.chain(oe).size() > 1) {
				LOG(debug) << "Edge (" << oe->source()->index() << "," << oe->target()->index() 
					<< ") splitted by introducing a crossing";
				//List<edge> chain = C.chain(oe);
				//for (auto it = chain.begin(); it != chain.end(); ++it) {
				//	/*remove(edges.begin(), edges.end(), *it);*/
				//	for (auto it2 = edges.begin(); it2 != edges.end(); ++it2) {
				//		auto it3 = it2;
				//		++it3;
				//		if (it3 != edges.end() && *it3 == *it) {
				//			edges.delSucc(it2);
				//			LOG(debug) << "  Ignore (" << C.original((*it)->source())->index() 
				//				<< "," << C.original((*it)->target())->index() << ")";
				//			break;
				//		}
				//	}
				//}
			}
			node s = e->source();
			node t = e->target();
			node os = C.original(s);
			node ot = C.original(t);
			auto itPair = deg2EdgesCopy.equal_range(oe);
			if (itPair.first == itPair.second) continue;
			bool deleted = false;
			for (auto it = itPair.first; it != itPair.second; ++it) {
				const vector<edge>& path = it->second;
				assert (path.size() >= 2);

				stringstream str;
				str << "Reverse path of degree 2 nodes between (" << s->index() << "," 
					<< t->index() << "):";
				for (edge f : path)
					str << " (" << f->source()->index() << "," << f->target()->index() << ")";
				LOG(LOG_LEVEL_DEBUG) << str.str();

				if (path[0]->source() != os && path[0]->target() != os)
					swap(os, ot);
				node u = C.copy(os);
				node v = NULL;
				for (int i=0; i<path.size()-1; ++i) {
					edge oe = path[i];

					if (!deleted && deg2Case3Edges.count(oe)>0) {
						//Can only happen once in Case 3, this edge is used as original-edge link, delete old copy
						int count = C.chain(oe).size();
						C.delCopy(e);
						deleted = true;
					}

					node ov = oe->target();
					if (ov == C.original(u)) ov = oe->source();
					v = C.newNode(ov);
					nodes.pushBack(v);
					edge e = newEdge(C, oe, u, v);
					edges.pushBack(e);
					u = v;
				}
				edge ne = newEdge(C, path[path.size() - 1], v, ot==NULL ? t : C.copy(ot));
				edges.pushBack(ne);

				reversedPaths++;
			}
			deg2EdgesCopy.erase(itPair.first, itPair.second);
		}
	}
	LOG(info) << "Count of reversed circles: " << reversedCircles;
	LOG(info) << "Count of reversed paths: " << reversedPaths;
}
edge SimplificationDeg12::newEdge(GraphCopy& C, edge origE, node copyU, node copyV) const
{
	if (C.copy(origE->source()) == copyU) {
		if (copyV->lastAdj() != NULL) {
			return C.newEdge(origE, copyU, copyV->lastAdj());
		}
	}
	if (C.copy(origE->target()) == copyV) {
		if (copyU->lastAdj() != NULL) {
			return C.newEdge(origE, copyU->lastAdj(), copyV);
		}
	}
	if (C.copy(origE->source()) == copyV) {
		if (copyU->lastAdj() != NULL) {
			return C.newEdge(origE, copyV, copyU->lastAdj());
		}
	}
	if (C.copy(origE->target()) == copyU) {
		if (copyV->lastAdj() != NULL) {
			return C.newEdge(origE, copyV->lastAdj(), copyU);
		}
	}
	assert (false);
	return NULL;
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