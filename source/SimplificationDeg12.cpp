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
	deg2Infos.clear();
	int circleCount = 0;
	int pathCount = 0;
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

		//Build node and edge list
		vector<node> nodes;
		vector<edge> edges;
		for (int i=nodes1.size()-1; i>0; --i) {
			node u = nodes1[i];
			node v = nodes1[i-1];
			node ov = simplifiedGraph.original(v);
			assert (ov != NULL);
			nodes.push_back(ov);
			edge e = simplifiedGraph.searchEdge(u, v);
			assert (e != NULL);
			edge eo = simplifiedGraph.original(e);
			assert (eo != NULL);
			edges.push_back(eo);
		}
		node u = nodes1[0];
		edge e = simplifiedGraph.searchEdge(u, v);
		assert (e != NULL);
		edge eo = simplifiedGraph.original(e);
		assert (eo != NULL);
		node ov = simplifiedGraph.original(v);
		nodes.push_back(ov);
		edges.push_back(eo);
		u = nodes2[0];
		e = simplifiedGraph.searchEdge(v, u);
		assert (e != NULL);
		eo = simplifiedGraph.original(e);
		assert (eo != NULL);
		edges.push_back(eo);
		for (int i=1; i<nodes2.size(); ++i) {
			node u = nodes2[i-1];
			node v = nodes2[i];
			node ou = simplifiedGraph.original(u);
			assert (ou != NULL);
			nodes.push_back(ou);
			edge e = simplifiedGraph.searchEdge(u, v);
			assert (e != NULL);
			edge eo = simplifiedGraph.original(e);
			assert (eo != NULL);
			edges.push_back(eo);
		}
		assert (nodes.size() + 1 == edges.size());

		//Delete nodes and edges
		for (edge oe : edges) {
			simplifiedGraph.delCopy(simplifiedGraph.copy(oe));
		}
		for (node on : nodes) {
			simplifiedGraph.delCopy(simplifiedGraph.copy(on));
		}

		//Now there exists three cases:
		//1. last1 == last2: The path of degree 2 nodes is a circle around one single node
		//2. adj(last1, last2)==true: The merging of the nodes would result in a double edge
		//3. otherwise: Normal case, simply add the merged edge

		node olast1 = simplifiedGraph.original(last1);
		node olast2 = simplifiedGraph.original(last2);
		assert (olast1 != NULL);
		assert (olast2 != NULL);

		Deg2Info info;
		info.sourceOriginal = olast1;
		info.targetOriginal = olast2;
		info.paths.push_back(make_pair(nodes, edges));
		info.edgeOriginal = NULL;
		info.deleteEdge = false;

		if (last1 == last2) {
			//Case 1:
			//It can happen that the handle node of the circle now has degree = 2
			if (last1->degree() == 2) {
				deg2Nodes.insert(last1); //add it
			}
			circleCount++;

		} else {

			int cost = 1;
			for (edge oe : edges) {
				cost = max(cost, edgeCosts.at(oe));
			}

			edge e = simplifiedGraph.searchEdge(last1, last2);
			if (e != NULL) {
				//Case 2:
				edge oe = simplifiedGraph.original(e);
				assert (oe != NULL);
				edgeCosts[oe] += cost;
				info.edgeOriginal = oe;
				info.deleteEdge = false;
				LOG(LOG_LEVEL_DEBUG) << "Reuse edge (" << e->source()->index() << "," << e->target()->index() << ")"
					<< " linked to original edge (" << oe->source()->index() << "," << oe->target()->index() << ")";
			} else {
				//Case 3:
				edge oe = edges[0];
				edge e = simplifiedGraph.newEdgeUnsave(oe, last1, last2);
				assert (e != NULL);
				edgeCosts[oe] = cost;
				info.edgeOriginal = oe;
				info.deleteEdge = true;
				LOG(LOG_LEVEL_DEBUG) << "Create new edge (" << e->source()->index() << "," << e->target()->index() << ")"
					<< " and link to original edge (" << oe->source()->index() << "," << oe->target()->index() << ")";
			}

			//It can happen that the handle nodes now have degree = 2
			if (last1->degree() == 2) {
				deg2Nodes.insert(last1); //add it
			}
			if (last2->degree() == 2) {
				deg2Nodes.insert(last2); //add it
			}

			pathCount++;
		}

		//insert Deg2Info
		auto it = deg2Infos.begin();
		for (; it != deg2Infos.end(); ++it) {
			if (it->sourceOriginal == info.sourceOriginal
				&& it->targetOriginal == info.targetOriginal) {
					break; //merge these
			}
		}
		if (it == deg2Infos.end()) {
			//new entry
			deg2Infos.push_back(info);
		} else {
			//merge entries
			Deg2Info info2 = *it;
			deg2Infos.erase(it);
			assert (info2.edgeOriginal == info.edgeOriginal);
			assert (info.paths.size() == 1);
			info2.deleteEdge = info2.deleteEdge || info.deleteEdge;
			info2.paths.push_back(info.paths[0]);
			deg2Infos.push_back(info2);
		}

	} //while (!deg2Nodes.empty())

	LOG(LOG_LEVEL_DEBUG) << "Degree 2 Infos contains:";
	for (const Deg2Info& info : deg2Infos) {
		stringstream s;
		s << " [" << info.sourceOriginal->index() << "," << info.targetOriginal->index() << "]";
		if (info.deleteEdge) {
			s << ", new edge";
		}
		if (info.sourceOriginal != info.targetOriginal) {
			s << ", original edge: (" << info.edgeOriginal->source()->index()
				<< "," << info.edgeOriginal->target()->index() << ")";
		}
		s << ", paths: {";
		for (int a=0; a<info.paths.size(); ++a) {
			if (a>0) s << ", ";
			s << "[";
			for (int b=0; b<info.paths[a].first.size(); ++b) {
				if (b>0) s << ", ";
				s << info.paths[a].first[b]->index();
			}
			s << "]";
		}
		s << "}";
		LOG(LOG_LEVEL_DEBUG) << s.str();
	}

	LOG(info) << "count of removed circles: " << circleCount;
	LOG(info) << "count of removed paths: " << pathCount;
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

void SimplificationDeg12::reverseCircle(GraphCopy& C, const Deg2Info& info) const
{
	node n = C.copy(info.sourceOriginal);
	assert (n != NULL);

	for (int j=0; j<info.paths.size(); ++j) {
		const vector<edge>& circle = info.paths[j].second;
		node u = n;
		node v = NULL;
		stringstream s;
		s << "Reverse circle around " << n->index() << ": [";
		for (int i=0; i<circle.size()-1; ++i) {
			edge oe = circle[i];
			node ov = oe->target();
			if (ov == C.original(u)) ov = oe->source();
			v = C.newNode(ov);
			edge e = newEdge(C, oe, u, v);

			if (i>0) s << ",";
			s << "(" << u << "," << v << ")";

			u = v;
		}
		newEdge(C, circle[circle.size()-1], u, n);
		s << "(" << u << "," << n << ")";
		s << "]";
		LOG(LOG_LEVEL_DEBUG) << s.str();
	}
}
void SimplificationDeg12::reversePath(GraphCopy& C, const Deg2Info& info) const
{
	edge oe = info.edgeOriginal;
	const List<edge>& chain = C.chain(oe);
	assert (!chain.empty());

	node source = C.copy(info.sourceOriginal);
	node target = C.copy(info.targetOriginal);
	assert (source != NULL);
	assert (target != NULL);

	if (chain.size() == 1) {

		stringstream s;

		//Simple case, no crossing
		if (info.deleteEdge) {
			s << ", delete introduced edge (" << chain.front()->source()->index()
				<< "," << chain.front()->target()->index() << ")";
			C.delCopy(chain.front());
		}

		s << ", paths: {";

		int j=0;
		for (const path_t& path : info.paths) {
			if (j>0) s << ", ";
			j++;
			s << "[";

			//create nodes
			vector<node> nodeCopy (path.first.size());
			for (int i=0; i<path.first.size(); ++i) {
				nodeCopy[i] = C.newNode(path.first[i]);
			}
			//create edges
			for (int i=0; i<path.second.size(); ++i) {
				node v,w;
				if (i==0) {
					v = source;
					w = nodeCopy[0];
				} else if (i==path.second.size()-1) {
					v = nodeCopy[nodeCopy.size() - 1];
					w = target;
				} else {
					v = nodeCopy[i-1];
					w = nodeCopy[i];
				}
				C.newEdgeUnsave(path.second[i], v, w);

				if (i>0) s << ", ";
				s << "(" << v << "," << w << ")";
			}

			s << "]";
		}
		s << "}";
		LOG(LOG_LEVEL_DEBUG) << "Reverse simple path from " << info.sourceOriginal->index() << " to "
			<< info.targetOriginal->index() << s.str();

	} else {
		//CROSSINGS !!!! 
		
		SList<adjEntry> crossingEdges; //the crossing edges
		auto it = chain.begin();
		for (int i=0; i<chain.size()-1; ++i) {
			edge e = *it; ++it;
			edge e2 = *it;
			adjEntry a = e->adjTarget();
			int counter = 0;
			while (a->theEdge() == e || a->theEdge() == e2 
					/*|| a->twinNode()->degree()==2*/) { //Introduced by a former expansion -> adj entries are flipped
				a = a->cyclicSucc();
				counter++;
				assert (counter<4); //must be found at index 1 or 3
			}
			assert (a != NULL);
			crossingEdges.pushBack(a->twin());
		}
		stringstream s;
		s << "Reverse crossing from " << info.sourceOriginal->index() << " to "	<< info.targetOriginal->index();
		s << ", edge chain: {";
		for (int i=0; i<chain.size()-1; ++i) {
			if (i>0) s << ", ";
			edge e = *chain.get(i);
			s << "(" << e->source()->index() << "," << e->target()->index() << ")";
		}
		s << ", (" << chain.back()->source()->index() << "," << chain.back()->target()->index() << ")";
		s << "}";
		s << ", crossing edges (ordered): {";
		for (int i=0; i<crossingEdges.size(); ++i) {
			if (i>0) s << ", ";
			s << "(" << (*crossingEdges.get(i))->twinNode()->index() << ","
				<< (*crossingEdges.get(i))->theNode()->index() << ")";
		}
		s << "}";
		LOG(LOG_LEVEL_DEBUG) << s.str();

		if (info.deleteEdge) {

			C.removeEdgePath(oe);

			/*
			stringstream str;
			edge first = C.original(*chain.begin());
			assert (first == info.edgeOriginal);
			//split first edge
			node s = chain.front()->source();
			node t = chain.front()->target();
			C.delCopy(chain.front());
			str << " Split (" << s->index() << "," << t->index() << ") into {";

			//create nodes
			const path_t &path = info.paths[0];
			vector<node> nodeCopy (path.first.size());
			for (int i=0; i<path.first.size(); ++i) {
				nodeCopy[i] = C.newNode(path.first[i]);
			}
			//create edges
			edge oe = NULL;
			for (int i=0; i<path.second.size(); ++i) {
				node v,w;
				if (i==0) {
					v = s;
					w = nodeCopy[0];
				} else if (i==path.second.size()-1) {
					v = nodeCopy[nodeCopy.size() - 1];
					w = t;
				} else {
					v = nodeCopy[i-1];
					w = nodeCopy[i];
				}
				edge newEdge = C.newEdgeUnsave(path.second[i], v, w);
				oe = path.second[i];

				if (i>0) str << ", ";
				str << "(" << newEdge->source()->index() << "," << newEdge->target()->index() << ")";
			}
			str << "}";
			LOG(LOG_LEVEL_DEBUG) << str.str();

			*/
		}

		//int start = info.deleteEdge ? 1 : 0;
		int start = 0;
		for (int i=start; i<info.paths.size(); ++i) {
			const path_t &path = info.paths[i];
			stringstream s;
			s << " Introduce path parallel to (" << source->index() << ","
				<< target->index() << ")";

			//create nodes
			vector<node> nodeCopy (path.first.size());
			for (int i=0; i<path.first.size(); ++i) {
				nodeCopy[i] = C.newNode(path.first[i]);
			}
			//create edges
			s << ", new edges: {";
			for (int i=0; i<path.second.size()-1; ++i) {
				node v,w;
				if (i==0) {
					v = source;
					w = nodeCopy[0];
				} else {
					v = nodeCopy[i-1];
					w = nodeCopy[i];
				}
				edge newEdge = C.newEdgeUnsave(path.second[i], v, w);
				if (i>0) s << ", ";
				s << "(" << newEdge->source()->index() << "," << newEdge->target()->index() << ")";
			}
			s << "}";

			//introduce crossings
			edge oe = path.second[path.second.size()-1];
			C.insertEdgePath(oe, crossingEdges);
			const List<edge>& edgePath = C.chain(oe);

			s << ", insert crossing edge path from " << C.copy(oe->source()) << " to " << C.copy(oe->target());
			s << ": {";
			for (int i=0; i<edgePath.size(); ++i) {
				if (i>0) s << ", ";
				s << "(" << (*edgePath.get(i))->source()->index() << "," 
					<< (*edgePath.get(i))->target()->index() << ")";
			}
			s << "}";

			LOG(LOG_LEVEL_DEBUG) << s.str();
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

	for (auto it1 = deg2Infos.rbegin(); it1 != deg2Infos.rend(); ++it1)
	{
		const Deg2Info &info = *it1;

		if (info.sourceOriginal == info.targetOriginal) {
			reverseCircle(C, info);
		} else {
			reversePath(C, info);
		}

	}
	
}
edge SimplificationDeg12::newEdge(GraphCopy& C, edge origE, node copyU, node copyV) const
{
	return C.newEdgeUnsave(origE, copyU, copyV);

	//if (C.copy(origE->source()) == copyU) {
	//	if (copyV->lastAdj() != NULL) {
	//		return C.newEdge(origE, copyU, copyV->lastAdj());
	//	}
	//}
	//if (C.copy(origE->target()) == copyV) {
	//	if (copyU->lastAdj() != NULL) {
	//		return C.newEdge(origE, copyU->lastAdj(), copyV);
	//	}
	//}
	//if (C.copy(origE->source()) == copyV) {
	//	if (copyU->lastAdj() != NULL) {
	//		return C.newEdge(origE, copyV, copyU->lastAdj());
	//	}
	//}
	//if (C.copy(origE->target()) == copyU) {
	//	if (copyV->lastAdj() != NULL) {
	//		return C.newEdge(origE, copyV->lastAdj(), copyU);
	//	}
	//}
	//return NULL;
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