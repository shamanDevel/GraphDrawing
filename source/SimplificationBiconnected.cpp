#include "stdafx.h"
#include <SimplificationBiconnected.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/planarity/BoyerMyrvold.h>
#include <algorithm>
#include <cassert>
#include <unordered_map>
#include <boost/log/trivial.hpp>

namespace shaman {

SimplificationBiconnected::SimplificationBiconnected(const Graph& originalGraph)
	: oGraph(&originalGraph)
{
	calculateComponents(originalGraph);
}


SimplificationBiconnected::~SimplificationBiconnected(void)
{
}

const vector<GraphCopy> SimplificationBiconnected::getComponents()
{
	vector<GraphCopy> nonPlanarComponents;
	for (int i=0; i<components.size(); ++i) {
		if (!planarity[i]) {
			nonPlanarComponents.push_back(components[i]);
		}
	}
	return nonPlanarComponents;
}

GraphCopy SimplificationBiconnected::reverseSimplification(const vector<GraphCopy>& components)
{
	GraphCopy G;
	G.createEmpty(*oGraph);

	//collect component vector
	vector<const GraphCopy*> comp(this->components.size());
	int j=0;
	for (int i=0; i<this->components.size(); ++i) {
		if (planarity[i]) {
			comp[i] = &this->components[i]; //planar -> use original one
		} else {
			comp[i] = &components[j]; //non-planar -> modifed ones
			j++;
		}
	}

	//collect and create nodes
	unordered_map<node, node> copiedNodes; //original node -> copied node
	unordered_map<node, node> dummyNodes; //new dummy node -> copied node
	for (const GraphCopy* GC : comp) {
		node n;
		forall_nodes(n, *GC) {
			node on = GC->original(n);
			if (on == NULL) {
				//dummy node
				auto it = dummyNodes.find(n);
				if (it == dummyNodes.end()) {
					//create new node
					node n2 = G.newNode();
					dummyNodes.emplace(n, n2);
				}
			} else {
				//copied / normal node
				auto it = copiedNodes.find(on);
				if (it == copiedNodes.end()) {
					//create new node
					node n2 = G.newNode(on);
					copiedNodes.emplace(on, n2);
				}
			}
		}
	}
	
	//create edges
	for (const GraphCopy* GC : comp) {
		edge e;
		forall_edges(e, *GC) {
			edge oe = GC->original(e);
			assert (oe != NULL);
			node v = e->source();
			node w = e->target();
			node v2, w2;
			if (GC->original(v) == NULL) {
				v2 = dummyNodes.at(v);
			} else {
				v2 = copiedNodes.at(GC->original(v));
			}
			if (GC->original(w) == NULL) {
				w2 = dummyNodes.at(w);
			} else {
				w2 = copiedNodes.at(GC->original(w));
			}
			G.newEdgeUnsave(oe, v2, w2);
		}
	}

	return G;
}

void SimplificationBiconnected::calculateComponents(const Graph& originalGraph)
{
	EdgeArray<int> edgeIndices (originalGraph);
	int count = biconnectedComponents(originalGraph, edgeIndices);
	BOOST_LOG_TRIVIAL(info) << count << " biconnected components found";
	
	components.resize(count);
	planarity.resize(count);

	int nonPlanarComp = 0;
	for (int i=0; i<count; ++i) {
		GraphCopy G (originalGraph);
		extractComponent(edgeIndices, i, G);
		components[i] = G;
		BoyerMyrvold bm;
		planarity[i] = bm.isPlanar(G);
		if (!planarity[i]) {
			nonPlanarComp++;
		}
	}
	BOOST_LOG_TRIVIAL(info) << nonPlanarComp << " of these components are non-planar";
}

void SimplificationBiconnected::extractComponent(const EdgeArray<int>& edgeIndices, int index, GraphCopy& G)
{
	edge oe;
	forall_edges(oe, G.original()) {
		edge e = G.copy(oe);
		assert (e != NULL);
		int i = edgeIndices[oe];
		if (i != index) {
			//not in this component -> remove edge
			G.delCopy(e);
		}
	}

	node on;
	forall_nodes(on, G.original()) {
		node n = G.copy(on);
		assert (n != NULL);
		if (n->degree() == 0) {
			G.delCopy(n);
		}
	}
}

}