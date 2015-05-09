#include "stdafx.h"
#include <SimplificationBiconnected.h>
#include <ogdf/basic/simple_graph_alg.h>
#include <ogdf/planarity/BoyerMyrvold.h>
#include <algorithm>
#include <cassert>

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
	GraphCopy G (*oGraph);
	
	//TODO!
	return G;
}

void SimplificationBiconnected::calculateComponents(const Graph& originalGraph)
{
	EdgeArray<int> edgeIndices (originalGraph);
	int count = biconnectedComponents(originalGraph, edgeIndices);
	cout << count << " biconnected components found" << endl;
	
	components.resize(count);
	planarity.resize(count);

	BoyerMyrvold bm;
	int nonPlanarComp = 0;
	for (int i=0; i<count; ++i) {
		GraphCopy G (originalGraph);
		extractComponent(edgeIndices, i, G);
		components[i] = G;
		planarity[i] = bm.isPlanar(G);
		if (!planarity[i]) {
			nonPlanarComp++;
		}
	}
	cout << nonPlanarComp << " of these components are non-planar" << endl;
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