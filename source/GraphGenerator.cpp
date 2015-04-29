#include "stdafx.h"

#include <GraphGenerator.h>
#include <boost/graph/graph_archetypes.hpp>
#include <boost/graph/edge_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>

using namespace std;

namespace shaman {

GraphGenerator::GraphGenerator(void)
	: randomEngine(chrono::system_clock::now().time_since_epoch().count())
{
}


GraphGenerator::~GraphGenerator(void)
{
}

bool GraphGenerator::removeEdge(ogdf::Graph& g, vector<ogdf::edge>& edges)
{
	int nodeCount = g.numberOfNodes();
	int edgeCount = g.numberOfEdges();
	vector<int> checkEdge(edgeCount);
	for (int e=0; e<edgeCount; ++e) {checkEdge[e]=e;}
	shuffle(checkEdge.begin(), checkEdge.end(), randomEngine);
	for (int i : checkEdge) {
		//remove the edge at index i
		ogdf::edge e = edges[i];
		ogdf::node u = e->source();
		ogdf::node v = e->target();
		g.delEdge(e); //TODO: use hideEdge, restoreEdge
		//check if the graph is still connected
		if (ogdf::isConnected(g)) {
			edges.erase(edges.begin() + i);
			return true;
		}
		//re-add edge
		g.newEdge(u, v);
	}
	return false;
}

boost::optional<ogdf::Graph> GraphGenerator::createRandomGraph(unsigned int n, unsigned int e)
{
	if (e < n-1) {
		//too less edges (less than a straight line)
		return boost::optional<ogdf::Graph>();
	}
	if (e > n*(n-1)/2) {
		//too many edges (more than the complete graph)
		return boost::optional<ogdf::Graph>();
	}
	//create random graph
	ogdf::Graph g;
	vector<ogdf::node> nodes (n);
	for (int i=0; i<n; ++i) {
		nodes[i] = g.newNode();
	}
	int edgeCount = 0;
	vector<ogdf::edge> edges;
	for (int u=0; u<n-1; ++u) {
		for (int v=u+1; v<n; ++v) {
			ogdf::edge e = g.newEdge(nodes[u], nodes[v]);
			edges.push_back(e);
			edgeCount ++;
		}
	}
	for (int i=edgeCount; i>e; --i) {
		removeEdge(g, edges);
	}

	return boost::optional<ogdf::Graph>(g);
}

boost::optional<ogdf::Graph> GraphGenerator::createRandomPlanarGraph(unsigned int s, unsigned int e)
{
	unsigned int nodeCount = s*s;
	if (e < nodeCount - 1) {
		//too less edges (less than a straight line)
		return boost::optional<ogdf::Graph>();
	}
	if (e > (s-1)*(s-1)*3 + (s-1)*2) {
		//too many edges (more than the complete graph)
		return boost::optional<ogdf::Graph>();
	}
	//create random planar graph
	ogdf::Graph g;
	vector<ogdf::node> nodes (s*s);
	for (int i=0; i<s*s; ++i) {
		nodes[i] = g.newNode();
	}
	int edgeCount = 0;
	vector<ogdf::edge> edges;
	for (int x=0; x<s-1; ++x) {
		for (int y=0; y<s-1; ++y) {
			edges.push_back(g.newEdge(nodes[x + y*s], nodes[x+1 + y*s]));
			edges.push_back(g.newEdge(nodes[x + y*s], nodes[x + (y+1)*s]));
			edges.push_back(g.newEdge(nodes[x + y*s], nodes[x+1 + (y+1)*s]));
			edgeCount += 3;
		}
	}
	for (int x=0; x<s-1; ++x) {
		int y = s-1;
		edges.push_back(g.newEdge(nodes[x + y*s], nodes[x+1 + y*s]));
		edgeCount++;
	}
	for (int y=0; y<s-1; ++y) {
		int x = s-1;
		edges.push_back(g.newEdge(nodes[x + y*s], nodes[x + (y+1)*s]));
		edgeCount++;
	}
	for (int i=edgeCount; i>e; --i) {
		if (!removeEdge(g, edges)) {
			//failed to remove edge
			return boost::optional<ogdf::Graph>();
		}
	}

	return boost::optional<ogdf::Graph>(g);
}

}