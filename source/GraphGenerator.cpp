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

bool GraphGenerator::removeEdge(Graph& g, vector<pair<int, int> >& edges)
{
	int nodeCount = num_vertices(g);
	int edgeCount = num_edges(g);
	vector<int> tmp(nodeCount);
	vector<int> checkEdge(edgeCount);
	for (int e=0; e<edgeCount; ++e) {checkEdge[e]=e;}
	shuffle(checkEdge.begin(), checkEdge.end(), randomEngine);
	for (int i : checkEdge) {
		//remove the edge at index i
		int u = edges[i].first;
		int v = edges[i].second;
		remove_edge(u, v, g);
		//check if the graph is still connected
		if (boost::connected_components(g, &tmp[0]) == 1) {
			edges.erase(edges.begin() + i);
			return true;
		}
		//re-add edge
		add_edge(u, v, g);
	}
	return false;
}

boost::optional<Graph> GraphGenerator::createRandomGraph(unsigned int n, unsigned int e)
{
	if (e < n-1) {
		//too less edges (less than a straight line)
		return boost::optional<Graph>();
	}
	if (e > n*(n-1)/2) {
		//too many edges (more than the complete graph)
		return boost::optional<Graph>();
	}
	//create random graph
	Graph g(n);
	int edgeCount = 0;
	vector<pair<int, int> > edges;
	for (int u=0; u<n-1; ++u) {
		for (int v=u+1; v<n; ++v) {
			add_edge(u, v, g);
			edges.push_back(make_pair(u, v));
			edgeCount ++;
		}
	}
	for (int i=edgeCount; i>e; --i) {
		removeEdge(g, edges);
	}
	return boost::optional<Graph>(g);
}

boost::optional<Graph> GraphGenerator::createRandomPlanarGraph(unsigned int s, unsigned int e)
{
	unsigned int nodeCount = s*s;
	if (e < nodeCount - 1) {
		//too less edges (less than a straight line)
		return boost::optional<Graph>();
	}
	if (e > s*s*3) {
		//too many edges (more than the complete graph)
		return boost::optional<Graph>();
	}
	//create random planar graph
	Graph g(s*s);
	int edgeCount = 0;
	vector<pair<int, int> > edges;
	for (int x=0; x<s-1; ++x) {
		for (int y=0; y<s-1; ++y) {
			pair<int, int> p;
			p = make_pair(x + y*s, x+1 + y*s);
			add_edge(p.first, p.second, g);
			edges.push_back(p);
			p = make_pair(x + y*s, x + (y+1)*s);
			add_edge(p.first, p.second, g);
			edges.push_back(p);
			p = make_pair(x + y*s, x+1 + (y+1)*s);
			add_edge(p.first, p.second, g);
			edges.push_back(p);
			edgeCount += 3;
		}
	}
	for (int i=edgeCount; i>e; --i) {
		if (!removeEdge(g, edges)) {
			//failed to remove edge
			return boost::optional<Graph>();
		}
	}
	return boost::optional<Graph>(g);
}

}