#pragma once

#include <ogdf_include.h>
#include <boost/optional.hpp>
#include <random>
#include <vector>

using namespace std;

namespace shaman {

class GraphGenerator
{
public:
	GraphGenerator(void);
	~GraphGenerator(void);

	///	\brief	Creates a random graph with the specified node and edge count.
	///			This graph is simple and connected, contains no self loops and double edges.
	///			It is not guarantered that the resulting graph is planar. Instead, when the edge count
	///			increases, the graph will not be planar.
	///			The lower bound of the edge count is nodeCount-1 (a tree).
	///			The upper bound of the edge count is nodeCount*(nodeCount-1)/2 (the complete graph).
	///			If the edge count is outside of this bounds, the returned optional is empty.
	///
	///	\param nodeCount	The count of nodes
	///	\param edgeCount	The count of edges
	///	\return	If the edge and node counts are valid, the boost::optional will contain the graph.
	boost::optional<ogdf::Graph> createRandomGraph(unsigned int nodeCount, unsigned int edgeCount);

	///	\brief	Creates a random planar graph with the specified node and edge count.
	///			If first creates a regular triangle mesh with the side length of sqrtNodeCount,
	///			Then it deletes as many edges until the specified edgeCount is achieved.
	///			Therefore, the resulting graph will contain sqrtNodeCount^2 nodes.
	///			This graph is simple and connected, contains no self loops and double edges.
	///			The lower bound of the edge count is sqrtNodeCount*sqrtNodeCount-1 (a tree).
	///			The upper bound of the edge count is (s-1)*(s-1)*3 + (s-1)*2 with s:=sqrtNodeCount (the complete grid).
	///			If the edge count is outside of this bounds, the returned optional is empty.
	///
	///	\param sqrtNodeCount	The side length of the triangle grid
	///	\param edgeCount		The count of edges
	///	\return	If the edge and node counts are valid, the boost::optional will contain the graph.
	boost::optional<ogdf::Graph> createRandomPlanarGraph(unsigned int sqrtNodeCount, unsigned int edgeCount);

private:
	//removes a random edge without disconnecting the graph
	bool removeEdge(ogdf::Graph& g, vector<ogdf::edge>& edges);

	std::default_random_engine randomEngine;
};

}