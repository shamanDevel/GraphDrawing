#pragma once

// This header defines the graph structure used in combination with boost
#include <boost/graph/adjacency_list.hpp>

namespace shaman {

	struct node_data_t {
		typedef boost::vertex_property_tag kind;
	};
	// The type of the node.
	// This specifies how the node was created and how it should be displayed
	enum NodeType {
		// A normal node
		DEFAULT, 
		// This node was created during edge subdivision (to transform multiple edge crossings into single edge crossings)
		SUBDIVISION, 
		// This node represents an edge crossing and should not be drawn.
		CROSSING
	};
	// Structure containing extra information for each node
	struct NodeData {
		// The type of the node.
		// This specifies how the node was created and how it should be displayed
		NodeType type;
		// The label number of the node.
		// Only valid if the node type is DEFAULT
		int label;
		// If the type of the node is SUBDIVISON, these variables store the id of the incident nodes of the original edge.
		// It is used to track adjacent edges
		// pathU < pathV
		int pathU, pathV;
		// If the type of the node is CROSSING, this variable stores the index of the crossing variable in the lp-model.
		int variable;
	};
	typedef boost::property<node_data_t, NodeData, boost::property<boost::vertex_index1_t, int> > NodeProperty;
	typedef boost::property<boost::edge_index_t, int> EdgeProperty;
	// The Graph type that is used
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, NodeProperty, EdgeProperty> Graph;

	typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
	typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
	typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;

}