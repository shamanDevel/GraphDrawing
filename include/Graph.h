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
	};
	typedef boost::property<node_data_t, NodeData> NodeProperty;
	// The Graph type that is used
	typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, NodeProperty> Graph;
}