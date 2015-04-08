#pragma once

#include <Graph.h>
#include <ogdf_include.h>
#include <utility>

namespace shaman {

// Converts the Boost-Graph to the OGDF-Graph
class GraphConverter
{
private:
	GraphConverter(void);
	~GraphConverter(void);

public:

	static struct Settings
	{
		float nodeSize;
		float edgeWidth;
		bool labelNodes;
		ogdf::String nodeColor;
	};

	///	\brief	Converts the Boost-Graph as defined as a typedef in Graph.h to an OGDF-Graph
	///	\param g		The input Boost-Graph
	///	\param outG		The output OGDF-Graph that is filled
	///	\param outGA	The output GraphAttributes-instance that is linked to outG, required for settings node attributes
	///					These flags are required:nodeGraphics, edgeGraphics, nodeLabel, nodeColor and edgeStyle 
	///	\param settings	Specifies the graphic settings
	static void convert(const Graph& g, ogdf::Graph& outG, ogdf::GraphAttributes& outGA, const Settings& settings);
};

}