#pragma once

#include <ogdf_include.h>
#include <vector>

namespace shaman {

using namespace std;
using namespace ogdf;

///	\brief	Simplifies a graph by splitting it into biconnected components.
///			Every biconnected component can then be solved on its once and then
///			the graph is united again.
///			Only these components that are non-planar are returned.
class SimplificationBiconnected
{
public:
	SimplificationBiconnected(const Graph& originalGraph);
	~SimplificationBiconnected(void);

	///	\brief	Returns the non-planar biconnected components
	const vector<GraphCopy> getComponents();

	///	\brief	Reverses the simplification
	///
	///	\param components	The components as returned by getComponents() after 
	///						solving the crossing minimization.
	///	\return	The combined graph
	GraphCopy reverseSimplification(const vector<GraphCopy>& components);

private:
	const Graph* oGraph;
	vector<GraphCopy> components;
	vector<bool> planarity;

	void calculateComponents(const Graph& originalGraph);
	void extractComponent(const EdgeArray<int>& edgeIndices, int index, GraphCopy& G);
};

}