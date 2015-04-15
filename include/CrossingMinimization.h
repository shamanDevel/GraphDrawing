#pragma once

#include <Graph.h>
#include <utility>
#include <boost/optional.hpp>

namespace shaman {

class CrossingMinimization
{
public:
	virtual ~CrossingMinimization() {}

	///	\brief	Solves the crossing problem to optimum.
	///			Input: the original graph, simple and connected
	///			Output: The optional contains the graph with inserted crossing nodes and is now planar and the crossing number.
	///				If the problem could not be solved, the optional is empty.
	///
	///	\param	originalG	The original graph
	///	\return	The resulting graph with crossing nodes and the crossing number, or an empty optional
	virtual boost::optional< std::pair<Graph, unsigned int> > solve(const Graph& originalGraph) = 0;

	// Known bounds for the crossing number

	// Returns the upper bound of the crossing number of the complete graph Kn.
	static int crKnUpper(int n) {
		return (int) ceil( (n/2) * ((n-1)/2) * ((n-2)/2) * ((n-3)/2) / 4.0);
	}

	// Returns a lower bound of the crossing number of an arbitrary (simple and connected) graph
	// with n nodes and m edges
	static int crGLower(int n, int m) {
		int cr = 0;
		if (n>2) {
			cr = max(cr, m - 3*n + 6); //Euler's formula
		}
		if (m >= 4*n) {
			cr = max(cr, m*m*m/(n*n*100)); //Leighton
		}
		cr = max(cr, (int) floor(m*m*m/(n*n*33.75) - 0.9*n)); //Pach and Tóth
		return cr;
	}
};

}