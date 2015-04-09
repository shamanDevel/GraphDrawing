#pragma once

#include <math.h>
#include <Graph.h>

using namespace std;

namespace shaman {

/// Performs the crossing minimization to the optimum.
/// Contains also some helper functions
class CrossingMinimization
{
public:
	CrossingMinimization(void);
	~CrossingMinimization(void);

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

	///	\brief	Replaces every edge of g with a path of length |E|.
	///			This is needed to enshure that a simpe drawing is also the optimum in a drawing where
	///			edges can cross multiple times.
	///
	///	\param g	The graph (input and output)
	void splitGraph(Graph& g);
};

}