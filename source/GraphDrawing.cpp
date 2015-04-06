// GraphDrawing.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <stdafx.h>
#include <iostream>
#include <boost/graph/graph_archetypes.hpp>
#include <boost/graph/adjacency_matrix.hpp>


int _tmain(int argc, _TCHAR* argv[])
{
	//Test if lp_solve is available
	lprec *lp;
	lp = make_lp(0,4);
	delete_lp(lp);

	//Test if OGDF is available
	ogdf::Graph G;
	ogdf::randomSimpleGraph(G, 10, 20);

	//Test if BOOST is available
	enum { A, B, C, D, E, F, N };
	const char* name = "ABCDEF";
	typedef boost::adjacency_matrix<boost::undirectedS> UGraph;
	UGraph ug(N);
	add_edge(B, C, ug);
	add_edge(B, F, ug);
	add_edge(C, A, ug);
	add_edge(D, E, ug);
	add_edge(F, A, ug);
	std::cout << "the edges incident to v: ";
	boost::graph_traits<UGraph>::out_edge_iterator e, e_end;
	boost::graph_traits<UGraph>::vertex_descriptor 
	s = vertex(0, ug);
	for (boost::tie(e, e_end) = out_edges(s, ug); e != e_end; ++e)
	std::cout << "(" << source(*e, ug) 
				<< "," << target(*e, ug) << ")" << endl;

	std::cout << "Press ENTER to exit" << std::endl;
	std::cin.get();
	return 0;
}

