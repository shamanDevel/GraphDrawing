// GraphDrawing.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <stdafx.h>
#include <lp_lib.h>
#include <ogdf\basic\basic.h>
#include <ogdf\basic\Graph.h>
#include <ogdf\basic\Graph_d.h>
#include <ogdf\basic\graph_generators.h>


int _tmain(int argc, _TCHAR* argv[])
{
	//Test if lp_solve is available
	lprec *lp;
	lp = make_lp(0,4);
	delete_lp(lp);

	//Test if OGDF is available
	ogdf::Graph G;
	ogdf::randomSimpleGraph(G, 10, 20);

	return 0;
}

