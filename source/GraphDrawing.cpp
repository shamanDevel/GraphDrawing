// GraphDrawing.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <stdafx.h>
#include <iostream>


int _tmain(int argc, _TCHAR* argv[])
{
	//Test if lp_solve is available
	lprec *lp;
	lp = make_lp(0,4);
	delete_lp(lp);

	//Test if OGDF is available
	ogdf::Graph G;
	ogdf::randomSimpleGraph(G, 10, 20);

	std::cout << "Press ENTER to exit" << std::endl;
	std::cin.get();
	return 0;
}

