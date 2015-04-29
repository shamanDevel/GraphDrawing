//Includes OGDF
#pragma once

#include <ogdf\basic\basic.h>
#include <ogdf\basic\Graph.h>
#include <ogdf\basic\Graph_d.h>
#include <ogdf\basic\GraphAttributes.h>
#include <ogdf\basic\graph_generators.h>
#include <ogdf\basic\simple_graph_alg.h>
#include <ogdf\basic\GraphCopy.h>

namespace ogdf {
	bool operator==(const NodeElement& u, const NodeElement& v);
	bool operator==(const EdgeElement& e, const EdgeElement& f);
}