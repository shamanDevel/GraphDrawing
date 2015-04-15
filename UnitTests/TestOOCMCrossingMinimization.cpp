#include "stdafx.h"
#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace shaman;
using namespace std;

//#define SAVE_GRAPHS

namespace UnitTests
{		
	TEST_CLASS(TestOOCMCrossingMinimization)
	{
	public:

		TEST_METHOD(TestCreateVariables)
		{
			GraphGenerator gen;
			OOCMCrossingMinimization cm(NULL);
			for (int n=5; n<50; ++n) {
				for (int i=0; i<5; ++i) {
					//create graph
					int m = i==0 ? n*(n-1)/2 : RandomInt(n-1, n*(n-1)/2);
					Graph g = *gen.createRandomGraph(n, m);

					//create variables
					vector<OOCMCrossingMinimization::crossing> outCrossings;
					vector<OOCMCrossingMinimization::crossingOrder> outCrossingOrders;
					cm.createVariables(g, outCrossings, outCrossingOrders);

					stringstream s;
					s << "n=" << n << " m=" << m << " -> variables=" 
						<< (outCrossings.size() + outCrossingOrders.size()) << endl;
					for (OOCMCrossingMinimization::crossing i : outCrossings) {
						s << "  {(" << i.first.first << "," << i.first.second;
						s << ")x(" << i.second.first << "," << i.second.second;
						s << ")}";
					}
					s << endl;
					for (OOCMCrossingMinimization::crossingOrder o : outCrossingOrders) {
						s << " ((" << get<0>(o).first << "," << get<0>(o).second
							<< ");(" << get<1>(o).first << "," << get<1>(o).second
							<< ");(" << get<2>(o).first << "," << get<2>(o).second
							<< "))";
					}
					s << endl;
					Logger::WriteMessage(s.str().c_str());
				}
			}
		}

		static int RandomInt(int min, int max) 
		{
			int r = rand();
			r = r % (max-min+1);
			r += min;
			return r;
		}

		static void TestAndSaveGraph(const Graph& g, const char* prefix)
		{
			int numNodes = num_vertices(g);
			int numEdges = num_edges(g);
			//convert
			ogdf::Graph G;
			ogdf::GraphAttributes GA(G, 
				ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
				| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
				| ogdf::GraphAttributes::nodeColor );
			GraphConverter::Settings settings;
			settings.labelNodes = true;
			settings.nodeSize = 10.0;
			settings.edgeWidth = 1.0;
			settings.nodeColor = "#AAAAAA";
			GraphConverter::convert(g, G, GA, settings);
			int numNodes2 = G.numberOfNodes();
			int numEdges2 = G.numberOfEdges();
			Assert::AreEqual(numNodes, numNodes2, L"Node count is not equal", LINE_INFO());
			Assert::AreEqual(numEdges, numEdges2, L"Edge count is not equal", LINE_INFO());
			//layout
			bool planar = boost::boyer_myrvold_planarity_test(g);
			if (planar) {
#if 0
				//use planar layout
				ogdf::PlanarDrawLayout pdl;
				pdl.sideOptimization(true);
				pdl.sizeOptimization(true);
				pdl.call(GA);
#else
				//use spring layout
				ogdf::FMMMLayout fmmm;
				fmmm.useHighLevelOptions(true);
				fmmm.unitEdgeLength(25.0); 
				fmmm.newInitialPlacement(true);
				fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
				fmmm.call(GA);
#endif
			} else {
#if 1
				//use spring layout
				ogdf::FMMMLayout fmmm;
				fmmm.useHighLevelOptions(true);
				fmmm.unitEdgeLength(10.0 * sqrt(numEdges)); 
				fmmm.newInitialPlacement(true);
				fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
				fmmm.call(GA);
#else
				ogdf::StressMajorization sm;
				sm.call(GA);
#endif
			}

			//save
			stringstream s;
			s << "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\graphs\\";
			s << prefix;
			s << "_n";
			s << numNodes;
			s << "_e";
			s << numEdges;
			if (planar) {
				s << "_planar";
			}
			s << ".svg";
			GA.writeSVG(s.str().c_str());
			cout << "write graph to " << s.str() << endl;
			//Assert::Fail(ToString(s.str()).c_str());
		}

	};
}