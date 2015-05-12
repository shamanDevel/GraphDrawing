// TestApp.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include "stdafx.h"

#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <string>
#include <cstdio>
#include <fstream>
#include <GraphGenerator.h>
#include <GraphConverter.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <SimplificationDeg12.h>
#include <SimplificationBiconnected.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/boyer_myrvold_planar_test.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>
#include <boost/filesystem.hpp>
#include <boost/graph/graphml.hpp>
#include <ogdf_include.h>
#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <ogdf\basic\simple_graph_alg.h>
#include <MILP.h>
#include <MILP_lp_solve.h>

using namespace shaman;
using namespace std;
using namespace ogdf;

#define SAVE_GRAPHS

int RandomInt(int min, int max) 
{
	int r = rand();
	r = r % (max-min+1);
	r += min;
	return r;
}

void SaveGraph(GraphAttributes& GA, const char* prefix)
{

	//layout
	BoyerMyrvold bm;
	bool planar = bm.isPlanar(GA.constGraph());
	if (planar) {
#if 1
		//use planar layout
		MixedModelLayout l;
		l.call(GA);
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
		fmmm.unitEdgeLength(10.0 * sqrt(GA.constGraph().numberOfEdges())); 
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
	s << GA.constGraph().numberOfNodes();
	s << "_e";
	s << GA.constGraph().numberOfEdges();
	if (planar) {
		s << "_planar";
	}
	//s << ".svg";
	//GA.writeSVG(s.str().c_str());
	s << ".gml";
	GA.writeGML(s.str().c_str());
	cout << "Graph saved to " << s.str() << endl;
}
void SaveGraph(const GraphCopy& G, const char* prefix)
{
#ifdef SAVE_GRAPHS

	int numNodes = G.numberOfNodes();
	int numEdges = G.numberOfEdges();
	//converter
	ogdf::GraphAttributes GA(G, 
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor | ogdf::GraphAttributes::nodeType
		| ogdf::GraphAttributes::edgeLabel );
	SList<node> nodes;
	G.allNodes(nodes);
	for (node n : nodes) {
		if (G.isDummy(n)) {
			GA.type(n) = Graph::NodeType::dummy;
			GA.width(n) = GA.height(n) = 3;
		} else {
			stringstream s;
			s << G.original(n)->index();
			GA.labelNode(n) = s.str().c_str();
			GA.width(n) = GA.height(n) = 15;
		}
	}
	SList<edge> edges;
	G.allEdges(edges);
	for (edge e : edges) {
		stringstream s;
		s << G.original(e)->index();
		GA.labelEdge(e) = s.str().c_str();
	}

	SaveGraph(GA, prefix);
#endif
}

void TestMinimization() {
	GraphGenerator gen;
	MILP* milp = new MILP_lp_solve();
	CrossingMinimization* cm = new OOCMCrossingMinimization(milp);
	for (int n=5; n<20; ++n) {
		//for (int i=0; i<5; ++i) {
			cout << "n=" << n << endl;
			Graph g = *gen.createRandomGraph(n, max(n*(n-1)/2, n-1));
			int m = g.numberOfEdges();

			unordered_map<edge, int> edgeCosts;
			edge e;
			forall_edges(e, g)
				edgeCosts[e] = 1;

			boost::optional< pair<GraphCopy, unsigned int> > result = cm->solve(g, edgeCosts);
			if (result) {
				cout << "graph solved, count of crossings: " << result->second << endl;
				stringstream s;
				s << "K" << n;
				SaveGraph(result->first, s.str().c_str());
			} else {
				cout << "unable to solve graph" << endl;
			}
			cin.get();
			cout << endl;
		//}
	}
}

struct RomeGraphDescr
{
	string fileName;
	int n;
	int m;
};

void scanRomeGraphs(string folder, vector<RomeGraphDescr>& target)
{
	target.clear();
	ifstream in = ifstream (folder + "Graphs.dat");
	if (!in.is_open()) {
		cerr << "Unable to open file!" << endl;
		return;
	}
	int n;
	int m;
	char name[512];
	//read file
	while (!in.eof()) {
		in >> n;
		in >> m;
		in >> name;
		if (strlen(name) == 0) break;
		RomeGraphDescr descr;
		descr.n = n;
		descr.m = m;
		descr.fileName = name;
		target.push_back(descr);
	}
	cout << target.size() << " Graph infos read" << endl;
	in.close();
}

void createRomeGraphsInfo(string folder)
{
	using namespace boost::filesystem;
	path p (folder);   // p reads clearer than argv[1] in the following code
	FILE* file;
	fopen_s(&file, (folder + "Graphs.dat").c_str(), "w");
	try
	{
		assert (exists(p));
		assert (is_directory(p));
		for (directory_iterator it = directory_iterator(p); it != directory_iterator(); ++it)
		{
			path f = it->path();
			if (f.extension() == ".graphml") {
				//load graph
				typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BoostGraph;
				BoostGraph G;
				ifstream is (f.c_str());
				boost::dynamic_properties properties;
				boost::read_graphml(is, G, properties);
				cout << "Graph " << f.filename() << " loaded";
				int n = num_vertices(G);
				int m = num_edges(G);
				bool planar = boost::boyer_myrvold_planarity_test(G);
				cout << " nodes=" << n << " edges=" << m << " planar=" << planar << endl;
				if (!planar) {
					fprintf(file, "%d %d %s\n", n, m, f.filename().string().c_str());
				}
			}
		}
	}
	catch (const filesystem_error& ex)
	{
		cerr << ex.what() << '\n';
	}
	catch (const boost::graph_exception& ex)
	{
		cerr << ex.what() << '\n';
	}
	fclose(file);
}

void RomeMinimization() {
	string folder = "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\example-data\\";
	vector<RomeGraphDescr> graphs;
	scanRomeGraphs(folder, graphs);

	int nMin = INT_MAX;
	int nMax = INT_MIN;
	for (const RomeGraphDescr& d : graphs) {
		nMin = min(nMin, d.n);
		nMax = max(nMax, d.n);
	}
	vector<RomeGraphDescr> graphs2;
	cout << "Choose a graph from the rome graph collection" << endl;
	//Select node count
	while (graphs2.empty()) {
		cout << "Enter the number of nodes between " << nMin << " and " << nMax << ": ";
		int n;
		cin >> n;
		for (const RomeGraphDescr& d : graphs) {
			if (d.n == n)
				graphs2.push_back(d);
		}
		if (graphs2.empty()) {
			cout << "No graphs with this node count found" << endl;
		}
	}
	//select edge count
	int mMin = INT_MAX;
	int mMax = INT_MIN;
	for (const RomeGraphDescr& d : graphs2) {
		mMin = min(mMin, d.m);
		mMax = max(mMax, d.m);
	}
	vector<RomeGraphDescr> graphs3;
	while (graphs3.empty()) {
		cout << "Enter the number of edges between " << mMin << " and " << mMax << ": ";
		int m;
		cin >> m;
		for (const RomeGraphDescr& d : graphs2) {
			if (d.m == m)
				graphs3.push_back(d);
		}
		if (graphs3.empty()) {
			cout << "No graphs with this edge count found" << endl;
		}
	}

	cout << graphs3.size() << " graphs with the specified node and edge count found, select a random one" << endl;
	int i = RandomInt(0, graphs3.size() - 1);
	RomeGraphDescr d = graphs3[i];

	cout << "Load " << d.fileName << " ... ";
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BoostGraph;
	BoostGraph BG;
	try {
		ifstream is (folder + d.fileName);
		boost::dynamic_properties properties;
		boost::read_graphml(is, BG, properties);
		bool planar = boost::boyer_myrvold_planarity_test(BG);
		assert (!planar);
	}
	catch (const boost::graph_exception& ex)
	{
		cerr << ex.what() << '\n';
		return;
	}
	Graph G;
	GraphConverter::convert(BG, G);
	SaveGraph(G, d.fileName.c_str());
	cout << " Loaded and saved" << endl;

	//Now simplify it
	GraphCopy GC1 (G);
	SimplificationDeg12 s (GC1);
	const GraphCopy& GC2 = s.getSimplifiedGraph();
	cout << "Graph simplified, contains now " << GC2.numberOfNodes() << " nodes and " 
		<< GC2.numberOfEdges() << " endges" << endl;

	//solve crossing minimization
	OOCMCrossingMinimization cm (new MILP_lp_solve());
	CrossingMinimization::solve_result_t result = cm.solve(GC2, s.getEdgeCosts());
	if (!result) {
		cerr << "Unable to solve crossing minimization" << endl;
		return;
	}
	GraphCopy GC3 = result->first;
	cout << "Crossing number: " << result->second << endl;

	GraphCopy GC4 = s.reverseSimplification(GC3);
	assert (GC4.consistencyCheck());
	cout << "Reversed Simplification" << endl;

	//save
	SaveGraph(GC4, (d.fileName + "Solved").c_str());
	cout << "Graph saved" << endl;
}

Graph createRandomNonPlanarGraph(int n) {
	GraphGenerator gen;
	BoyerMyrvold bm;
	int m = n;
	while (true) {
		Graph G = *gen.createRandomGraph(n, m);
		if (bm.isPlanar(G)) {
			m++;
		} else {
			return G;
		}
	}
}

void AssertGraphEquality(const Graph& G, const GraphCopy& GC)
{
	assert(GC.consistencyCheck());
	assert(G.consistencyCheck());
	assert(G.numberOfNodes() == GC.numberOfNodes());
	assert(G.numberOfEdges() == GC.numberOfEdges());
	node v;
	//check every node
	forall_nodes(v, GC) {
		node ov = GC.original(v);
		assert(ov != NULL);
		//find node in source graph
		node u;
		node ou = NULL;
		forall_nodes(u, G) {
			if (u == ov) {
				assert(ou == NULL);
				ou = u;
			}
		}
		assert(ou != NULL);
		//check adjacent edges
		assert(ou->degree() == v->degree());
		adjEntry adj;
		forall_adj(adj, v) {
			node n = adj->twinNode();
			node on = GC.original(n);
			assert(on != NULL);
			adjEntry adj2;
			bool found = false;
			forall_adj(adj2, ou) {
				node n2 = adj2->twinNode();
				if (n2 == on) {
					assert(!found);
					found = true;
				}
			}
			assert(found);
		}
	}
}

void Test_SimplificationDeg12_K5_1() {
	//Define a test graph that collapses to a K5
	BoyerMyrvold bm;
	Graph G;
	vector<node> nodes(41);
	for (int i=0; i<=40; ++i) nodes[i] = G.newNode();
	int edges[52][2] = {
		{0,7}, {0,6}, {0,9}, {0,18}, {0,19}, {0,35},
		{1,8}, {1,9}, {1,26}, {1,10}, {1,20}, {1,16}, {1,28},
		{2,11}, {2,18}, {2,12}, {2,22}, {2,25}, {2,17},
		{3,14}, {3,19}, {3,20}, {3,13}, {3,21}, {3,23},
		{4,35}, {4,5}, {4,16}, {4,17}, {4,15},
		{5,6}, {7,8}, {10,11}, {12,13}, {14,15},
		{21,22}, {23,24}, {24,25},
		{26,31}, {26,27}, {27,28}, {28,29}, {30,31}, {31,32},
		{33,34}, {33,35}, {34,35}, {33,36}, {33,37}, {37,38}, {37,39}, {37,40}
	};
	for (int i=0; i<52; ++i) {
		G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
	}
	assert (!bm.isPlanar(G));
	SaveGraph(GraphCopy(G), "BlownK5");

	SimplificationDeg12 s (G);
	const GraphCopy& GC = s.getSimplifiedGraph();
	SaveGraph(GC, "SimplifiedK5");

	//GC now should be a K5 with nodes 0..4
	assert(5 == GC.numberOfNodes());
	assert(10 == GC.numberOfEdges());
	node u;
	forall_nodes(u, GC) {
		assert(4 == u->degree());
	}
	assert(isConnected(GC));
	assert(isSimpleUndirected(GC));

	//reverse simplificatoin
	const GraphCopy GC2 (GC);
	GraphCopy GC3 = s.reverseSimplification(GC2);
	assert (GC3.consistencyCheck());
	edge e;
	vector<pair<int, int> > edgeList;
	forall_edges(e, GC3) {
		edgeList.push_back(make_pair(GC3.original(e->source())->index(), GC3.original(e->target())->index()));
	}
	sort(edgeList.begin(), edgeList.end());
	cout << "Edges:";
	for (pair<int, int> pii : edgeList) {
		cout << " (" << pii.first << "," << pii.second << ")";
	}
	cout << endl;
	assert (!bm.isPlanar(GC3));
	SaveGraph(GC3, "ReversedK5");

	//Test it
	AssertGraphEquality(G, GC3);

	//Now solve the small graph
	OOCMCrossingMinimization cm (new MILP_lp_solve());
	CrossingMinimization::solve_result_t result = cm.solve(GC, s.getEdgeCosts());
	assert(result);
	GraphCopy GC5 = result->first;
	cout << "Crossing number: " << result->second << endl;

	//reverse simplification
	GraphCopy GC6 = s.reverseSimplification(GC5);
	SaveGraph(GC6, "ReversedSolvedK5");
	assert (GC6.consistencyCheck());

}

void Test_SimplificationDeg12_K5_2() {
	//Define a test graph that collapses to a K5
	Graph G;
	vector<node> nodes(13);
	for (int i=0; i<=12; ++i) nodes[i] = G.newNode();
	int edges[21][2] = {
		{0,2}, {0,3}, {0,4}, {1,2}, {1,3}, {1,4}, {2,3}, {2,4}, {3,4},
		{0,5}, {0,6}, {5,7}, {6,7},
		{7,8}, {7,9}, {8,10}, {9,10},
		{10,11}, {10,12}, {11,1}, {12,1}
	};
	int edgeCosts[10][3] = {
		{0,1,2}, {0,2,1}, {0,3,1}, {0,4,1},
		{1,2,1}, {1,3,1}, {1,4,1},
		{2,3,1}, {2,4,1}, {3,4,1}
	};
	for (int i=0; i<21; ++i) {
		G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
	}
	SaveGraph(G, "BlownEdgeK5");

	SimplificationDeg12 s (G);
	const GraphCopy& GC = s.getSimplifiedGraph();
	const unordered_map<edge, int>& edgeCostMap = s.getEdgeCosts();
	const GraphCopy& GC2 (GC);
	SaveGraph(GC, "SimplifiedEdgeK5");

	//GC now should be a K5 with nodes 0..4
	assert(5 == GC2.numberOfNodes());
	assert(10 == GC2.numberOfEdges());
	node u;
	forall_nodes(u, GC2) {
		assert(4 == u->degree());
	}
	assert(isConnected(GC2));
	assert(isSimpleUndirected(GC2));
	for (int i=0; i<10; ++i) {
		edge e = GC2.searchEdge(GC2.copy(nodes[edgeCosts[i][0]]), GC.copy(nodes[edgeCosts[i][1]]));
		assert (e!=NULL);
		int costExp = edgeCosts[i][2];
		int costActual = edgeCostMap.at(GC2.original(e));
		assert(costExp == costActual);
	}

	const GraphCopy GC3 (GC2);
	GraphCopy GC4 = s.reverseSimplification(GC3);
	SaveGraph(GC4, "ReversedEdgeK5");

	//Test it
	AssertGraphEquality(G, GC4);
}


void showBiconnectedComponents(Graph& G, const char* prefix)
{
	ogdf::GraphAttributes GA(G, 
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor | ogdf::GraphAttributes::nodeType
		| ogdf::GraphAttributes::edgeLabel);
	SList<node> nodes;
	G.allNodes(nodes);
	for (node n : nodes) {
		stringstream s;
		s << n->index();
		GA.labelNode(n) = s.str().c_str();
		GA.width(n) = GA.height(n) = 15;
	}

	EdgeArray<int> edgeArray (G);
	int count = biconnectedComponents(G, edgeArray);
	cout << count << " biconnected components found" << endl;
	edge e;
	forall_edges(e, G) {
		stringstream s;
		s << edgeArray[e];
		GA.labelEdge(e) = s.str().c_str();
	}

	SaveGraph(GA, prefix);
	SimplificationBiconnected sb (G);
	int i = 0;
	const vector<GraphCopy>& components = sb.getComponents();
	for (const GraphCopy& GC : components) {
		stringstream s;
		s << prefix << "_" << (++i);
		SaveGraph(GC, s.str().c_str());
	}

	vector<GraphCopy> modifiedComponents(components.size());
	for (int j=0; j<components.size(); ++j) {
		modifiedComponents[j] = GraphCopy(components[j]);
	}

	GraphCopy G2 = sb.reverseSimplification(modifiedComponents);
	AssertGraphEquality(G, G2);
}
void TestBiconnectedComponents() {
	//First Graph:
	{
	Graph G;
	vector<node> nodes(11);
	for (int i=0; i<=10; ++i) nodes[i] = G.newNode();
	int edges[24][2] = {
		{1,2},{1,3},{1,4},{1,5},{2,3},{2,4},{2,5},{3,4},{3,5},{4,5},
		{6,7},{6,8},{6,9},{6,10},{7,8},{7,9},{7,10},{8,9},{8,10},{9,10},
		{0,2},{0,3},
		{0,7},{0,10}
	};
	for (int i=0; i<24; ++i) {
		G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
	}
	showBiconnectedComponents(G, "Biconnected1");	
	}

	//Second Graph:
	{
	Graph G;
	vector<node> nodes(41);
	for (int i=0; i<=40; ++i) nodes[i] = G.newNode();
	int edges[52][2] = {
		{0,7}, {0,6}, {0,9}, {0,18}, {0,19}, {0,35},
		{1,8}, {1,9}, {1,26}, {1,10}, {1,20}, {1,16}, {1,28},
		{2,11}, {2,18}, {2,12}, {2,22}, {2,25}, {2,17},
		{3,14}, {3,19}, {3,20}, {3,13}, {3,21}, {3,23},
		{4,35}, {4,5}, {4,16}, {4,17}, {4,15},
		{5,6}, {7,8}, {10,11}, {12,13}, {14,15},
		{21,22}, {23,24}, {24,25},
		{26,31}, {26,27}, {27,28}, {28,29}, {30,31}, {31,32},
		{33,34}, {33,35}, {34,35}, {33,36}, {33,37}, {37,38}, {37,39}, {37,40}
	};
	for (int i=0; i<52; ++i) {
		G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
	}
	showBiconnectedComponents(G, "Biconnected2");
	}

	//Third Graph:
	{
	Graph G;
	vector<node> nodes(13);
	for (int i=0; i<=12; ++i) nodes[i] = G.newNode();
	int edges[21][2] = {
		{0,2}, {0,3}, {0,4}, {1,2}, {1,3}, {1,4}, {2,3}, {2,4}, {3,4},
		{0,5}, {0,6}, {5,7}, {6,7},
		{7,8}, {7,9}, {8,10}, {9,10},
		{10,11}, {10,12}, {11,1}, {12,1}
	};
	for (int i=0; i<21; ++i) {
		G.newEdge(nodes[edges[i][0]], nodes[edges[i][1]]);
	}
	showBiconnectedComponents(G, "Biconnected3");
	}
}

void Test_SimplificationBiconnected() {
	BoyerMyrvold bm;
	for (int n=7; n<=20; ++n) {
		for (int i=0; i<5; ++i) {
			Graph G = createRandomNonPlanarGraph(n);
			
			SimplificationBiconnected s (G);
			const vector<GraphCopy>& components = s.getComponents();

			vector<GraphCopy> modifiedComponents(components.size());
			for (int j=0; j<components.size(); ++j) {
				modifiedComponents[j] = GraphCopy(components[j]);
			}

			GraphCopy G2 = s.reverseSimplification(modifiedComponents);

			AssertGraphEquality(G, G2);
		}
	}
}


Graph createK5() {
	GraphGenerator gen;
	return *gen.createRandomGraph(5, 10);
}
void blowEdge(Graph& G, int nu, int nv, vector<int> blow)
{
	List<node> nodes;
	G.allNodes(nodes);
	node u = *nodes.get(nu);
	node v = *nodes.get(nv);
	G.delEdge(G.searchEdge(u, v));
	for (int b : blow) {
		if (b==1) {
			G.newEdge(u, v);
			continue;
		}

		vector<node> nodes (b-1);
		for (int i=0; i<b-1; ++i) {
			nodes[i] = G.newNode();
		}
		G.newEdge(u, nodes[0]);
		for (int i=1; i<b-1; ++i) {
			G.newEdge(nodes[i-1], nodes[i]);
		}
		G.newEdge(nodes[b-2], v);
	}
}
vector<int> makeVector(int i1, int i2=0, int i3=0, int i4=0, int i5=0)
{
	vector<int> v;
	v.push_back(i1);
	if (i2 != 0) {
		v.push_back(i2);
		if (i3 != 0) {
			v.push_back(i3);
			if (i4 != 0) {
				v.push_back(i4);
				if (i5 != 0) {
					v.push_back(i5);
				}
			}
		}
	}
	return v;
}
void TestDeg2SimplificationSolveImpl(Graph& G, string name)
{
	SaveGraph(G, name.c_str());

	SimplificationDeg12 s (G);
	GraphCopy GC1 = s.getSimplifiedGraph();
	
	GraphCopy GC2 = s.reverseSimplification(GraphCopy(GC1));
	AssertGraphEquality(G, GC2);
	
	MILP_lp_solve lp;
	OOCMCrossingMinimization oocm (&lp);
	CrossingMinimization::solve_result_t result = oocm.solve(GC1, s.getEdgeCosts());
	GraphCopy GC3 = result->first;
	
	BoyerMyrvold bm1;
	assert (bm1.isPlanar(GC3));

	GraphCopy GC4 = s.reverseSimplification(GC3);
	SaveGraph(GC4, (name + "Solved").c_str());
	assert (GC4.consistencyCheck());
	BoyerMyrvold bm2;
	assert (bm2.isPlanar(GC4));
}
void TestDeg2SimplificationSolve() 
{
	//First Test: Blow one edge without crossing
	Graph G = createK5();
	blowEdge(G, 0, 1, makeVector(2, 2));
	TestDeg2SimplificationSolveImpl(G, "Blow1");

	//Second Test: Blow every edge
	G = createK5();
	for (int x=0; x<5; ++x)
		for (int y=x+1; y<5; ++y)
			blowEdge(G, x, y, makeVector(2,2));
	TestDeg2SimplificationSolveImpl(G, "Blow2");

	//Third Test: Add path to every edge
	G = createK5();
	for (int x=0; x<5; ++x)
		for (int y=x+1; y<5; ++y)
			blowEdge(G, x, y, makeVector(1,3));
	TestDeg2SimplificationSolveImpl(G, "Blow3");
}


int _tmain(int argc, _TCHAR* argv[])
{
	//TestMinimization();
	//createRomeGraphsInfo("C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\example-data\\");
	//RomeMinimization();
	//Test_SimplificationDeg12_K5_1();
	//TestBiconnectedComponents();
	//Test_SimplificationBiconnected();
	TestDeg2SimplificationSolve();

	cout << "Press a key to exit ... " << endl;
	cin.clear();
	cin.get();
	return 0;
}

