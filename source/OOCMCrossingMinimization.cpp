#include "stdafx.h"
#include <OOCMCrossingMinimization.h>
#include <ogdf_include.h>
#include <ogdf/basic/GraphCopy.h>
#include <ogdf/basic/SList.h>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/null.hpp>
#include <boost/log/trivial.hpp>
#include <algorithm>
#include <limits>
#include <map>
#include <math.h>

namespace std {
	bool operator==(const shaman::OOCMCrossingMinimization::crossing& a,
		const shaman::OOCMCrossingMinimization::crossing& b) {
		return ( (*a.first == *b.first && *a.second == *b.second)
			|| (*a.second == *b.first && *a.first == *b.second) );
	}
	bool operator!=(const shaman::OOCMCrossingMinimization::crossing& a, 
		const shaman::OOCMCrossingMinimization::crossing& b) {
		return ! (a==b);
	}
	bool operator==(const shaman::OOCMCrossingMinimization::crossingOrder& a,
		const shaman::OOCMCrossingMinimization::crossingOrder& b) {
			return ( *get<0>(a) == *get<0>(b) && *get<1>(a) == *get<1>(b) && *get<2>(a) == *get<2>(b) );
	}
	bool operator!=(const shaman::OOCMCrossingMinimization::crossingOrder& a, 
		const shaman::OOCMCrossingMinimization::crossingOrder& b) {
			return !(a==b);
	}
}

namespace shaman {

using namespace std;
using namespace ogdf;

#define PRINT_CROSSING(os, c) (os) << " (" << (c).first->source()->index() << "," << (c).first->target()->index() << ")x(" << (c).second->source()->index() << "," << (c).second->target()->index() << ")"
#define PRINT_CROSSING_ORDER(os, o) (os) << " (" << get<0>(o)->source()->index() << "," << get<0>(o)->target()->index() << "),(" << get<1>(o)->source()->index() << "," << get<1>(o)->target()->index() << "),(" << get<2>(o)->source()->index() << "," << get<2>(o)->target()->index() << ")"

#define LOG_REALIZE_DEBUG_ENABLED 0
#define LOG_LEVEL_REALIZE_ERROR error
#define LOG_KURATOWSKI_DEBUG_ENABLED 0
#define LOG_LEVEL_KURATOWSKI_ERROR error
#define LOG_LEVEL_LO_ERROR error
#define LOG_LEVEL_SOLVE_INFO info
#define LOG_LEVEL_SOLVE_WARNING warning
#define LOG(level) BOOST_LOG_TRIVIAL(level)

OOCMCrossingMinimization::OOCMCrossingMinimization(MILP* lp)
	: lp(lp)
{
}


OOCMCrossingMinimization::~OOCMCrossingMinimization(void)
{
}

OOCMCrossingMinimization::solve_result_t OOCMCrossingMinimization::solve(
	const ogdf::GraphCopy& originalGraph, const edge_cost_t& edgeCosts)
{
	//pre-check for planarity
	if (boyerMyrvold.isPlanar(originalGraph)) {
		return solve_result_t(make_pair(originalGraph, 0));
	}
	//pre-check if every node is not a dummy
	node testN;
	forall_nodes(testN, originalGraph) {
		if (originalGraph.isDummy(testN)) {
			LOG(error) << "Input graph contains dummy nodes";
			return solve_result_t();
		}
	}

	int n = originalGraph.numberOfNodes();
	int m = originalGraph.numberOfEdges();
	boost::iostreams::stream< boost::iostreams::null_sink > nullOstream( ( boost::iostreams::null_sink() ) );

	//create variables
	SList<edge> edgeVector;
	originalGraph.allEdges(edgeVector);
	vector<crossing> crossings;
	vector<crossingOrder> crossingOrders;
	crossingOrderMap_t crossingOrderMap;
	createVariables(originalGraph, crossings, crossingOrders);
	createCrossingOrdersMap(crossingOrders, crossingOrderMap);
	vector<bool> variables (crossings.size() + crossingOrders.size());
	unordered_map<node, int> crossingNodes;

	//setup lp model
	if (!lp->initialize(crossings.size() + crossingOrders.size()))
		return solve_result_t();
	for (unsigned int i=1; i<=crossings.size() + crossingOrders.size(); ++i) {
		if (!lp->setVariableType(i, MILP::VariableType::Binary))
			return solve_result_t();
	}
	if (!setObjectiveFunction(crossings, lp, originalGraph, edgeCosts))
		return solve_result_t();
	int crLower = max(1, crGLower(n, m));
	int crUpper = crKnUpper(n);
	if (!addCrossingNumberConstraints(crossings, crLower, crUpper, lp))
		return solve_result_t();
	if (!addLinearOrderingConstraints(edgeVector, crossings, crossingOrderMap, lp))
		return solve_result_t();

	//Main loop
	vector<bool> oldVariables;
	while (true) 
	{
		MILP::real* resultVariables;
		MILP::real objective;
		MILP::SolveResult result = lp->solve(&objective, &resultVariables);
		LOG(LOG_LEVEL_SOLVE_INFO) << "Solved, result: " << (int) result;
		if (result != MILP::SolveResult::Optimal) {
			return solve_result_t();
		}
		LOG(LOG_LEVEL_SOLVE_INFO) << "objective: " << objective;
		for (unsigned int i=0; i<variables.size(); ++i) {
			variables[i] = resultVariables[i] >= 0.999;
		}
		{
		stringstream s;
		s << "variables set to one:";
		for (unsigned int i=0; i<crossings.size(); ++i) {
			if (variables[i]) {
				const crossing& c = crossings[i];
				PRINT_CROSSING(s, c);
			}
		}
		for (unsigned int i=0; i<crossingOrders.size(); ++i) {
			if (variables[i + crossings.size()]) {
				const crossingOrder& o = crossingOrders[i];
				PRINT_CROSSING_ORDER(s, o);
			}
		}
		LOG(LOG_LEVEL_SOLVE_INFO) << s.str();
		}
		if (variables == oldVariables) {
			LOG(LOG_LEVEL_SOLVE_WARNING) << "Nothing has changed between this loop and the last loop -> terminate";
			return solve_result_t();
		}
		oldVariables = variables;

		//realize graph
		crossingNodes.clear();
		GraphCopy G (originalGraph);
		realize(originalGraph, G, crossings, crossingOrderMap, variables, crossingNodes);

		//check if the graph is now planar
		SList<KuratowskiWrapper> kuratowski_edges;
		if (boyerMyrvold.planarEmbed(GraphCopySimple(G), kuratowski_edges, BoyerMyrvoldPlanar::doFindUnlimited, true)) 
		{
			//graph is planar
			return solve_result_t(make_pair(G, (unsigned int) floor(objective + 0.5)));
		}
		else
		{
			LOG(LOG_LEVEL_SOLVE_INFO) << "Number of detected Kuratowski-Subdivisions: " << kuratowski_edges.size();
			for (KuratowskiWrapper w : kuratowski_edges) {
				//cout << "Kuratowki-Subgraph:";
				//for (const auto& e : w.edgeList) {
				//	cout << " (" << e->source()->index() << "," << e->target()->index() << ")" ;
				//}
				//cout << "  count=" << kuratowski_edges.size();
				//cout << endl;

				if (!addKuratowkiConstraints(edgeVector, crossings, crossingOrderMap, variables, w, G, crossingNodes, lp))
					return solve_result_t();

				//break; //only one kuratowski subgraph
			}
			LOG(LOG_LEVEL_SOLVE_INFO) << "Kuratowski constraints added";
		}

		//if (true) {
		//	return solve_result_t();
		//}
	}

}

void OOCMCrossingMinimization::createVariables(
	const Graph& originalG, vector<crossing>& outCrossings, vector<crossingOrder>& outCrossingOrders)
{
	outCrossings.clear();
	outCrossingOrders.clear();
	int m = originalG.numberOfEdges();
	SList<edge> edgeVector;
	originalG.allEdges(edgeVector);
	//create x_{e,f} variables: all combinations of edges (unordered)
	for (SList<edge>::iterator it1 = edgeVector.begin(); it1 != edgeVector.end(); ++it1) {
		for (SList<edge>::iterator it2 = it1.succ(); it2 != edgeVector.end(); ++it2) {
			//ignore adjacent and same edges
			const edge& e = *it1;
			const edge& f = *it2;
			if (e->source() == f->source() || e->source() == f->target()
				|| e->target() == f->source() || e->target() == f->target())
				continue;

			crossing c = minmax(*it1, *it2);
			outCrossings.push_back(c);
		}
	}
	sort(outCrossings.begin(), outCrossings.end());
	//create y_(e,f,g) variables: all ordered combinations of three edges
	for (edge e : edgeVector) {
		for (edge f : edgeVector) {
			if (e==f) continue;
			for (edge g : edgeVector) {
				if (e==g || f==g) continue;
				crossingOrder o = make_tuple(e, f, g);
				outCrossingOrders.push_back(o);
			}
		}
	}
}

void OOCMCrossingMinimization::createCrossingOrdersMap(
	const vector<crossingOrder>& crossingOrders, crossingOrderMap_t& outMap)
{
	outMap.clear();
	for (unsigned int i=0; i<crossingOrders.size(); ++i) {
		//outMap.emplace(crossingOrders[i], i);
		const crossingOrder& o = crossingOrders[i];
		outMap[get<0>(o)][get<1>(o)][get<2>(o)] = i;
	}
}

void OOCMCrossingMinimization::realize(const ogdf::GraphCopy& originalG, 
	GraphCopy& G, const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrdersMap,
	const vector<bool>& variableAssignment, unordered_map<ogdf::node, int>& crossingNodes)
{	
	int crossingsSize = crossings.size();
	int io = 0;
	//maps the source edges to the crossing edges with the inducing variable
	unordered_map< edge, vector< pair<edge, int> > > crossingMap;

	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].first;

		//collect all crossings with this edge
		vector< pair<edge, int> >& ex = crossingMap[e];
		int j = i;
		for (; j<crossingsSize && crossings[j].first==e; ++j) {
			if (variableAssignment[j]) {
				bool duplicate = false;
				for (const pair<edge, int>& i : ex) {
					if (i.second == j) {
						duplicate = true;
						break;
					}
				}
				if (duplicate) continue;

				ex.push_back(make_pair(crossings[j].second, j));
			}
		}
		//i = j;
	}
	for (int i=0; i<crossingsSize; ++i) {
		if (!variableAssignment[i]) continue;

		edge e = crossings[i].second;

		//collect all crossings with this edge
		vector< pair<edge, int> >& ex = crossingMap[e];
		for (int j = 0; j<crossingsSize; ++j) {
			if (variableAssignment[j] && crossings[j].second==e )
				//&& indexOf(ex, pair<edge, int>(crossings[j].first, j))==-1)
			{
				bool duplicate = false;
				for (const pair<edge, int>& i : ex) {
					if (i.second == j) {
						duplicate = true;
						break;
					}
				}
				if (duplicate) continue;
				ex.push_back(make_pair(crossings[j].first, j));
			}
		}
	}

	//specify order
	for (auto& a : crossingMap) {
		edge e = a.first;
		vector< pair<edge, int> >& ex = a.second;

		if (ex.size() > 1) {
			map<crossingOrder, bool> cache;
			for (pair<edge, int> f : ex) {
				for (pair<edge, int> g : ex) {
					crossingOrder o = make_tuple(e, f.first, g.first);
					//map< crossingOrder, int >::const_iterator it = crossingOrdersMap.find(o);
					//if (it == crossingOrdersMap.end()) 
					//	continue;
					//int index = it->second + crossings.size();
					const auto& it1 = crossingOrdersMap.find(e);
					if (it1 == crossingOrdersMap.end()) continue;
					const auto& it2 = it1->second.find(f.first);
					if (it2 == it1->second.end()) continue;
					const auto& it3 = it2->second.find(g.first);
					if (it3 == it2->second.end()) continue;
					int index = it3->second + crossings.size();
					bool result = variableAssignment[index];
					cache[o] = result;
				}
			}
			sort(ex.begin(), ex.end(), [=](const pair<edge, int>& f, const pair<edge, int>& g) -> bool
			{
				crossingOrder o = make_tuple(e, f.first, g.first);
				map<crossingOrder, bool>::const_iterator it = cache.find(o);
				if (it == cache.end())
					return false;
				else
					return it->second;
			});
		}

	}

	//replace all edges with their copied counterparts
	unordered_map< edge, vector< pair<edge, int> > > crossingMap2;
	for (const auto& entry : crossingMap) {
		vector< pair<edge, int> > ex (entry.second.size());
		for (int i=0; i<entry.second.size(); ++i) {
			ex[i] = make_pair(G.copy(originalG.original(entry.second[i].first)), entry.second[i].second);
		}
		crossingMap2.emplace(G.copy(originalG.original(entry.first)), ex);
	}
	crossingMap = crossingMap2;

	//debug
#if LOG_REALIZE_DEBUG_ENABLED == 1
	{
		stringstream s;
		for (const auto& a : crossingMap) {
			edge e = a.first;
			vector< pair<edge, int> > ex = a.second;
			s << "edge (" << e->source()->index() << "," << e->target()->index() << ") crosses with";
			for (pair<edge, int> f : ex)
				s << " (" << f.first->source()->index() << "," << f.first->target()->index() << ")->" << f.second;
		}
		LOG(debug) << s.str();
	}
#endif

	//now split edges
	while (!crossingMap.empty())
	{
		edge e = crossingMap.begin()->first;
		vector< pair<edge, int> > ex = crossingMap.begin()->second;
		int s1 = crossingMap.size();
		crossingMap.erase(e);
		assert(crossingMap.size() == s1-1);

#if LOG_REALIZE_DEBUG_ENABLED == 1
		stringstream s;
		s << "Introduce crossing between "
			<< "(" << e->source()->index() << "," << e->target()->index() << ")"
			<< " and";
		s << endl;
#endif

		//split edges
		node u = e->source();
		const node eTarget = e->target();
		int counter = 0;
		for (pair<edge, int>& f : ex) {
#if LOG_REALIZE_DEBUG_ENABLED == 1
			if (counter > 0)
				s << endl;
			else
				counter++;
			s << " (" << f.first->source()->index() << "," << f.first->target()->index() << ")->" << f.second;
#endif

			//check where f crosses e
			vector< pair<edge, int> > fx = crossingMap[f.first];
			crossingMap.erase(f.first);
			int index = indexOf(fx, make_pair(e, f.second));
			if (index < 0) {
				stringstream str;
				str << "ERROR: edge (" << e->source()->index() << "," << e->target()->index() << ")"
					<< " not found in edge list of (" << f.first->source()->index() << "," << f.first->target()->index() << "): ";
				for (pair<edge, int> h : fx) {
					str << " (" << h.first->source()->index() << "," << h.first->target()->index() << ")";
				}
				LOG(LOG_LEVEL_REALIZE_ERROR) << str;
				assert(index >= 0);
				return;
			}

			//introduce crossing node
//			node n = G.newNode();
//			crossingNodes.emplace(n, f.second);
//#if LOG_REALIZE_DEBUG_ENABLED == 1
//			s << " -> node " << n->index();
//#endif
//
//			//remove old and add new edges
//			edge d1 = G.searchEdge(u, eTarget);
//			edge d2 = f.first;
//			assert (d1 != NULL);
//			assert (d2 != NULL);
//			G.delEdge(d1);
//			node fSource = f.first->source();
//			node fTarget = f.first->target();
//			G.delEdge(d2);
//			edge d1c1 = G.newEdge(u, n);
//			edge d1c2 = G.newEdge(eTarget, n);
//			edge ef1 = G.newEdge(fSource, n);
//			edge ef2 = G.newEdge(fTarget, n);
//			u = n;

			//introduce crossing
			edge d1 = G.searchEdge(u, eTarget);
			edge d2 = f.first;
			node fSource = f.first->source();
			node fTarget = f.first->target();
			edge e = G.insertCrossing(d1, d2, false);
			node n = e->source();
			assert (n != u);
			assert (n != eTarget);
			assert (n != fSource);
			assert (n != fTarget);
			assert (G.isDummy(n));
			crossingNodes.emplace(n, f.second);
#if LOG_REALIZE_DEBUG_ENABLED == 1
			s << " -> node " << n->index();
#endif
			u = n;
			edge ef1 = G.searchEdge(fSource, n);
			edge ef2 = G.searchEdge(fTarget, n);
			assert (ef1 != NULL);
			assert (ef2 != NULL);

			//split fx
			vector<pair<edge, int>> fx1, fx2;
#if LOG_REALIZE_DEBUG_ENABLED == 1
			s << " fx1:";
#endif
			for (int i=0; i<index; ++i) {
				fx1.push_back(fx[i]);
#if LOG_REALIZE_DEBUG_ENABLED == 1
				s << " (" << fx[i].first->source()->index() << "," << fx[i].first->target()->index() << ")";
#endif
				assert (crossingMap.count(fx[i].first) > 0);
				vector<pair<edge, int>>& fxx = crossingMap[fx[i].first];
				//replace(fxx.begin(), fxx.end(), f, make_pair((edge) minmax(f.first.first, node), f.second));
				for (pair<edge, int>& h : fxx) {
					if (h.first == f.first) h.first = ef1; //minmax(f.first.first, node);
				}
#if LOG_REALIZE_DEBUG_ENABLED == 1
				s << "[fxx:";
				for (pair<edge, int> h : crossingMap[fx[i].first])
					s << "(" << h.first->source()->index() << "," << h.first->target()->index() << ")";
				s << "]";
#endif
			}
#if LOG_REALIZE_DEBUG_ENABLED == 1
			s << " fx2:";
#endif
			for (unsigned int i=index+1; i<fx.size(); ++i) {
				fx2.push_back(fx[i]);
#if LOG_REALIZE_DEBUG_ENABLED == 1
				s << " (" << fx[i].first->source()->index() << "," << fx[i].first->target()->index() << ")";
#endif
				assert (crossingMap.count(fx[i].first) > 0);
				vector<pair<edge, int>>& fxx = crossingMap[fx[i].first];
				//replace(fxx.begin(), fxx.end(), f, make_pair((edge) minmax(f.first.second, node), f.second));
				for (pair<edge, int>& h : fxx) {
					if (h.first == f.first) h.first = ef2; //minmax(f.first.second, node);
				}
#if LOG_REALIZE_DEBUG_ENABLED == 1
				s << "[fxx:";
				for (pair<edge, int> h : crossingMap[fx[i].first])
					s << "(" << h.first->source()->index() << "," << h.first->target()->index() << ")";
				s << "]";
#endif
			}
			//reverse(fx2.begin(), fx2.end()); //not needed anymore when using G.insertCrossing(...)
			if (!fx1.empty()) {
				crossingMap.emplace(ef1, fx1); 
			}
			if (!fx2.empty()) {
				crossingMap.emplace(ef2, fx2);
			}

		} //end split edges

#if LOG_REALIZE_DEBUG_ENABLED == 1
		LOG(debug) << s.str();
#endif
	} //end while(!crossingMap.empty())

	return;
}

bool OOCMCrossingMinimization::setObjectiveFunction(
	const vector<crossing>& crossings, MILP* lp, const ogdf::GraphCopy& originalGraph, const edge_cost_t& edgeCosts)
{
	vector<MILP::real> row (crossings.size());
	vector<int> colno (crossings.size());
	for (unsigned int i=0; i<crossings.size(); ++i) {
		edge oe = originalGraph.original(crossings[i].first);
		edge of = originalGraph.original(crossings[i].second);
		assert (oe != NULL);
		assert (of != NULL);
		int cost = edgeCosts.at(oe) * edgeCosts.at(of);
		row[i] = cost; 
		colno[i] = i+1;
	}
	bool result = lp->setObjectiveFunction(crossings.size(), &row[0], &colno[0], MILP::Direction::Minimize);
	return result;
}

bool OOCMCrossingMinimization::addCrossingNumberConstraints(
	const vector<crossing>& crossings, int crLower, int crUpper, MILP* lp)
{
	int count = crossings.size();
	vector<MILP::real> row (count);
	vector<int> colno (count);
	for (int i=0; i<count; ++i) {
		row[i] = 1;
		colno[i] = i+1;
	}
	return lp->addConstraint(count, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, crLower)
		&& lp->addConstraint(count, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, crUpper);
}

bool OOCMCrossingMinimization::addLinearOrderingConstraints(
	const SList<edge>& edges, const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap, MILP* lp)
{
	//This method is very slow (~15sec for the K8)
	//Improve it

	int countX = crossings.size();
	int countY = crossingOrderMap.size();
	
	if (!lp->setAddConstraintMode(true)) 
		return false;

	map<crossing, int> crossingMap;
	for (int i=0; i<countX; ++i)
		crossingMap[crossings[i]] = i;

	vector<MILP::real> row(4);
	vector<int> colno(4);
	for (const auto& m1 : crossingOrderMap) {
		const edge& e = m1.first;
		for (const auto& m2 : m1.second) {
			const edge& f = m2.first;
			for (const auto& m3 : m2.second) {
				const edge& g = m3.first;
				int index = m3.second;
				//add crossing-existence constraint
				map<crossing, int>::iterator it1 = crossingMap.find(minmax(e, f));
				int x_ef = -1;
				if (it1 != crossingMap.end()) x_ef = it1->second + 1;
				map<crossing, int>::iterator it2 = crossingMap.find(minmax(e, g));
				int x_eg = -1;
				if (it2 != crossingMap.end()) x_eg = it2->second + 1;
				row[1] = -1; colno[1] = index + countX + 1;
				if (x_ef > 0) {
					row[0] = 1; colno[0] = x_ef;
					if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
						return false;
				}
				if (x_eg > 0) {
					row[0] = 1; colno[0] = x_eg;
					if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, 0))
						return false;
				}
				//add order-existence constraint
				//TODO: constraint is added twice, check f<g
				if (x_ef > 0 && x_eg > 0) {
					row[0] = 1; colno[0] = index + countX + 1;
					row[1] = 1; colno[1] = crossingOrderMap.at(e).at(g).at(f) + countX + 1;
					row[2] = -1; colno[2] = x_ef;
					row[3] = -1; colno[3] = x_eg;
					if (!lp->addConstraint(4, &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, -1))
						return false;
				}
				//add mirror-order constraint
				//TODO: constraint is added twice, check f<g
				row[0] = 1; colno[0] = index + countX + 1;
				row[1] = 1; colno[1] = crossingOrderMap.at(e).at(g).at(f) + countX + 1;
				if (!lp->addConstraint(2, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, 1))
				return false;
			}
		}
	}

	//add cyclic-order-constraint
	for (const edge& e : edges) {
		if (crossingOrderMap.count(e) == 0) {
			LOG(LOG_LEVEL_LO_ERROR) << "no crossing order found for edge e=(" << e->source()->index() 
				<< "," << e->target()->index() << ")";
			return false;
		}
		const auto& me = crossingOrderMap.at(e);
		for (const auto& m1 : me) {
			const edge& f = m1.first;
			for (const auto& m2 : m1.second) {
				const edge& g = m2.first;
				const int i1 = m2.second;
				for (edge h : edges) {
					if (e!=h && h>f && h>g) {
						const auto& it1 = me.find(g);
						if (it1 == me.end()) continue;
						const auto& it2 = it1->second.find(h);
						if (it2 == it1->second.end()) continue;
						const int i2 = it2->second;
						const auto& it3 = me.find(h);
						if (it3 == me.end()) continue;
						const auto& it4 = it3->second.find(f);
						if (it4 == it3->second.end()) continue;
						const int i3 = it4->second;

						row[0] = 1; colno[0] = i1 + countX + 1;
						row[1] = 1; colno[1] = i2 + countX + 1;
						row[2] = 1; colno[2] = i3 + countX + 1;
						if (!lp->addConstraint(3, &row[0], &colno[0], MILP::ConstraintType::LessThanEqual, 2))
							return false;
					}
				}
			}
		}
	}

	if (!lp->setAddConstraintMode(false))
		return false;

	return true;
}

bool OOCMCrossingMinimization::addKuratowkiConstraints(const SList<edge>& edges,
	const vector<crossing>& crossings, const crossingOrderMap_t& crossingOrderMap,
	const vector<bool>& variableAssignment, const KuratowskiWrapper& kuratowski_edges, 
	const GraphCopy& realizedGraph, const unordered_map<node, int>& crossingNodes, MILP* lp)
{
	int crossingCount = crossings.size();

	//simplifyKuratowskiSubgraph(kuratowski_edges);

	//create a mapping from the variable index to the crossings (0-based because the assignment is 0-based)
	map<int, crossing> crossingVariableMap;
	for (int i=0; i<crossingCount; ++i) {
		crossingVariableMap[i] = crossings[i];
	}

	//collect set of all nodes in the kuratowski subdivision
	map<node, int> kuratowskiNodes;
	for (edge e : kuratowski_edges.edgeList) {
		kuratowskiNodes[e->source()]++;
		kuratowskiNodes[e->target()]++;
	}
	//maps the nodes created by the path subdivisions to their end-nodes (called kuratowski nodes)
	map<node, pair<node, node>> kuratowskiPathToNodes;
	for (pair<node, int> p : kuratowskiNodes) {
		//if (p.second != 2) continue; //a subdivision node always has degree two
		if (!realizedGraph.isDummy(p.first)) continue; //a subdivision node has no parent -> dummy

		//follow the path in the edge list until a node with deg(n)>2 is reached
		const node n = p.first;
		node u = NULL,v = NULL;
		for (edge e : kuratowski_edges.edgeList) {
			node t;
			if (e->source() == n)
				t = e->target();
			else if (e->target() == n)
				t = e->source();
			else
				continue;
			node ot = n;
			while (kuratowskiNodes.at(t) == 2) {
				node t2 = NULL;
				for (edge e2 : kuratowski_edges.edgeList) {
					if (e2->source() == t && e2->target() != ot)
						t2 = e2->target();
					else if (e2->target() == t && e2->source() != ot)
						t2 = e2->source();
					else
						continue;
					break;
				}
				assert (t2 != NULL);
				//next node
				ot = t;
				t = t2;
			}
			if (u == NULL)
				u = t;
			else {
				v = t;
				break;
			}
		}
		assert (u != NULL);
		assert (v != NULL);
		kuratowskiPathToNodes[n] = minmax(u, v);
	}

#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "Kuratowski-Nodes:";
	for (pair<node, int> p : kuratowskiNodes) {
		s << "  " << p.second << "x" << p.first->index();
	}
	s << endl;
	s << "  Subdivision-Nodes go to:";
	for (pair<node, pair<node, node> > p : kuratowskiPathToNodes) {
		s << "  " << p.first->index() << "->{" << p.second.first->index() << "," << p.second.second->index() << "}";
	}
	LOG(debug) << s.str();
	}
#endif

	//create Zk, the set of all induced crossings whose dummy nodes are part of the kuratowski subdivision
	set<crossing> Zk;
	for (pair<node, int> p : kuratowskiNodes) {
		node n = p.first;
		unordered_map<node, int>::const_iterator it = crossingNodes.find(n);
		if (it != crossingNodes.end()) {
			//it is a crossing node
			int variable = it->second;
			Zk.insert(crossingVariableMap.at(variable));
		}
	}
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "Zk:";
	for (const crossing& c : Zk) {
		PRINT_CROSSING(s, c);
	}
	LOG(debug) << s.str();
	}
#endif

	//create Yk
	set<crossingOrder> Yk;
	for (const crossing& c1 : Zk) {
		for (const crossing& c2 : Zk) {
			edge e,f,g;
			if (c1.first == c2.first) {
				e = c1.first; f = c1.second; g = c2.second;
			} else if (c1.first == c2.second) {
				e = c1.first; f = c1.second; g = c2.first;
			} else if (c1.second == c2.first) {
				e = c1.second; f = c1.first; g = c2.second;
			} else if (c1.second == c2.second) {
				e = c1.second; f = c1.first; g = c2.first;
			} else {
				continue;
			}
			int y_efg = getCrossingOrderVariable(crossingOrderMap, e, f, g, -1);
			if (y_efg == -1) continue;
			if (!variableAssignment[y_efg + crossingCount]) continue; //y_e,f,g does not exists or is false

			//check if there is no other edge h in between
			bool existsH = false;
			for (const crossing& c3 : Zk) {
				edge h;
				if (c3.first == e) {
					h = c3.second;
				} else if (c3.second == e) {
					h = c3.first;
				} else {
					continue;
				}
				if (h==f || h==g) continue;
				int y_efh = getCrossingOrderVariable(crossingOrderMap, e, f, h, -1);
				if (y_efh == -1) continue;
				int y_ehg = getCrossingOrderVariable(crossingOrderMap, e, h, g, -1);
				if (y_ehg == -1) continue;
				if (variableAssignment[y_efh + crossingCount] && variableAssignment[y_ehg + crossingCount]) {
					existsH = true; //an edge h found that crosses between f and g
					break;
				}
			}
			if (existsH) continue;

			//triple found
			Yk.insert(crossingOrder(e, f, g));
		}
	}
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "Yk:";
	for (const crossingOrder& o : Yk) {
		PRINT_CROSSING_ORDER(s, o);
	}
	LOG(debug) << s.str();
	}
#endif
	//Create Xk
	set<crossing> Xk;
	for (const crossing& c : Zk) {
		const edge& e = c.first;
		const edge& f = c.second;
		for (const edge& g : edges) {
			set<crossingOrder> s;
			s.insert(crossingOrder(e, f, g));
			s.insert(crossingOrder(e, g, f));
			s.insert(crossingOrder(f, e, g));
			s.insert(crossingOrder(f, g, e));
			if (is_disjoint(s, Yk)) {
				Xk.insert(c);
			}
		}
	}
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "Xk:";
	for (const crossing& c : Xk) {
		PRINT_CROSSING(s, c);
	}
	LOG(debug) << s.str();
	}
#endif

	//Create CrPairs(K)
	set<crossing> CrPairs;
	set<pair<node, node>> kuratowskiEdgesInG; //the kuratowski edges in the original graph
	for (pair<node, int> p : kuratowskiNodes) {
		node n = p.first;
		if (crossingNodes.count(n) > 0) continue; //crossing node
		//it is a node in the original graph, no walk along all outgoing edges until another original node is found
		vector<node> targetNodes;
		for (const auto& e : kuratowski_edges.edgeList) {
			node v;
			if (e->source() == n)
				v = e->target();
			else if (e->target() == n)
				v = e->source();
			else
				continue;
			if (crossingNodes.count(v) > 0) continue;
			targetNodes.push_back(v);
		}
		//add found targets
		for (node v : targetNodes)
			kuratowskiEdgesInG.insert(minmax(n, v));
	}
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "Paths in K that are edges in G:";
	for (const auto& e : kuratowskiEdgesInG) {
		s << " (" << e.first << "," << e.second << ")";
	}
	LOG(debug) << s.str();
	}
#endif

	//now loop over all pairs of these edges, find non-adjacent ones
	for (const auto& e : kuratowskiEdgesInG) {
		node u1 = e.first;
		node v1 = e.second;
		set<node> s1;
		if (kuratowskiPathToNodes.count(u1)>0) {
			//subdivision-node, replace it
			pair<node, node> p = kuratowskiPathToNodes.at(u1);
			s1.insert(p.first); s1.insert(p.second);
		} else {
			s1.insert(u1);
		}
		if (kuratowskiPathToNodes.count(v1)>0) {
			//subdivision-node, replace it
			pair<node, node> p = kuratowskiPathToNodes.at(v1);
			s1.insert(p.first); s1.insert(p.second);
		} else {
			s1.insert(v1);
		}

		for (const pair<node, node>& f : kuratowskiEdgesInG) {
			node u2 = f.first;
			node v2 = f.second;
			set<node> s2;
			if (kuratowskiPathToNodes.count(u2)>0) {
				//subdivision-node, replace it
				pair<node, node> p = kuratowskiPathToNodes.at(u2);
				s2.insert(p.first); s1.insert(p.second);
			} else {
				s2.insert(u2);
			}
			if (kuratowskiPathToNodes.count(v2)>0) {
				//subdivision-node, replace it
				pair<node, node> p = kuratowskiPathToNodes.at(v2);
				s2.insert(p.first); s2.insert(p.second);
			} else {
				s2.insert(v2);
			}

			if (is_disjoint(s1, s2)) {
				//add this crossing
				//pair<edge, edge> c = minmax(e, f);
				//if (binary_search(crossings.begin(), crossings.end(), c)) {
				//vector<crossing>::const_iterator it = find(crossings.begin(), crossings.end(), c);
				vector<crossing>::const_iterator it;
				for (it = crossings.begin(); it!=crossings.end(); ++it) {
					node u1 = realizedGraph.copy(it->first->source());
					node v1 = realizedGraph.copy(it->first->target());
					node u2 = realizedGraph.copy(it->second->source());
					node v2 = realizedGraph.copy(it->second->target());
					if (( (u1 == e.first && v1 == e.second)
						 || (v1 == e.first && u1 == e.second))
						&& ( (u2 == f.first && v2 == f.second)
						 || (v2 == f.first && u2 == f.second)) ) {
							 break; //found
					}
				}
				if (it ==  crossings.end()) continue;
				int index = it - crossings.begin();
				if (variableAssignment[index]) continue; //Already form a crossing, but not used
				//add it
				CrPairs.insert(*it);
			}
		}
	}
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "CrPairs:";
	for (const crossing& c : CrPairs) {
		PRINT_CROSSING(s, c);
	}
	LOG(debug) << s.str();
	}
#endif

#if defined(DEBUG_BUILD) || defined(_DEBUG)
	//Validation
	//Every variable in CrPairs must be assigned to zero
	for (const crossing& c : CrPairs) {
		int index = indexOf(crossings, c); //TODO: binary search
		assert (index >= 0);
		bool assignment = variableAssignment[index];
		assert (assignment == false);
	}
	//Every variable in Xk and Yk must be assigned to one
	for (const crossing& c : Xk) {
		int index = indexOf(crossings, c);
		assert (index >= 0);
		bool assignment = variableAssignment[index];
		assert (assignment == true);
	}
	for (const crossingOrder& o : Yk) {
		int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
		bool assignment = variableAssignment[index + crossingCount];
		assert (assignment == true);
	}
#endif

	//now setup the equation
	vector<MILP::real> row;
	vector<int> colno;
	for (const crossing& c : CrPairs) {
		int index = indexOf(crossings, c); //TODO: binary search
		if (index >= 0) {
			row.push_back(1);
			colno.push_back(index + 1);
		}
	}
	for (const crossing& c : Xk) {
		int index = indexOf(crossings, c); //TODO: binary search
		assert (index >= 0);
		int index2 = indexOf(colno, index + 1);
		if (index2 >= 0) {
			//variable already used
			//row[index2]--;
			row[index2] = -1;
		} else {
			row.push_back(-1);
			colno.push_back(index + 1);
		}
	}
	for (const crossingOrder& o : Yk) {
		int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
		row.push_back(-1);
		colno.push_back(index + crossingCount + 1);
	}
	int rhs = 1 - Xk.size() - Yk.size();

	//Debug
#if LOG_KURATOWSKI_DEBUG_ENABLED == 1
	{
	stringstream s;
	s << "new constraint:" << endl;
	for (unsigned int i=0; i<colno.size(); ++i) {
		if (row[i] > 0)
			s << " + ";
		else if (row[i] == 0)
			s << " + 0*";
		else
			s << " - ";
		if (colno[i] <= crossingCount) {
			const crossing& c = crossings[colno[i]-1];
			s << "x";
			PRINT_CROSSING(s, c);
		} else {
			for (const crossingOrder o : Yk) {
				int index = crossingOrderMap.at(get<0>(o)).at(get<1>(o)).at(get<2>(o));
				if (index == colno[i] - 1 - crossingCount) {
					s << "y";
					PRINT_CROSSING_ORDER(s, o);
					break;
				}
			}
		}
	}
	s << " >= " << rhs;
	LOG(debug) << s.str();
	}
#endif

	//send to LP
	if (row.empty()) {
		LOG(LOG_LEVEL_KURATOWSKI_ERROR) << "empty constraint";
		return true;
	}
	return lp->addConstraint(colno.size(), &row[0], &colno[0], MILP::ConstraintType::GreaterThanEqual, rhs);
}


//void OOCMCrossingMinimization::simplifyKuratowskiSubgraph(
//	kuratowski_edges_t& kuratowski_edges)
//{
//	int size = kuratowski_edges.size();
//	//removes all redundant edges from the kuratowski subgraph
//	unordered_map<int, int> nodeOccurence;
//	//add node occurence
//	for (auto e : kuratowski_edges) {
//		nodeOccurence[e.m_source]++;
//		nodeOccurence[e.m_target]++;
//	}
//	//loop as long as nodes with an occurence of 1 are there
//	while(true)
//	{
//		bool found = false;
//		for(unordered_map<int, int>::value_type entry : nodeOccurence) {
//			if (entry.second == 1) {
//				//remove the edge going out from this node
//				remove_if(kuratowski_edges.begin(), kuratowski_edges.end(),
//					[=](boost::graph_traits<Graph>::edge_descriptor e) mutable -> bool {
//						if (e.m_source == entry.first) {
//							nodeOccurence[e.m_target]--;
//							return true;
//						} else if (e.m_target == entry.first) {
//							nodeOccurence[e.m_source]--;
//							return true;
//						} else {
//							return false;
//						}
//				});
//				nodeOccurence.erase(entry.first);
//				size--;
//				found = true;
//				break;
//			}
//		}
//		if (!found) break;
//	}
//	kuratowski_edges.resize(size);
//}

}