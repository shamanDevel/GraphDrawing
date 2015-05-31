#include "testui.h"
#include "QtBoostLogger.h"

#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/log/trivial.hpp>

#include <ogdf\planarlayout\PlanarDrawLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <ogdf\planarity\PlanarizationLayout.h>

#include <unordered_map>

#include <GraphConverter.h>

using namespace std;
using namespace shaman;

TestUI::TestUI(QWidget *parent)
	: QMainWindow(parent), originalLayout(0), solvedLayout(0), solverThread(NULL)
{
	setupUi();
	setupLogBackend();
	folder = "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\example-data\\";
	scanRomeGraphs(folder, graphs);
	initGraphTable();
	switchUIState(0);
}

TestUI::~TestUI()
{

}

void TestUI::setupUi()
{
    this->setObjectName(QStringLiteral("TestUIClass"));
    this->resize(1121, 746);
    
	centralWidget = new QWidget(this);
    centralWidget->setObjectName(QStringLiteral("centralWidget"));

	outputConsole = new QPlainTextEdit(centralWidget);
	clearOutputButton = new QToolButton(centralWidget);
	showDebugButton = new QToolButton(centralWidget);
	outputConsole->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::TextSelectableByKeyboard);
	clearOutputButton->setIcon(QIcon(QString(":/TestUI/clear.png")));
	clearOutputButton->setToolTip(QString("Clears the console"));
	showDebugButton->setIcon(QIcon(QString(":/TestUI/bug.png")));
	showDebugButton->setToolTip(QString("Show debug output"));
	clearOutputButton->setCheckable(false);
	showDebugButton->setCheckable(true);
	QGridLayout* outputLayout = new QGridLayout();
	outputLayout->addWidget(clearOutputButton, 0, 0, 1, 1);
	outputLayout->addWidget(showDebugButton, 1, 0, 1, 1);
	outputLayout->addWidget(outputConsole, 0, 1, 3, 1);
	outputLayout->setColumnStretch(1, 1);
	outputLayout->setRowStretch(2, 1);
	outputLayout->setSpacing(2);
	connect(clearOutputButton, SIGNAL(clicked()), this, SLOT(clearOutput()));

	nodeCountFilter = new QLineEdit(centralWidget);
	edgeCountFilter = new QLineEdit(centralWidget);
	QLabel *controlTitle = new QLabel(QString("Choose Graph"), centralWidget);
	nodeCountLabel = new QLabel(QString("Node Count (XX-XX):"), centralWidget);
	edgeCountLabel = new QLabel(QString("Edge Count (XX-XX):"), centralWidget);
	QLabel *listTitle = new QLabel(QString("Select Graph"), centralWidget);
	graphList = new QTableWidget(centralWidget);
	simplifyButton = new QPushButton(QString("Simplify"), centralWidget);
	solveButton = new QPushButton(QString("Solve and combine"), centralWidget);
	cancelButton = new QPushButton(QString("Cancel"), centralWidget);
	QStringList header;
	header << "n" << "m" << "Name";
	graphList->setColumnCount(3);
	graphList->setHorizontalHeaderLabels(header);
	QGridLayout* controlLayout = new QGridLayout();
	controlLayout->addWidget(controlTitle, 0, 0, 1, 2 );
	controlLayout->addWidget(nodeCountLabel, 1, 0, 1, 1);
	controlLayout->addWidget(nodeCountFilter, 1, 1, 1, 1);
	controlLayout->addWidget(edgeCountLabel, 2, 0, 1, 1);
	controlLayout->addWidget(edgeCountFilter, 2, 1, 1, 1);
	controlLayout->addWidget(listTitle, 3, 0, 1, 2);
	controlLayout->addWidget(graphList, 4, 0, 1, 2);
	controlLayout->addWidget(simplifyButton, 5, 0, 1, 2);
	controlLayout->addWidget(solveButton, 6, 0, 1, 2);
	controlLayout->addWidget(cancelButton, 7, 0, 1, 2);
	controlLayout->setRowStretch(4, 1);
	controlLayout->setColumnStretch(1, 1);
	controlLayout->setSpacing(2);
	connect(nodeCountFilter, SIGNAL(textChanged(const QString&)), this, SLOT(filterGraphs()));
	connect(edgeCountFilter, SIGNAL(textChanged(const QString&)), this, SLOT(filterGraphs()));
	connect(graphList, SIGNAL(cellClicked(int, int)), this, SLOT(graphSelected(int, int)));
	connect(simplifyButton, SIGNAL(clicked()), this, SLOT(simplifyGraph()));
	connect(solveButton, SIGNAL(clicked()), this, SLOT(solveGraph()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancelSolving()));

	originalGraphLayoutComboBox = new QComboBox(centralWidget);
	originalGraphView = new GraphView(centralWidget);
	solvedGraphLayoutComboBox = new QComboBox(centralWidget);
	solvedGraphView = new GraphView(centralWidget);
	QLabel* originalGraphTitle = new QLabel(QString("Original Graph    "), centralWidget);
	QLabel* solvedGraphTitle = new QLabel(QString("Solved Graph    "), centralWidget);
	QLabel *originalGraphLayoutLabel = new QLabel(QString("Layout:"), centralWidget);
	QLabel *solvedGraphLayoutLabel = new QLabel(QString("Layout:"), centralWidget);
	crossingNumberLabel = new QLabel(QString(""), centralWidget);
	originalGraphLayoutComboBox->addItems(QStringList() << "Spring (FMMM)" << "Planarization" );
	solvedGraphLayoutComboBox->addItems(QStringList() << "Mixed Model" << "Planar Draw" << "Planar Straight" << "Spring (FMMM)" );
	originalLayout = 0;
	saveOriginalGraphButton = new QPushButton();
	QGridLayout* graphLayoutLeft = new QGridLayout();
	graphLayoutLeft->addWidget(originalGraphTitle, 0, 0, 1, 1);
	graphLayoutLeft->addWidget(originalGraphLayoutLabel, 0, 1, 1, 1);
	graphLayoutLeft->addWidget(originalGraphLayoutComboBox, 0, 2, 1, 1);
	graphLayoutLeft->addItem(new QSpacerItem(0, 0, QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Expanding), 0, 3, 1, 1);
	graphLayoutLeft->addWidget(originalGraphView, 1, 0, 1, 4);
	graphLayoutLeft->setColumnStretch(3, 1);
	graphLayoutLeft->setRowStretch(1, 1);
	graphLayoutLeft->setSpacing(2);
	QGridLayout* graphLayoutRight = new QGridLayout();
	graphLayoutRight->addWidget(solvedGraphTitle, 0, 0, 1, 1);
	graphLayoutRight->addWidget(solvedGraphLayoutLabel, 0, 1, 1, 1);
	graphLayoutRight->addWidget(solvedGraphLayoutComboBox, 0, 2, 1, 1);
	graphLayoutRight->addWidget(crossingNumberLabel, 0, 3, 1, 1);
	graphLayoutRight->addItem(new QSpacerItem(0, 0, QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Expanding), 0, 4, 1, 1);
	graphLayoutRight->addWidget(solvedGraphView, 1, 0, 1, 5);
	graphLayoutRight->setColumnStretch(4, 1);
	graphLayoutRight->setRowStretch(1, 1);
	graphLayoutRight->setSpacing(2);
	connect(originalGraphLayoutComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(originalLayoutChanged(int)));

	QGridLayout* centralLayout = new QGridLayout(centralWidget);
	centralLayout->addLayout(controlLayout, 0, 0, 1, 1);
	centralLayout->addLayout(graphLayoutLeft, 0, 1, 1, 1);
	centralLayout->addLayout(graphLayoutRight, 0, 2, 1, 1);
	centralLayout->addLayout(outputLayout, 1, 0, 1, 3);
	centralLayout->setColumnStretch(1, 1);
	centralLayout->setColumnStretch(2, 1);
	centralLayout->setRowStretch(0, 1);
	centralLayout->setSpacing(5);
	centralWidget->setLayout(centralLayout);

	this->setCentralWidget(centralWidget);
	this->setWindowTitle(QString("Optimal Crossing Minimization"));
}

void TestUI::scanRomeGraphs(string folder, vector<RomeGraphDescr>& target)
{
	target.clear();
	ifstream in = ifstream (folder + "Graphs.dat");
	if (!in.is_open()) {
		BOOST_LOG_TRIVIAL(fatal) << "Unable to open file!";
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
	BOOST_LOG_TRIVIAL(info) << target.size() << " Graph infos read";
	in.close();
	//sort
	sort(target.begin(), target.end(), 
		[](const RomeGraphDescr& a, const RomeGraphDescr& b)
        {
            if (a.n < b.n) return true;
			if (a.n > b.n) return false;
			return a.m < b.m;
        });
}

void TestUI::setupLogBackend()
{
	//create sink
	logging::add_common_attributes();
	boost::shared_ptr< logging::core > core = logging::core::get();
	boost::shared_ptr< QtBoostLogger > backend = boost::make_shared< QtBoostLogger >();
	typedef sinks::synchronous_sink< QtBoostLogger > sink_t;
	boost::shared_ptr< sink_t > sink = boost::make_shared< sink_t > (backend);
	namespace expr = boost::log::expressions;
	sink->set_formatter(
		expr::stream
            << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%H:%M:%S")
            << ": [" << logging::trivial::severity
            << "] " << expr::smessage
	);
	core->add_sink(sink);
	//connect to outputConsole
	connect(backend.get(), SIGNAL(messageConsumed(QString)), 
		outputConsole, SLOT(appendPlainText(QString)));
}

void TestUI::initGraphTable()
{
	graphList->setRowCount(graphs.size());
	for (int i=0; i<graphs.size(); ++i) {
		QTableWidgetItem* i1 = new QTableWidgetItem(tr("%1").arg(graphs[i].n));
		QTableWidgetItem* i2 = new QTableWidgetItem(tr("%1").arg(graphs[i].m));
		QTableWidgetItem* i3 = new QTableWidgetItem(QString(graphs[i].fileName.c_str()));
		graphList->setItem(i, 0, i1);
		graphList->setItem(i, 1, i2);
		graphList->setItem(i, 2, i3);
		graphList->setRowHeight(i, 20);
	}
	graphList->verticalHeader()->setVisible(false);
	graphList->setColumnWidth(0, 50);
	graphList->setColumnWidth(1, 50);
	graphList->setColumnWidth(2, 130);
	graphList->setSelectionMode(QAbstractItemView::SingleSelection);
	graphList->setSelectionBehavior(QAbstractItemView::SelectRows);
}

void TestUI::filterGraphs()
{
	QString nodeCount = nodeCountFilter->text();
	QString edgeCount = edgeCountFilter->text();
	for (int i=0; i<graphList->rowCount(); ++i) {
		bool visible = true;
		if (nodeCount.length() > 0 && (graphList->item(i, 0)->text() != nodeCount)) visible = false;
		if (edgeCount.length() > 0 && (graphList->item(i, 1)->text() != edgeCount)) visible = false;
		graphList->setRowHidden(i, !visible);
	}
}

void TestUI::graphSelected(int row, int column)
{
	const RomeGraphDescr& descr = graphs[row];
	BOOST_LOG_TRIVIAL(info) << "load " << descr.fileName;
	typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> BoostGraph;
	BoostGraph BG;
	try {
		ifstream is (folder + descr.fileName);
		boost::dynamic_properties properties;
		boost::read_graphml(is, BG, properties);
	}
	catch (const boost::graph_exception& ex)
	{
		cerr << ex.what() << '\n';
		return;
	}
	originalG.clear();
	GraphConverter::convert(BG, originalG);
	BOOST_LOG_TRIVIAL(info) << "Graph loaded, contains " << originalG.numberOfNodes() << " nodes and "
		<< originalG.numberOfEdges() << " edges";
	originalGA = ogdf::GraphAttributes(originalG,
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor | ogdf::GraphAttributes::nodeType
		| ogdf::GraphAttributes::edgeLabel);
	ogdf::node n;
	forall_nodes(n, originalG) {
		originalGA.width(n) = originalGA.height(n) = 20;
		stringstream s;
		s << n->index();
		originalGA.labelNode(n) = s.str().c_str();
	}
	ogdf::edge e;
	forall_edges(e, originalG) {
		stringstream s;
		s << e->index();
		originalGA.labelEdge(e) = s.str().c_str();
	}
	layoutOriginalGraph();
	if (state != 1)
		switchUIState(1);
}

void TestUI::clearOutput()
{
	outputConsole->clear();
}

void TestUI::originalLayoutChanged(int index)
{
	originalLayout = index;
	if (!originalG.empty()) {
		layoutOriginalGraph();
	}
}

void TestUI::layoutOriginalGraph()
{
	if (originalLayout == 0) { //FMMM
		ogdf::FMMMLayout fmmm;
		fmmm.useHighLevelOptions(true);
		fmmm.unitEdgeLength(25.0); 
		fmmm.newInitialPlacement(true);
		fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
		fmmm.call(originalGA);
		BOOST_LOG_TRIVIAL(info) << "FMMMLayout called";
	} else if (originalLayout==1) { //Stress Majorization
		ogdf::PlanarizationLayout pl;
		pl.preprocessCliques(true);
		pl.call(originalGA);
		BOOST_LOG_TRIVIAL(info) << "Planarization called";
	} else {
		BOOST_LOG_TRIVIAL(fatal) << "Unknown layout for original graph: " << originalLayout;
	}
	originalGraphView->showGraph(originalGA);
}

void TestUI::switchUIState(int newState)
{
	state = newState;
	if (state == 0) {
		//initial state
		originalG.clear();
		originalGraphLayoutComboBox->setEnabled(false);
		solvedGraphLayoutComboBox->setEnabled(false);
		simplifyButton->setEnabled(false);
		solveButton->setEnabled(false);
		cancelButton->setEnabled(false);
		nodeCountFilter->setEnabled(true);
		edgeCountFilter->setEnabled(true);
		graphList->setEnabled(true);
	} else if (state == 1) {
		//graph loaded
		originalGraphLayoutComboBox->setEnabled(true);
		solvedGraphLayoutComboBox->setEnabled(false);
		simplifyButton->setEnabled(true);
		solveButton->setEnabled(false);
	} else if (state == 2) {
		simplifyButton->setEnabled(false);
		nodeCountFilter->setEnabled(false);
		edgeCountFilter->setEnabled(false);
		graphList->setEnabled(false);
		solveButton->setEnabled(false);
		cancelButton->setEnabled(true);
	} else if (state == 3) {
		//simplifyed
		simplifyButton->setEnabled(false);
		solveButton->setEnabled(true);
		nodeCountFilter->setEnabled(true);
		edgeCountFilter->setEnabled(true);
		graphList->setEnabled(true);
		cancelButton->setEnabled(false);
		crossingNumberLabel->setText(QString(""));
	} else if (state == 4) {
		//solve
		nodeCountFilter->setEnabled(false);
		edgeCountFilter->setEnabled(false);
		graphList->setEnabled(false);
		solveButton->setEnabled(false);
		cancelButton->setEnabled(true);
	} else if (state == 5) {
		//solved
		solveButton->setEnabled(false);
		cancelButton->setEnabled(false);
		solvedGraphLayoutComboBox->setEnabled(true);
		nodeCountFilter->setEnabled(true);
		edgeCountFilter->setEnabled(true);
		graphList->setEnabled(true);
	}
}

void TestUI::killWaitSolverThread()
{
	if (solverThread != NULL) {
		if (solverThread->isRunning()) {
			solverThread->requestInterruption();
			solverThread->goOn();
			solverThread->wait();
		}
		delete solverThread;
		solverThread = NULL;
	}
}

void TestUI::simplifyGraph()
{
	//create solver thread and simplify
	killWaitSolverThread();
	solverThread = new SolverThread(originalG);
	connect(solverThread, SIGNAL(simplifyingFinished()), this, SLOT(simplifiedGraph()));
	solverThread->start();
	switchUIState(2);
}

void TestUI::simplifiedGraph()
{
	disconnect(solverThread, SIGNAL(simplifyingFinished()), this, SLOT(simplifiedGraph()));

	//visualize simplification
	vector<const GraphCopy*> components = solverThread->getSimplifiedComponents();
	Graph G;
	unordered_map<node, node> map;
	for (int i=0; i<components.size(); ++i) {
		const GraphCopy C = *components[i];
		node n;
		forall_nodes(n, C) {
			node c = G.newNode();
			node og = C.original(n);
			if (map.count(og) == 0)
				map[og] = c;
		}
		edge e;
		forall_edges(e, C) {
			node u = map.at(C.original(e->source()));
			node v = map.at(C.original(e->target()));
			if (u==nullptr || v==nullptr) {
				BOOST_LOG_TRIVIAL(fatal) << "no copy node found!";
			}
			edge c = G.newEdge(u, v);
			if (c==nullptr) {
				BOOST_LOG_TRIVIAL(fatal) << "edge is null";
			}
		}
	}
	GraphAttributes GA(G,
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor | ogdf::GraphAttributes::nodeType
		| ogdf::GraphAttributes::edgeLabel);
	for (auto p : map) {
		node o = p.first;
		node c = p.second;
		GA.labelNode(c) = originalGA.labelNode(o);
		GA.width(c) = originalGA.width(o);
		GA.height(c) = originalGA.height(o);
		GA.x(c) = originalGA.x(o);
		GA.y(c) = originalGA.y(o);
	}
	solvedGraphView->showGraph(GA);

	switchUIState(3);
}

void TestUI::solveGraph()
{
	connect(solverThread, SIGNAL(finished()), this, SLOT(solvedGraph()));
	switchUIState(4);
	solverThread->goOn();
}

void TestUI::cancelSolving()
{
	solverThread->terminateSolving();
	killWaitSolverThread();
	switchUIState(1);
}

void TestUI::solvedGraph()
{
	int crossingNumber = solverThread->getCrossingNumber();
	if (crossingNumber < 0) {
		//terminated
		switchUIState(2);
	} else {
		crossingNumberLabel->setText(QString("  Crossing Number: %1").arg(crossingNumber));
		const GraphCopy solvedG = *solverThread->getSolvedGraph();
		//show graph
		solvedGA = ogdf::GraphAttributes(solvedG,
			ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
			| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
			| ogdf::GraphAttributes::nodeColor | ogdf::GraphAttributes::nodeType
			| ogdf::GraphAttributes::edgeLabel);
		ogdf::node n;
		forall_nodes(n, solvedG) {
			ogdf::node on = solvedG.original(n);
			if (on == nullptr) {
				//dummy
				solvedGA.width(n) = solvedGA.height(n) = 0;
				solvedGA.labelNode(n) = "";
			} else {
				solvedGA.width(n) = solvedGA.height(n) = 20;
				stringstream s;
				s << on->index();
				solvedGA.labelNode(n) = s.str().c_str();
			}
		}
		ogdf::edge e;
		forall_edges(e, solvedG) {
			stringstream s;
			s << solvedG.original(e)->index();
			originalGA.labelEdge(e) = s.str().c_str();
		}
		layoutSolvedGraph();
		switchUIState(5);
	}
	delete solverThread;
	solverThread = NULL;
}

void TestUI::layoutSolvedGraph()
{
	if (solvedLayout == 0) {
		BOOST_LOG_TRIVIAL(info) << "MixedModelLayout called";
		MixedModelLayout l;
		l.call(solvedGA);
	}
	solvedGraphView->showGraph(solvedGA);
}