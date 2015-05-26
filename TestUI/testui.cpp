#include "testui.h"
#include <boost/log/trivial.hpp>
#include "QtBoostLogger.h"
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

TestUI::TestUI(QWidget *parent)
	: QMainWindow(parent)
{
	setupUi();
	setupLogBackend();
	string folder = "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\example-data\\";
	scanRomeGraphs(folder, graphs);
	initGraphTable();

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

	nodeCountFilter = new QLineEdit(centralWidget);
	edgeCountFilter = new QLineEdit(centralWidget);
	QLabel *controlTitle = new QLabel(QString("Choose Graph"), centralWidget);
	nodeCountLabel = new QLabel(QString("Node Count (XX-XX):"), centralWidget);
	edgeCountLabel = new QLabel(QString("Edge Count (XX-XX):"), centralWidget);
	QLabel *listTitle = new QLabel(QString("Select Graph"), centralWidget);
	graphList = new QTableWidget(centralWidget);
	simplifyButton = new QPushButton(QString("Simplify"), centralWidget);
	solveButton = new QPushButton(QString("Solve"), centralWidget);
	combineButton = new QPushButton(QString("Combine"), centralWidget);
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
	controlLayout->addWidget(combineButton, 7, 0, 1, 2);
	controlLayout->addWidget(cancelButton, 8, 0, 1, 2);
	controlLayout->setRowStretch(4, 1);
	controlLayout->setColumnStretch(1, 1);
	controlLayout->setSpacing(2);
	connect(nodeCountFilter, SIGNAL(textChanged(const QString&)), this, SLOT(filterGraphs()));
	connect(edgeCountFilter, SIGNAL(textChanged(const QString&)), this, SLOT(filterGraphs()));
	connect(graphList, SIGNAL(cellClicked(int, int)), this, SLOT(graphSelected(int, int)));

	originalGraphLayoutComboBox = new QComboBox(centralWidget);
	originalGraphView = new GraphView(centralWidget);
	solvedGraphLayoutComboBox = new QComboBox(centralWidget);
	solvedGraphView = new GraphView(centralWidget);
	QLabel* originalGraphTitle = new QLabel(QString("Original Graph    "), centralWidget);
	QLabel* solvedGraphTitle = new QLabel(QString("Solved Graph    "), centralWidget);
	QLabel *originalGraphLayoutLabel = new QLabel(QString("Layout:"), centralWidget);
	QLabel *solvedGraphLayoutLabel = new QLabel(QString("Layout:"), centralWidget);
	crossingNumberLabel = new QLabel(QString("  Crossing Number: ?"), centralWidget);
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
}