#include "testui.h"

TestUI::TestUI(QWidget *parent)
	: QMainWindow(parent)
{
	setupUi();
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
	outputWidget = new QWidget(centralWidget);
	controlWidget = new QWidget(centralWidget);
	graphWidgetLeft = new QWidget(centralWidget);
	graphWidgetRight = new QWidget(centralWidget);
	QGridLayout* centralLayout = new QGridLayout(centralWidget);
	centralLayout->addWidget(controlWidget, 0, 0, 1, 1);
	centralLayout->addWidget(graphWidgetLeft, 0, 1, 1, 1);
	centralLayout->addWidget(graphWidgetRight, 0, 2, 1, 1);
	centralLayout->addWidget(outputWidget, 1, 0, 1, 3);
	centralLayout->setColumnStretch(1, 1);
	centralLayout->setColumnStretch(2, 1);
	centralLayout->setRowStretch(0, 1);
	centralLayout->setSpacing(0);
	centralWidget->setLayout(centralLayout);

	outputConsole = new QPlainTextEdit(outputWidget);
	clearOutputButton = new QToolButton(outputWidget);
	showDebugButton = new QToolButton(outputWidget);
	clearOutputButton->setIcon(QIcon(QString(":/TestUI/clear.png")));
	clearOutputButton->setToolTip(QString("Clears the console"));
	showDebugButton->setIcon(QIcon(QString(":/TestUI/bug.png")));
	showDebugButton->setToolTip(QString("Show debug output"));
	clearOutputButton->setCheckable(false);
	showDebugButton->setCheckable(true);
	QGridLayout* outputLayout = new QGridLayout(outputWidget);
	outputLayout->addWidget(clearOutputButton, 0, 0, 1, 1);
	outputLayout->addWidget(showDebugButton, 1, 0, 1, 1);
	outputLayout->addWidget(outputConsole, 0, 1, 3, 1);
	outputLayout->setColumnStretch(1, 1);
	outputLayout->setRowStretch(2, 1);
	outputLayout->setSpacing(2);
	outputWidget->setLayout(outputLayout);

	nodeCountSpinner = new QSpinBox(controlWidget);
	edgeCountSpinner = new QSpinBox(controlWidget);
	QLabel *controlTitle = new QLabel(QString("Choose Graph"), controlWidget);
	nodeCountLabel = new QLabel(QString("Node Count (XX-XX):"), controlWidget);
	edgeCountLabel = new QLabel(QString("Edge Count (XX-XX):"), controlWidget);
	QLabel *listTitle = new QLabel(QString("Select Graph"), controlWidget);
	graphList = new QListWidget(controlWidget);
	simplifyButton = new QPushButton(QString("Simplify"), controlWidget);
	solveButton = new QPushButton(QString("Solve"), controlWidget);
	combineButton = new QPushButton(QString("Combine"), controlWidget);
	cancelButton = new QPushButton(QString("Cancel"), controlWidget);
	QGridLayout* controlLayout = new QGridLayout(controlWidget);
	controlLayout->addWidget(controlTitle, 0, 0, 1, 2 );
	controlLayout->addWidget(nodeCountLabel, 1, 0, 1, 1);
	controlLayout->addWidget(nodeCountSpinner, 1, 1, 1, 1);
	controlLayout->addWidget(edgeCountLabel, 2, 0, 1, 1);
	controlLayout->addWidget(edgeCountSpinner, 2, 1, 1, 1);
	controlLayout->addWidget(listTitle, 3, 0, 1, 2);
	controlLayout->addWidget(graphList, 4, 0, 1, 2);
	controlLayout->addWidget(simplifyButton, 5, 0, 1, 2);
	controlLayout->addWidget(solveButton, 6, 0, 1, 2);
	controlLayout->addWidget(combineButton, 7, 0, 1, 2);
	controlLayout->addWidget(cancelButton, 8, 0, 1, 2);
	controlLayout->setRowStretch(4, 1);
	controlLayout->setColumnStretch(1, 1);
	controlLayout->setSpacing(2);
	controlWidget->setLayout(controlLayout);

	originalGraphLayoutComboBox = new QComboBox(graphWidgetLeft);
	originalGraphView = new GraphView(graphWidgetLeft);
	solvedGraphLayoutComboBox = new QComboBox(graphWidgetRight);
	solvedGraphView = new GraphView(graphWidgetRight);
	QLabel* originalGraphTitle = new QLabel(QString("Original Graph    "), graphWidgetLeft);
	QLabel* solvedGraphTitle = new QLabel(QString("Solved Graph    "), graphWidgetRight);
	QLabel *originalGraphLayoutLabel = new QLabel(QString("Layout:"), graphWidgetLeft);
	QLabel *solvedGraphLayoutLabel = new QLabel(QString("Layout:"), graphWidgetRight);
	crossingNumberLabel = new QLabel(QString("Crossing Number: ?"), graphWidgetRight);
	QGridLayout* graphLayoutLeft = new QGridLayout(graphWidgetLeft);
	graphLayoutLeft->addWidget(originalGraphTitle, 0, 0, 1, 1);
	graphLayoutLeft->addWidget(originalGraphLayoutLabel, 0, 1, 1, 1);
	graphLayoutLeft->addWidget(originalGraphLayoutComboBox, 0, 2, 1, 1);
	graphLayoutLeft->addItem(new QSpacerItem(0, 0, QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Expanding), 0, 3, 1, 1);
	graphLayoutLeft->addWidget(originalGraphView, 1, 0, 1, 4);
	graphLayoutLeft->setColumnStretch(3, 1);
	graphLayoutLeft->setRowStretch(1, 1);
	graphLayoutLeft->setSpacing(2);
	graphWidgetLeft->setLayout(graphLayoutLeft);
	QGridLayout* graphLayoutRight = new QGridLayout(graphWidgetRight);
	graphLayoutRight->addWidget(solvedGraphTitle, 0, 0, 1, 1);
	graphLayoutRight->addWidget(solvedGraphLayoutLabel, 0, 1, 1, 1);
	graphLayoutRight->addWidget(solvedGraphLayoutComboBox, 0, 2, 1, 1);
	graphLayoutRight->addWidget(crossingNumberLabel, 0, 3, 1, 1);
	graphLayoutRight->addItem(new QSpacerItem(0, 0, QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Expanding), 0, 4, 1, 1);
	graphLayoutRight->addWidget(solvedGraphView, 1, 0, 1, 5);
	graphLayoutRight->setColumnStretch(4, 1);
	graphLayoutRight->setRowStretch(1, 1);
	graphLayoutRight->setSpacing(2);
	graphWidgetRight->setLayout(graphLayoutRight);

	this->setCentralWidget(centralWidget);
	this->setWindowTitle(QString("Optimal Crossing Minimization"));
}