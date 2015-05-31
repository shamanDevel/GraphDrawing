#ifndef TESTUI_H
#define TESTUI_H

#include <string>
#include <vector>
#include <algorithm>

#include <QDebug>
#include <QtWidgets/QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QLineEdit>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/GraphCopy.h>

#include <SimplificationBiconnected.h>
#include <SimplificationDeg12.h>

#include "GraphView.h"
#include "SolverThread.h"

using namespace std;
using namespace shaman;

class TestUI : public QMainWindow
{
	Q_OBJECT

public:
	TestUI(QWidget *parent = 0);
	~TestUI();

private:
	struct RomeGraphDescr
	{
		string fileName;
		int n;
		int m;
	};

	void setupUi();
	void setupLogBackend();
	void scanRomeGraphs(string folder, vector<RomeGraphDescr>& target);
	void initGraphTable();
	void layoutOriginalGraph();
	void switchUIState(int newState);
	void layoutSolvedGraph();
	void killWaitSolverThread();

private slots:
	void filterGraphs();
	void graphSelected(int row, int column);
	void clearOutput();
	void originalLayoutChanged(int index);
	void simplifyGraph();
	void simplifiedGraph();
	void solveGraph();
	void solvedGraph();
	void cancelSolving();
	void saveOriginalGraph();
	void saveSolvedGraph();

private:
	string folder;
	// 0: no graph loaded; 1: graph loaded; 2: graph simplifying,
	// 3: graph simplified; 4: graph solving; 5: graph solved and merged
	int state;

	QWidget *centralWidget;

	QPlainTextEdit *outputConsole;
	QToolButton *clearOutputButton;
	QToolButton *showDebugButton;

	QLineEdit* nodeCountFilter;
	QLineEdit* edgeCountFilter;
	QLabel* nodeCountLabel;
	QLabel* edgeCountLabel;
	QTableWidget *graphList;
	QPushButton *simplifyButton;
	QPushButton *solveButton;
	QPushButton *cancelButton;

	QComboBox *originalGraphLayoutComboBox;
	QComboBox *solvedGraphLayoutComboBox;
	GraphView *originalGraphView;
	GraphView *solvedGraphView;
	QLabel *crossingNumberLabel;
	QPushButton *saveOriginalGraphButton;
	QPushButton *saveSolvedGraphButton;

	vector<RomeGraphDescr> graphs;
	std::string selectedGraphName;
	ogdf::Graph originalG;
	ogdf::GraphAttributes originalGA;
	//0: FMMM, 1: Stress Majorization
	int originalLayout;

	SolverThread* solverThread;
	int crossingNumber;
	float simplificationTime;
	float solvingTime;
	float combiningTime;

	ogdf::GraphCopy solvedG;
	ogdf::GraphAttributes solvedGA;
	int solvedLayout;
};

#endif // TESTUI_H
