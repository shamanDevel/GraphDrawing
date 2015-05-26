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

#include "GraphView.h"

using namespace std;

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

private slots:
	void filterGraphs();
	void graphSelected(int row, int column);

private:
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
	QPushButton *combineButton;
	QPushButton *cancelButton;

	QComboBox *originalGraphLayoutComboBox;
	QComboBox *solvedGraphLayoutComboBox;
	GraphView *originalGraphView;
	GraphView *solvedGraphView;
	QLabel *crossingNumberLabel;

	vector<RomeGraphDescr> graphs;
};

#endif // TESTUI_H
