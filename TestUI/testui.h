#ifndef TESTUI_H
#define TESTUI_H

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

#include "GraphView.h"

class TestUI : public QMainWindow
{
	Q_OBJECT

public:
	TestUI(QWidget *parent = 0);
	~TestUI();

private:
	void setupUi();

private:
	QWidget *centralWidget;
	QWidget *outputWidget;
	QWidget *controlWidget;
	QWidget *graphWidgetLeft;
	QWidget *graphWidgetRight;

	QPlainTextEdit *outputConsole;
	QToolButton *clearOutputButton;
	QToolButton *showDebugButton;

	QSpinBox* nodeCountSpinner;
	QSpinBox* edgeCountSpinner;
	QLabel* nodeCountLabel;
	QLabel* edgeCountLabel;
	QListWidget *graphList;
	QPushButton *simplifyButton;
	QPushButton *solveButton;
	QPushButton *combineButton;
	QPushButton *cancelButton;

	QComboBox *originalGraphLayoutComboBox;
	QComboBox *solvedGraphLayoutComboBox;
	GraphView *originalGraphView;
	GraphView *solvedGraphView;
	QLabel *crossingNumberLabel;
};

#endif // TESTUI_H
