#pragma once

#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>
#include <vector>
#include <ogdf/basic/GraphCopy.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>
#include <SimplificationBiconnected.h>
#include <SimplificationDeg12.h>

using namespace std;
using namespace ogdf;
using namespace shaman;

class SolverThread : public QThread
{
	Q_OBJECT

public:
	SolverThread(const Graph& originalGraph, QObject* parent = NULL);
	~SolverThread(void);

	virtual void run();
	void terminateSolving();

	void setShowDebugOutput(bool showDebugOutput) {
		this->showDebugOutput = showDebugOutput;
	}
	vector<const GraphCopy*> getSimplifiedComponents();
	vector<const GraphCopy*> getSolvedComponents();
	const GraphCopy* getSolvedGraph();
	int getCrossingNumber() const {return crossingNumber;}
	float getSimplificationTime() const {return simplificationTime;}
	float getSolvingTime() const {return solvingTime;}
	float getCombiningTime() const {return combiningTime;}

signals:
	void simplifyingFinished();
	void solvingFinished();
	void combiningFinished();

public slots:
	void goOn();

private:
	bool simplifyGraph();
	bool solveGraph();
	bool combineGraph();

	const Graph* originalGraph;
	SimplificationBiconnected* bicon;
	vector<SimplificationDeg12*> deg12;
	vector< GraphCopy > results;
	GraphCopy solvedGraph;

	QMutex* mutex;
	QWaitCondition* condition;

	int crossingNumber;
	float simplificationTime;
	float solvingTime;
	float combiningTime;
	bool showDebugOutput;
};

