#pragma once

#include <QtCore/QThread>
#include <vector>
#include <ogdf/basic/GraphCopy.h>
#include <CrossingMinimization.h>
#include <OOCMCrossingMinimization.h>

using namespace std;
using namespace ogdf;
using namespace shaman;

class SolverThread : public QThread
{
	Q_OBJECT

public:
	SolverThread(QObject* parent = NULL);
	~SolverThread(void);

	virtual void run();
	void terminateSolving();

	void setGraphs(const vector< pair<GraphCopy, unordered_map<edge, int> > >& graphs);
	void setShowDebugOutput(bool debugOutput) {
		showDebugOutput = debugOutput;
	}

	int getCrossingNumber() const;
	const vector<GraphCopy>& getSolvedGraphs() const;
	float getCalculationTime() const;

private:
	const vector< pair<GraphCopy, unordered_map<edge, int> > >* originalGraphs;
	vector<GraphCopy> solvedGraphs;
	int crossingNumber;
	float calculationTime;
	bool showDebugOutput;
};

