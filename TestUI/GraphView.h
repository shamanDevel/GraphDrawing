#pragma once

#include <QtWidgets/QGraphicsView>

#include <ogdf/basic/GraphAttributes.h>

class GraphView : public QGraphicsView
{
	Q_OBJECT

public:
	GraphView(QWidget* parent)
		: QGraphicsView(parent)
	{}
	~GraphView(void) {}

public slots:
	void showGraph(const ogdf::GraphAttributes& GA);
};

