#pragma once

#include <QtWidgets/QGraphicsView>

#include <ogdf/basic/GraphAttributes.h>

class GraphView : public QGraphicsView
{
public:
	GraphView(QWidget* parent)
		: QGraphicsView(parent)
	{}
	~GraphView(void) {}

	void showGraph(const ogdf::GraphAttributes& GA);
};

