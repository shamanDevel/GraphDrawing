#include "GraphView.h"

#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsSimpleTextItem>
#include <vector>
#include <boost/log/trivial.hpp>

using namespace std;

void GraphView::showGraph(const ogdf::GraphAttributes& GA)
{
	QGraphicsScene* scene = new QGraphicsScene(this);

	QPen blackPen(Qt::PenStyle::SolidLine);
	blackPen.setColor(QColor(0,0,0));
	QBrush grayBrush(QColor(200,200,200));

	qreal minX = INT_MAX;
	qreal minY = INT_MAX;
	qreal maxX = INT_MIN;
	qreal maxY = INT_MIN;

	//draw edges
	ogdf::edge e;
	forall_edges(e, GA.constGraph()) {
		const ogdf::DPolyline bend = GA.bends(e);
		vector<ogdf::DPoint> points;
		points.reserve(bend.size() + 2);
		points.push_back(ogdf::DPoint(GA.x(e->source()), GA.y(e->source())));
		for (auto it = bend.begin(); it != bend.end(); it = it.succ()) {
			const ogdf::DPoint p = *it;
			points.push_back(p);
			minX = min(minX, p.m_x);
			minY = min(minY, p.m_y);
			maxX = max(maxX, p.m_x);
			maxY = max(maxY, p.m_y);
		}
		points.push_back(ogdf::DPoint(GA.x(e->target()), GA.y(e->target())));
		for (int i=1; i<points.size(); ++i) {
			QGraphicsLineItem* item = scene->addLine(points[i-1].m_x, points[i-1].m_y, points[i].m_x, points[i].m_y, blackPen);
		}
	}

	//draw nodes
	ogdf::node n;
	forall_nodes(n, GA.constGraph()) {
		double x = GA.x(n);
		double y = GA.y(n);
		double w = GA.width(n);
		double h = GA.height(n);
		scene->addEllipse(x - (w/2), y - (w/2), w, h, blackPen, grayBrush);
		QGraphicsSimpleTextItem* text = scene->addSimpleText(QString(GA.labelNode(n).cstr()));
		QRectF rect = text->boundingRect();
		text->setPos(x - rect.width()/2, y - rect.height()/2);
		minX = min(minX, x);
		minY = min(minY, y);
		maxX = max(maxX, x);
		maxY = max(maxY, y);
	}

	this->setScene(scene);
	minX-=50; minY-=50;
	maxX+=50; maxY+=50;
	this->setSceneRect(minX, minY, maxX-minX, maxY-minY);
	this->fitInView(minX, minY, maxX-minX, maxY-minY);
	this->setRenderHint(QPainter::Antialiasing, true);
	this->setRenderHint(QPainter::TextAntialiasing, true);
}
