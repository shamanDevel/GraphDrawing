#include "LatexExporter.h"

#include <boost/log/trivial.hpp>
#include <QtWidgets/QFileDialog>
#include <QtCore/QString>
#include <fstream>

using namespace std;
using namespace ogdf;

LatexExporter::LatexExporter(void)
{
}


LatexExporter::~LatexExporter(void)
{
}

void LatexExporter::save(const GraphAttributes& GA, const string& fileName, 
						 vector<string> comments, QWidget* parent)
{
	QString file = QFileDialog::getSaveFileName(parent, QString("Save as LaTeX/TikZ file"), 
		QString((fileName + ".tex").c_str()), QString("*.tex"));
	if (file.isNull() || file.isEmpty()) {
		BOOST_LOG_TRIVIAL(info) << "No file selected";
	} else {
		BOOST_LOG_TRIVIAL(info) << "Save to " << file.toStdString();

		ofstream o (file.toStdString(), ofstream::out);
		o << "\\begin{tikzpicture}" << endl;

		for (string comment : comments) {
			o << "% " << comment << endl;
		}

		o << "\t\\tikzstyle{vertex}=[circle,fill=black!25,minimum size=20,inner sep=0pt]" << endl;
		o << "\t\\tikzstyle{crossing}=[minimum size=0pt,inner sep=0pt]" << endl;
		o << "\t\\tikzstyle{edge}=[thin]" << endl;

		node n;
		double minX = 1e20;
		double minY = 1e20;
		double maxX = -1e20;
		double maxY = -1e20;
		forall_nodes(n, GA.constGraph()) {
			minX = min(minX, GA.x(n));
			minY = min(minY, GA.y(n));
			maxX = max(maxX, GA.x(n));
			maxY = max(maxY, GA.y(n));
		}
		double offsetX = minX;
		double offsetY = minY;
		double factor = 5.0 / max(maxX-minX, maxY-minY);

		forall_nodes(n, GA.constGraph()) {
			o << "\t\\node";
			if (GA.labelNode(n).length() == 0) {
				//dummy
				o << "[crossing]";
			} else {
				//normal node
				o << "[vertex]";
			}
			o << " (" << n->index() << ")";
			double x = GA.x(n);
			double y = GA.y(n);
			x -= offsetX;
			y -= offsetY;
			x *= factor;
			y *= factor;
			o << " at (" << x << "," << y << ")";
			o << " {" << GA.labelNode(n).cstr() << "};";
			o << endl;
		}

		edge e;
		forall_edges(e, GA.constGraph()) {
			o << "\t\\draw[edge] ";
			o << "(" << e->source()->index() << ")";
			const ogdf::DPolyline bend = GA.bends(e);
			for (auto it = bend.begin(); it != bend.end(); it = it.succ()) {
				const ogdf::DPoint p = *it;
				double x = p.m_x;
				double y = p.m_y;
				x -= offsetX;
				y -= offsetY;
				x *= factor;
				y *= factor;
				o << " -- (" << x << "," << y << ") coordinate";
			}
			o << " -- (" << e->target()->index() << ");";
			o << endl;
		}

		o << "\\end{tikzpicture}" << endl;
		o.close();
	}
}