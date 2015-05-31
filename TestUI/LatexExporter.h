#pragma once

#include <QtWidgets/QWidget>
#include <ogdf/basic/GraphAttributes.h>
#include <string>
#include <vector>

class LatexExporter
{
private:
	LatexExporter(void);
	~LatexExporter(void);

public:
	static void save(const ogdf::GraphAttributes& GA, const std::string& fileName, 
		std::vector<std::string> comments, QWidget* parent = 0);
};

