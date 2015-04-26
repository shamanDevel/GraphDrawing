#include "stdafx.h"
#include <ogdf_include.h>

namespace ogdf {
	bool operator==(const NodeElement& u, const NodeElement& v) {
		return u.index() == v.index();
	}
	bool operator==(const EdgeElement& e, const EdgeElement& f) {
		return ( (*e.source() == *f.source() && *e.target() == *f.target())
			|| (*e.target() == *f.source() && *e.source() == *f.target()) );
	}
}