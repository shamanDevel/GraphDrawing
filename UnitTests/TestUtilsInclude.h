#include "CppUnitTest.h"
#include "CppUnitTestAssert.h"
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <set>
#include <math.h>
#include <algorithm>
#include <tuple>
#include <ogdf\planarlayout\MixedModelLayout.h>
#include <ogdf\energybased\FMMMLayout.h>
#include <ogdf\energybased\StressMajorizationSimple.h>
#include <ogdf\planarity\BoyerMyrvold.h>
#include <boost/log/core.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/frontend_requirements.hpp>
#include <boost/log/attributes/attribute_value_set.hpp>
#include <boost/log/core/record_view.hpp>

namespace logging = boost::log;
namespace sinks = boost::log::sinks;

using namespace Microsoft::VisualStudio::CppUnitTestFramework;
using namespace std;
using namespace ogdf;