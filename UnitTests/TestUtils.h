
// To be included within a class

static int RandomInt(int min, int max) 
{
	int r = rand();
	r = r % (max-min+1);
	r += min;
	return r;
}

static void SaveGraph(const Graph& G, const char* prefix)
{

	int numNodes = G.numberOfNodes();
	int numEdges = G.numberOfEdges();
	//converter
	ogdf::GraphAttributes GA(G, 
		ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::edgeGraphics
		| ogdf::GraphAttributes::nodeLabel | ogdf::GraphAttributes::edgeStyle
		| ogdf::GraphAttributes::nodeColor );
	SList<node> nodes;
	G.allNodes(nodes);
	for (node n : nodes) {
		stringstream s;
		s << n->index();
		GA.labelNode(n) = s.str().c_str();
	}

	//layout
	BoyerMyrvold bm;
	bool planar = bm.isPlanar(G);
	if (planar) {
#if 1
		//use planar layout
		MixedModelLayout l;
		l.call(GA);
#else
		//use spring layout
		ogdf::FMMMLayout fmmm;
		fmmm.useHighLevelOptions(true);
		fmmm.unitEdgeLength(25.0); 
		fmmm.newInitialPlacement(true);
		fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
		fmmm.call(GA);
#endif
	} else {
#if 1
		//use spring layout
		ogdf::FMMMLayout fmmm;
		fmmm.useHighLevelOptions(true);
		fmmm.unitEdgeLength(10.0 * sqrt(numEdges)); 
		fmmm.newInitialPlacement(true);
		fmmm.qualityVersusSpeed(ogdf::FMMMLayout::qvsGorgeousAndEfficient);
		fmmm.call(GA);
#else
		ogdf::StressMajorization sm;
		sm.call(GA);
#endif
	}

	//save
	stringstream s;
	s << "C:\\Users\\Sebastian\\Documents\\C++\\GraphDrawing\\graphs\\";
	s << prefix;
	s << "_n";
	s << numNodes;
	s << "_e";
	s << numEdges;
	if (planar) {
		s << "_planar";
	}
	s << ".svg";
	GA.writeSVG(s.str().c_str());

}

template< typename CharT >
class UnitTest_output_backend :
	public sinks::basic_formatted_sink_backend< CharT, sinks::concurrent_feeding >
{
    //! Base type
    typedef sinks::basic_formatted_sink_backend< CharT, sinks::concurrent_feeding > base_type;

public:
    //! Character type
    typedef typename base_type::char_type char_type;
    //! String type to be used as a message text holder
    typedef typename base_type::string_type string_type;

public:
    /*!
     * Constructor. Initializes the sink backend.
     */
	UnitTest_output_backend() {}
    /*!
     * Destructor
     */
	~UnitTest_output_backend() {}

    /*!
     * The method passes the formatted message to debugger
     */
    void consume(logging::record_view const& rec, string_type const& formatted_message)
	{
		Logger::WriteMessage((formatted_message + '\n').c_str());
	}
};

static void RegisterDebugBoostSink()
{
	boost::shared_ptr< logging::core > core = logging::core::get();
	typedef sinks::synchronous_sink< UnitTest_output_backend<char> > sink_t;
	boost::shared_ptr< sink_t > sink = boost::make_shared< sink_t > ();
	core->add_sink(sink);
}