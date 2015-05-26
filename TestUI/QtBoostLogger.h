#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <boost/log/core.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sinks/basic_sink_backend.hpp>
#include <boost/log/sinks/frontend_requirements.hpp>
#include <boost/log/attributes/attribute_value_set.hpp>
#include <boost/log/core/record_view.hpp>

namespace logging = boost::log;
namespace sinks = boost::log::sinks;

class QtBoostLogger
	: public QObject, public sinks::basic_formatted_sink_backend< char, sinks::concurrent_feeding >
{
	Q_OBJECT

	typedef sinks::basic_formatted_sink_backend< char, sinks::concurrent_feeding > base_type;
public:
	typedef base_type::char_type char_type;
	typedef base_type::string_type string_type;

	QtBoostLogger(void)
	{
	}

	~QtBoostLogger(void)
	{
	}

	void consume(logging::record_view const& rec, string_type const& formatted_message)
	{
		//Logger::WriteMessage((formatted_message + '\n').c_str());
		messageConsumed(QString(formatted_message.c_str()));
	}

signals:
	void messageConsumed(QString formatted_message);
};

