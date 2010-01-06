/**
 *  \file Log.cpp   \brief Logging and error reporting support.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#include "IMP/log.h"
#include "IMP/internal/log_internal.h"
#include "IMP/exception.h"

#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/operations.hpp>
#include <boost/iostreams/filtering_stream.hpp>


IMP_BEGIN_INTERNAL_NAMESPACE

LogLevel log_level= TERSE;
LogTarget log_target= COUT;
unsigned int log_indent=0;

namespace {
class LogStream:
  public boost::iostreams::filtering_stream<boost::iostreams::output>,
  public boost::noncopyable  {
  typedef boost::iostreams::filtering_stream<boost::iostreams::output> P;
  std::ostream *out_;
  std::string prefix_;

  struct IndentFilter: public boost::iostreams::output_filter {
    LogStream *ps_;
    bool to_indent_;
    IndentFilter(LogStream *ps): ps_(ps), to_indent_(false){};
    template <typename Sink>
    bool put(Sink &sink, char c) {
      if (c=='\n') {
        to_indent_=true;
      } else if (to_indent_) {
        for (unsigned int i=0; i< log_indent; ++i) {
          boost::iostreams::put(sink, ' ');
        }
        to_indent_=false;
      }
      return boost::iostreams::put(sink, c);
    }
  };

  struct LogSink: boost::iostreams::sink {
    LogStream *ps_;
    LogSink(LogStream *ps): ps_(ps){}
    unsigned int write(const char *s, std::streamsize n) {
      ps_->out_->write(s, n);
      return n;
    }
  };
  friend class LogSink;
  friend class IndentFilter;
 public:
  LogStream(std::ostream *out): out_(out) {
    P::push(IndentFilter(this));
    P::push(LogSink(this));
  }
  void set_stream(std::ostream *out) {
    out_=out;
  }
};
}


IMP_END_INTERNAL_NAMESPACE

IMP_BEGIN_NAMESPACE

namespace {
  IMP_CHECK_CODE(double initialized=11111111);
  std::ofstream fstream;
  internal::LogStream stream(&std::cout);
}


void set_log_level(LogLevel l) {
  IMP_USAGE_CHECK(l >= SILENT && l < ALL_LOG,
            "Setting log to invalid level: " << l,
            ValueException);
  internal::log_level=l;
}

void set_log_target(LogTarget l)
{
  if (l== COUT) {
    stream.set_stream(&std::cout);
  } else if (l==CERR) {
    stream.set_stream(&std::cerr);
  } else {
    stream.set_stream(&fstream);
  }
}

void set_log_file(std::string l) {
  if (!l.empty()) {
    fstream.open(l.c_str());
    if (!fstream.is_open()) {
      IMP_FAILURE("Error opening log file " << l);
    } else {
      internal::log_target=FILE;
    }
  } else {
    fstream.close();
    internal::log_target=COUT;
  }
}

void log_write(std::string str) {
  IMP_INTERNAL_CHECK(initialized=11111111,
             "You connot use the log before main is called.");
  stream.write(str.c_str(), str.size());
  stream.strict_sync();
}

IMP_END_NAMESPACE
