/**
 *  \file internal/utility.h
 *  \brief Various useful utilities
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPBASE_INTERNAL_LOG_STREAM_H
#define IMPBASE_INTERNAL_LOG_STREAM_H

#include "../base_config.h"
#include "../file.h"
#include "static.h"
#include <istream>
#include <sstream>
#include <boost/iostreams/categories.hpp>
#include <boost/iostreams/operations.hpp>
#include <boost/iostreams/filtering_stream.hpp>


IMPBASE_BEGIN_NAMESPACE
namespace internal {
class LogStream:
  public boost::iostreams::filtering_stream<boost::iostreams::output>,
  public boost::noncopyable  {
  typedef boost::iostreams::filtering_stream<boost::iostreams::output> P;
  TextOutput out_;
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
        if (print_time) {
          std::ostringstream oss;
          oss << log_timer.elapsed();
          std::string str= oss.str();
          for (unsigned int i=0; i< str.size(); ++i) {
            boost::iostreams::put(sink, str[i]);
          }
          boost::iostreams::put(sink, ':');
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
      ps_->out_.get_stream().write(s, n);
      return n;
    }
  };
  friend struct LogSink;
  friend struct IndentFilter;
 public:
  LogStream(): out_(TextOutput(std::cout)) {
    P::push(IndentFilter(this));
    P::push(LogSink(this));
  }
  ~LogStream() {
    // make sure nothing is written during destruction
    set_log_level(SILENT);
  }
  void set_stream(TextOutput out) {
    // temporarily disable writes, otherwise at log level MEMORY the log is
    // displayed using the old out_ object, which is in the process of being
    // freed (generally this leads to a segfault)
    LogLevel old = get_log_level();
    set_log_level(SILENT);
    out_=out;
    set_log_level(old);
  }
  TextOutput get_stream() const {
    return out_;
  }
};
}
IMPBASE_END_NAMESPACE

#endif  /* IMPBASE_INTERNAL_LOG_STREAM_H */
