/**
 *  \file log.h     \brief Logging and error reporting support.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_LOG_H
#define IMP_LOG_H

#include "config.h"
#include "internal/log_internal.h"
#include "utility.h"

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cassert>
#include <string>

IMP_BEGIN_NAMESPACE

/** \defgroup log Logging
    \defgroup assert Error checking and reporting

    By default \imp performs a variety of runtime error checks. These
    can be controlled using the IMP::set_check_level function. Call
    IMP::set_check_level with IMP::NONE to disable all checks when you
    are performing your optimization as opposed to testing your
    code. Make sure you run your code with the level set to at least
    CHEAP before running your final optimization to make sure that
    \imp is used correctly.

    Use the gdbinit file provided in tools to automatically have gdb
    break when errors are detected.
 */

//! The log levels supported by IMP
/** \ingroup log
    DEFAULT is only local logging (like in IMP::Object), it means use
    the global log level
    VERBOSE prints very large amounts of information. It should be enough
    to allow the computational flow to be understood.
    TERSE prints a few lines per restraint or per state each time
    the score is evaluated.
    WARNING prints only warnings.
    MEMORY print information about allocations and deallocations to debug
    memory issues
 */
enum LogLevel {DEFAULT=-1, SILENT=0, WARNING=1, TERSE=2, VERBOSE=3,
               MEMORY=4};

//! The targets for IMP logging
/** \ingroup log
 */
enum LogTarget {COUT, FILE, CERR};


#if !defined SWIG && !defined(IMP_DOXYGEN)
namespace internal {
  IMPEXPORT extern LogLevel log_level;
}
#endif

//! Set the current log level for IMP
/** \ingroup log
 */
inline void set_log_level(LogLevel l) {
  internal::log_level=l;
}

//! Set the target of logs
/** \ingroup log
 */
IMPEXPORT inline void set_log_target(LogTarget l)
{
  internal::Log::get().set_target(l);
}

//! Get the current log level for IMP
/** \ingroup log
 */
inline LogLevel get_log_level()
{
  return internal::log_level;
}

//! Get the target of logs
/** \ingroup log
 */
IMPEXPORT inline LogTarget get_log_target()
{
  return LogTarget(internal::Log::get().get_target());
}

//! Set the file name for the IMP log; must be called if a file is to be used.
/** \ingroup log
 */
IMPEXPORT inline void set_log_file(std::string l)
{
  internal::Log::get().set_filename(l);
}

//! Determine whether a given log level should be output.
/** \note This probably should not be called in C++.
    \ingroup log
 */
IMPEXPORT inline bool is_log_output(LogLevel l)
{
  return l <= get_log_level();
}


//! The stream to output a particular log level to.
/** \note This probably should not be called in C++.
    \ingroup log
 */
IMPEXPORT inline std::ostream& get_log_stream(LogLevel l)
{
  return internal::Log::get().get_stream(l);
}


#ifndef IMP_DISABLE_LOGGING


//! Execute the code block if a certain level logging is on
/**
   The next code block (delimited by { }) is executed if
   get_check_level() <= level.
 */
#define IMP_IF_LOG(level)\
  if (level <= ::IMP::get_log_level())


//! Write an entry to a log.
/** \param[in] level The IMP::Log_Level for the message
    \param[in] expr A stream expression to be sent to the output stream
    \ingroup log
 */
#define IMP_LOG(level, expr) if (IMP::is_log_output(level)) \
    { IMP::get_log_stream(level) << expr << std::flush;};

//! Write an entry to a log. This is to be used for objects with no operator<<.
/** \param[in] level The IMP::Log_Level for the message
    \param[in] expr An expression which writes something to IMP_STREAM
    \ingroup log
 */
#define IMP_LOG_WRITE(level, expr) if (IMP::is_log_output(level)) \
    {std::ostream &IMP_STREAM= IMP::get_log_stream(level); expr;}

#else
#define IMP_LOG(l,e)
#define IMP_LOG_WRITE(l,e)
#define IMP_IF_LOG(l) if (false)
#endif



//! Write a warning to a log.
/** \param[in] expr An expression to be output to the log. It is prefixed
                    by "WARNING"
    \ingroup log
 */
#define IMP_WARN(expr) if (IMP::is_log_output(IMP::WARNING)) \
    { IMP::get_log_stream(IMP::WARNING) << "WARNING  " << expr << std::flush;};

//! Write an entry to a log. This is to be used for objects with no operator<<.
/** \param[in] expr An expression which writes something to IMP_STREAM.
                    It is prefixed by "WARNING"
    \ingroup log
 */
#define IMP_WARN_WRITE(expr) if (IMP::is_log_output(IMP::WARNING)) \
    {std::ostream &IMP_STREAM= IMP::get_log_stream(IMP::Log::WARNING); expr;}



//! Write a warning to standard error.
/** \param[in] expr An expression to be output to std::cerr. It is prefixed
                    by "ERROR"
    \ingroup log
 */
#define IMP_ERROR(expr) std::cerr << "ERROR: " << expr << std::endl;

//! Write an entry to standard error; for objects with no operator<<.
/** \param[in] expr An expression which writes something to IMP_STREAM.
                    It is prefixed by "ERROR"
    \ingroup log
 */
#define IMP_ERROR_WRITE(expr) { \
  std::ostream &IMP_STREAM = std::cerr; \
  std:cerr<< "ERROR "; \
  expr; \
  std::cerr << std::endl; \
}


//! Set the log level
/** \ingroup log
 */
#define IMP_SET_LOG_LEVEL(level) IMP::Log::get()::set_level(level);

IMP_END_NAMESPACE

#endif  /* IMP_LOG_H */
