/**
 *  \file base/log.h
 *  \brief Logging and error reporting support.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_LOG_H
#define IMPBASE_LOG_H

#include "base_config.h"
#include "enums.h"
#include "WarningContext.h"
#include "internal/log.h"
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>

IMPBASE_BEGIN_NAMESPACE

/** \name Logging
    \anchor log
    \imp provides tools for controlling the amount of log output produced
    and directing it to the terminal or a file. Only log messages tagged
    with a lower level than the current LogLevel are emitted. In addition
    to a global log level (get_log_level(), set_log_level()), each
    IMP::Object has an internal log level (IMP::Object::get_log_level(),
    IMP::Object::set_log_level()) which is used when executing code on
    that object.

    Logging is provided by IMP/log.h.

    People implementing IMP::Object classes should also see IMP_OBJECT_LOG()
    and IMP::SetLogState.

    All logging is disabled when \imp is built using \c build='fast'.
    @{
 */

//! Push a new log context onto the stack
/** A log context is, eg, a function name.
 */
IMPBASEEXPORT void push_log_context(const char *functionname,
                                    const void *object);

//! pop the top log context
IMPBASEEXPORT void pop_log_context();

//! Write a string to the log
IMPBASEEXPORT void add_to_log(std::string to_write);


//! Set the current global log level
/** Note that this should not, currently, be used directly
    during Model::evaluate() calls. */
IMPBASEEXPORT void set_log_level(LogLevel l);

//! Set whether log messages are tagged with the current log time
IMPBASEEXPORT void set_log_timer(bool tb);

//! Reset the log timer
IMPBASEEXPORT void reset_log_timer();


//! Get the currently active log level
/** This may not always match the value passed to set_log_level()
    as objects can temporarily override the global level
    while they are evaluating.
 */
inline LogLevel get_log_level()
{
  return internal::log_level;
}

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
inline bool get_is_log_output(LogLevel l)
{
  return l <= get_log_level();
}
#endif

/** @} */

IMPBASE_END_NAMESPACE


#endif  /* IMPBASE_LOG_H */
