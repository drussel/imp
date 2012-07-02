/**
 *  \file base/check_macros.h
 *  \brief Exception definitions and assertions.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_CHECK_MACROS_H
#define IMPBASE_CHECK_MACROS_H

#include "base_config.h"
#include "exception.h"
#include <iostream>
#include <cmath>

#ifdef IMP_DOXYGEN
/** Catch any IMP exception thrown by expr and terminate with an
    error message. Use this for basic error handling in main functions
    in C++. Do not use within the \imp library.
*/
#define IMP_CATCH_AND_TERMINATE(expr)

//! Execute the code block if a certain level checks are on
/**
   The next code block (delimited by { }) is executed if
   get_check_level() <= level.

   For example:
    \code
    IMP_CHECK_CODE(CHECK_USAGE) {
        base::Vector<Particle*> testp(input.begin(), input.end());
        std::sort(testp.begin(), testp.end());
        IMP_USAGE_CHECK(std::unique(testp.begin(), testp.end()) == testp.end(),
                        "Duplicate particles found in the input list.");
    }
    \endcode
*/
#define IMP_IF_CHECK(level)

/** \copydoc IMP_IF_CHECK

    Do the check with a given probability.
 */
#define IMP_IF_CHECK_PROBABILISTIC(level, probability)

//! Only compile the code if checks are enabled
/** For example
    \code
    IMP_CHECK_CODE({
        base::Vector<Particle*> testp(input.begin(), input.end());
        std::sort(testp.begin(), testp.end());
        IMP_USAGE_CHECK(std::unique(testp.begin(), testp.end()) == testp.end(),
                        "Duplicate particles found in the input list.");
    });
    \endcode
**/
#define IMP_CHECK_CODE(expr)


/** \brief An assertion to check for internal errors in \imp. An
    IMP::ErrorException will be thrown.

    Since it is a debug-only check and no attempt should be made to
    recover from it, the exception type cannot be specified.

    For example:
    \code
    IMP_INTERNAL_CHECK((3.14-PI) < .01,
                       "PI is not close to 3.14. It is instead " << PI);
    \endcode

    \note if the code is compiled with 'fast', or the check level is
    less than IMP::USAGE_AND_INTERNAL, the check is not performed.  Do
    not use asserts as a shorthand to throw exceptions (throw the
    exception yourself); use them only to check for logic errors.

    \param[in] expr The assertion expression.
    \param[in] message Write this message if the assertion fails.
*/
#define IMP_INTERNAL_CHECK(expr, message)


/** This is like IMP_INTERNAL_CHECK, however designed to check if
    two floating point numbers are almost equal. The check looks something
    like
    \code
    std::abs(a-b) < .1*(a+b)+.1
    \endcode
    Using this makes such tests a bit easier to spot and not mess up.
*/
#define IMP_INTERNAL_CHECK_FLOAT_EQUAL(expra, exprb, message)

//! A runtime test for incorrect usage of a class or method.
/** \param[in] expr The assertion expression.
    \param[in] message Write this message if the assertion fails.

    It should be used to check arguments to function. For example
    \code
    IMP_USAGE_CHECK(positive_argument >0,
                    "Argument positive_argument to function my_function "
                    << " must be positive. Instead got " << positive_argument);
    \endcode

    \note if the build is 'fast', or the check level
    is less than IMP::USAGE, the check is not performed. Do not use these
    checks as a shorthand to throw necessary exceptions (throw the
    exception yourself); use them only to check for errors, such as
    inappropriate input.
 */
#define IMP_USAGE_CHECK(expr, message)

/** This is like IMP_USAGE_CHECK, however designed to check if
    two floating point numbers are almost equal. The check looks something
    like
    \code
    std::abs(a-b) < .1*(a+b)+.1
    \endcode
    Using this makes such tests a bit easier to spot and not mess up.
*/
#define IMP_USAGE_CHECK_FLOAT_EQUAL(expra, exprb, message)


//! Throw an exception with a message
/** The exception thrown must inherit from Exception and not be
    UsageException or InternalException as those are reserved for
    disableable checks (the IMP_INTERNAL_CHECK() and IMP_USAGE_CHECK()
    macros).
    \code
    IMP_THROW("Could not open file " << file_name,
              IOException);
    \endcode
 */
#define IMP_THROW(message, exception_name)


//! A runtime failure for IMP.
/** \param[in] message Failure message to write.
    This macro is used to provide nice error messages when there is
    an internal error in \imp. It causes an IMP::InternalException to be
    thrown.
*/
#define IMP_FAILURE(message)

//! Use this to make that the method is not implemented yet
/**
 */
#define IMP_NOT_IMPLEMENTED

#else // IMP_DOXYGEN

#define IMP_CATCH_AND_TERMINATE(expr)                   \
  try {                                                 \
    expr;                                               \
  } catch (const IMP::base::Exception &e) {             \
    std::cerr << "Application terminated with error :"  \
              << e.what() << std::endl;                 \
    exit(1);                                            \
  }

#define IMP_THROW(message, exception_name)do {                          \
    /* to bring in exceptions for backward compat */                    \
    using namespace IMP::base;                                          \
    std::ostringstream oss;                                             \
    oss << message << std::endl;                                        \
  BOOST_STATIC_ASSERT((!(boost::is_base_of<IMP::base::UsageException,   \
                          exception_name>::value)                       \
                       && !(boost::is_base_of<IMP::base::InternalException, \
                             exception_name>::value)                    \
                       && (boost::is_base_of<IMP::base::Exception,      \
                            exception_name>::value)));                  \
  throw exception_name(oss.str().c_str());                              \
  } while (true)

#define IMP_FAILURE(message) do {                                       \
  std::ostringstream oss;                                               \
  oss << message << std::endl;                                          \
  IMP::base::handle_error(oss.str().c_str());                           \
  throw IMP::base::InternalException(oss.str().c_str());                \
  } while (true)
#define IMP_NOT_IMPLEMENTED do {                                        \
    IMP::base::handle_error("This method is not implemented.");         \
    throw IMP::base::InternalException("Not implemented");              \
  } while(true)

#if IMP_BUILD < IMP_FAST
#define IMP_IF_CHECK(level)                      \
  using IMP::base::NONE;                         \
  using IMP::base::USAGE;                        \
  using IMP::base::USAGE_AND_INTERNAL;           \
  if (level <= ::IMP::base::get_check_level())

#define IMP_IF_CHECK_PROBABILISTIC(level, prob) \
  if (level <= ::IMP::base::get_check_level() \
      && boost::uniform_real<>(0,1)(IMP::base::random_number_generator) < prob)


#define IMP_CHECK_CODE(expr) expr

#define IMP_INTERNAL_CHECK(expr, message)                       \
  do {                                                          \
    if (IMP::base::get_check_level()                            \
        >= IMP::base::USAGE_AND_INTERNAL && !(expr)) {          \
      std::ostringstream oss;                                   \
      oss << "Internal check failure: " << message << std::endl \
          << "  File \"" << __FILE__ << "\", line " << __LINE__ \
          << IMP::base::get_context_message()                   \
          << std::endl;                                         \
      IMP::base::handle_error(oss.str().c_str());               \
      throw IMP::base::InternalException(oss.str().c_str());    \
    }                                                           \
  } while(false)

#define IMP_INTERNAL_CHECK_FLOAT_EQUAL(expra, exprb, message)           \
  IMP_INTERNAL_CHECK(std::abs((expra)-(exprb)) <                        \
                     .1*std::abs((expra)+(exprb))+.1,                   \
                     (expra) << " != " << (exprb)                       \
                     << " - " << message)


#define IMP_USAGE_CHECK(expr, message)                          \
  do {                                                          \
    if (IMP::base::get_check_level() >= IMP::base::USAGE && !(expr)) {        \
      std::ostringstream oss;                                   \
      oss << "Usage check failure: " << message                 \
          << IMP::base::get_context_message()                         \
          << std::endl;                                         \
      IMP::base::handle_error(oss.str().c_str());                     \
      throw IMP::base::UsageException(oss.str().c_str());             \
    }                                                           \
  } while (false)
#define IMP_USAGE_CHECK_FLOAT_EQUAL(expra, exprb, message)              \
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  IMP_USAGE_CHECK(std::abs((expra)-(exprb))                             \
                  < .1*std::abs((expra)+(exprb))+.1,                    \
                  expra << " != " << exprb                              \
=======
  IMP_USAGE_CHECK(std::abs(expra-exprb) < .1*std::abs(expra+exprb)+.1,  \
                  expra << " != " << exprb                              \
                  <<" within "<< .1*std::abs(expra+exprb)+.1            \
>>>>>>> print out values when equal check fails
=======
  IMP_USAGE_CHECK(std::abs(expra-exprb) < .1*std::abs(expra+exprb)+.1,  \
                  expra << " != " << exprb                              \
                  <<" within "<< .1*std::abs(expra+exprb)+.1            \
>>>>>>> print out values when equal check fails
=======
  IMP_USAGE_CHECK(std::abs(expra-exprb) < .1*std::abs(expra+exprb)+.1,  \
                  expra << " != " << exprb                              \
                  <<" within "<< .1*std::abs(expra+exprb)+.1            \
>>>>>>> print out values when equal check fails
=======
  IMP_USAGE_CHECK(std::abs(expra-exprb) < .1*std::abs(expra+exprb)+.1,  \
                  expra << " != " << exprb                              \
                  <<" within "<< .1*std::abs(expra+exprb)+.1            \
>>>>>>> print out values when equal check fails
                  <<" - " <<  message)

#else // IMP_BUILD < IMP_FAST
#define IMP_IF_CHECK(level) if (0)
#define IMP_IF_CHECK_PROBABILISTIC(level, prob) if (0)
#define IMP_CHECK_CODE(expr)
#define IMP_INTERNAL_CHECK(expr, message)
#define IMP_INTERNAL_CHECK_FLOAT_EQUAL(expra, exprb, message)
#define IMP_USAGE_CHECK(expr, message)
#define IMP_USAGE_CHECK_FLOAT_EQUAL(expra, exprb, message)
#endif // IMP_BUILD < IMP_FAST



#endif // IMP_DOXYGEN

#if defined(IMP_DOXYGEN) || IMP_BUILD == IMP_FAST
//! Perform some basic validity checks on the object for memory debugging
#define IMP_CHECK_OBJECT(obj)
#define IMP_CHECK_OBJECT_IF_NOT_nullptr(obj)
#else
#define IMP_CHECK_OBJECT(obj) do {                                      \
    IMP_INTERNAL_CHECK((obj), "nullptr object");                           \
    IMP_INTERNAL_CHECK((obj)->get_is_valid(), "Check object "           \
                       << static_cast<const void*>(obj)                 \
                       << " was previously freed");                     \
} while (false)

#define IMP_CHECK_OBJECT_IF_NOT_nullptr(obj) do {                          \
    if (obj) {                                                          \
      IMP_INTERNAL_CHECK((obj)->get_is_valid(), "Check object "         \
                         << static_cast<const void*>(obj)               \
                         << " was previously freed");                   \
    }                                                                   \
  } while (false)
#endif


#endif  /* IMPBASE_CHECK_MACROS_H */
