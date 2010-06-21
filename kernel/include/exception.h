/**
 *  \file exception.h     \brief Exception definitions and assertions.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_EXCEPTION_H
#define IMP_EXCEPTION_H

#include "kernel_config.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

#include <cassert>
#include <cstring>
#include <string>
#include <iostream>
#include <new>
#include <sstream>

IMP_BEGIN_NAMESPACE


/**
    \name Error checking and reporting
    \anchor assert

    By default \imp performs a variety of runtime error checks. These
    can be controlled using the IMP::set_check_level function. Call
    IMP::set_check_level with IMP::NONE to disable all checks when you
    are performing your optimization as opposed to testing your
    code. Make sure you run your code with the level set to at least
    USAGE before running your final optimization to make sure that
    \imp is used correctly.

    Error handling is provided by IMP/exception.h,

    Use the \c gdbinit file provided in \c tools to automatically have \c gdb
    break when \imp errors are detected.
    @{
 */

//! The general base class for \imp exceptions
/** Exceptions should be used to report all errors that occur within \imp.
*/
class IMPEXPORT Exception
{
  struct refstring {
    char message_[4096];
    int ct_;
  };
  refstring *str_;
 public:
  const char *what() const throw() {
    return str_? str_->message_: NULL;
  }
  Exception(const char *message) {
    str_= new (std::nothrow) refstring();
    if (str_ != NULL) {
      str_->ct_=1;
      std::strncpy(str_->message_, message, 4095);
      str_->message_[4095]='\0';
    }
  }
  /* \note By making the destructor virtual and providing an implementation in
      each derived class, we force a strong definition of the exception object
      in the kernel DSO. This allows exceptions to be passed between DSOs.
  */
  virtual ~Exception() throw();

  Exception(const Exception &o) {copy(o);}
  Exception &operator=(const Exception &o) {
    destroy();
    copy(o);
    return *this;
  }
 private:
  void destroy() {
    if (str_ != NULL) {
      --str_->ct_;
      if (str_->ct_==0) delete str_;
    }
  }
  void copy(const Exception &o) {
    str_=o.str_;
    if (str_!= NULL) ++str_->ct_;
  }
};

//! Determine the level of runtime checks performed
/** - NONE means that minimial checks are
    used.
    - USAGE means that checks of input values to functions
    and classes are verified.
    - USAGE_AND_INTERNAL adds checks that \imp itself is
    correct. Turn these on if you suspect an \imp bug or are
    developing Restraints or other \imp classes.
*/
enum CheckLevel {NONE=0, USAGE=1, USAGE_AND_INTERNAL=2};

//! Determine the maximum check level that can be used for this build
/** For example, 'fast' builds can't use any checks.
 */
IMPEXPORT CheckLevel get_maximum_check_level();

#if !defined(SWIG) && !defined(IMP_DOXYGEN)
namespace internal {
  IMPEXPORT extern CheckLevel check_mode;
}
#endif


//! Control runtime checks in the code
/** The default level of checks is USAGE for release builds and
    USAGE_AND_INTERNAL for debug builds.
*/
inline void set_check_level(CheckLevel tf) {
  internal::check_mode= tf;
}

//! Get the current audit mode
/**
 */
inline CheckLevel get_check_level() {
  return internal::check_mode;
}


//! Set whether exception messages are printed or not
/** By default the error message associated with thrown exceptions are printed
    when using from C++, but not from Python (since the error messages of
    an unhandled exception are printed by the Python runtime).
*/
IMPEXPORT void set_print_exceptions(bool tf);

#ifdef IMP_DOXYGEN
//! Execute the code block if a certain level checks are on
/**
   The next code block (delimited by { }) is executed if
   get_check_level() <= level.

   For example:
    \code
    IMP_CHECK_CODE(CHECK_USAGE) {
        std::vector<Particle*> testp(input.begin(), input.end());
        std::sort(testp.begin(), testp.end());
        IMP_USAGE_CHECK(std::unique(testp.begin(), testp.end()) == testp.end(),
                        "Duplicate particles found in the input list.");
    }
    \endcode
*/
#define IMP_IF_CHECK(level)

//! Only compile the code if checks are enabled
/** For example
    \code
    IMP_CHECK_CODE({
        std::vector<Particle*> testp(input.begin(), input.end());
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

#define IMP_THROW(message, exception_name)do {                          \
  std::ostringstream oss;                                               \
  oss << message << std::endl;                                          \
  BOOST_STATIC_ASSERT((!(boost::is_base_of<IMP::UsageException,         \
                          exception_name>::value)                       \
                       && !(boost::is_base_of<IMP::InternalException,   \
                             exception_name>::value)                    \
                       && (boost::is_base_of<IMP::Exception,            \
                            exception_name>::value)));                  \
  throw exception_name(oss.str().c_str());                              \
  } while (true)
#define IMP_FAILURE(message) do {                                       \
  std::ostringstream oss;                                               \
  oss << message << std::endl;                                          \
  IMP::internal::assert_fail(oss.str().c_str());                        \
  throw InternalException(oss.str().c_str());                           \
  } while (true)
#define IMP_NOT_IMPLEMENTED do {                                        \
    IMP::internal::assert_fail("This method is not implemented.");      \
    throw InternalException("Not implemented");                         \
  } while(true)

#if IMP_BUILD < IMP_FAST
#define IMP_IF_CHECK(level)                     \
  if (level <= ::IMP::get_check_level())
#define IMP_CHECK_CODE(expr) expr

#define IMP_INTERNAL_CHECK(expr, message)                       \
  do {                                                          \
    if (IMP::get_check_level() >= IMP::USAGE_AND_INTERNAL && !(expr)) { \
      std::ostringstream oss;                                   \
      oss << message << std::endl                               \
          << "  File \"" << __FILE__ << "\", line " << __LINE__ \
          << std::endl;                                         \
      IMP::internal::assert_fail(oss.str().c_str());            \
      throw IMP::InternalException(oss.str().c_str());          \
    }                                                           \
  } while(false)
#define IMP_USAGE_CHECK(expr, message)                          \
  do {                                                          \
    if (IMP::get_check_level() >= IMP::USAGE && !(expr)) {      \
      std::ostringstream oss;                                   \
      oss << message << std::endl;                              \
      IMP::internal::assert_fail(oss.str().c_str());            \
      throw IMP::UsageException(oss.str().c_str());             \
    }                                                           \
  } while (false)
#else // IMP_BUILD < IMP_FAST
#define IMP_IF_CHECK(level) if (0)
#define IMP_CHECK_CODE(expr)
#define IMP_INTERNAL_CHECK(expr, message)
#define IMP_USAGE_CHECK(expr, message)
#endif // IMP_BUILD < IMP_FAST



#endif // IMP_DOXYGEN




#if !defined(SWIG) && !defined(IMP_DOXYGEN)
namespace internal {
IMPEXPORT void assert_fail(const char *msg);
}
#endif




/** @} */



//! A general exception for an intenal error in IMP.
/** This exception is thrown by the IMP_INTERNAL_CHECK() and
    IMP_FAILURE() macros. It should never be caught.
 */
struct IMPEXPORT InternalException: public Exception
{
  InternalException(const char *msg="Fatal error"): Exception(msg){}
  ~InternalException() throw();
};

//! An exception for an invalid usage of \imp
/** It is thrown by the IMP_USAGE_CHECK() macro. It should never be
    caught internally to \imp, but it one may be able to recover from
    it being thrown.

    \advanceddoc
    As the usage checks are disabled in fast mode,
    UsageExceptions are not considered part of the API and hence
    should not be documented or checked in test cases.
 */
class IMPEXPORT UsageException : public Exception
{
 public:
  UsageException(const char *t): Exception(t){}
  ~UsageException() throw();
};

//! An exception for an invalid value being passed to \imp
/** The equivalent Python type also derives from Python's ValueError.
 */
class IMPEXPORT ValueException : public Exception
{
 public:
  ValueException(const char *t): Exception(t){}
  ~ValueException() throw();
};


//! An exception for a request for an invalid member of a container
/** The equivalent Python type also derives from Python's IndexError.
 */
class IMPEXPORT IndexException: public Exception
{
 public:
  //! Create exception with an error message
  IndexException(const char *t): Exception(t){}
  ~IndexException() throw();
};

//! An input/output exception
/** This exception should be used when an IO
    operation fails in a way that leaves the internal state OK. For
    example, failure to open a file should result in an IOException.

    It is OK to catch such exceptions in \imp.

    The equivalent Python type also derives from Python's IOError.
 */
class IMPEXPORT IOException: public Exception
{
 public:
  IOException(const char *t): Exception(t){}
  ~IOException() throw();
};


/** \brief An exception which is thrown when the Model has
    attributes with invalid values.

    It may be OK to catch an \imp ModelException, when, for example,
    the catcher can simply re-randomize the optimized coordinates and
    restart the optimization. Sampling protocols, such as
    IMP::core::MCCGSampler, tend to do this.
 */
class IMPEXPORT ModelException: public Exception
{
 public:
  //! Create exception with an error message
  ModelException(const char *t): Exception(t){}
  ~ModelException() throw();
};

IMP_END_NAMESPACE

#endif  /* IMP_EXCEPTION_H */
