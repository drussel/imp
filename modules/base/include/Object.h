/**
 *  \file base/Object.h
 *  \brief A shared base class to help in debugging and things.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_OBJECT_H
#define IMPBASE_OBJECT_H

#include "base_config.h"
#include "RefCounted.h"
#include "exception.h"
#include "VersionInfo.h"
#include "base_macros.h"
#include "log.h"
#include "types.h"
#include <IMP/compatibility/hash.h>
#include <boost/functional/hash.hpp>


IMPBASE_BEGIN_NAMESPACE
class RefCounted;

//! Common base class for heavy weight \imp objects.
/** The base class for non value-type objects in \imp.
    Anything inheriting from IMP::Object has the following
    properties:
    - has a method Object::show() which writes one or more lines of text
      to a stream
    - has embedded information about the module and version which can be
      accessed using Object::get_version_info(). This information can be
      used to log what version of software is used to compute a result.
    - it has a local logging level which can override the global one
      allowing fine grained logging control.
    - the object keeps track of whether it has been been used. See the
      IMP::Object::set_was_used() method for an explanation.

    Objects can be outputted to standard streams using operator<<()
    which will call the Object::show() method.

    \advanceddoc Types inheriting from Object should always be created using
    \c new in C++ and passed using pointers and stored using
    IMP::Pointer objects. Note that you have to be careful of cycles
    and so must use IMP::WeakPointer objects to break cycles. See
    IMP::RefCounted for more information on reference counting. IMP_NEW()
    can help shorten creating a ref counted pointer. See IMP::Pointer for
    more information.
 */
class IMPBASEEXPORT Object: public RefCounted
{
  std::string name_;
  int compare(const Object &o) const {
    if (&o < this) return 1;
    else if (&o > this) return -1;
    else return 0;
  }
protected:
  Object(std::string name);
  IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Object);
public:


  //! Set the logging level used in this object
  /** Each object can be assigned a different log level in order to,
      for example, suppress messages for verbose and uninteresting
      objects. If set to DEFAULT, the global level as returned by
      IMP::get_log_level() is used, otherwise
      the local one is used. Methods in classes inheriting from
      Object should start with IMP_OBJECT_LOG to change the log
      level to the local one for this object and increase
      the current indent.
   */
  virtual void set_log_level(LogLevel l) {
    IMP_USAGE_CHECK(l <= MEMORY && l >= DEFAULT, "Setting to invalid log level "
              << l);
#if IMP_BUILD < IMP_FAST
    log_level_=l;
#endif
  }

  /** Each object can be assigned a different check level too.
   */
  virtual void set_check_level(CheckLevel l) {
#if IMP_BUILD < IMP_FAST
    check_level_=l;
#endif
  }

#ifndef IMP_DOXYGEN
  LogLevel get_log_level() const {
#if IMP_BUILD >= IMP_FAST
      return SILENT;
#else
      return log_level_;
#endif
  }
  CheckLevel get_check_level() const {
#if IMP_BUILD >= IMP_FAST
      return NONE;
#else
      return check_level_;
#endif
  }
#endif // IMP_DOXYGEN

  //! Return a string identifying the type of the object
  virtual std::string get_type_name() const=0;

  //! Print out one or more lines of text describing the object
  void show(std::ostream &out=std::cout) const {
    out << get_name()
        << "(" << get_type_name() << ")\n";
    do_show(out);
  }


  IMP_HASHABLE_INLINE(Object, return boost::hash_value(this););

#ifndef IMP_DOXYGEN
  void _debugger_show() const {
    show(std::cout);
  }

  //! Return a string version of the object, can be used in the debugger
  std::string get_string() const {
    std::ostringstream oss;
    show(oss);
    return oss.str();
  }
#endif // IMP_DOXYGEN

  //! Get information about the module and version of the object
  virtual IMP::base::VersionInfo get_version_info() const=0;

 /** @name Names
      All objects have names to aid in debugging and inspection
      of the state of the system. These names are not necessarily unique
      and should not be used to store data or as keys into a table. Use
      the address of the object instead since objects cannot be copied.
      @{
  */
  const std::string& get_name() const {
    return name_;
  }
  void set_name(std::string name) {
    name_=name;
  }
  /* @} */


  /** \imp provides warnings when objects are never used before they are
      destroyed. Examples of use include adding an IMP::Restraint to an
      IMP::Model. If an object is not properly marked as used, or your
      code is the one using it, call set_was_used(true) on the object.
  */
  void set_was_used(bool tf) const {
#if IMP_BUILD < IMP_FAST
    was_owned_=tf;
#endif
  }

#ifndef IMP_DOXYGEN
  // swig needs to know to wrap this function
  virtual void do_show(std::ostream &out) const =0;

  void _on_destruction() {
#if IMP_BUILD < IMP_FAST
    LogLevel old=IMP::base::get_log_level();
    if (log_level_!= DEFAULT) {
      IMP::base::set_log_level(log_level_);
    }
    log_level_=old;
#endif
  }
#endif

 private:
  Object(const Object &): RefCounted() {}
  const Object& operator=(const Object &) {return *this;}

#if IMP_BUILD < IMP_FAST
  static void add_live_object(Object*o);
  static void remove_live_object(Object*o);

  LogLevel log_level_;
  CheckLevel check_level_;
  mutable bool was_owned_;
  double check_value_;
#endif
};


#if !defined(IMP_DOXYGEN) && !defined(IMP_SWIG)
inline std::ostream &operator<<(std::ostream &out, const Object& o) {
  o.show(out);
  return out;
}
#endif



/** Up (or down) cast an \imp Object-derived class. If the cast
    does not succeed a ValueException will be thrown. Use a
    \c dynamic_cast if you prefer to have a NULL returned.
 */
template <class O, class I>
inline O* object_cast(I *o) {
  O *ret= dynamic_cast<O*>(o);
  if (!ret) {
    if (!o) {
      IMP_THROW("Cannot cast NULL pointer to desired type.", ValueException);
    } else {
      IMP_THROW("Object " << o->get_name() << " cannot be cast to "
                << "desired type.", ValueException);
    }
  }
  return ret;
}

#ifndef IMP_DOXYGEN
inline void show(std::ostream &out, Object *o) {
  out << "\"" << o->get_name() << "\"";
}

template <class OS>
inline void show_objects(const OS &os, std::ostream &out) {
  out << "[";
  for (unsigned int i=0; i< os.size(); ++i) {
    if (i!= 0) out << ", ";
    out << '"' << os[i]->get_name() << '"';
  }
  out << "]";
}
#endif



IMPBASE_END_NAMESPACE

#ifdef IMP_DOXYGEN
//! Perform some basic validity checks on the object for memory debugging
#define IMP_CHECK_OBJECT(obj)
#elif IMP_BUILD < IMP_FAST
#define IMP_CHECK_OBJECT(obj) do {                                      \
    IMP_INTERNAL_CHECK((obj), "NULL object");                           \
    IMP_INTERNAL_CHECK((obj)->get_is_valid(), "Check object "           \
                       << static_cast<const void*>(obj)                 \
                       << " was previously freed");                     \
} while (false)
#else
#define IMP_CHECK_OBJECT(obj)
#endif


/** When accepting objects as arguments, it is good practice to wrap them
    in a reference counted pointer. This ensures that they are freed if
    they are passed as temporaries. Put this macro call as one of the first
    lines in the function.
*/
#define IMP_ACCEPT_OBJECT(obj) IMP::Pointer<Object> imp_control##obj(obj);

#ifdef IMP_DOXYGEN
//! Set the log level to the object's log level.
/** All non-trivial Object methods should start with this. It creates a
    RAII-style object which sets the log level to the local one,
    if appropriate, until it goes out of scope.
 */
#define IMP_OBJECT_LOG

//! Beginning logging for a non-member function
/**
 */
#define IMP_FUNCTION_LOG

#endif

// recommended by http://gcc.gnu.org/gcc/Function-Names.html
#if defined(_MSC_VER)
#  define __func__ __FUNCTION__
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ < 199901L
# if __GNUC__ >= 2
#  define __func__ __FUNCTION__
# else
#  define __func__ "<unknown>"
# endif
#endif

#if IMP_BUILD < IMP_FAST
#define IMP_OBJECT_LOG \
  IMP::base::SetLogState log_state_guard__(this->get_log_level());      \
  IMP::base::SetCheckState check_state_guard__(this->get_check_level()); \
  IMP_CHECK_OBJECT(this);                                               \
  IMP::base::CreateLogContext log_context__(__func__, this)


#define IMP_FUNCTION_LOG                                                \
  IMP::base::CreateLogContext log_context__(__func__)


#else // fast
#define IMP_OBJECT_LOG
#define IMP_FUNCTION_LOG
#endif // fast

#endif  /* IMPBASE_OBJECT_H */
