/**
 *  \file base/declare_Object.h
 *  \brief A shared base class to help in debugging and things.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_DECLARE_OBJECT_H
#define IMPBASE_DECLARE_OBJECT_H

#include "base_config.h"
#include "RefCounted.h"
#include "ref_counted_macros.h"
#include "enums.h"
#include "hash_macros.h"
#include "showable_macros.h"
#include "VersionInfo.h"
#include <IMP/compatibility/hash.h>
#include <boost/functional/hash.hpp>


IMPBASE_BEGIN_NAMESPACE

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
    - in python, there is a method Class::get_from(Object *o) that attempts
      to case o to an object of type Class and throws and exception if it
      fails.
    - the object keeps track of whether it has been been used. See the
      IMP::Object::set_was_used() method for an explanation.

    Objects can be outputted to standard streams using operator<<()
    which will call the Object::show() method.

    \headerfile Object.h "IMP/base/Object.h"

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
  void set_log_level(LogLevel l);

  /** Each object can be assigned a different check level too.
   */
  void set_check_level(CheckLevel l) {
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

  IMP_SHOWABLE(Object);

#ifndef IMP_DOXYGEN
  // swig needs to know to wrap this function
  virtual void do_show(std::ostream &out) const =0;

  void _on_destruction();
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


IMPBASE_END_NAMESPACE

#endif  /* IMPBASE_DECLARE_OBJECT_H */
