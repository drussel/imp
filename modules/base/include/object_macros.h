/**
 *  \file IMP/base/object_macros.h
 *  \brief Various general useful macros for IMP.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPBASE_OBJECT_MACROS_H
#define IMPBASE_OBJECT_MACROS_H
#include "base_config.h"
#include "ref_counted_macros.h"
#include "Vector.h"
#include "Pointer.h"
#include "WeakPointer.h"


//! Define the basic things needed by any Object
/** This defines
    - IMP::base::Object::get_version_info()
    - a private destructor
    and declares
    - IMP::base::Object::do_show()
*/
#define IMP_OBJECT_INLINE(Name, show, destructor)                       \
  public:                                                               \
  IMP_IMPLEMENT_INLINE(virtual std::string get_type_name() const,       \
                        return #Name);                                  \
  IMP_IMPLEMENT_INLINE( virtual ::IMP::base::VersionInfo                \
                        get_version_info() const,                       \
  return ::IMP::base::VersionInfo(get_module_name(),                    \
                                  get_module_version()));               \
  /** \brief For python, cast a generic Object to this type. Throw a
      ValueException of object is not the right type.*/                 \
static Name* get_from(IMP::base::Object *o) {                           \
  return IMP::base::object_cast<Name>(o);                               \
}                                                                       \
IMP_IMPLEMENT_INLINE(virtual void do_show(std::ostream &out) const,     \
                      show);                                            \
IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, IMP::base::Object::_on_destruction(); \
                                  destructor;)


//! Define the basic things needed by any Object
/** This defines
    - IMP::base::Object::get_version_info()
    - a private destructor
    and declares
    - IMP::base::Object::do_show()
*/
#define IMP_OBJECT(Name)                                                \
  public:                                                               \
  IMP_IMPLEMENT_INLINE(virtual std::string get_type_name() const,       \
                        return #Name);                                  \
  IMP_IMPLEMENT_INLINE( virtual ::IMP::base::VersionInfo                \
                        get_version_info() const,                       \
  return ::IMP::base::VersionInfo(get_module_name(),                    \
                                  get_module_version()));               \
  /** \brief For Python, cast a generic Object to this type. Throw a
      ValueException if object is not the right type.*/                 \
static Name* get_from(IMP::base::Object *o) {                           \
  return IMP::base::object_cast<Name>(o);                               \
  }                                                                     \
IMP_IMPLEMENT(virtual void do_show(std::ostream &out) const);           \
IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, IMP::base::Object::_on_destruction();)






//! Define the basic things needed by any internal Object
/** \see IMP_OBJECT
    This version also defines IMP::base::Object::do_show()
*/
#define IMP_INTERNAL_OBJECT(Name)                                       \
  public:                                                               \
  virtual ::IMP::base::VersionInfo get_version_info() const {           \
    return  ::IMP::base::VersionInfo(get_module_name(),                 \
                                     get_module_version());             \
  }                                                                     \
  virtual std::string get_type_name() const {                           \
    return #Name;                                                       \
  }                                                                     \
private:                                                                \
virtual void do_show(std::ostream & =std::cout) const {                 \
}                                                                       \
IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name,                                 \
                                  IMP::base::Object::_on_destruction();)


#ifdef IMP_DOXYGEN
//! Define the types for storing sets of objects
/** The macro defines the types PluralName and PluralNameTemp.
    PluralName should be Names unless the English spelling is
    different.
 */
#define IMP_OBJECTS(Name, PluralName)

#else

#define IMP_OBJECTS(Name, PluralName)                           \
  typedef IMP::base::Vector<IMP::base::Pointer<Name> >          \
  PluralName;                                                   \
  typedef IMP::base::Vector<IMP::base::WeakPointer<Name> >      \
  PluralName##Temp;

#endif


#define IMP_GENERIC_OBJECT(Name, lcname, targument, carguments, cparguments) \
  typedef Generic##Name<targument> Name;                                \
  template <class targument>                                            \
  Generic##Name<targument>* create_##lcname carguments {                \
    return new Generic##Name<targument>cparguments;                      \
  }


//! Declare a ref counted pointer to a new object
/** \param[in] Typename The namespace qualified type being declared
    \param[in] varname The name for the ref counted pointer
    \param[in] args The arguments to the constructor, or ()
    if there are none.
    Please read the documentation for IMP::Pointer before using.
*/
#define IMP_NEW(Typename, varname, args)        \
  IMP::base::Pointer<Typename> varname(new Typename args)



/** When accepting objects as arguments, it is good practice to wrap them
    in a reference counted pointer. This ensures that they are freed if
    they are passed as temporaries. Put this macro call as one of the first
    lines in the function.
*/
#define IMP_ACCEPT_OBJECT(obj) IMP::Pointer<Object> imp_control##obj(obj);



#ifdef _MSC_VER
// VC doesn't understand friends properly
#define IMP_REF_COUNTED_DESTRUCTOR(Name)        \
  public:                                       \
  virtual ~Name(){}                             \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)

#define IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, dest)              \
  public:                                                          \
  virtual ~Name(){dest}                                            \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)


#define IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Name)     \
  public:                                               \
  virtual ~Name()


#elif defined(SWIG)
// SWIG doesn't do friends right either, but we don't care as much
#define IMP_REF_COUNTED_DESTRUCTOR(Name)                \
  public:                                               \
  virtual ~Name(){}                                     \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)

#define IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, dest)              \
  public:                                                          \
  virtual ~Name(){dest}                                            \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)


#define IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Name)     \
  public:                                               \
  virtual ~Name()


#elif defined(IMP_DOXYGEN)
/* The destructor is unprotected for SWIG since if it is protected
   SWIG does not wrap the Python proxy distruction and so does not
   dereference the ref counted pointer. SWIG also gets confused
   on template friends.
*/
//! Ref counted objects should have private destructors
/** This macro defines a private destructor and adds the appropriate
    friend methods so that the class can be used with ref counting.
    By defining a private destructor, you make it so that the object
    cannot be declared on the stack and so must be ref counted.

    \see IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR()
    \see IMP::base::RefCounted
*/
#define IMP_REF_COUNTED_DESTRUCTOR(Name)


/** Like IMP_REF_COUNTED_DESTRUCTOR(), but the destructor is only
    declared, not defined.
*/
#define IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Name)

/** Like IMP_REF_COUNTED_DESTRUCTOR(), but the destructor is declared
    inline.
*/
#define IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, destructor)


#else
#define IMP_REF_COUNTED_DESTRUCTOR(Name)                                \
  protected:                                                            \
  template <class T, class E> friend struct IMP::base::internal::RefStuff; \
  virtual ~Name(){}                                                     \
public:                                                                 \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)

#define IMP_REF_COUNTED_INLINE_DESTRUCTOR(Name, dest)                   \
  protected:                                                            \
  template <class T, class E> friend struct IMP::base::internal::RefStuff; \
  virtual ~Name(){dest}                                                 \
public:                                                                 \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)


#define IMP_REF_COUNTED_NONTRIVIAL_DESTRUCTOR(Name)                     \
  protected:                                                            \
  template <class T, class E> friend struct IMP::base::internal::RefStuff; \
  virtual ~Name();                                                      \
public:                                                                 \
  IMP_REQUIRE_SEMICOLON_CLASS(destructor)
#endif

#endif  /* IMPBASE_OBJECT_MACROS_H */