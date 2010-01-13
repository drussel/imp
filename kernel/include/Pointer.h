/**
 *  \file Pointer.h
 *  \brief A NULL-initialized pointer to an IMP Object.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMP_POINTER_H
#define IMP_POINTER_H


#include "WeakPointer.h"
#include "RefCounted.h"
#include "Object.h"
#include "internal/ref_counting.h"
#include "internal/OwnerPointer.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

IMP_BEGIN_NAMESPACE

//! A reference counted pointer to an object.
/** The object being pointed to must inherit from IMP::RefCountedObject.
    Use an IMP::WeakPointer to break cycles or to point to
    non-ref-counted objects.

    \param[in] O The type of IMP::RefCounted-derived object to point to
 */
template <class O>
class Pointer: public WeakPointer<O>
{
  // Make sure O is not RefCounted itself as RefCounted is not polymorphic
  BOOST_STATIC_ASSERT((!boost::is_base_of<O, RefCounted>::value));

  void check(const RefCounted *){}
  void check(const Object *o) {
    if (o) IMP_CHECK_OBJECT(o);
  }

  typedef WeakPointer<O> P;
  typedef Pointer<O> This;

  void set_pointer(O* p) {
    if (p == P::o_) return;
    if (P::o_) internal::unref(P::o_);
    if (p) internal::ref(p);
    check(p);
    P::o_=p;
  }
  // issue with commas
  BOOST_STATIC_ASSERT((boost::is_base_of<RefCounted, O>::value));

public:
  /** copy constructor */
  Pointer(const Pointer &o) {
    set_pointer(o.o_);
  }
  /** copy from another */
  Pointer& operator=(const Pointer &o){
    set_pointer(o.o_);
    return *this;
  }
  //! initialize to NULL
  Pointer() {}
  /** initialize from a pointer */
  Pointer(O* o) {
    IMP_INTERNAL_CHECK(o, "Can't initialize with NULL pointer");
    set_pointer(o);
  }
  /** drop control of the object */
  ~Pointer(){
    set_pointer(NULL);
  }

  //! Set it from a possibly NULL pointer.
  Pointer<O>& operator=(O* o) {
    set_pointer(o);
    return *this;
  }
};


//! Make a ref counted pointer to an object. Useful for temporaries.
template <class T>
Pointer<T> make_pointer(T*t) {
  return Pointer<T>(t);
}

IMP_END_NAMESPACE

#endif  /* IMP_POINTER_H */
