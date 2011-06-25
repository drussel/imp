/**
 *  \file WeakPointer.h
 *  \brief A NULL-initialized pointer to an Object.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_WEAK_POINTER_H
#define IMP_WEAK_POINTER_H

#include "Object.h"
#include "macros.h"
#include "exception.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/mpl/not.hpp>
#include <boost/mpl/and.hpp>
#include <IMP/compatibility/hash.h>

IMP_BEGIN_NAMESPACE

//! A weak pointer to an IMP::Object or IMP::RefCountedObject.
/** WeakPointers do not do reference counting and do not claim ownership
    of the pointed object. As a result, they can be used to break cycles
    in reference counted pointers. For example, since an IMP::Model
    contains a reference counted pointer to an IMP::Particle, the
    IMP::Particle has a WeakPointer back to the Model.

    \param[in] O The type of IMP::Object-derived object to point to
 */
template <class O>
class WeakPointer
{

  void set_pointer(O* p) {
    if (p == o_) return;
    if (p) {
      //IMP_CHECK_OBJECT(p);
    }
    o_=p;
  }
  void audit(const void *t) const {
    IMP_INTERNAL_CHECK(t != NULL, "Pointer is NULL");
  }
  void audit(const RefCounted*t) const {
    IMP_INTERNAL_CHECK(t != NULL, "Pointer is NULL");
    IMP_INTERNAL_CHECK(t->get_ref_count() >0, "Ref count is null");
  }
  void audit(const Object*t) const {
    IMP_INTERNAL_CHECK(t != NULL, "Pointer is NULL");
    IMP_CHECK_OBJECT(t);
  }
  void audit() const {
    audit(o_);
  }
protected:
  IMP_NO_DOXYGEN(O* o_);
public:
  //! initialize to NULL
  WeakPointer(): o_(NULL) {}
  /** initialize from a pointer */
  explicit WeakPointer(O* o): o_(NULL) {
    IMP_INTERNAL_CHECK(o, "Can't initialize with NULL pointer");
    set_pointer(o);
  }
  const O& operator*() const {
    audit();
    return *o_;
  }
  O& operator*()  {
    audit();
    return *o_;
  }
  O* operator->() const {
    audit();
    return o_;
  }
  O* operator->() {
    audit();
    return o_;
  }
  //! get the raw pointer
  O* get() const {
    audit();
    return o_;
  }
  //! Set it from a possibly NULL pointer.
  WeakPointer<O>& operator=(O* o) {
    set_pointer(o);
    return *this;
  }

  IMP_COMPARISONS_1(WeakPointer, o_);

  //! Return true if the pointer is not NULL
  bool operator!() const {
    return !o_;
  }

  //! convert to the raw pointer
  operator O*() const {
    return o_;
  }

  IMP_HASHABLE_INLINE(WeakPointer, return boost::hash_value(o_););
};

IMP_END_NAMESPACE

#endif  /* IMP_WEAK_POINTER_H */
