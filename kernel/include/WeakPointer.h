/**
 *  \file WeakPointer.h
 *  \brief A NULL-initialized pointer to an Object.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_WEAK_POINTER_H
#define IMP_WEAK_POINTER_H
#include "internal/PointerBase.h"


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
struct WeakPointer: internal::PointerBase<O, internal::WeakPointerTraits> {
  typedef  internal::PointerBase<O, internal::WeakPointerTraits> P;
  template <class Any>
  WeakPointer(const Any &o): P(o){}
  WeakPointer(){}
  using P::operator=;
};

IMP_END_NAMESPACE

#endif  /* IMP_WEAK_POINTER_H */
