/**
 *  \file ref_counting.h
 *  \brief Helpers to handle reference counting.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMP_REF_COUNTING_H
#define IMP_REF_COUNTING_H

#include "../RefCounted.h"
#include "../log.h"

#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>

IMP_BEGIN_INTERNAL_NAMESPACE


template <class O>
void unref(O* o)
{
  if (!o) return;
  // need to know about possible virtual destructors
  // or the correct non-virtual one
  const RefCounted *rc= o;
  IMP_INTERNAL_CHECK(rc->count_ !=0, "Too many unrefs on object");
  --rc->count_;
  IMP_LOG(MEMORY, "Unrefing object " << rc << std::endl);
  if (rc->count_==0) {
    delete o;
  }
}

template <class O>
void release(O* o)
{
  if (!o) return;
  // need to know about possible virtual destructors
  // or the correct non-virtual one
  const RefCounted *rc= o;
  IMP_INTERNAL_CHECK(rc->count_ !=0, "Release called on unowned object");
  --rc->count_;
  //IMP_INTERNAL_CHECK(rc->count_ == 0, "Release called on shared object.");
}


template <class O>
void ref(O* o)
{
  if (!o) return;
  const RefCounted *r= o;
  IMP_LOG(MEMORY, "Refing object " << r << std::endl);
  ++r->count_;
}

IMP_END_INTERNAL_NAMESPACE

#endif  /* IMP_REF_COUNTING_H */
