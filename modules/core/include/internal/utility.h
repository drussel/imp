/**
 *  \file core/internal/utility.h
 *  \brief Various important functionality
 *         for implementing decorators.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 *
 */

#ifndef IMPCORE_INTERNAL_UTILITY_H
#define IMPCORE_INTERNAL_UTILITY_H

#include "../macros.h"

// All below can go away once decoratorbase is updated
#include "../config.h"
#include <IMP/Decorator.h>

IMPCORE_BEGIN_INTERNAL_NAMESPACE
/** Stupid hack since I can't change the kernel. This should go away
 and the calls to it above should be replace by P::is_instance_of*/
template <class T> inline bool parent_instance(::IMP::Particle *p) {
  return T::particle_is_instance(p);
}
/** inline is needed, much to my surprise.
 */
template <>
inline bool parent_instance<Decorator>(::IMP::Particle *p) {
  return true;
}

IMPCORE_END_INTERNAL_NAMESPACE

#endif  /* IMPCORE_INTERNAL_UTILITY_H */
