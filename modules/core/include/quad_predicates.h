/**
 *  \file core/quad_predicates.h    \brief Define some predicates.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPCORE_QUAD_PREDICATES_H
#define IMPCORE_QUAD_PREDICATES_H

#include "core_config.h"
#include <IMP/QuadPredicate.h>
#include "internal/container_helpers.h"

IMPCORE_BEGIN_NAMESPACE

/** Always return a constant value.
 */
class IMPCOREEXPORT ConstantQuadPredicate: public QuadPredicate {
  int v_;
public:
  ConstantQuadPredicate(int v,
                             std::string name="ConstQuadPredicate%1%");
  IMP_INDEX_QUAD_PREDICATE(ConstantQuadPredicate, {
      IMP_UNUSED(m); IMP_UNUSED(pi);
      return v_;
    });
};

/** Return a unique predicate value for each unordered set of ParticleTypes
    (see Typed).
*/

class IMPCOREEXPORT UnorderedTypeQuadPredicate: public QuadPredicate {
public:
  UnorderedTypeQuadPredicate(std::string name
                             ="UnorderedTypeQuadPredicate%1%");
  IMP_INDEX_QUAD_PREDICATE(UnorderedTypeQuadPredicate, {
      return internal::get_type_hash(m, pi);
    });
};

/** Return a unique predicate value for each order list of ParticleTypes
    (see Typed).
*/
class IMPCOREEXPORT OrderedTypeQuadPredicate: public QuadPredicate {
public:
  OrderedTypeQuadPredicate(std::string name
                             ="OrderedTypeQuadPredicate%1%");
  IMP_INDEX_QUAD_PREDICATE(OrderedTypeQuadPredicate, {
      return internal::get_ordered_type_hash(m, pi);
    });
  int get_value(const core::ParticleTypes& types) {
    return internal::get_ordered_type_hash(types);
  }
};

/** Return true if all members of the tuple are the same. */
class IMPCOREEXPORT AllSameQuadPredicate: public QuadPredicate {
public:
  AllSameQuadPredicate(std::string name
                             ="AllSameQuadPredicate%1%");
  IMP_INDEX_QUAD_PREDICATE(AllSameQuadPredicate, {
      return internal::get_all_same(m, pi);
    });
};


IMPCORE_END_NAMESPACE

#endif  /* IMPCORE_QUAD_PREDICATES_H */
