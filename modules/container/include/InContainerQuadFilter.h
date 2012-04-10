/**
 *  \file InContainerQuadFilter.h    \brief A filter for Quads.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H
#define IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H

#include "container_config.h"
#include <IMP/QuadPredicate.h>
#include <IMP/QuadContainer.h>
#include <IMP/internal/container_helpers.h>

IMPCONTAINER_BEGIN_NAMESPACE


//! A filter which returns true if a container containers the Quad
/** This predicate returns 1 if the passed tuple is in the container.
    \note Only the exact tuple, not permutations of it are searched for.

 */
class IMPCONTAINEREXPORT InContainerQuadFilter :
    public QuadPredicate
{
  IMP::OwnerPointer<QuadContainer> c_;
public:
  InContainerQuadFilter(QuadContainer *c,
                             std::string name="QuadFilter %1%");

  IMP_INDEX_QUAD_PREDICATE(InContainerQuadFilter,{
      IMP_UNUSED(m);
      return c_->get_contains_index(pi);
    });
};


IMP_OBJECTS(InContainerQuadFilter, InContainerQuadFilters);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H */
