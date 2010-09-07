/**
 *  \file InContainerQuadFilter.h    \brief A filter for Quads.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H
#define IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H

#include "container_config.h"
#include <IMP/QuadFilter.h>
#include <IMP/QuadContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE


//! A filter which returns true if a container containers the Quad
/** Stores a searchable shared collection of Quads.
    \ingroup restraints

    Implementors should see IMP_QUAD_FILTER().
 */
class IMPCONTAINEREXPORT InContainerQuadFilter : public QuadFilter
{
  IMP::internal::OwnerPointer<QuadContainer> c_;
public:
  InContainerQuadFilter(QuadContainer *c,
                             std::string name="QuadFilter %1%");

  IMP_QUAD_FILTER(InContainerQuadFilter);
};

inline bool InContainerQuadFilter
::get_contains_particle_quad(const ParticleQuad& p) const {
  return c_->get_contains_particle_quad(p);
}

IMP_OBJECTS(InContainerQuadFilter, InContainerQuadFilters);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_IN_CONTAINER_QUAD_FILTER_H */
