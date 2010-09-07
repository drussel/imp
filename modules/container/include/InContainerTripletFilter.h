/**
 *  \file InContainerTripletFilter.h    \brief A filter for Triplets.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 Sali Lab. All rights reserved.
 */

#ifndef IMPCONTAINER_IN_CONTAINER_TRIPLET_FILTER_H
#define IMPCONTAINER_IN_CONTAINER_TRIPLET_FILTER_H

#include "container_config.h"
#include <IMP/TripletFilter.h>
#include <IMP/TripletContainer.h>

IMPCONTAINER_BEGIN_NAMESPACE


//! A filter which returns true if a container containers the Triplet
/** Stores a searchable shared collection of Triplets.
    \ingroup restraints

    Implementors should see IMP_TRIPLET_FILTER().
 */
class IMPCONTAINEREXPORT InContainerTripletFilter : public TripletFilter
{
  IMP::internal::OwnerPointer<TripletContainer> c_;
public:
  InContainerTripletFilter(TripletContainer *c,
                             std::string name="TripletFilter %1%");

  IMP_TRIPLET_FILTER(InContainerTripletFilter);
};

inline bool InContainerTripletFilter
::get_contains_particle_triplet(const ParticleTriplet& p) const {
  return c_->get_contains_particle_triplet(p);
}

IMP_OBJECTS(InContainerTripletFilter, InContainerTripletFilters);

IMPCONTAINER_END_NAMESPACE

#endif  /* IMPCONTAINER_IN_CONTAINER_TRIPLET_FILTER_H */
