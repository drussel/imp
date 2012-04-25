/**
 *  \file example/creating_restraints.h
 *  \brief A simple unary function.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */


#ifndef IMPEXAMPLE_CREATING_RESTRAINTS_H
#define IMPEXAMPLE_CREATING_RESTRAINTS_H

#include "example_config.h"
#include <IMP/container/ClosePairContainer.h>
#include <IMP/container/generic.h>
#include <IMP/container/ConsecutivePairContainer.h>
#include <IMP/container/ListSingletonContainer.h>
#include <IMP/core/SphereDistancePairScore.h>
#include <IMP/core/DistancePairScore.h>


IMPEXAMPLE_BEGIN_NAMESPACE

/** Restraint the passed particles to be connected in a chain. The distance
    between consecutive particles is length_factor*the sum of the radii.

    Note, this assumes that all such chains will be disjoint and so you can
    use the container::ExclusiveConsecutivePairFilter if you want to filter
    out all pairs of particles connected by such chain restraints.

    The restraint is not added to the model.
*/
inline Restraint* create_chain_restraint(const ParticlesTemp &ps,
                                         double length_factor,
                                         double k,
                                         std::string name) {
  IMP_USAGE_CHECK(!ps.empty(), "No Particles passed.");
  double scale = core::XYZR(ps[0]).get_radius();
  IMP_NEW(core::HarmonicDistancePairScore, hdps, (length_factor*2.0*scale,
                                                  k, "chain linker %1%"));
  // the true, says that the particles will be in no other
  // ConsecutivePairContainer
  // this accelerates certain computations
  IMP_NEW(container::ExclusiveConsecutivePairContainer, cpc,
          (ps, name+" consecutive pairs"));
  Pointer<Restraint> r= container::create_restraint(hdps.get(), cpc.get(),
                                                    "chain restraint %1%");
  // make sure it is not freed
  return r.release();
}


/** Create an excluded-volume style ClosePairsContainer based score. */
inline container::ClosePairContainer*
create_excluded_volume(const ParticlesTemp &ps,
                       double k,
                       std::string name) {
  IMP_USAGE_CHECK(!ps.empty(), "No Particles passed.");
  Model *m= ps[0]->get_model();
  double scale = core::XYZR(ps[0]).get_radius();
  IMP_NEW(container::ListSingletonContainer, cores_container,
          (ps, name+" list"));
  // Create a close pair container, with a distance bound of 0 and a slack
  // that is proportional to the particle radius
  IMP_NEW(container::ClosePairContainer,
          cpc, (cores_container, 0, scale*.3));
  IMP_NEW(core::SoftSpherePairScore, hlb, (k));
  Pointer<Restraint> r= container::create_restraint(hlb.get(), cpc.get());
  m->add_restraint(r);
  return cpc.release();
}

IMPEXAMPLE_END_NAMESPACE

#endif  /* IMPEXAMPLE_CREATING_RESTRAINTS_H */
