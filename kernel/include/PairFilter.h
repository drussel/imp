/**
 *  \file PairFilter.h    \brief A filter for particle_pairs.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_FILTER_H
#define IMP_PAIR_FILTER_H

#include "kernel_config.h"
#include "Particle.h"
#include "utility.h"
#include "VersionInfo.h"
#include "base_types.h"
#include "VersionInfo.h"

IMP_BEGIN_NAMESPACE


//! A shared filter for particle_pairs
/** Stores a searchable shared collection of particle_pairs.
    \ingroup restraints

    Implementors should see IMP_PAIR_FILTER().
 */
class IMPEXPORT PairFilter : public Object
{
public:
  PairFilter(std::string name="PairFilter %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular filter.
   */
  virtual bool get_contains_particle_pair(const ParticlePair& p) const =0;

  /** \name Interactions
      Return the set of particles used when applied to the passed
      list.
      @{
  */
  virtual ParticlesTemp get_input_particles(const ParticlePair& p) const=0;
  virtual ObjectsTemp get_input_objects(const ParticlePair& p) const=0;
  /** @} */

  IMP_REF_COUNTED_DESTRUCTOR(PairFilter);
};

IMP_OUTPUT_OPERATOR(PairFilter);

IMP_OBJECTS(PairFilter,PairFilters);

IMP_END_NAMESPACE

#endif  /* IMP_PAIR_FILTER_H */
