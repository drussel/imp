/**
 *  \file QuadFilter.h    \brief A filter for Quads.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_QUAD_FILTER_H
#define IMPKERNEL_QUAD_FILTER_H

#include "kernel_config.h"
#include "base_types.h"
#include "macros.h"
#include "VersionInfo.h"
#include "ParticleTuple.h"
#include "internal/container_helpers.h"

IMP_BEGIN_NAMESPACE


//! A shared filter for Quads
/** Stores a searchable shared collection of Quads.
    \ingroup restraints

    Implementors should see IMP_QUAD_FILTER().
 */
class IMPEXPORT QuadFilter : public base::Object
{
public:
  QuadFilter(std::string name="QuadFilter %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular filter.
   */
  virtual bool get_contains(const ParticleQuad& p) const =0;

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular filter.
   */
  virtual bool get_contains(Model *m,
                            const ParticleIndexQuad& p) const {
    return get_contains(internal::get_particle(m,p));
  }

  /** \name Interactions
      Return the set of particles used when applied to the passed
      list.
      @{
  */
  virtual ParticlesTemp get_input_particles(Particle* p) const=0;
  virtual ContainersTemp get_input_containers(Particle* p) const=0;
  /** @} */

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  virtual void filter_in_place(Model *m, ParticleIndexQuads& ps) const;
  virtual void filter_in_place(ParticleQuadsTemp& ps) const;
#endif

  IMP_REF_COUNTED_DESTRUCTOR(QuadFilter);
};

IMP_OUTPUT_OPERATOR(QuadFilter);

IMP_OBJECTS(QuadFilter,QuadFilters);

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_QUAD_FILTER_H */
