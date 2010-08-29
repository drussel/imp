/**
 *  \file QuadFilter.h    \brief A filter for particle_quads.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_QUAD_FILTER_H
#define IMP_QUAD_FILTER_H

#include "kernel_config.h"
#include "Particle.h"
#include "utility.h"
#include "VersionInfo.h"
#include "base_types.h"
#include "VersionInfo.h"
#include "ParticleTuple.h"
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>


IMP_BEGIN_NAMESPACE


//! A shared filter for particle_quads
/** Stores a searchable shared collection of particle_quads.
    \ingroup restraints

    Implementors should see IMP_QUAD_FILTER().
 */
class IMPEXPORT QuadFilter : public Object
{
public:
  QuadFilter(std::string name="QuadFilter %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular filter.
   */
  virtual bool get_contains_particle_quad(const ParticleQuad& p) const =0;

  /** \name Interactions
      Return the set of particles used when applied to the passed
      list.
      @{
  */
  virtual ParticlesTemp get_input_particles(const ParticleQuad& p) const=0;
  virtual ContainersTemp get_input_containers(const ParticleQuad& p) const=0;
  /** @} */

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  virtual void filter_in_place(ParticleQuadsTemp& ps) const {
  }
#endif

  IMP_REF_COUNTED_DESTRUCTOR(QuadFilter);
};

IMP_OUTPUT_OPERATOR(QuadFilter);

IMP_OBJECTS(QuadFilter,QuadFilters);

IMP_END_NAMESPACE

#endif  /* IMP_QUAD_FILTER_H */
