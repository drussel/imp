/**
 *  \file SingletonFilter.h    \brief A filter for Singletons.
 *
 *  This file is generated by a script (core/tools/make-containers).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_SINGLETON_FILTER_H
#define IMP_SINGLETON_FILTER_H

#include "kernel_config.h"
#include "base_types.h"
#include "macros.h"
#include "VersionInfo.h"
#include "ParticleTuple.h"
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>


IMP_BEGIN_NAMESPACE


//! A shared filter for Singletons
/** Stores a searchable shared collection of Singletons.
    \ingroup restraints

    Implementors should see IMP_SINGLETON_FILTER().
 */
class IMPEXPORT SingletonFilter : public Object
{
public:
  SingletonFilter(std::string name="SingletonFilter %1%");

  /** \note This function may be linear. Be aware of the complexity
      bounds of your particular filter.
   */
  virtual bool get_contains_particle(Particle* p) const =0;

  /** \name Interactions
      Return the set of particles used when applied to the passed
      list.
      @{
  */
  virtual ParticlesTemp get_input_particles(Particle* p) const=0;
  virtual ContainersTemp get_input_containers(Particle* p) const=0;
  /** @} */

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  virtual void filter_in_place(ParticlesTemp& ps) const {
    IMP_FAILURE("filter_in_place not overloaded properly.");
  }
#endif

  IMP_REF_COUNTED_DESTRUCTOR(SingletonFilter);
};

IMP_OUTPUT_OPERATOR(SingletonFilter);

IMP_OBJECTS(SingletonFilter,SingletonFilters);

IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_FILTER_H */
