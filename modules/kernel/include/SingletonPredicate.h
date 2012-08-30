/**
 *  \file IMP/SingletonPredicate.h
 *  \brief Define SingletonPredicate.
 *
 *  WARNING This file was generated from NAMEPredicate.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_SINGLETON_PREDICATE_H
#define IMPKERNEL_SINGLETON_PREDICATE_H

#include "kernel_config.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"
#include "internal/container_helpers.h"

IMP_BEGIN_NAMESPACE

//! Abstract predicate function
/** A predicate is a function which returns one of a discrete set of
    values (eg -1, 0, 1 depending on whether a value is negative, zero
    or positive). SingletonPredicates will evaluate the predicate for the passed
    particles.

    Implementers should check out IMP_SINGLETON_PREDICATE().
*/
class IMPEXPORT SingletonPredicate : public base::Object
{
 public:
  typedef Particle* Argument;
  typedef ParticleIndex IndexArgument;
  SingletonPredicate(std::string name="SingletonPredicate %1%");
  //! Compute the predicate.
  virtual int get_value(Particle* vt) const =0;

  /** Implementations
      for these are provided by the IMP_SINGLETON_PREDICATE()
      macro.
  */
  virtual Ints get_value(const ParticlesTemp &o) const {
    Ints ret(o.size());
    for (unsigned int i=0; i< o.size(); ++i) {
      ret[i]+= get_value(o[i]);
    }
    return ret;
  }

#if !defined(IMP_DOXYGEN) && !defined(SWIG)
  virtual void remove_if_equal(Model *m,
                               ParticleIndexes& ps, int v) const;
  virtual void remove_if_not_equal(Model *m,
                                   ParticleIndexes& ps, int v) const;
#endif

  //! Compute the predicate and the derivative if needed.
  virtual int get_value_index(Model *m, ParticleIndex vt) const {
    return get_value(internal::get_particle(m, vt));
  }

  /** Implementations
      for these are provided by the IMP_SINGLETON_PREDICATE()
      macro.
  */
  virtual Ints get_value_index(Model *m,
                                const ParticleIndexes &o) const {
    Ints ret(o.size());
    for (unsigned int i=0; i< o.size(); ++i) {
      ret[i]+= get_value_index(m, o[i]);
    }
    return ret;
  }



  /** Get the set of particles read when applied to the arguments. */
  virtual ParticlesTemp
    get_input_particles(Particle *p) const =0;

  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;

  IMP_REF_COUNTED_DESTRUCTOR(SingletonPredicate);
};


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_SINGLETON_PREDICATE_H */
