/**
 *  \file IMP/SingletonDerivativeModifier.h
 *  \brief A Modifier on ParticlesTemp
 *
 *  WARNING This file was generated from NAMEDerivativeModifier.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_SINGLETON_DERIVATIVE_MODIFIER_H
#define IMPKERNEL_SINGLETON_DERIVATIVE_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "internal/container_helpers.h"


IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticlesTemp
/** The primary function of such a class is to change
    the derivatives of the passed particles.

    Implementors should see and
    IMP_SINGLETON_DERIVATIVE_MODIFIER() and
    IMP::SingletonModifier.
 */
class IMPEXPORT SingletonDerivativeModifier : public base::Object
{
public:
  typedef Particle* Argument;
  typedef ParticleIndex IndexArgument;
  SingletonDerivativeModifier(std::string name="SingletonModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(Particle*,
                     DerivativeAccumulator &da) const=0;

 /** Apply the function to a single value*/
  virtual void apply_index(Model *m, ParticleIndex v,
                           DerivativeAccumulator &da) const {
    apply(internal::get_particle(m, v), da);
  }

  /** Apply the function to a collection of ParticlesTemp */
  virtual void apply_indexes(Model *m, const ParticleIndexes &o,
                             DerivativeAccumulator &da) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply_index(m, o[i], da);
    }
  }

  /** Get the set of particles read when applied to the arguments.*/
  virtual ParticlesTemp
    get_input_particles(Particle* p) const =0;
  /** Get the set of particles modifier when applied to the arguments.*/
  virtual ParticlesTemp
    get_output_particles(Particle *p) const =0;
  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;
  /** Get the set of output containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_output_containers(Particle *p) const =0;
};


IMP_OBJECTS(SingletonDerivativeModifier,SingletonDerivativeModifiers);


IMP_END_NAMESPACE

#endif  /* IMPKERNEL_SINGLETON_DERIVATIVE_MODIFIER_H */
