/**
 *  \file SingletonDerivativeModifier.h
 *  \brief A Modifier on ParticlesTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_SINGLETON_DERIVATIVE_MODIFIER_H
#define IMP_SINGLETON_DERIVATIVE_MODIFIER_H

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
class IMPEXPORT SingletonDerivativeModifier : public Object
{
public:
  typedef Particle* Argument;
  SingletonDerivativeModifier(std::string name="SingletonModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(Particle*,
                     DerivativeAccumulator &da) const=0;
  /** Apply the function to a collection of ParticlesTemp */
  virtual void apply(const ParticlesTemp &o,
                     DerivativeAccumulator &da) const {
    for (unsigned int i=0; i< o.size(); ++i) {
      apply(o[i], da);
    }
  }

 /** Apply the function to a single value*/
  virtual void apply(Model *m, ParticleIndex v,
                     DerivativeAccumulator &da) const {
    apply(internal::get_particle(m, v), da);
  }

  /** Apply the function to a collection of ParticlesTemp */
  virtual void apply(Model *m, const ParticleIndexes &o,
                     DerivativeAccumulator &da) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply(m, o[i], da);
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


//! Create a functor which can be used with build in C++ and python commands
/** For example, you can do
    \code
    std::for_each(particles.begin(), particles.end(),
                  IMP::SingletonFunctor(new IMP::core::Transform(tr)));
    IMP::for_each(particles,
                  IMP::SingletonFunctor(new IMP::core::Transform(tr)));
    \endcode
    in C++ (the second can be used with when \c particles is a temporary
    value) or
    \verbatim
    map(SingletonFunctor(Transform(tr)), particles)
    \endverbatim
    in python.

    \see IMP::SingletonModifier
 */
class SingletonDerivativeFunctor {
  Pointer<const SingletonDerivativeModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the SingletonModifier and the optional DerivativeAccumulator
  SingletonDerivativeFunctor(const SingletonDerivativeModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.");
  }
  void operator()( Particle* p) const {
    f_->apply(p, *da_);
  }
};

IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_DERIVATIVE_MODIFIER_H */
