/**
 *  \file SingletonModifier.h    \brief A Modifier on ParticlesTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_SINGLETON_MODIFIER_H
#define IMP_SINGLETON_MODIFIER_H

#include "kernel_config.h"
#include "internal/container_helpers.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "VectorOfRefCounted.h"
#include "ParticleTuple.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticlesTemp
/** The primary function of such a class is to change
    the passed particles.

    A given SingletonModifier may only work when passed a
    DerivativeAccumulator or when not passed one.

    \see IMP::SingletonFunctor

    Implementors should see IMP_SINGLETON_MODIFIER() and
    IMP_SINGLETON_MODIFIER_DA().
 */
class IMPEXPORT SingletonModifier : public Object
{
public:
  typedef Particle* Argument;
  SingletonModifier(std::string name="SingletonModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(Particle*,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This SingletonModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a single value*/
  virtual void apply(Particle*) const {
    IMP_FAILURE("This SingletonModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticlesTemp */
  virtual void apply(const ParticlesTemp &) const {
    IMP_FAILURE("This SingletonModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticlesTemp */
  virtual void apply(const ParticlesTemp &,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This SingletonModifier must be called without a"
                << " DerivativeAccumulator.");
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

IMP_OUTPUT_OPERATOR(SingletonModifier);


IMP_OBJECTS(SingletonModifier,SingletonModifiers);

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
class SingletonFunctor {
  Pointer<const SingletonModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the SingletonModifier and the optional DerivativeAccumulator
  SingletonFunctor(const SingletonModifier *f): f_(f), da_(NULL){}
  SingletonFunctor(const SingletonModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.");
  }
  void operator()( Particle* p) const {
    if (da_) {
      f_->apply(p, *da_);
    } else {
      f_->apply(p);
    }
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_SINGLETON_MODIFIER_H */
