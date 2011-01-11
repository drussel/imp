/**
 *  \file TripletModifier.h    \brief A Modifier on ParticleTripletsTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_TRIPLET_MODIFIER_H
#define IMP_TRIPLET_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticleTripletsTemp
/** The primary function of such a class is to change
    the passed particles.

    A given TripletModifier may only work when passed a
    DerivativeAccumulator or when not passed one.

    \see IMP::TripletFunctor

    Implementors should see IMP_TRIPLET_MODIFIER() and
    IMP_TRIPLET_MODIFIER_DA().
 */
class IMPEXPORT TripletModifier : public Object
{
public:
  typedef ParticleTriplet Argument;
  TripletModifier(std::string name="TripletModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(const ParticleTriplet&,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This TripletModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a single value*/
  virtual void apply(const ParticleTriplet&) const {
    IMP_FAILURE("This TripletModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticleTripletsTemp */
  virtual void apply(const ParticleTripletsTemp &) const {
    IMP_FAILURE("This TripletModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticleTripletsTemp */
  virtual void apply(const ParticleTripletsTemp &,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This TripletModifier must be called without a"
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

IMP_OUTPUT_OPERATOR(TripletModifier);


IMP_OBJECTS(TripletModifier,TripletModifiers);

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

    \see IMP::TripletModifier
 */
class TripletFunctor {
  Pointer<const TripletModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the TripletModifier and the optional DerivativeAccumulator
  TripletFunctor(const TripletModifier *f): f_(f), da_(NULL){}
  TripletFunctor(const TripletModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.");
  }
  void operator()( ParticleTriplet p) const {
    if (da_) {
      f_->apply(p, *da_);
    } else {
      f_->apply(p);
    }
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_TRIPLET_MODIFIER_H */
