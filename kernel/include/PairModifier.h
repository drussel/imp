/**
 *  \file PairModifier.h    \brief A Modifier on ParticlePairs
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_MODIFIER_H
#define IMP_PAIR_MODIFIER_H

#include "kernel_config.h"
#include "internal/container_helpers.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "VectorOfRefCounted.h"
#include "ParticleTuple.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticlePairs
/** The primary function of such a class is to change
    the passed particles.

    A given PairModifier may only work when passed a
    DerivativeAccumulator or when not passed one.

    \see IMP::PairFunctor

    Implementors should see IMP_PAIR_MODIFIER() and
    IMP_PAIR_MODIFIER_DA().
 */
class IMPEXPORT PairModifier : public Object
{
public:
  PairModifier(std::string name="PairModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(const ParticlePair& vt,
                     DerivativeAccumulator &da) const {
    IMP_FAILURE("This PairModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a single value*/
  virtual void apply(const ParticlePair& vt) const {
    IMP_FAILURE("This PairModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticlePairs */
  virtual void apply(const ParticlePairsTemp &o) const {
    IMP_FAILURE("This PairModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticlePairs */
  virtual void apply(const ParticlePairsTemp &o,
                     DerivativeAccumulator &da) const {
    IMP_FAILURE("This PairModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Get the set of interactions induced by applying to the
      argument.*/
  virtual ParticlesList
    get_interacting_particles(const ParticlePair& vt) const =0;

  /** Get the set of particles read when applied to the arguments.*/
  virtual ParticlesTemp
    get_input_particles(const ParticlePair& vt) const =0;
  /** Get the set of particles modifier when applied to the arguments.*/
  virtual ParticlesTemp
    get_output_particles(const ParticlePair& vt) const =0;
  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(const ParticlePair& vt) const =0;
  /** Get the set of output containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_output_containers(const ParticlePair& vt) const =0;
};

IMP_OUTPUT_OPERATOR(PairModifier);


IMP_OBJECTS(PairModifier,PairModifiers);

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

    \see IMP::PairModifier
 */
class PairFunctor {
  Pointer<const PairModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the PairModifier and the optional DerivativeAccumulator
  PairFunctor(const PairModifier *f): f_(f), da_(NULL){}
  PairFunctor(const PairModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.");
  }
  void operator()( ParticlePair p) const {
    if (da_) {
      f_->apply(p, *da_);
    } else {
      f_->apply(p);
    }
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_PAIR_MODIFIER_H */
