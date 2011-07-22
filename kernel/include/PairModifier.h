/**
 *  \file PairModifier.h    \brief A Modifier on ParticlePairsTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_MODIFIER_H
#define IMP_PAIR_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "internal/container_helpers.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticlePairsTemp
/** The primary function of such a class is to change
    the passed particles.

    \see IMP::PairFunctor

    Implementors should see IMP_PAIR_MODIFIER(). Also see
    PairDerivativeModifier.
 */
class IMPEXPORT PairModifier : public Object
{
public:
  typedef ParticlePair Argument;
  PairModifier(std::string name="PairModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(const ParticlePair&) const =0;

  /** Apply the function to a collection of ParticlePairsTemp */
  virtual void apply(const ParticlePairsTemp &o) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply(o[i]);
    }
  }

 /** Apply the function to a single value*/
  virtual void apply(Model *m, const ParticleIndexPair& v) const {
    apply(internal::get_particle(m, v));
  }

  /** Apply the function to a collection of ParticlePairsTemp */
  virtual void apply(Model *m, const ParticleIndexPairs &o) const {
    for (unsigned int i=0; i < o.size(); ++i) {
      apply(m, o[i]);
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
public:
  //! Store the PairModifier and the optional DerivativeAccumulator
  PairFunctor(const PairModifier *f): f_(f){}
  void operator()( ParticlePair p) const {
    f_->apply(p);
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_PAIR_MODIFIER_H */
