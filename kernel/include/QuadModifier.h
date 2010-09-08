/**
 *  \file QuadModifier.h    \brief A Modifier on ParticleQuadsTemp
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_QUAD_MODIFIER_H
#define IMP_QUAD_MODIFIER_H

#include "kernel_config.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "ParticleTuple.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of ParticleQuadsTemp
/** The primary function of such a class is to change
    the passed particles.

    A given QuadModifier may only work when passed a
    DerivativeAccumulator or when not passed one.

    \see IMP::QuadFunctor

    Implementors should see IMP_QUAD_MODIFIER() and
    IMP_QUAD_MODIFIER_DA().
 */
class IMPEXPORT QuadModifier : public Object
{
public:
  typedef ParticleQuad Argument;
  QuadModifier(std::string name="QuadModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(const ParticleQuad&,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This QuadModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a single value*/
  virtual void apply(const ParticleQuad&) const {
    IMP_FAILURE("This QuadModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticleQuadsTemp */
  virtual void apply(const ParticleQuadsTemp &) const {
    IMP_FAILURE("This QuadModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of ParticleQuadsTemp */
  virtual void apply(const ParticleQuadsTemp &,
                     DerivativeAccumulator &) const {
    IMP_FAILURE("This QuadModifier must be called without a"
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

IMP_OUTPUT_OPERATOR(QuadModifier);


IMP_OBJECTS(QuadModifier,QuadModifiers);

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

    \see IMP::QuadModifier
 */
class QuadFunctor {
  Pointer<const QuadModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the QuadModifier and the optional DerivativeAccumulator
  QuadFunctor(const QuadModifier *f): f_(f), da_(NULL){}
  QuadFunctor(const QuadModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.");
  }
  void operator()( ParticleQuad p) const {
    if (da_) {
      f_->apply(p, *da_);
    } else {
      f_->apply(p);
    }
  }
};



IMP_END_NAMESPACE

#endif  /* IMP_QUAD_MODIFIER_H */
