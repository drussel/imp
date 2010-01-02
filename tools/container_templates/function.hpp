/**
 *  \file GroupnameModifier.h    \brief A Modifier on Classnames
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMP_GROUPNAME_MODIFIER_H
#define IMP_GROUPNAME_MODIFIER_H

#include "config.h"
#include "internal/container_helpers.h"
#include "DerivativeAccumulator.h"
#include "base_types.h"
#include "VectorOfRefCounted.h"

IMP_BEGIN_NAMESPACE
// to keep swig happy
class Particle;

//! A base class for modifiers of Classnames
/** The primary function of such a class is to change
    the passed particles.

    A given GroupnameModifier may only work when passed a
    DerivativeAccumulator or when not passed one.

    \see IMP::GroupnameFunctor

    Implementors should see IMP_GROUPNAME_MODIFIER() and
    IMP_GROUPNAME_MODIFIER_DA().
 */
class IMPEXPORT GroupnameModifier : public Object
{
public:
  GroupnameModifier(std::string name="GroupnameModifier %1%");

  /** Apply the function to a single value*/
  virtual void apply(PassValue vt,
                     DerivativeAccumulator &da) const {
    IMP_FAILURE("This GroupnameModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a single value*/
  virtual void apply(PassValue vt) const {
    IMP_FAILURE("This GroupnameModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of Classnames */
  virtual void apply(const ClassnamesTemp &o) const {
    IMP_FAILURE("This GroupnameModifier must be called with a"
                << " DerivativeAccumulator.");
  }

  /** Apply the function to a collection of Classnames */
  virtual void apply(const ClassnamesTemp &o,
                     DerivativeAccumulator &da) const {
    IMP_FAILURE("This GroupnameModifier must be called without a"
                << " DerivativeAccumulator.");
  }

  /** Get the set of interactions induced by applying to the
      argument.*/
  virtual ParticlesList
    get_interacting_particles(PassValue vt) const =0;

  /** Get the set of particles read when applied to the arguments.*/
  virtual ParticlesTemp
    get_input_particles(PassValue vt) const =0;
  /** Get the set of particles modifier when applied to the arguments.*/
  virtual ParticlesTemp
    get_output_particles(PassValue vt) const =0;
  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(PassValue vt) const =0;
  /** Get the set of output containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_output_containers(PassValue vt) const =0;
};

IMP_OUTPUT_OPERATOR(GroupnameModifier)


//! A collection
typedef VectorOfRefCounted<GroupnameModifier*> GroupnameModifiers;

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

    \see IMP::GroupnameModifier
 */
class GroupnameFunctor {
  Pointer<const GroupnameModifier> f_;
  DerivativeAccumulator *da_;
public:
  //! Store the GroupnameModifier and the optional DerivativeAccumulator
  GroupnameFunctor(const GroupnameModifier *f): f_(f), da_(NULL){}
  GroupnameFunctor(const GroupnameModifier *f,
                   DerivativeAccumulator *da): f_(f), da_(da){
    IMP_USAGE_CHECK(da_,
                    "The passed derivative accumulator should not be null.",
                    InvalidStateException);
  }
  void operator()( Value p) const {
    if (da_) {
      f_->apply(p, *da_);
    } else {
      f_->apply(p);
    }
  }
};


IMP_END_NAMESPACE

#endif  /* IMP_GROUPNAME_MODIFIER_H */
