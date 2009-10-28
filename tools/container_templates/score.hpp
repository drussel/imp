/**
 *  \file GroupnameScore.h    \brief Define GroupnameScore.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-9 Sali Lab. All rights reserved.
 */

#ifndef IMP_GROUPNAME_SCORE_H
#define IMP_GROUPNAME_SCORE_H

#include "config.h"
#include "base_types.h"
#include "Interaction.h"
#include "Particle.h"
#include "DerivativeAccumulator.h"

IMP_BEGIN_NAMESPACE

//! Abstract score function
/** GroupnameScores will evaluate the score and derivatives
    for the passed particles. Use in conjunction with various
    restraints such as IMP::core::GroupnamesRestraint or
    IMP::core::GroupnameRestraint.

    Implementers should check out IMP_GROUPNAME_SCORE().
*/
class IMPEXPORT GroupnameScore : public Object
{
public:
  GroupnameScore(std::string name="GroupnameScore %1%");
  //! Compute the score and the derivative if needed.
  virtual double evaluate(ClassnameArguments,
                          DerivativeAccumulator *da) const = 0;

  /** An implementations
      for this is provided by the IMP_SINGLETON_SCORE,
      IMP_PAIR_SCORE macros.
  */
  virtual double evaluate(const ClassnamesTemp &o,
                          DerivativeAccumulator *da) const = 0;

  /** \name Incremental evaluation
      Compute how much the score has changed since the last evaluate
      (and writing derivatives if they have changed). Implementations
      for these are provided by the IMP_SINGLETON_SCORE,
      IMP_PAIR_SCORE macros.
      @{
  */
  virtual double evaluate_change(ClassnameArguments,
                                 DerivativeAccumulator *da) const = 0;

  virtual double evaluate_change(const ClassnamesTemp &o,
                                 DerivativeAccumulator *da) const = 0;
  virtual double evaluate_prechange(ClassnameArguments,
                                    DerivativeAccumulator *da) const = 0;
  virtual double evaluate_prechange(const ClassnamesTemp &o,
                                    DerivativeAccumulator *da) const = 0;

  /** @} */

  /** Get the set of interaction induced by applying to the
      argument. */
  virtual ParticlesList
    get_interacting_particles(ClassnameArguments) const =0;

  /** Get the set of particles read when applied to the arguments. */
  virtual ParticlesTemp
    get_input_particles(ClassnameArguments) const =0;

  IMP_REF_COUNTED_DESTRUCTOR(GroupnameScore)
};

IMP_OUTPUT_OPERATOR(GroupnameScore);

IMP_END_NAMESPACE

#endif  /* IMP_GROUPNAME_SCORE_H */
