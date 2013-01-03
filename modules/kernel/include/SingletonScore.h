/**
 *  \file IMP/SingletonScore.h
 *  \brief Define SingletonScore.
 *
 *  WARNING This file was generated from NAMEScore.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_SINGLETON_SCORE_H
#define IMPKERNEL_SINGLETON_SCORE_H

#include "kernel_config.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"
#include "internal/container_helpers.h"
#include <IMP/base/utility_macros.h>
#include "input_output_macros.h"

IMP_BEGIN_NAMESPACE

//! Abstract score function
/** SingletonScores will evaluate the score and derivatives
    for the passed particles. Use in conjunction with various
    restraints such as IMP::core::SingletonsRestraint or
    IMP::core::SingletonRestraint.

    Implementers should check out IMP_SINGLETON_SCORE().
*/
class IMPEXPORT SingletonScore : public base::Object
{
 public:
  typedef Particle* Argument;
  typedef ParticleIndex IndexArgument;
  typedef Particle* PassArgument;
  typedef ParticleIndex PassIndexArgument;
  typedef SingletonModifier Modifier;
  SingletonScore(std::string name="SingletonScore %1%");
  //! Compute the score and the derivative if needed.
  IMP_DEPRECATED_WARN
    virtual double evaluate(Particle* vt,
                            DerivativeAccumulator *da) const;

  //! Compute the score and the derivative if needed.
  virtual double evaluate_index(Model *m, ParticleIndex vt,
                                DerivativeAccumulator *da) const;

  /** Implementations
      for these are provided by the IMP_SINGLETON_SCORE()
      macro.
  */
  virtual double evaluate_indexes(Model *m,
                                  const ParticleIndexes &o,
                                  DerivativeAccumulator *da,
                                  unsigned int lower_bound,
                                  unsigned int upper_bound) const ;

  //! Compute the score and the derivative if needed.
  virtual double evaluate_if_good_index(Model *m,
                                        ParticleIndex vt,
                                        DerivativeAccumulator *da,
                                        double max)  const;

  /** Implementations
      for these are provided by the IMP_SINGLETON_SCORE()
      macro.
  */
  virtual double evaluate_if_good_indexes(Model *m,
                                          const ParticleIndexes &o,
                                          DerivativeAccumulator *da,
                                          double max,
                                          unsigned int lower_bound,
                                          unsigned int upper_bound)
    const;
  /** Decompose this pair score acting on the pair into a set of
      restraints. The scoring function and derivatives should
      be equal to the current score. The defualt implementation
      just returns this object bound to the pair.*/
  Restraints create_current_decomposition(Model *m,
                                          ParticleIndex vt) const;

  /** Overide this to return your own decomposition.*/
  IMP_PROTECTED_METHOD(virtual Restraints,
                       do_create_current_decomposition,
                       (Model *m, ParticleIndex vt), const,);

  IMP_INPUTS_DECL(SingletonScore);

  IMP_REF_COUNTED_DESTRUCTOR(SingletonScore);
};

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_SINGLETON_SCORE_H */
