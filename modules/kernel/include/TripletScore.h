/**
 *  \file IMP/TripletScore.h
 *  \brief Define TripletScore.
 *
 *  WARNING This file was generated from NAMEScore.hpp
 *  in tools/maintenance/container_templates/kernel
 *  by tools/maintenance/make-container.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_TRIPLET_SCORE_H
#define IMPKERNEL_TRIPLET_SCORE_H

#include "kernel_config.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"
#include "internal/container_helpers.h"
#include <IMP/base/utility_macros.h>
#include "input_output_macros.h"

IMP_BEGIN_NAMESPACE

//! Abstract score function
/** TripletScores will evaluate the score and derivatives
    for the passed particles. Use in conjunction with various
    restraints such as IMP::core::TripletsRestraint or
    IMP::core::TripletRestraint.

    Implementers should check out IMP_TRIPLET_SCORE().
*/
class IMPEXPORT TripletScore : public base::Object
{
 public:
  typedef ParticleTriplet Argument;
  typedef ParticleIndexTriplet IndexArgument;
  typedef const ParticleTriplet& PassArgument;
  typedef const ParticleIndexTriplet& PassIndexArgument;
  typedef TripletModifier Modifier;
  TripletScore(std::string name="TripletScore %1%");
  //! Compute the score and the derivative if needed.
  IMP_DEPRECATED_WARN
    virtual double evaluate(const ParticleTriplet& vt,
                            DerivativeAccumulator *da) const;

  //! Compute the score and the derivative if needed.
  virtual double evaluate_index(Model *m, const ParticleIndexTriplet& vt,
                                DerivativeAccumulator *da) const;

  /** Implementations
      for these are provided by the IMP_TRIPLET_SCORE()
      macro.
  */
  virtual double evaluate_indexes(Model *m,
                                  const ParticleIndexTriplets &o,
                                  DerivativeAccumulator *da,
                                  unsigned int lower_bound,
                                  unsigned int upper_bound) const ;

  //! Compute the score and the derivative if needed.
  virtual double evaluate_if_good_index(Model *m,
                                        const ParticleIndexTriplet& vt,
                                        DerivativeAccumulator *da,
                                        double max)  const;

  /** Implementations
      for these are provided by the IMP_TRIPLET_SCORE()
      macro.
  */
  virtual double evaluate_if_good_indexes(Model *m,
                                          const ParticleIndexTriplets &o,
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
                                          const ParticleIndexTriplet& vt) const;

  /** Overide this to return your own decomposition.*/
  IMP_PROTECTED_METHOD(virtual Restraints,
                       do_create_current_decomposition,
                       (Model *m, const ParticleIndexTriplet& vt), const,);

  IMP_INPUTS_DECL(TripletScore);

  IMP_REF_COUNTED_DESTRUCTOR(TripletScore);
};

IMP_END_NAMESPACE

#endif  /* IMPKERNEL_TRIPLET_SCORE_H */
