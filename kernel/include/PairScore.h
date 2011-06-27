/**
 *  \file PairScore.h    \brief Define PairScore.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_SCORE_H
#define IMP_PAIR_SCORE_H

#include "kernel_config.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"

IMP_BEGIN_NAMESPACE

//! Abstract score function
/** PairScores will evaluate the score and derivatives
    for the passed particles. Use in conjunction with various
    restraints such as IMP::core::PairsRestraint or
    IMP::core::PairRestraint.

    Implementers should check out IMP_PAIR_SCORE().
*/
class IMPEXPORT PairScore : public Object
{
 public:
  typedef ParticlePair Argument;
  PairScore(std::string name="PairScore %1%");
  //! Compute the score and the derivative if needed.
  virtual double evaluate(const ParticlePair& vt,
                          DerivativeAccumulator *da) const =0;

  /** Implementations
      for these are provided by the IMP_PAIR_SCORE()
      macro.
  */
  virtual double evaluate(const ParticlePairsTemp &o,
                          DerivativeAccumulator *da) const =0;


  //! Compute the score and the derivative if needed.
  virtual double evaluate_if_good(const ParticlePair& vt,
                                  DerivativeAccumulator *da,
                                  double max) const =0;

  /** Implementations
      for these are provided by the IMP_PAIR_SCORE()
      macro.
  */
  virtual double evaluate_if_good(const ParticlePairsTemp &o,
                                  DerivativeAccumulator *da,
                                  double max) const =0;

  /** Get the set of particles read when applied to the arguments. */
  virtual ParticlesTemp
    get_input_particles(Particle *p) const =0;

  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;

  /** Decompose this pair score acting on the pair into a set of
      restraints. The scoring function and derivatives should
      be equal to the current score. The defualt implementation
      just returns this object bound to the pair.*/
  Restraints get_instant_decomposition(const ParticlePair& vt) const;

  IMP_REF_COUNTED_DESTRUCTOR(PairScore);
};

IMP_OBJECTS(PairScore,PairScores);


/** A PairScoreRestraint is a restraint where the score (and
    derivative values) can be decomposed into an application
    of a PairScore onto a ParticlePair.
*/
class IMPEXPORT PairScoreRestraint: public Restraint {
public:
  PairScoreRestraint(std::string name);
  virtual PairScore *get_score() const =0;
  virtual ParticlePair get_argument() const=0;
};

IMP_OBJECTS(PairScoreRestraint, PairScoreRestraints);

/** A PairsScoreRestraint is a restraint where the score (and
    derivative values) can be decomposed into a series of applications
    of a PairScore onto a ParticlePair.
*/
class IMPEXPORT PairsScoreRestraint: public Restraint {
public:
  PairsScoreRestraint(std::string name);
  virtual PairScore *get_score() const =0;
  virtual ParticlePairsTemp get_arguments() const=0;
};

IMP_OBJECTS(PairsScoreRestraint, PairsScoreRestraints);

IMP_END_NAMESPACE

#endif  /* IMP_PAIR_SCORE_H */
