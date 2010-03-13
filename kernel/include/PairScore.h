/**
 *  \file PairScore.h    \brief Define PairScore.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2010 IMP Inventors. All rights reserved.
 */

#ifndef IMP_PAIR_SCORE_H
#define IMP_PAIR_SCORE_H

#include "kernel_config.h"
#include "base_types.h"
#include "Particle.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"
#include "internal/container_helpers.h"

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
  PairScore(std::string name="PairScore %1%");
  //! Compute the score and the derivative if needed.
  virtual double evaluate(const ParticlePair& vt,
                          DerivativeAccumulator *da) const = 0;

#if !defined(IMP_DOXYGEN) && 2 != 1
  // backwards compatibility
  virtual double evaluate(Particle *a, Particle *b,
                          DerivativeAccumulator *da) const {
    return evaluate(ParticlePair(a,b), da);
  }
#endif

  /** Implementations
      for these are provided by the IMP_PAIR_SCORE()
      macro.
  */
  virtual double evaluate(const ParticlePairsTemp &o,
                          DerivativeAccumulator *da) const = 0;

  /** \name Incremental evaluation
      Compute how much the score has changed since the last evaluate
      (and writing derivatives if they have changed). Implementations
      for these are provided by the IMP_PAIR_SCORE() macro.
      @{
  */
  virtual double evaluate_change(const ParticlePair& vt,
                                 DerivativeAccumulator *da) const = 0;

  virtual double evaluate_change(const ParticlePairsTemp &o,
                                 DerivativeAccumulator *da) const = 0;
  virtual double evaluate_prechange(const ParticlePair& vt,
                                    DerivativeAccumulator *da) const = 0;
  virtual double evaluate_prechange(const ParticlePairsTemp &o,
                                    DerivativeAccumulator *da) const = 0;
  //! Return true if the score for vt might have changed.
  virtual bool get_is_changed(const ParticlePair& vt) const =0;
  /** @} */

  /** Get the set of interaction induced by applying to the
      argument. */
  virtual ParticlesList
    get_interacting_particles(const ParticlePair& vt) const =0;

  /** Get the set of particles read when applied to the arguments. */
  virtual ParticlesTemp
    get_input_particles(const ParticlePair& vt) const =0;

  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(const ParticlePair& vt) const =0;

  IMP_REF_COUNTED_DESTRUCTOR(PairScore);
};

IMP_OBJECTS(PairScore,PairScores);

IMP_END_NAMESPACE

#endif  /* IMP_PAIR_SCORE_H */
