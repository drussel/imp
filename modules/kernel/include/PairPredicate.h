/**
 *  \file PairPredicate.h    \brief Define PairPredicate.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2011 IMP Inventors. All rights reserved.
 */

#ifndef IMPKERNEL_PAIR_PREDICATE_H
#define IMPKERNEL_PAIR_PREDICATE_H

#include "kernel_config.h"
#include "base_types.h"
#include "ParticleTuple.h"
#include "DerivativeAccumulator.h"
#include "internal/container_helpers.h"

IMP_BEGIN_NAMESPACE

//! Abstract predicate function
/** A predicate is a function which returns one of a discrete set of
    values (eg -1, 0, 1 depending on whether a value is negative, zero
    or positive). PairPredicates will evaluate the predicate for the passed
    particles.

    Implementers should check out IMP_PAIR_PREDICATE().
*/
class IMPEXPORT PairPredicate : public base::Object
{
 public:
  typedef ParticlePair Argument;
  PairPredicate(std::string name="PairPredicate %1%");
  //! Compute the predicate.
  virtual int get_value(const ParticlePair& vt) const =0;

  /** Implementations
      for these are provided by the IMP_PAIR_PREDICATE()
      macro.
  */
  virtual Ints get_value(const ParticlePairsTemp &o) const {
    Ints ret(o.size());
    for (unsigned int i=0; i< o.size(); ++i) {
      ret[i]+= get_value(o[i]);
    }
    return ret;
  }



  //! Compute the predicate and the derivative if needed.
  virtual int get_value_index(Model *m, const ParticleIndexPair& vt) const {
    return get_value(internal::get_particle(m, vt));
  }

  /** Implementations
      for these are provided by the IMP_PAIR_PREDICATE()
      macro.
  */
  virtual Ints get_value_index(Model *m,
                                const ParticleIndexPairs &o) const {
    Ints ret(o.size());
    for (unsigned int i=0; i< o.size(); ++i) {
      ret[i]+= get_value_index(m, o[i]);
    }
    return ret;
  }



  /** Get the set of particles read when applied to the arguments. */
  virtual ParticlesTemp
    get_input_particles(Particle *p) const =0;

  /** Get the set of input containers when this modifier is applied to
      the arguments. */
  virtual ContainersTemp
    get_input_containers(Particle *p) const =0;

  /** Decompose this pair predicate acting on the pair into a set of
      restraints. The scoring function and derivatives should
      be equal to the current predicate. The defualt implementation
      just returns this object bound to the pair.*/
  Restraints create_current_decomposition(const ParticlePair& vt) const;

  IMP_REF_COUNTED_DESTRUCTOR(PairPredicate);
};

IMP_OBJECTS(PairPredicate,PairPredicates);



IMP_END_NAMESPACE

#endif  /* IMPKERNEL_PAIR_PREDICATE_H */
