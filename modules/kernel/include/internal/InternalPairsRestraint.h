/**
 *  \file CorePairsRestraint.h
 *  \brief Apply a PairScore to each Pair in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_INTERNAL_INTERNAL_PAIRS_RESTRAINT_H
#define IMPKERNEL_INTERNAL_INTERNAL_PAIRS_RESTRAINT_H

#include "../kernel_config.h"

#include <IMP/base/Pointer.h>
#include "../PairScore.h"
#include "../PairContainer.h"
#include "../Restraint.h"

#include <iostream>

IMP_BEGIN_INTERNAL_NAMESPACE

//! Applies a PairScore to each Pair in a list.
/** This restraint stores the used particles in a ParticlePairsTemp.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListPairContainer is used and the
    {add_, set_, clear_}particle_pair{s} methods can be used.

    \see PairRestraint
 */
class IMPEXPORT InternalPairsRestraint :
  public Restraint
{
  IMP::OwnerPointer<PairScore> ss_;
  IMP::OwnerPointer<PairContainer> pc_, ac_, rc_;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  InternalPairsRestraint(PairScore *ss,
                      PairContainer *pc,
                      std::string name="PairsRestraint %1%");

  IMP_RESTRAINT(InternalPairsRestraint);

  ParticlePairsTemp get_arguments() const {
    return pc_->get();
  }

  PairScore* get_score() const {
    return ss_;
  }

  PairContainer* get_container() const {
    return pc_;
  }
#ifndef IMP_DOXYGEN
  Restraints do_create_decomposition() const;

  Restraints do_create_current_decomposition() const;
#endif
  double unprotected_evaluate_if_good(DerivativeAccumulator *da,
                                      double max) const;
};

IMP_END_INTERNAL_NAMESPACE

#endif  /* IMPKERNEL_INTERNAL_INTERNAL_PAIRS_RESTRAINT_H */
