/**
 *  \file CoreTripletsRestraint.h
 *  \brief Apply a TripletScore to each Triplet in a list.
 *
 *  This file is generated by a script (core/tools/make-container).
 *  Do not edit directly.
 *
 *  Copyright 2007-2012 IMP Inventors. All rights reserved.
 *
 */

#ifndef IMPKERNEL_INTERNAL_INTERNAL_TRIPLETS_RESTRAINT_H
#define IMPKERNEL_INTERNAL_INTERNAL_TRIPLETS_RESTRAINT_H

#include "../kernel_config.h"

#include <IMP/base/Pointer.h>
#include "../TripletScore.h"
#include "../TripletContainer.h"
#include "../Restraint.h"

#include <iostream>

IMP_BEGIN_INTERNAL_NAMESPACE

//! Applies a TripletScore to each Triplet in a list.
/** This restraint stores the used particles in a ParticleTripletsTemp.
    The container used can be set so that the list can be shared
    with other containers (or a nonbonded list can be used). By default
    a ListTripletContainer is used and the
    {add_, set_, clear_}particle_triplet{s} methods can be used.

    \see TripletRestraint
 */
class IMPEXPORT InternalTripletsRestraint :
  public Restraint
{
  IMP::OwnerPointer<TripletScore> ss_;
  IMP::OwnerPointer<TripletContainer> pc_, ac_, rc_;
public:

 //! Create the restraint with a shared container
  /** \param[in] ss The function to apply to each particle.
      \param[in] pc The container containing the stored particles. This
      container is not copied.
      \param[in] name The object name
   */
  InternalTripletsRestraint(TripletScore *ss,
                      TripletContainer *pc,
                      std::string name="TripletsRestraint %1%");

  IMP_RESTRAINT(InternalTripletsRestraint);

  ParticleTripletsTemp get_arguments() const {
    return pc_->get();
  }

  TripletScore* get_score() const {
    return ss_;
  }

  TripletContainer* get_container() const {
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

#endif  /* IMPKERNEL_INTERNAL_INTERNAL_TRIPLETS_RESTRAINT_H */